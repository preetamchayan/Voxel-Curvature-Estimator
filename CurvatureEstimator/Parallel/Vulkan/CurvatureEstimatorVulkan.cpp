#include "CurvatureEstimatorVulkan.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr uint32_t kLocalSize = 64;
constexpr int kMaxCurveLength = 32;
constexpr uint32_t kOperationInnerSpace = 0;
constexpr uint32_t kOperationMarkInterior = 1;
constexpr uint32_t kOperationMarkFrontier = 2;
constexpr uint32_t kOperationEstimateCurvature = 3;

constexpr size_t kVoxelsPerPackedWord = 10;
constexpr uint32_t kPackedVoxelMask = 0x7u;

std::vector<uint32_t> packVoxels(const std::vector<unsigned char>& voxels) {
    std::vector<uint32_t> packed((voxels.size() + kVoxelsPerPackedWord - 1) /
                                  kVoxelsPerPackedWord, 0u);
    for (size_t id = 0; id < voxels.size(); ++id) {
        packed[id / kVoxelsPerPackedWord] |=
            (static_cast<uint32_t>(voxels[id]) & kPackedVoxelMask) <<
            ((id % kVoxelsPerPackedWord) * 3u);
    }
    return packed;
}

void unpackVoxels(const std::vector<uint32_t>& packed, std::vector<unsigned char>& voxels) {
    for (size_t id = 0; id < voxels.size(); ++id) {
        voxels[id] = static_cast<unsigned char>(
            (packed[id / kVoxelsPerPackedWord] >> ((id % kVoxelsPerPackedWord) * 3u)) &
             kPackedVoxelMask);
    }
}

struct PushConstants {
    int operation;
    int width;
    int height;
    int depth;
    int plane;
    int curveLength;
    int surfaceVoxelCount;
    int padding;
};

void checkVk(VkResult result, const char* operation) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(std::string(operation) + " failed with Vulkan error " + std::to_string(result));
    }
}

std::vector<char> readBinaryFile(const char* path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error(std::string("Unable to open Vulkan curvature shader: ") + path);
    }

    const size_t size = static_cast<size_t>(file.tellg());
    if (size == 0 || size % sizeof(uint32_t) != 0) {
        throw std::runtime_error(std::string("Invalid Vulkan curvature SPIR-V file: ") + path);
    }

    std::vector<char> bytes(size);
    file.seekg(0);
    file.read(bytes.data(), static_cast<std::streamsize>(size));
    return bytes;
}
}

CurvatureEstimatorVulkan::CurvatureEstimatorVulkan() {
    VkApplicationInfo applicationInfo{};
    applicationInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    applicationInfo.pApplicationName = "VoxelCurvatureEstimator";
    applicationInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    applicationInfo.pEngineName = "VoxelCurvatureEstimator";
    applicationInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    applicationInfo.apiVersion = VK_API_VERSION_1_0;

    VkInstanceCreateInfo instanceInfo{};
    instanceInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instanceInfo.pApplicationInfo = &applicationInfo;
    checkVk(vkCreateInstance(&instanceInfo, nullptr, &m_instance), "vkCreateInstance");

    uint32_t deviceCount = 0;
    checkVk(vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr), "vkEnumeratePhysicalDevices");
    if (deviceCount == 0) {
        throw std::runtime_error("No Vulkan physical devices found.");
    }
    std::vector<VkPhysicalDevice> devices(deviceCount);
    checkVk(vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data()), "vkEnumeratePhysicalDevices");
    for (VkPhysicalDevice device : devices) {
        const uint32_t family = findComputeQueueFamily(device);
        if (family != UINT32_MAX) {
            m_physicalDevice = device;
            m_computeQueueFamilyIndex = family;
            break;
        }
    }
    if (m_physicalDevice == VK_NULL_HANDLE) {
        throw std::runtime_error("No Vulkan compute queue family found.");
    }

    float priority = 1.0f;
    VkDeviceQueueCreateInfo queueInfo{};
    queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueInfo.queueFamilyIndex = m_computeQueueFamilyIndex;
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &priority;

    VkDeviceCreateInfo deviceInfo{};
    deviceInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    deviceInfo.queueCreateInfoCount = 1;
    deviceInfo.pQueueCreateInfos = &queueInfo;
    checkVk(vkCreateDevice(m_physicalDevice, &deviceInfo, nullptr, &m_device), "vkCreateDevice");
    vkGetDeviceQueue(m_device, m_computeQueueFamilyIndex, 0, &m_computeQueue);

    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.queueFamilyIndex = m_computeQueueFamilyIndex;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    checkVk(vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool), "vkCreateCommandPool");

    VkCommandBufferAllocateInfo commandBufferInfo{};
    commandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    commandBufferInfo.commandPool = m_commandPool;
    commandBufferInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    commandBufferInfo.commandBufferCount = 1;
    checkVk(vkAllocateCommandBuffers(m_device, &commandBufferInfo, &m_commandBuffer), "vkAllocateCommandBuffers");

    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(m_physicalDevice, &properties);
    createPipeline();
    std::cout << "Initializing Vulkan curvature environment on " << properties.deviceName
              << " (max storage-buffer range: " << properties.limits.maxStorageBufferRange
              << " bytes)..." << std::endl;
}

CurvatureEstimatorVulkan::~CurvatureEstimatorVulkan() {
    cleanup();
}

uint32_t CurvatureEstimatorVulkan::findComputeQueueFamily(VkPhysicalDevice device) const {
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());
    for (uint32_t index = 0; index < count; ++index) {
        if (families[index].queueCount > 0 && (families[index].queueFlags & VK_QUEUE_COMPUTE_BIT)) {
            return index;
        }
    }
    return UINT32_MAX;
}

uint32_t CurvatureEstimatorVulkan::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const {
    VkPhysicalDeviceMemoryProperties memoryProperties{};
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memoryProperties);
    for (uint32_t index = 0; index < memoryProperties.memoryTypeCount; ++index) {
        if ((typeFilter & (1u << index)) &&
            (memoryProperties.memoryTypes[index].propertyFlags & properties) == properties) {
            return index;
        }
    }
    throw std::runtime_error("No host-visible, coherent Vulkan memory type found.");
}

void CurvatureEstimatorVulkan::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                                             VkBuffer& buffer, VkDeviceMemory& memory) const {
    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = std::max<VkDeviceSize>(size, 4);
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    checkVk(vkCreateBuffer(m_device, &bufferInfo, nullptr, &buffer), "vkCreateBuffer");

    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
    VkMemoryAllocateInfo allocationInfo{};
    allocationInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocationInfo.allocationSize = requirements.size;
    allocationInfo.memoryTypeIndex = findMemoryType(
        requirements.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    checkVk(vkAllocateMemory(m_device, &allocationInfo, nullptr, &memory), "vkAllocateMemory");
    checkVk(vkBindBufferMemory(m_device, buffer, memory, 0), "vkBindBufferMemory");
}

void CurvatureEstimatorVulkan::createPipeline() {
    std::array<VkDescriptorSetLayoutBinding, 3> bindings{};
    for (uint32_t binding = 0; binding < bindings.size(); ++binding) {
        bindings[binding].binding = binding;
        bindings[binding].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[binding].descriptorCount = 1;
        bindings[binding].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    checkVk(vkCreateDescriptorSetLayout(m_device, &layoutInfo, nullptr, &m_descriptorSetLayout), "vkCreateDescriptorSetLayout");

    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushRange.size = sizeof(PushConstants);
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushRange;
    checkVk(vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &m_pipelineLayout), "vkCreatePipelineLayout");

    const std::vector<char> shaderCode = readBinaryFile("CurvatureEstimator/Parallel/Vulkan/CurvatureEstimatorKernel.spv");
    VkShaderModuleCreateInfo shaderInfo{};
    shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    shaderInfo.codeSize = shaderCode.size();
    shaderInfo.pCode = reinterpret_cast<const uint32_t*>(shaderCode.data());
    VkShaderModule shader = VK_NULL_HANDLE;
    checkVk(vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shader), "vkCreateShaderModule");

    VkPipelineShaderStageCreateInfo stageInfo{};
    stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stageInfo.module = shader;
    stageInfo.pName = "main";
    VkComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = m_pipelineLayout;
    checkVk(vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_pipeline), "vkCreateComputePipelines");
    vkDestroyShaderModule(m_device, shader, nullptr);
}

void CurvatureEstimatorVulkan::submitAndWait() {
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    VkFence fence = VK_NULL_HANDLE;
    checkVk(vkCreateFence(m_device, &fenceInfo, nullptr, &fence), "vkCreateFence");
    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffer;
    checkVk(vkQueueSubmit(m_computeQueue, 1, &submitInfo, fence), "vkQueueSubmit");
    checkVk(vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");
    vkDestroyFence(m_device, fence, nullptr);
}

void CurvatureEstimatorVulkan::preprocessVoxels(std::vector<unsigned char>& voxels, const Dimensions3i& dims) {
    std::cout << "Preprocessing curvature voxels using Vulkan..." << std::endl;
    VkBuffer voxelsBuffer = VK_NULL_HANDLE;
    VkDeviceMemory voxelsMemory = VK_NULL_HANDLE;
    VkBuffer unusedCurvaturesBuffer = VK_NULL_HANDLE;
    VkDeviceMemory unusedCurvaturesMemory = VK_NULL_HANDLE;
    VkBuffer unusedSurfaceIdsBuffer = VK_NULL_HANDLE;
    VkDeviceMemory unusedSurfaceIdsMemory = VK_NULL_HANDLE;
    std::vector<uint32_t> shaderVoxels = packVoxels(voxels);
    const VkDeviceSize voxelBytes = shaderVoxels.size() * sizeof(uint32_t);
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(m_physicalDevice, &properties);
    if (voxelBytes > properties.limits.maxStorageBufferRange) {
        throw std::runtime_error(
            "Vulkan curvature voxel buffer requires " + std::to_string(voxelBytes) +
            " bytes, exceeding this device's maxStorageBufferRange of " +
            std::to_string(properties.limits.maxStorageBufferRange) +
            " bytes. Chunk preprocessing into storage-buffer-range-sized slabs before dispatching.");
    }
    createBuffer(voxelBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, voxelsBuffer, voxelsMemory);
    createBuffer(sizeof(int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, unusedCurvaturesBuffer, unusedCurvaturesMemory);
    createBuffer(sizeof(int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, unusedSurfaceIdsBuffer, unusedSurfaceIdsMemory);

    void* mapped = nullptr;
    checkVk(vkMapMemory(m_device, voxelsMemory, 0, voxelBytes, 0, &mapped), "vkMapMemory preprocess upload");
    std::memcpy(mapped, shaderVoxels.data(), voxelBytes);
    vkUnmapMemory(m_device, voxelsMemory);

    VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3};
    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.maxSets = 1;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    checkVk(vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &m_descriptorPool), "vkCreateDescriptorPool");
    VkDescriptorSetAllocateInfo allocationInfo{};
    allocationInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocationInfo.descriptorPool = m_descriptorPool;
    allocationInfo.descriptorSetCount = 1;
    allocationInfo.pSetLayouts = &m_descriptorSetLayout;
    checkVk(vkAllocateDescriptorSets(m_device, &allocationInfo, &m_descriptorSet), "vkAllocateDescriptorSets");

    std::array<VkDescriptorBufferInfo, 3> infos{{
        {voxelsBuffer, 0, VK_WHOLE_SIZE},
        {unusedCurvaturesBuffer, 0, VK_WHOLE_SIZE},
        {unusedSurfaceIdsBuffer, 0, VK_WHOLE_SIZE},
    }};
    std::array<VkWriteDescriptorSet, 3> writes{};
    for (uint32_t binding = 0; binding < writes.size(); ++binding) {
        writes[binding].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[binding].dstSet = m_descriptorSet;
        writes[binding].dstBinding = binding;
        writes[binding].descriptorCount = 1;
        writes[binding].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[binding].pBufferInfo = &infos[binding];
    }
    vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

        // Packed voxel preprocessing path.
    checkVk(vkResetCommandBuffer(m_commandBuffer, 0), "vkResetCommandBuffer preprocess");
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    checkVk(vkBeginCommandBuffer(m_commandBuffer, &beginInfo), "vkBeginCommandBuffer preprocess");
    vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipeline);
    vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);

    const auto dispatch = [&](int operation, int plane, uint32_t xGroups, uint32_t yGroups, uint32_t zGroups) {
        PushConstants constants{operation, dims.width, dims.height, dims.depth, plane, 0, 0, 0};
        vkCmdPushConstants(m_commandBuffer, m_pipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(constants), &constants);
        vkCmdDispatch(m_commandBuffer, xGroups, yGroups, zGroups);
        VkMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        vkCmdPipelineBarrier(m_commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             0, 1, &barrier, 0, nullptr, 0, nullptr);
    };
    // The shader uses local_size_x = 64, so X dispatch counts are workgroup counts.
    // Y and Z local sizes are one and therefore use their logical dimensions directly.
    const auto xGroups = [](int count) {
        return static_cast<uint32_t>((count + static_cast<int>(kLocalSize) - 1) / static_cast<int>(kLocalSize));
    };
    dispatch(kOperationInnerSpace, 2, xGroups(dims.width), dims.height, 1);
    dispatch(kOperationInnerSpace, 1, xGroups(dims.depth), dims.width, 1);
    dispatch(kOperationInnerSpace, 0, xGroups(dims.height), dims.depth, 1);
    dispatch(kOperationMarkInterior, 0, xGroups(dims.width), dims.height, dims.depth);
    dispatch(kOperationMarkFrontier, 0, xGroups(dims.width), dims.height, dims.depth);
    checkVk(vkEndCommandBuffer(m_commandBuffer), "vkEndCommandBuffer preprocess");
    submitAndWait();

    checkVk(vkMapMemory(m_device, voxelsMemory, 0, voxelBytes, 0, &mapped), "vkMapMemory preprocess download");
    std::memcpy(shaderVoxels.data(), mapped, voxelBytes);
    vkUnmapMemory(m_device, voxelsMemory);
    unpackVoxels(shaderVoxels, voxels);

    const size_t surfaceVoxelCount = static_cast<size_t>(std::count(voxels.begin(), voxels.end(), static_cast<unsigned char>(1)));
    const size_t interiorVoxelCount = static_cast<size_t>(std::count(voxels.begin(), voxels.end(), static_cast<unsigned char>(2)));
    std::cout << "Vulkan curvature preprocessing result: "
              << surfaceVoxelCount << " surface voxels, "
              << interiorVoxelCount << " interior voxels" << std::endl;

    vkDestroyDescriptorPool(m_device, m_descriptorPool, nullptr); m_descriptorPool = VK_NULL_HANDLE;
    vkDestroyBuffer(m_device, unusedSurfaceIdsBuffer, nullptr); vkFreeMemory(m_device, unusedSurfaceIdsMemory, nullptr);
    vkDestroyBuffer(m_device, unusedCurvaturesBuffer, nullptr); vkFreeMemory(m_device, unusedCurvaturesMemory, nullptr);
    vkDestroyBuffer(m_device, voxelsBuffer, nullptr); vkFreeMemory(m_device, voxelsMemory, nullptr);
}

void CurvatureEstimatorVulkan::estimateCurvature(int curvLength, const std::vector<unsigned char>& voxels,
                                                 std::vector<int>& curvatures, const Dimensions3i& dims) {
    if (curvLength < 1 || curvLength > kMaxCurveLength) {
        throw std::runtime_error("Vulkan curvature curve length must be in [1, 32].");
    }
    if (voxels.size() != curvatures.size()) {
        throw std::runtime_error("Vulkan curvature input and output sizes differ.");
    }

    std::vector<int> surfaceVoxelIds;
    for (size_t id = 0; id < voxels.size(); ++id) {
        if (voxels[id] == 1) surfaceVoxelIds.push_back(static_cast<int>(id));
    }
    std::fill(curvatures.begin(), curvatures.end(), std::numeric_limits<int>::max());
    std::cout << "Estimating curvature using Vulkan..." << std::endl;
    std::cout << "Vulkan curvature dispatch: " << surfaceVoxelIds.size() << " surface voxels out of " << voxels.size() << " grid voxels" << std::endl;

    VkBuffer voxelsBuffer = VK_NULL_HANDLE, curvaturesBuffer = VK_NULL_HANDLE, surfaceIdsBuffer = VK_NULL_HANDLE;
    VkDeviceMemory voxelsMemory = VK_NULL_HANDLE, curvaturesMemory = VK_NULL_HANDLE, surfaceIdsMemory = VK_NULL_HANDLE;
    std::vector<uint32_t> shaderVoxels = packVoxels(voxels);
    std::vector<int> surfaceCurvatures(surfaceVoxelIds.size(), std::numeric_limits<int>::max());
    const VkDeviceSize voxelBytes = shaderVoxels.size() * sizeof(uint32_t);
    const VkDeviceSize curvatureBytes = std::max<size_t>(surfaceCurvatures.size(), 1) * sizeof(int);
    const VkDeviceSize idsBytes = std::max<size_t>(surfaceVoxelIds.size(), 1) * sizeof(int);
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(m_physicalDevice, &properties);
    if (voxelBytes > properties.limits.maxStorageBufferRange ||
        curvatureBytes > properties.limits.maxStorageBufferRange ||
        idsBytes > properties.limits.maxStorageBufferRange) {
        throw std::runtime_error("Packed Vulkan curvature buffers exceed this device's maxStorageBufferRange.");
    }
    createBuffer(voxelBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, voxelsBuffer, voxelsMemory);
    createBuffer(curvatureBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, curvaturesBuffer, curvaturesMemory);
    createBuffer(idsBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, surfaceIdsBuffer, surfaceIdsMemory);
    void* mapped = nullptr;
    checkVk(vkMapMemory(m_device, voxelsMemory, 0, voxelBytes, 0, &mapped), "vkMapMemory voxel upload"); std::memcpy(mapped, shaderVoxels.data(), voxelBytes); vkUnmapMemory(m_device, voxelsMemory);
    checkVk(vkMapMemory(m_device, curvaturesMemory, 0, curvatureBytes, 0, &mapped), "vkMapMemory curvature upload"); std::memcpy(mapped, surfaceCurvatures.data(), surfaceCurvatures.size() * sizeof(int)); vkUnmapMemory(m_device, curvaturesMemory);
    checkVk(vkMapMemory(m_device, surfaceIdsMemory, 0, idsBytes, 0, &mapped), "vkMapMemory ID upload"); if (!surfaceVoxelIds.empty()) std::memcpy(mapped, surfaceVoxelIds.data(), surfaceVoxelIds.size() * sizeof(int)); vkUnmapMemory(m_device, surfaceIdsMemory);

    VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3};
    VkDescriptorPoolCreateInfo poolInfo{}; poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; poolInfo.maxSets = 1; poolInfo.poolSizeCount = 1; poolInfo.pPoolSizes = &poolSize;
    checkVk(vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &m_descriptorPool), "vkCreateDescriptorPool estimate");
    VkDescriptorSetAllocateInfo allocationInfo{}; allocationInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; allocationInfo.descriptorPool = m_descriptorPool; allocationInfo.descriptorSetCount = 1; allocationInfo.pSetLayouts = &m_descriptorSetLayout;
    checkVk(vkAllocateDescriptorSets(m_device, &allocationInfo, &m_descriptorSet), "vkAllocateDescriptorSets estimate");
    std::array<VkDescriptorBufferInfo, 3> infos{{{voxelsBuffer, 0, VK_WHOLE_SIZE}, {curvaturesBuffer, 0, VK_WHOLE_SIZE}, {surfaceIdsBuffer, 0, VK_WHOLE_SIZE}}};
    std::array<VkWriteDescriptorSet, 3> writes{};
    for (uint32_t binding = 0; binding < writes.size(); ++binding) { writes[binding].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[binding].dstSet = m_descriptorSet; writes[binding].dstBinding = binding; writes[binding].descriptorCount = 1; writes[binding].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; writes[binding].pBufferInfo = &infos[binding]; }
    vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

    if (!surfaceVoxelIds.empty()) {
        checkVk(vkResetCommandBuffer(m_commandBuffer, 0), "vkResetCommandBuffer estimate");
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        checkVk(vkBeginCommandBuffer(m_commandBuffer, &beginInfo), "vkBeginCommandBuffer estimate");
        vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipeline);
        vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);

        VkMemoryBarrier uploadBarrier{};
        uploadBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        uploadBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
        uploadBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        vkCmdPipelineBarrier(m_commandBuffer, VK_PIPELINE_STAGE_HOST_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &uploadBarrier,
                             0, nullptr, 0, nullptr);

        PushConstants constants{static_cast<int>(kOperationEstimateCurvature), dims.width, dims.height, dims.depth, 0, curvLength, static_cast<int>(surfaceVoxelIds.size()), 0};
        vkCmdPushConstants(m_commandBuffer, m_pipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(constants), &constants);
        vkCmdDispatch(m_commandBuffer, static_cast<uint32_t>((surfaceVoxelIds.size() + kLocalSize - 1) / kLocalSize), 1, 1);

        VkMemoryBarrier readbackBarrier{};
        readbackBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        readbackBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        readbackBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(m_commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &readbackBarrier,
                             0, nullptr, 0, nullptr);
        checkVk(vkEndCommandBuffer(m_commandBuffer), "vkEndCommandBuffer estimate");
        submitAndWait();
    }

    checkVk(vkMapMemory(m_device, curvaturesMemory, 0, curvatureBytes, 0, &mapped), "vkMapMemory curvature download");
    if (!surfaceCurvatures.empty()) std::memcpy(surfaceCurvatures.data(), mapped, surfaceCurvatures.size() * sizeof(int));
    vkUnmapMemory(m_device, curvaturesMemory);
    for (size_t surfaceIndex = 0; surfaceIndex < surfaceVoxelIds.size(); ++surfaceIndex) {
        curvatures[static_cast<size_t>(surfaceVoxelIds[surfaceIndex])] = surfaceCurvatures[surfaceIndex];
    }
    vkDestroyDescriptorPool(m_device, m_descriptorPool, nullptr); m_descriptorPool = VK_NULL_HANDLE;
    vkDestroyBuffer(m_device, surfaceIdsBuffer, nullptr); vkFreeMemory(m_device, surfaceIdsMemory, nullptr);
    vkDestroyBuffer(m_device, curvaturesBuffer, nullptr); vkFreeMemory(m_device, curvaturesMemory, nullptr);
    vkDestroyBuffer(m_device, voxelsBuffer, nullptr); vkFreeMemory(m_device, voxelsMemory, nullptr);
}

void CurvatureEstimatorVulkan::cleanup() {
    if (m_device != VK_NULL_HANDLE) vkDeviceWaitIdle(m_device);
    if (m_descriptorPool != VK_NULL_HANDLE) vkDestroyDescriptorPool(m_device, m_descriptorPool, nullptr);
    if (m_pipeline != VK_NULL_HANDLE) vkDestroyPipeline(m_device, m_pipeline, nullptr);
    if (m_pipelineLayout != VK_NULL_HANDLE) vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
    if (m_descriptorSetLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(m_device, m_descriptorSetLayout, nullptr);
    if (m_commandBuffer != VK_NULL_HANDLE) vkFreeCommandBuffers(m_device, m_commandPool, 1, &m_commandBuffer);
    if (m_commandPool != VK_NULL_HANDLE) vkDestroyCommandPool(m_device, m_commandPool, nullptr);
    if (m_device != VK_NULL_HANDLE) vkDestroyDevice(m_device, nullptr);
    if (m_instance != VK_NULL_HANDLE) vkDestroyInstance(m_instance, nullptr);
}
