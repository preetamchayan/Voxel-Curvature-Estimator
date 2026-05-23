#include "VulkanEnv.h"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>

static void checkVk(VkResult res, const char* msg) {
    if (res != VK_SUCCESS) {
        std::cerr << "Vulkan error (" << res << ") at " << msg << std::endl;
        std::exit(1);
    }
}

VulkanEnv::VulkanEnv()
{
    std::cout << "Initializing Vulkan environment..." << std::endl;
    createInstance();
    pickPhysicalDevice();
    createLogicalDevice();
    createCommandPoolAndBuffer();
}

VulkanEnv::~VulkanEnv()
{
    std::cout << "Cleaning up Vulkan environment..." << std::endl;
    cleanup();
}

uint32_t VulkanEnv::findComputeQueueFamily(VkPhysicalDevice device)
{
    uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);
    if (queueFamilyCount == 0) return 0xFFFFFFFFu;
    std::vector<VkQueueFamilyProperties> families(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, families.data());
    for (uint32_t i = 0; i < queueFamilyCount; ++i) {
        if (families[i].queueCount > 0 && (families[i].queueFlags & VK_QUEUE_COMPUTE_BIT)) {
            // prefer compute-only queues
            if ((families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0) return i;
            return i;
        }
    }
    return 0xFFFFFFFFu;
}

uint32_t VulkanEnv::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties)
{
    VkPhysicalDeviceMemoryProperties memProperties;
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProperties);

    for (uint32_t i = 0; i < memProperties.memoryTypeCount; ++i) {
        if ((typeFilter & (1u << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }

    std::cerr << "Failed to find suitable memory type." << std::endl;
    std::exit(1);
}

void VulkanEnv::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &bufferMemory)
{
    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    checkVk(vkCreateBuffer(m_device, &bufferInfo, nullptr, &buffer), "vkCreateBuffer");

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(m_device, buffer, &memRequirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

    checkVk(vkAllocateMemory(m_device, &allocInfo, nullptr, &bufferMemory), "vkAllocateMemory");
    checkVk(vkBindBufferMemory(m_device, buffer, bufferMemory, 0), "vkBindBufferMemory");
}

std::vector<char> VulkanEnv::readFile(const std::string &filename)
{
    std::ifstream file(filename, std::ios::ate | std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return {};
    }

    size_t fileSize = (size_t)file.tellg();
    std::vector<char> buffer(fileSize);
    file.seekg(0);
    file.read(buffer.data(), fileSize);
    file.close();
    return buffer;
}

void VulkanEnv::createDescriptorSetLayout()
{
    VkDescriptorSetLayoutBinding bindings[4];
    for (uint32_t i = 0; i < 4; ++i) {
        bindings[i].binding = i;
        bindings[i].descriptorCount = 1;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[i].pImmutableSamplers = nullptr;
        bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 4;
    layoutInfo.pBindings = bindings;

    checkVk(vkCreateDescriptorSetLayout(m_device, &layoutInfo, nullptr, &m_descriptorSetLayout), "vkCreateDescriptorSetLayout");

    // push constant range for small parameters (numFaces, dims, offsets)
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushRange.offset = 0;
    pushRange.size = 32; // enough for several ints

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushRange;

    checkVk(vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &m_pipelineLayout), "vkCreatePipelineLayout");
}

void VulkanEnv::createDescriptorPoolAndSet()
{
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSize.descriptorCount = 4;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    poolInfo.maxSets = 1;

    checkVk(vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &m_descriptorPool), "vkCreateDescriptorPool");

    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = m_descriptorPool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &m_descriptorSetLayout;

    checkVk(vkAllocateDescriptorSets(m_device, &allocInfo, &m_descriptorSet), "vkAllocateDescriptorSets");

    VkDescriptorBufferInfo facesBufInfo{};
    facesBufInfo.buffer = m_facesBuffer;
    facesBufInfo.offset = 0;
    facesBufInfo.range = VK_WHOLE_SIZE;

    VkDescriptorBufferInfo vertsBufInfo{};
    vertsBufInfo.buffer = m_verticesBuffer;
    vertsBufInfo.offset = 0;
    vertsBufInfo.range = VK_WHOLE_SIZE;

    VkDescriptorBufferInfo voxelsBufInfo{};
    voxelsBufInfo.buffer = m_voxelsBuffer;
    voxelsBufInfo.offset = 0;
    voxelsBufInfo.range = VK_WHOLE_SIZE;

    VkDescriptorBufferInfo totalBufInfo{};
    totalBufInfo.buffer = m_totalSizeBuffer;
    totalBufInfo.offset = 0;
    totalBufInfo.range = VK_WHOLE_SIZE;

    VkWriteDescriptorSet descriptorWrites[4];
    descriptorWrites[0] = {};
    descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[0].dstSet = m_descriptorSet;
    descriptorWrites[0].dstBinding = 0;
    descriptorWrites[0].descriptorCount = 1;
    descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrites[0].pBufferInfo = &facesBufInfo;

    descriptorWrites[1] = {};
    descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[1].dstSet = m_descriptorSet;
    descriptorWrites[1].dstBinding = 1;
    descriptorWrites[1].descriptorCount = 1;
    descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrites[1].pBufferInfo = &vertsBufInfo;

    descriptorWrites[2] = {};
    descriptorWrites[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[2].dstSet = m_descriptorSet;
    descriptorWrites[2].dstBinding = 2;
    descriptorWrites[2].descriptorCount = 1;
    descriptorWrites[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrites[2].pBufferInfo = &voxelsBufInfo;

    descriptorWrites[3] = {};
    descriptorWrites[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[3].dstSet = m_descriptorSet;
    descriptorWrites[3].dstBinding = 3;
    descriptorWrites[3].descriptorCount = 1;
    descriptorWrites[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrites[3].pBufferInfo = &totalBufInfo;

    vkUpdateDescriptorSets(m_device, 4, descriptorWrites, 0, nullptr);
}

void VulkanEnv::createComputePipeline(const std::string &spvPath)
{
    auto code = readFile(spvPath);
    if (code.empty()) {
        std::cerr << "Compute SPIR-V not found: " << spvPath << std::endl;
        std::exit(1);
    }
    if (code.size() % 4 != 0) {
        std::cerr << "SPIR-V size not a multiple of 4: " << spvPath << std::endl;
        std::exit(1);
    }

    VkShaderModuleCreateInfo moduleInfo{};
    moduleInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    moduleInfo.codeSize = code.size();
    moduleInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

    VkShaderModule shaderModule;
    checkVk(vkCreateShaderModule(m_device, &moduleInfo, nullptr, &shaderModule), "vkCreateShaderModule");

    VkPipelineShaderStageCreateInfo stageInfo{};
    stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stageInfo.module = shaderModule;
    stageInfo.pName = "main";

    VkComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = m_pipelineLayout;

    checkVk(vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_computePipeline), "vkCreateComputePipelines");

    vkDestroyShaderModule(m_device, shaderModule, nullptr);
}

void VulkanEnv::destroyBuffers()
{
    if (m_facesBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(m_device, m_facesBuffer, nullptr);
        m_facesBuffer = VK_NULL_HANDLE;
    }
    if (m_facesMemory != VK_NULL_HANDLE) {
        vkFreeMemory(m_device, m_facesMemory, nullptr);
        m_facesMemory = VK_NULL_HANDLE;
    }
    if (m_verticesBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(m_device, m_verticesBuffer, nullptr);
        m_verticesBuffer = VK_NULL_HANDLE;
    }
    if (m_verticesMemory != VK_NULL_HANDLE) {
        vkFreeMemory(m_device, m_verticesMemory, nullptr);
        m_verticesMemory = VK_NULL_HANDLE;
    }
    if (m_voxelsBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(m_device, m_voxelsBuffer, nullptr);
        m_voxelsBuffer = VK_NULL_HANDLE;
    }
    if (m_voxelsMemory != VK_NULL_HANDLE) {
        vkFreeMemory(m_device, m_voxelsMemory, nullptr);
        m_voxelsMemory = VK_NULL_HANDLE;
    }
    if (m_totalSizeBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(m_device, m_totalSizeBuffer, nullptr);
        m_totalSizeBuffer = VK_NULL_HANDLE;
    }
    if (m_totalSizeMemory != VK_NULL_HANDLE) {
        vkFreeMemory(m_device, m_totalSizeMemory, nullptr);
        m_totalSizeMemory = VK_NULL_HANDLE;
    }
}

void VulkanEnv::createInstance()
{
    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "VoxelizerVulkan";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "NoEngine";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_0;

    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;

    VkResult res = vkCreateInstance(&createInfo, nullptr, &m_instance);
    checkVk(res, "vkCreateInstance");
}

void VulkanEnv::pickPhysicalDevice()
{
    uint32_t deviceCount = 0;
    checkVk(vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr), "vkEnumeratePhysicalDevices");
    if (deviceCount == 0) {
        std::cerr << "No Vulkan physical devices found." << std::endl;
        std::exit(1);
    }
    std::vector<VkPhysicalDevice> devices(deviceCount);
    checkVk(vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data()), "vkEnumeratePhysicalDevices");

    for (auto &dev : devices) {
        uint32_t idx = findComputeQueueFamily(dev);
        if (idx != 0xFFFFFFFFu) {
            m_physicalDevice = dev;
            m_computeQueueFamilyIndex = idx;
            break;
        }
    }
    if (m_physicalDevice == VK_NULL_HANDLE) {
        std::cerr << "No suitable Vulkan device with compute capability found." << std::endl;
        std::exit(1);
    }
}

void VulkanEnv::createLogicalDevice()
{
    float priority = 1.0f;
    VkDeviceQueueCreateInfo queueCreateInfo{};
    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCreateInfo.queueFamilyIndex = m_computeQueueFamilyIndex;
    queueCreateInfo.queueCount = 1;
    queueCreateInfo.pQueuePriorities = &priority;

    // Query support for VK_KHR_8bit_storage (via features2)
    VkPhysicalDevice8BitStorageFeatures storage8Features{};
    storage8Features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_8BIT_STORAGE_FEATURES;
    VkPhysicalDeviceFeatures2 features2{};
    features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    features2.pNext = &storage8Features;
    vkGetPhysicalDeviceFeatures2(m_physicalDevice, &features2);

    m_supports8bit = storage8Features.storageBuffer8BitAccess == VK_TRUE;

    // Check device extensions for VK_KHR_8bit_storage
    uint32_t extCount = 0;
    vkEnumerateDeviceExtensionProperties(m_physicalDevice, nullptr, &extCount, nullptr);
    std::vector<VkExtensionProperties> extProps(extCount);
    if (extCount > 0) vkEnumerateDeviceExtensionProperties(m_physicalDevice, nullptr, &extCount, extProps.data());
    bool has8bitExt = false;
    for (const auto &e : extProps) {
        if (std::strcmp(e.extensionName, "VK_KHR_8bit_storage") == 0) { has8bitExt = true; break; }
    }

    if (m_supports8bit && has8bitExt) {
        std::cout << "Device supports VK_KHR_8bit_storage; enabling 8-bit storage features." << std::endl;
    } else {
        std::cout << "VK_KHR_8bit_storage not available or not supported; falling back to 32-bit bitset voxel storage." << std::endl;
        m_supports8bit = false;
    }

    VkPhysicalDeviceFeatures deviceFeatures{};

    VkDeviceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.queueCreateInfoCount = 1;
    createInfo.pQueueCreateInfos = &queueCreateInfo;
    createInfo.pEnabledFeatures = &deviceFeatures;

    std::vector<const char*> enabledExtensions;
    if (has8bitExt) enabledExtensions.push_back("VK_KHR_8bit_storage");
    if (!enabledExtensions.empty()) {
        createInfo.enabledExtensionCount = static_cast<uint32_t>(enabledExtensions.size());
        createInfo.ppEnabledExtensionNames = enabledExtensions.data();
    }

    VkPhysicalDevice8BitStorageFeatures device8Features{};
    device8Features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_8BIT_STORAGE_FEATURES;
    device8Features.storageBuffer8BitAccess = m_supports8bit ? VK_TRUE : VK_FALSE;
    if (m_supports8bit) {
        createInfo.pNext = &device8Features;
    }

    checkVk(vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device), "vkCreateDevice");
    vkGetDeviceQueue(m_device, m_computeQueueFamilyIndex, 0, &m_computeQueue);
}

void VulkanEnv::createCommandPoolAndBuffer()
{
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.queueFamilyIndex = m_computeQueueFamilyIndex;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

    checkVk(vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool), "vkCreateCommandPool");

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = m_commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;

    checkVk(vkAllocateCommandBuffers(m_device, &allocInfo, &m_commandBuffer), "vkAllocateCommandBuffers");
}

void VulkanEnv::cleanup()
{
    if (m_device != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(m_device);
    }

    if (m_computePipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(m_device, m_computePipeline, nullptr);
        m_computePipeline = VK_NULL_HANDLE;
    }
    if (m_pipelineLayout != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
        m_pipelineLayout = VK_NULL_HANDLE;
    }
    if (m_descriptorSetLayout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(m_device, m_descriptorSetLayout, nullptr);
        m_descriptorSetLayout = VK_NULL_HANDLE;
    }
    if (m_descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(m_device, m_descriptorPool, nullptr);
        m_descriptorPool = VK_NULL_HANDLE;
    }
    if (m_commandBuffer != VK_NULL_HANDLE && m_commandPool != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(m_device, m_commandPool, 1, &m_commandBuffer);
        m_commandBuffer = VK_NULL_HANDLE;
    }
    if (m_commandPool != VK_NULL_HANDLE) {
        vkDestroyCommandPool(m_device, m_commandPool, nullptr);
        m_commandPool = VK_NULL_HANDLE;
    }
    if (m_device != VK_NULL_HANDLE) {
        vkDestroyDevice(m_device, nullptr);
        m_device = VK_NULL_HANDLE;
    }
    if (m_instance != VK_NULL_HANDLE) {
        vkDestroyInstance(m_instance, nullptr);
        m_instance = VK_NULL_HANDLE;
    }
}

void VulkanEnv::voxelize(
    std::vector<unsigned char> &voxels,
    const std::vector<Point3i> &vertices,
    const std::vector<Face> &faces,
    const BBox3i &scaledBounds,
    const Dimensions3i &dims)
{
    std::cout << "Voxelizing using Vulkan (preparing buffers, descriptors, pipeline)..." << std::endl;

    // Flatten faces and vertices into host arrays (ints)
    std::vector<int> flatFaces;
    flatFaces.reserve(faces.size() * 3);
    for (const auto &f : faces) {
        flatFaces.push_back(f.v1);
        flatFaces.push_back(f.v2);
        flatFaces.push_back(f.v3);
    }

    std::vector<int> flatVertices;
    flatVertices.reserve(vertices.size() * 3);
    for (const auto &v : vertices) {
        flatVertices.push_back(v.x);
        flatVertices.push_back(v.y);
        flatVertices.push_back(v.z);
    }

    VkDeviceSize facesSize = sizeof(int) * flatFaces.size();
    VkDeviceSize vertsSize = sizeof(int) * flatVertices.size();
    // Use a 32-bit bitset to store voxels on the device (1 bit per voxel)
    size_t numVoxels = static_cast<size_t>(dims.width) * static_cast<size_t>(dims.height) * static_cast<size_t>(dims.depth);
    size_t numWords = (numVoxels + 31) / 32;
    VkDeviceSize voxelsSizeWords = sizeof(uint32_t) * numWords;
    VkDeviceSize totalSizeSize = sizeof(uint32_t);

    // Create host-visible storage buffers for simplicity
    createBuffer(facesSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, m_facesBuffer, m_facesMemory);
    createBuffer(vertsSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, m_verticesBuffer, m_verticesMemory);
    createBuffer(voxelsSizeWords, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, m_voxelsBuffer, m_voxelsMemory);
    createBuffer(totalSizeSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, m_totalSizeBuffer, m_totalSizeMemory);

    // Upload initial data by mapping host-visible memory
    void* data = nullptr;
    checkVk(vkMapMemory(m_device, m_facesMemory, 0, facesSize, 0, &data), "vkMapMemory faces");
    std::memcpy(data, flatFaces.data(), (size_t)facesSize);
    vkUnmapMemory(m_device, m_facesMemory);

    checkVk(vkMapMemory(m_device, m_verticesMemory, 0, vertsSize, 0, &data), "vkMapMemory verts");
    std::memcpy(data, flatVertices.data(), (size_t)vertsSize);
    vkUnmapMemory(m_device, m_verticesMemory);

    checkVk(vkMapMemory(m_device, m_voxelsMemory, 0, voxelsSizeWords, 0, &data), "vkMapMemory voxels");
    std::memset(data, 0, (size_t)voxelsSizeWords);
    vkUnmapMemory(m_device, m_voxelsMemory);

    uint32_t zero = 0;
    checkVk(vkMapMemory(m_device, m_totalSizeMemory, 0, totalSizeSize, 0, &data), "vkMapMemory totalSize");
    std::memcpy(data, &zero, sizeof(uint32_t));
    vkUnmapMemory(m_device, m_totalSizeMemory);

    // Descriptor set layout, pool/set and pipeline
    createDescriptorSetLayout();
    createDescriptorPoolAndSet();

    // NOTE: you must provide a compiled SPIR-V compute shader that implements the
    // `voxelize_faces` kernel. Place it at Voxelizer/Vulkan/VoxelizerKernel.spv
    createComputePipeline("Voxelizer/Vulkan/VoxelizerKernel.spv");

    // Record compute dispatch
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    checkVk(vkBeginCommandBuffer(m_commandBuffer, &beginInfo), "vkBeginCommandBuffer");

    vkCmdBindPipeline(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);
    vkCmdBindDescriptorSets(m_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);

    // push constants: numFaces, dims.width, dims.height, dims.depth, xmin, ymin, zmin
    int pushConsts[7];
    pushConsts[0] = static_cast<int>(faces.size());
    pushConsts[1] = dims.width;
    pushConsts[2] = dims.height;
    pushConsts[3] = dims.depth;
    pushConsts[4] = scaledBounds.xmin;
    pushConsts[5] = scaledBounds.ymin;
    pushConsts[6] = scaledBounds.zmin;

    vkCmdPushConstants(m_commandBuffer, m_pipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pushConsts), pushConsts);

    const uint32_t localSize = 64u;
    uint32_t groupCount = (static_cast<uint32_t>(faces.size()) + localSize - 1) / localSize;
    vkCmdDispatch(m_commandBuffer, groupCount, 1, 1);

    checkVk(vkEndCommandBuffer(m_commandBuffer), "vkEndCommandBuffer");

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffer;

    VkFence fence;
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    checkVk(vkCreateFence(m_device, &fenceInfo, nullptr, &fence), "vkCreateFence");

    checkVk(vkQueueSubmit(m_computeQueue, 1, &submitInfo, fence), "vkQueueSubmit");
    checkVk(vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");
    vkDestroyFence(m_device, fence, nullptr);

    // Read back voxels (packed 32-bit words -> expand to unsigned char per voxel)
    void* hostPtr = nullptr;
    checkVk(vkMapMemory(m_device, m_voxelsMemory, 0, voxelsSizeWords, 0, &hostPtr), "vkMapMemory read voxels");
    uint32_t* words = reinterpret_cast<uint32_t*>(hostPtr);
    for (size_t i = 0; i < numVoxels; ++i) {
        uint32_t w = words[i >> 5];
        voxels[i] = ((w >> (i & 31u)) & 1u) ? 1 : 0;
    }
    vkUnmapMemory(m_device, m_voxelsMemory);

    // Read back total size
    uint32_t totalSize = 0;
    checkVk(vkMapMemory(m_device, m_totalSizeMemory, 0, totalSizeSize, 0, &hostPtr), "vkMapMemory read totalSize");
    std::memcpy(&totalSize, hostPtr, sizeof(uint32_t));
    vkUnmapMemory(m_device, m_totalSizeMemory);

    std::cout << "Total faces processed: " << totalSize << std::endl;

    // Destroy per-run objects and free buffers
    if (m_computePipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(m_device, m_computePipeline, nullptr);
        m_computePipeline = VK_NULL_HANDLE;
    }
    if (m_pipelineLayout != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
        m_pipelineLayout = VK_NULL_HANDLE;
    }
    if (m_descriptorSetLayout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(m_device, m_descriptorSetLayout, nullptr);
        m_descriptorSetLayout = VK_NULL_HANDLE;
    }
    if (m_descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(m_device, m_descriptorPool, nullptr);
        m_descriptorPool = VK_NULL_HANDLE;
    }

    destroyBuffers();
}