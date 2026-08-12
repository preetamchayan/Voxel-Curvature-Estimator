#pragma once

#include <vulkan/vulkan.h>
#include "../../MeshVoxelizerBase.h"
#include <cstdint>
#include <vector>
#include <string>

class MeshVoxelizerVulkan : public MeshVoxelizerBase {
public:
    MeshVoxelizerVulkan();
    ~MeshVoxelizerVulkan();
    void voxelize(
        std::vector<unsigned char> &voxels,
        const std::vector<Point3i> &vertices,
        const std::vector<Face> &faces,
        const BBox3i &scaledBounds,
        const Dimensions3i &dims) override;

private:
    // Core Vulkan objects
    VkInstance m_instance = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;
    VkQueue m_computeQueue = VK_NULL_HANDLE;
    uint32_t m_computeQueueFamilyIndex = 0xFFFFFFFFu;

    VkCommandPool m_commandPool = VK_NULL_HANDLE;
    VkCommandBuffer m_commandBuffer = VK_NULL_HANDLE;

    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorSet m_descriptorSet = VK_NULL_HANDLE;
    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_computePipeline = VK_NULL_HANDLE;

    // Buffers + memory for compute
    VkBuffer m_facesBuffer = VK_NULL_HANDLE;
    VkDeviceMemory m_facesMemory = VK_NULL_HANDLE;
    VkBuffer m_verticesBuffer = VK_NULL_HANDLE;
    VkDeviceMemory m_verticesMemory = VK_NULL_HANDLE;
    VkBuffer m_voxelsBuffer = VK_NULL_HANDLE;
    VkDeviceMemory m_voxelsMemory = VK_NULL_HANDLE;
    VkBuffer m_totalSizeBuffer = VK_NULL_HANDLE;
    VkDeviceMemory m_totalSizeMemory = VK_NULL_HANDLE;
    bool m_supports8bit = false;

    // Helpers
    uint32_t findComputeQueueFamily(VkPhysicalDevice device);
    void createInstance();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createCommandPoolAndBuffer();
    void cleanup();

    // Memory / buffer helpers
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);
    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &bufferMemory);
    std::vector<char> readFile(const std::string &filename);

    // Descriptor / pipeline helpers
    void createDescriptorSetLayout();
    void createDescriptorPoolAndSet();
    void createComputePipeline(const std::string &spvPath);
    void destroyBuffers();
};