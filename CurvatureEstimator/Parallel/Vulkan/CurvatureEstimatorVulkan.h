#pragma once

#include "../../CurvatureEstimatorBase.h"

#include <vulkan/vulkan.h>

#include <cstdint>
#include <vector>

class CurvatureEstimatorVulkan : public CurvatureEstimatorBase {
public:
    CurvatureEstimatorVulkan();
    ~CurvatureEstimatorVulkan() override;

    void preprocessVoxels(std::vector<unsigned char>& voxels,
                          const Dimensions3i& dims) override;
    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;

private:
    VkInstance m_instance = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;
    VkQueue m_computeQueue = VK_NULL_HANDLE;
    uint32_t m_computeQueueFamilyIndex = UINT32_MAX;
    VkCommandPool m_commandPool = VK_NULL_HANDLE;
    VkCommandBuffer m_commandBuffer = VK_NULL_HANDLE;
    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    VkDescriptorSet m_descriptorSet = VK_NULL_HANDLE;
    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_pipeline = VK_NULL_HANDLE;

    uint32_t findComputeQueueFamily(VkPhysicalDevice device) const;
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;
    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkBuffer& buffer,
                      VkDeviceMemory& memory) const;
    void createPipeline();
    void submitAndWait();
    void cleanup();
};