#pragma once

#include "../CurvatureEstimatorBaseEnv.h"

class CurvatureEstimatorVulkanEnv : public CurvatureEstimatorBaseEnv {
public:
    CurvatureEstimatorVulkanEnv();
    ~CurvatureEstimatorVulkanEnv();
    void estimateCurvature(std::vector<unsigned char> &voxels,
                           const BBox3i &scaledBounds,
                           const Dimensions3i &dims) override;
};