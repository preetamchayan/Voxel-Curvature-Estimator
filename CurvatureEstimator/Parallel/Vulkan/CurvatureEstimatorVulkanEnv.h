#pragma once

#include "../../CurvatureEstimatorBaseEnv.h"

class CurvatureEstimatorVulkanEnv : public CurvatureEstimatorBaseEnv {
public:
    CurvatureEstimatorVulkanEnv();
    ~CurvatureEstimatorVulkanEnv();
    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;
};