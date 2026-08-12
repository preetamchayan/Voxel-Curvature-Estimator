#pragma once

#include "../../CurvatureEstimatorBase.h"

class CurvatureEstimatorVulkan : public CurvatureEstimatorBase {
public:
    CurvatureEstimatorVulkan();
    ~CurvatureEstimatorVulkan();
    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;
};