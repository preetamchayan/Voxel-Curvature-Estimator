#pragma once

#include "../../CurvatureEstimatorBase.h"

class CurvatureEstimatorDirectX : public CurvatureEstimatorBase {
public:
    CurvatureEstimatorDirectX();
    ~CurvatureEstimatorDirectX();
    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;
};