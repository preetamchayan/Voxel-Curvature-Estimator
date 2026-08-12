#pragma once

#include "../../CurvatureEstimatorBase.h"

class CurvatureEstimatorCuda : public CurvatureEstimatorBase {
public:
    CurvatureEstimatorCuda();
    ~CurvatureEstimatorCuda();
    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;
};