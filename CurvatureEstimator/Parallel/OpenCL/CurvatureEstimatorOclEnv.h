#pragma once

#include "../../CurvatureEstimatorBaseEnv.h"

class CurvatureEstimatorOclEnv : public CurvatureEstimatorBaseEnv {
public:
    CurvatureEstimatorOclEnv();
    ~CurvatureEstimatorOclEnv();
    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;
};