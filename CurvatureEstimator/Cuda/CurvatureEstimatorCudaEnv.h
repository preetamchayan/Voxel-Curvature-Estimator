#pragma once

#include "../CurvatureEstimatorBaseEnv.h"

class CurvatureEstimatorCudaEnv : public CurvatureEstimatorBaseEnv {
public:
    CurvatureEstimatorCudaEnv();
    ~CurvatureEstimatorCudaEnv();
    void estimateCurvature(std::vector<unsigned char> &voxels,
                           const BBox3i &scaledBounds,
                           const Dimensions3i &dims) override;
};