#pragma once

#include "../CurvatureEstimatorBaseEnv.h"

class CurvatureEstimatorDirectxEnv : public CurvatureEstimatorBaseEnv {
public:
    CurvatureEstimatorDirectxEnv();
    ~CurvatureEstimatorDirectxEnv();
    void estimateCurvature(std::vector<unsigned char> &voxels,
                           const BBox3i &scaledBounds,
                           const Dimensions3i &dims) override;
};