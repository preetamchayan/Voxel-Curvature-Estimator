#pragma once

#include "../../CurvatureEstimatorBaseEnv.h"

class CurvatureEstimatorDirectXEnv : public CurvatureEstimatorBaseEnv {
public:
    CurvatureEstimatorDirectXEnv();
    ~CurvatureEstimatorDirectXEnv();
    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;
};