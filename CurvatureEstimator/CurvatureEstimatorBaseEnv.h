#pragma once

#include "../Helper/GeometryTypes.h"
#include <vector>

class CurvatureEstimatorBaseEnv {
public:
    virtual ~CurvatureEstimatorBaseEnv() = default;
    virtual void estimateCurvature(std::vector<unsigned char> &voxels,
                                   const BBox3i &scaledBounds,
                                   const Dimensions3i &dims) = 0;
};