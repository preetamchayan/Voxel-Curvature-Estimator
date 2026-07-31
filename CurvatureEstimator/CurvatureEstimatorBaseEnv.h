#pragma once

#include "../Helper/GeometryTypes.h"
#include <vector>

class CurvatureEstimatorBaseEnv {
public:
    virtual ~CurvatureEstimatorBaseEnv() = default;
    virtual void estimateCurvature(int curvLength,
                                   std::vector<Point3i> &voxels,
                                   std::vector<int>& curvatures,
                                   const Dimensions3i &dims) = 0;
};