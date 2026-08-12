#pragma once

#include "../Helper/GeometryTypes.h"
#include <vector>

class CurvatureEstimatorBase {
public:
    virtual ~CurvatureEstimatorBase() = default;

    // Converts a raw voxel shell grid into the curvature-ready grid:
    // 0 = empty, 1 = surface/frontier voxel, 2 = interior voxel.
    virtual void preprocessVoxels(std::vector<unsigned char>& voxels,
                                  const Dimensions3i& dims) = 0;

    virtual void estimateCurvature(int curvLength,
                                   const std::vector<unsigned char>& voxels,
                                   std::vector<int>& curvatures,
                                   const Dimensions3i& dims) = 0;
};