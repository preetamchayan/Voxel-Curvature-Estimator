#pragma once

#include "GeometryTypes.h"
#include <vector>

class CurvatureEstimator {
public:
    CurvatureEstimator(const std::vector<unsigned char>& voxels, const BBox3i& bounds);
    std::vector<double> estimateCurvature();
private:
    std::vector<unsigned char> m_voxels;
    BBox3i m_bounds;
    int m_width, m_height, m_depth;
    std::vector<Point3i> getNeighbors(int x, int y, int z);
    double computeCurvatureAtVoxel(int x, int y, int z);
};