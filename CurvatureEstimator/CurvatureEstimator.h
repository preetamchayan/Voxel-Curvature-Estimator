#pragma once

#include "../Helper/GeometryTypes.h"
#include "../Helper/MeshLoader/MeshLoader.h"
#include "../Helper/OcTree/OcTree.h"
#include "CurvatureEstimatorBaseEnv.h"
#include <vector>
#include <array>
#include <cmath>
#include <limits>

// OS-neutral     -- Windows/Linux
// Vendor-neutral -- NVIDIA/AMD/Intel/Qualcomm
// Device-neutral -- CPU, GPU, embedded devices
#define SERIAL 0
#define PARALLEL_OPENCL 1  // OS-neutral,   Vendor-neutral, Device-neutral
#define PARALLEL_VULKAN 2  // OS-neutral,   Vendor-neutral, GPU-only
#define PARALLEL_CUDA 3    // OS-neutral,   NVIDIA-only,    GPU-only
#define PARALLEL_DIRECTX 4 // Windows-only, Vendor-neutral, GPU-only

#ifndef CURVATURE_ESTIMATOR_MODE
#define CURVATURE_ESTIMATOR_MODE SERIAL
#endif

class CurvatureEstimator{
public:
    CurvatureEstimator(const std::vector<unsigned char>& voxels, const BBox3i& bounds);
    void estimateCurvature(std::vector<unsigned char> &voxels,
                           const BBox3i &scaledBounds,
                           const Dimensions3i &dims);
private:
    std::vector<unsigned char> m_voxels;
    BBox3i m_bounds;
    int m_width, m_height, m_depth;
    CurvatureEstimatorBaseEnv* m_baseEnv;
    std::vector<Point3i> getNeighbors(int x, int y, int z);
    double computeCurvatureAtVoxel(int x, int y, int z);
};