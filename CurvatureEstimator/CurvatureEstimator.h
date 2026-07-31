#pragma once

// OS-neutral     -- Windows / Linux (NOT MacOS due to lack of OpenCL / Vulkan / CUDA support)
// Vendor-neutral -- NVIDIA / AMD / Intel / Adreno (Qualcomm) / Mali (ARM) (NOT Apple Silicon)
// Device-neutral -- CPU, GPU, embedded devices
#define SERIAL 0
#define PARALLEL_OPENCL 1  // OS-neutral,   Vendor-neutral, Device-neutral
#define PARALLEL_VULKAN 2  // OS-neutral,   Vendor-neutral, GPU-only
#define PARALLEL_CUDA 3    // OS-neutral,   NVIDIA-only,    GPU-only
#define PARALLEL_DIRECTX 4 // Windows-only, Vendor-neutral, GPU-only
#define PARALLEL_METAL 5   // MacOS-only,   Apple-only,     GPU-only

#ifndef CURVATURE_ESTIMATOR_MODE
#define CURVATURE_ESTIMATOR_MODE SERIAL
#endif

#include "../Helper/GeometryTypes.h"
#include "../Helper/OcTree/OcTree.h"
#include "CurvatureEstimatorBaseEnv.h"

#include <vector>

class CurvatureEstimator {
public:
    CurvatureEstimator(
        std::vector<unsigned char>& voxels,
        OcTree* ocTree,
        const BBox3i& bounds,
        const Dimensions3i& dims
    );
    ~CurvatureEstimator();
    void estimateCurvature(int curveLength);
    const std::vector<Point3i>& getVoxels() const;
    BBox3i getSceneBounds() const;
    void exportCurvatureOBJ(const std::string& filename) const;
private:
    std::vector<Point3i> m_voxels;
    std::vector<int> m_curvatures;
    BBox3i m_bounds;
    Dimensions3i m_dims;
    OcTree* m_ocTree;
    OcTree* m_ocTreeInnerVoxel;
    CurvatureEstimatorBaseEnv* m_baseEnv;
    std::vector<Color> m_colors;
private:
    void computeInnerSpaceAndFrontierVoxels(std::vector<unsigned char>& voxels);
    void computeInnerSpaceVoxels(
        std::vector<unsigned char>& voxels,
        std::vector<OcTree>& ocTrees,
        int R, int C, int D, int plane
    );
    void markInteriorVoxels(
        std::vector<unsigned char>& voxels,
        std::vector<OcTree>& ocTrees
    );
    void computeFrontierVoxels(std::vector<unsigned char>& voxels);
    void averageCurvature();
    std::vector<Color>& writeMaterialFile(int maxCurvature, std::ofstream& fp);
};