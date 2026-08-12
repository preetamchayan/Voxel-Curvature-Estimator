#pragma once

// OS-neutral     -- Windows / Linux (NOT MacOS due to lack of OpenCL / Vulkan / CUDA support)
// Vendor-neutral -- NVIDIA / AMD / Intel / Adreno (Qualcomm) / Mali (ARM) (NOT Apple Silicon)
// Device-neutral -- CPU, GPU, embedded devices
#define SERIAL 0    // OS-neutral,   Vendor-neutral, CPU-only
#define OPENCL 1    // OS-neutral,   Vendor-neutral, Device-neutral
#define VULKAN 2    // OS-neutral,   Vendor-neutral, GPU-only
#define CUDA 3      // OS-neutral,   NVIDIA-only,    GPU-only
#define DIRECTX 4   // Windows-only, Vendor-neutral, GPU-only
#define METAL 5     // MacOS-only,   Apple-only,     GPU-only

#ifndef CURVATURE_ESTIMATOR
#define CURVATURE_ESTIMATOR OPENCL
#endif

#include "../Helper/GeometryTypes.h"
#include "CurvatureEstimatorBase.h"

#include <vector>
#include <string>
#include <fstream>

class CurvatureEstimator {
public:
    CurvatureEstimator(
        std::vector<unsigned char>& voxels,
        const BBox3i& bounds,
        const Dimensions3i& dims
    );
    ~CurvatureEstimator();
    void estimateCurvature(int curveLength);
    const std::vector<unsigned char>& getVoxels() const;
    BBox3i getSceneBounds() const;
    void exportCurvatureOBJ(const std::string& filename) const;
private:
    std::vector<unsigned char>& m_voxels;
    std::vector<int> m_curvatures;
    BBox3i m_bounds;
    Dimensions3i m_dims;
    int m_curveLength;
    CurvatureEstimatorBase* m_base;
    std::vector<Color> m_colors;
private:
    void averageCurvature();
    void writeMaterialFile(int maxCurvature, std::ofstream& fp, std::vector<Color>& colors);
};