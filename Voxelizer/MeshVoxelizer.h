#pragma once

#include "../Helper/GeometryTypes.h"
#include "../Helper/MeshLoader/MeshLoader.h"
#include "MeshVoxelizerBase.h"
#include <vector>

// OS-neutral     -- Windows / Linux (NOT MacOS due to lack of OpenCL / Vulkan / CUDA support)
// Vendor-neutral -- NVIDIA / AMD / Intel / Adreno (Qualcomm) / Mali (ARM) (NOT Apple Silicon)
// Device-neutral -- CPU, GPU, embedded devices
#define SERIAL 0           // OS-neutral,   Vendor-neutral, CPU-only
#define PARALLEL_OPENCL 1  // OS-neutral,   Vendor-neutral, Device-neutral
#define PARALLEL_VULKAN 2  // OS-neutral,   Vendor-neutral, GPU-only
#define PARALLEL_CUDA 3    // OS-neutral,   NVIDIA-only,    GPU-only
#define PARALLEL_DIRECTX 4 // Windows-only, Vendor-neutral, GPU-only
#define PARALLEL_METAL 5   // MacOS-only,   Apple-only,     GPU-only

#ifndef VOXELIZER_MODE
#define VOXELIZER_MODE PARALLEL_DIRECTX
#endif

class MeshVoxelizer {
private:
    std::vector<unsigned char> m_voxels;
    BBox3i m_scaledBounds;
    BBox3d m_unscaledBounds;
    Dimensions3i m_dims;
    MeshVoxelizerBase* m_base;

public:
    MeshVoxelizer(const BBox3d& bounds);
    ~MeshVoxelizer();
    void voxelize(const MeshLoader& mesh, float scale);
    std::vector<unsigned char>& getVoxels();
    BBox3i getSceneBounds() const;
    Dimensions3i getDimensions() const;
    void exportVoxelsOBJ(const std::string& filename) const;
    int getVoxelCount() const;
    void getRecommendedScaleRange(int& s_low, int& s_high) const;
};