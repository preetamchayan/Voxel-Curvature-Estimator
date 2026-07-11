#pragma once

#include "../Helper/GeometryTypes.h"
#include "../Helper/MeshLoader/MeshLoader.h"
#include "../Helper/OcTree/OcTree.h"
#include "MeshVoxelizerBaseEnv.h"
#include <vector>

// OS-neutral     -- Windows / Linux (NOT MacOS due to lack of OpenCL / Vulkan / CUDA support)
// Vendor-neutral -- NVIDIA / AMD / Intel / Adreno (Qualcomm) / Mali (ARM) (NOT Apple Silicon)
// Device-neutral -- CPU, GPU, embedded devices
#define SERIAL 0
#define PARALLEL_OPENCL 1  // OS-neutral,   Vendor-neutral, Device-neutral
#define PARALLEL_VULKAN 2  // OS-neutral,   Vendor-neutral, GPU-only
#define PARALLEL_CUDA 3    // OS-neutral,   NVIDIA-only,    GPU-only
#define PARALLEL_DIRECTX 4 // Windows-only, Vendor-neutral, GPU-only
#define PARALLEL_METAL 5   // MacOS-only,   Apple-only,     GPU-only

#ifndef VOXELIZE_MODE
#define VOXELIZE_MODE PARALLEL_METAL
#endif

class MeshVoxelizer {
private:
    std::vector<unsigned char> m_voxels;
    BBox3i m_scaledBounds;
    BBox3d m_unscaledBounds;
    Dimensions3i m_dims;
    OcTree* m_ocTree;
    MeshVoxelizerBaseEnv* m_baseEnv;

public:
    MeshVoxelizer(const BBox3d& bounds);
    ~MeshVoxelizer();
    void voxelize(const MeshLoader& mesh, float scale);
    const std::vector<unsigned char>& getVoxels() const;
    void getVoxels(std::vector<Point3i>& voxels) const;
    BBox3i getSceneBounds() const;
    void exportVoxelsOBJ(const std::string& filename) const;
    int getVoxelCount() const;
    void getRecommendedScaleRange(int& s_low, int& s_high) const;
    bool search(Point3i p) const;
    bool remove(Point3i p) const;
};