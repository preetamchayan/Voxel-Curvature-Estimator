#pragma once

#include "../Helper/GeometryTypes.h"
#include "../Helper/MeshLoader/MeshLoader.h"
#include "../Helper/OcTree/OcTree.h"
#include "VoxelizerBaseEnv.h"
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

#ifndef VOXELIZE_MODE
#define VOXELIZE_MODE SERIAL
#endif

class Voxelizer {
private:
    std::vector<unsigned char> m_voxels;
    BBox3i m_scaledBounds;
    BBox3d m_unscaledBounds;
    Dimensions3i m_dims;
    OcTree* m_ocTree;
    VoxelizerBaseEnv* m_baseEnv;
    int m_voxelCount;

    // Private helper functions for serial mode
    void swap(int* a, int* b);
    std::vector<Point2i> getPixelDSS2D(Point2i point1, Point2i point2, Values values, Flags flags);
    std::vector<Point2i> DSS2D(Point2i point1, Point2i point2);
    void DSS3D(Point3i p1, Point3i p2);
    void digitalTriangle2D(Point2i p1, Point2i p2, Point2i p3, Plane plane, char axis);
    void digitalTriangle3D(Point3i p1, Point3i p2, Point3i p3);

public:
    Voxelizer(BBox3d bounds);
    ~Voxelizer();
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