#include "MeshVoxelizer.h"
#include "../Helper/HelperFunctions.h"
#include <cmath>

// Include only the selected parallel environment header to avoid optional dependency
#if VOXELIZE_MODE == PARALLEL_OPENCL
#include "Parallel/OpenCL/MeshVoxelizerOclEnv.h"
#elif VOXELIZE_MODE == PARALLEL_VULKAN
#include "Parallel/Vulkan/MeshVoxelizerVulkanEnv.h"
#elif VOXELIZE_MODE == PARALLEL_CUDA
#include "Parallel/Cuda/MeshVoxelizerCudaEnv.h"
#elif VOXELIZE_MODE == PARALLEL_DIRECTX
#include "Parallel/DirectX/MeshVoxelizerDirectXEnv.h"
#elif VOXELIZE_MODE == PARALLEL_METAL
#include "Parallel/Metal/MeshVoxelizerMetalEnv.h"
#else
#include "Serial/MeshVoxelizerSerialEnv.h"
#endif
#include <algorithm>
#include <climits>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cassert>

int maxSize = INT_MIN;

MeshVoxelizer::MeshVoxelizer(const BBox3d& bounds) : m_unscaledBounds(bounds) {
    m_baseEnv = nullptr;
}

MeshVoxelizer::~MeshVoxelizer() {
    if (m_baseEnv) {
        delete m_baseEnv;
    }
}

BBox3i MeshVoxelizer::getSceneBounds() const {
    return m_scaledBounds;
}

Dimensions3i MeshVoxelizer::getDimensions() const {
    return m_dims;
}

std::vector<unsigned char>& MeshVoxelizer::getVoxels() {
    return m_voxels;
}

int MeshVoxelizer::getVoxelCount() const {
    return std::count(m_voxels.begin(), m_voxels.end(), 1);
}

void MeshVoxelizer::getRecommendedScaleRange(int& s_low, int& s_high) const {
    double dx = m_unscaledBounds.xmax - m_unscaledBounds.xmin;
    double dy = m_unscaledBounds.ymax - m_unscaledBounds.ymin;
    double dz = m_unscaledBounds.zmax - m_unscaledBounds.zmin;
    double maximum = std::max({dx, dy, dz});
    if (maximum == 0) {
        s_low = 1;
        s_high = 1;
        return;
    }
    int p = 0;
    while ((int)(maximum * (float)p) < 1000) p += 50;
    s_high = p;
    s_low = p / 10;
}

void MeshVoxelizer::voxelize(const MeshLoader& mesh, float scale) {
    const auto& vertices = mesh.getVertices();
    const auto& faces = mesh.getFaces();

    m_scaledBounds.xmax = INT_MIN; m_scaledBounds.xmin = INT_MAX;
    m_scaledBounds.ymax = INT_MIN; m_scaledBounds.ymin = INT_MAX;
    m_scaledBounds.zmax = INT_MIN; m_scaledBounds.zmin = INT_MAX;

    // Scale vertices and find bounds
    std::vector<Point3i> intVertices(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        Point3f point3f;
        point3f.x = vertices[i].x * scale;
        point3f.y = vertices[i].y * scale;
        point3f.z = vertices[i].z * scale;

        Point3i point3i;
        point3i.x = static_cast<int>(std::round(point3f.x));
        point3i.y = static_cast<int>(std::round(point3f.y));
        point3i.z = static_cast<int>(std::round(point3f.z));

        intVertices[i] = point3i;
        m_scaledBounds.xmax = std::max(m_scaledBounds.xmax, point3i.x);
        m_scaledBounds.xmin = std::min(m_scaledBounds.xmin, point3i.x);
        m_scaledBounds.ymax = std::max(m_scaledBounds.ymax, point3i.y);
        m_scaledBounds.ymin = std::min(m_scaledBounds.ymin, point3i.y);
        m_scaledBounds.zmax = std::max(m_scaledBounds.zmax, point3i.z);
        m_scaledBounds.zmin = std::min(m_scaledBounds.zmin, point3i.z);
    }

    int R = m_scaledBounds.xmax - m_scaledBounds.xmin + 3;
    int C = m_scaledBounds.ymax - m_scaledBounds.ymin + 3;
    int D = m_scaledBounds.zmax - m_scaledBounds.zmin + 3;

    m_dims.width  = R;
    m_dims.height = C;
    m_dims.depth  = D;

    m_voxels.assign(R * C * D, 0);

#if VOXELIZE_MODE == SERIAL
    m_baseEnv = new MeshVoxelizerSerialEnv();
#elif VOXELIZE_MODE == PARALLEL_OPENCL
    m_baseEnv = new MeshVoxelizerOclEnv();
#elif VOXELIZE_MODE == PARALLEL_VULKAN
    m_baseEnv = new MeshVoxelizerVulkanEnv();
#elif VOXELIZE_MODE == PARALLEL_CUDA
    m_baseEnv = new MeshVoxelizerCudaEnv();
#elif VOXELIZE_MODE == PARALLEL_DIRECTX
    m_baseEnv = new MeshVoxelizerDirectXEnv();
#elif VOXELIZE_MODE == PARALLEL_METAL
    m_baseEnv = new MeshVoxelizerMetalEnv();
#else
    #error "Invalid VOXELIZE_MODE defined. Please define it as SERIAL, PARALLEL_OPENCL, PARALLEL_VULKAN, PARALLEL_CUDA  or PARALLEL_DIRECTX." 
#endif
    if (m_baseEnv) {
        m_baseEnv->voxelize(m_voxels, intVertices, faces, m_scaledBounds, m_dims);
    }
}

void MeshVoxelizer::exportVoxelsOBJ(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return;
    }

    std::ofstream logFile("output/voxel_export_log.txt");
    if (logFile.is_open()) {
        logFile << "Exporting voxel mesh to OBJ format\n";
        logFile << "Grid bounds: (" << m_scaledBounds.xmin << ", " << m_scaledBounds.ymin << ", " << m_scaledBounds.zmin << ") to (" << m_scaledBounds.xmax << ", " << m_scaledBounds.ymax << ", " << m_scaledBounds.zmax << ")\n";
        logFile << "Total voxels: " << getVoxelCount() << "\n";
    }

    file << "# Voxel mesh exported from Voxelizer\n";
    file << "# Grid bounds: (" << m_scaledBounds.xmin << ", " << m_scaledBounds.ymin << ", "
         << m_scaledBounds.zmin << ") to (" << m_scaledBounds.xmax << ", " << m_scaledBounds.ymax << ", "
         << m_scaledBounds.zmax << ")\n\n";

    std::cout << "Exporting " << getVoxelCount() << " occupied voxels to OBJ file." << std::endl;

    for (int z = 0; z < m_dims.depth; ++z) {
        for (int y = 0; y < m_dims.height; ++y) {
            for (int x = 0; x < m_dims.width; ++x) {
                const size_t id = static_cast<size_t>(x) +
                                  static_cast<size_t>(y) * m_dims.width +
                                  static_cast<size_t>(z) * m_dims.width * m_dims.height;
                if (m_voxels[id] != 1) continue;

                Point3i voxel(
                    x + m_scaledBounds.xmin,
                    y + m_scaledBounds.ymin,
                    z + m_scaledBounds.zmin
                );
                if (logFile.is_open()) {
                    logFile << "(" << voxel.x << ", " << voxel.y << ", " << voxel.z << ")\n";
                }
                writePointToVoxel(voxel, file);
            }
        }
    }

    logFile.close();
    file.close();
    std::cout << "Exported voxel mesh to " << filename << std::endl;
}