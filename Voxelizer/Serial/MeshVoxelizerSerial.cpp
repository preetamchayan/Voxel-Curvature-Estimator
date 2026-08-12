#include "MeshVoxelizerSerial.h"
#include <iostream>
#include <cassert>

void MeshVoxelizerSerial::voxelize(
    std::vector<unsigned char>& voxels,
    const std::vector<Point3i>& vertices,
    const std::vector<Face>& faces,
    const BBox3i& bounds,
    const Dimensions3i& dims) {
    
    assert(voxels.size() == static_cast<size_t>(dims.width * dims.height * dims.depth));
    int faceCount = 0;
    for (const auto& face : faces) {
        Point3i p1 = vertices[face.v1];
        Point3i p2 = vertices[face.v2];
        Point3i p3 = vertices[face.v3];
        m_triangleVoxelizer.voxelizeTriangle(p1, p2, p3, voxels, bounds, dims);
        faceCount++;
    }

    std::cout << "Total faces processed: " << faceCount << std::endl;
    int voxelCount = 0;
    for (unsigned char voxel : voxels) {
        if (voxel == 1) voxelCount++;
    }
    std::cout << "Total unique voxels marked in grid: " << voxelCount << std::endl;
}