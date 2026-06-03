#include "MeshVoxelizerSerialEnv.h"
#include <iostream>
#include <cassert>

MeshVoxelizerSerialEnv::MeshVoxelizerSerialEnv(OcTree* ocTree) : m_ocTree(ocTree) {}

void MeshVoxelizerSerialEnv::voxelize(
    std::vector<unsigned char>& voxels,
    const std::vector<Point3i>& vertices,
    const std::vector<Face>& faces,
    const BBox3i& bounds,
    const Dimensions3i& dims) {
    
    assert(m_ocTree != nullptr);
    int faceCount = 0;
    for (const auto& face : faces) {
        Point3i p1 = vertices[face.v1];
        Point3i p2 = vertices[face.v2];
        Point3i p3 = vertices[face.v3];
        m_triangleVoxelizer.voxelizeTriangle(p1, p2, p3, m_ocTree);
        faceCount++;

        // for (const auto& voxel : m_voxels)
        //     m_ocTree->insert(voxel);
    }

    std::cout << "Total faces processed: " << faceCount << std::endl;
    std::cout << "Total unique voxels inserted in octree: " << m_ocTree->getVoxelCount() << std::endl;
}