#pragma once

#include "../MeshVoxelizerBaseEnv.h"
#include "TriangleVoxelizer/TriangleVoxelizer.h"

class MeshVoxelizerSerialEnv : public MeshVoxelizerBaseEnv {
public:
    MeshVoxelizerSerialEnv() = default;
    void voxelize(
        std::vector<unsigned char>& voxels,
        const std::vector<Point3i>& vertices,
        const std::vector<Face>& faces,
        const BBox3i& bounds,
        const Dimensions3i& dims) override;

private:
    TriangleVoxelizer m_triangleVoxelizer;
};