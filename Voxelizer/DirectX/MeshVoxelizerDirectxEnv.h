#pragma once

// #include "../../Helper/GeometryTypes.h"
#include "../MeshVoxelizerBaseEnv.h"

class MeshVoxelizerDirectxEnv : public MeshVoxelizerBaseEnv {
public:
    MeshVoxelizerDirectxEnv();
    ~MeshVoxelizerDirectxEnv();
    void voxelize(std::vector<unsigned char> &voxels,
                  const std::vector<Point3i> &vertices,
                  const std::vector<Face> &faces,
                  const BBox3i &scaledBounds,
                  const Dimensions3i &dims) override;
};