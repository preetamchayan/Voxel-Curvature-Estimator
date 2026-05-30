#pragma once

// #include "../../Helper/GeometryTypes.h"
#include "../VoxelizerBaseEnv.h"

class VoxelizerDirectxEnv : public VoxelizerBaseEnv {
public:
    VoxelizerDirectxEnv();
    ~VoxelizerDirectxEnv();
    void voxelize(std::vector<unsigned char> &voxels,
                  const std::vector<Point3i> &vertices,
                  const std::vector<Face> &faces,
                  const BBox3i &scaledBounds,
                  const Dimensions3i &dims) override;
};