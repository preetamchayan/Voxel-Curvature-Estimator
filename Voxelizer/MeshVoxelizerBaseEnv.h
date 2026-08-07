#pragma once

#include "../Helper/GeometryTypes.h"
#include "../Helper/MeshLoader/MeshLoader.h"
#include <vector>

class MeshVoxelizerBaseEnv {

public:
    virtual ~MeshVoxelizerBaseEnv() = default;
    virtual void voxelize(std::vector<unsigned char> &voxels,
                          const std::vector<Point3i> &vertices,
                          const std::vector<Face> &faces,
                          const BBox3i &scaledBounds,
                          const Dimensions3i &dims) = 0;
};