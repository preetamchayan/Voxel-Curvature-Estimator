#pragma once

// #include "../../Helper/GeometryTypes.h"
#include "../BaseEnv.h"

class DirectxEnv : public BaseEnv {
public:
    DirectxEnv();
    ~DirectxEnv();
    void voxelize(std::vector<unsigned char> &voxels,
                  const std::vector<Point3i> &vertices,
                  const std::vector<Face> &faces,
                  const BBox3i &scaledBounds,
                  const Dimensions3i &dims) override;
};