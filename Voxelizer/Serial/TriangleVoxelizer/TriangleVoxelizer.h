#pragma once

#include "../DSSCreator/DSSCreator.h"

class TriangleVoxelizer {
public:
    void voxelizeTriangle(
        Point3i p1,
        Point3i p2,
        Point3i p3,
        std::vector<unsigned char>& voxels,
        const BBox3i& bounds,
        const Dimensions3i& dims
    );

private:
    std::vector<Point2i> computeTriEdgePixels(Point2i p1, Point2i p2, Point2i p3);

private:
    DSSCreator m_dssCreator;
};