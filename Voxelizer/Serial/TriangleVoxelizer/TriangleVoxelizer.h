#pragma once

#include "../DSSCreator/DSSCreator.h"

class TriangleVoxelizer {
public:
    void voxelizeTriangle(Point3i p1, Point3i p2, Point3i p3, OcTree* ocTree);

private:
    std::vector<Point2i> computeTriEdgePixels(Point2i p1, Point2i p2, Point2i p3);

private:
    std::vector<Point3i> m_voxels;
    DSSCreator m_dssCreator;
};