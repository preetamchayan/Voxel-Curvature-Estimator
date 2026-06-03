#pragma once

#include "../../../Helper/GeometryTypes.h"
#include "../../../Helper/OcTree/OcTree.h"

#include <vector>

class DSSCreator {
public:
    std::vector<Point2i> rasterizeDSS(Point2i point1, Point2i point2);
    void voxelizeDSS(Point3i p1, Point3i p2, OcTree* ocTree);

private:
    std::vector<Point2i> bresenhamLineDrawing(Point2i point1, Point2i point2, Values values, Flags flags);
};