#pragma once

#include "../../../Helper/GeometryTypes.h"
#include <vector>

class DSSCreator {
public:
    std::vector<Point2i> rasterizeSegment(Point2i point1, Point2i point2);
    void voxelizeSegment(
        Point3i p1,
        Point3i p2,
        std::vector<unsigned char>& voxels,
        const BBox3i& bounds,
        const Dimensions3i& dims
    );

private:
    std::vector<Point2i> bresenhamLineDrawing(Point2i point1, Point2i point2, Values values, Flags flags);
};