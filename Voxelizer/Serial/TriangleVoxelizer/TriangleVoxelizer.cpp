#include "TriangleVoxelizer.h"
#include "../../../Helper/HelperFunctions.h"
#include <algorithm>
#include <cmath>
#include <cassert>
#include <climits>

void TriangleVoxelizer::voxelizeTriangle(Point3i p1, Point3i p2, Point3i p3, OcTree* ocTree) {
    Point3i p12, p13;

    // determining the co-efficients of the plane equation of the triangle
    p12.x = (p2.x - p1.x);
    p12.y = (p2.y - p1.y);
    p12.z = (p2.z - p1.z);
    p13.x = (p3.x - p1.x);
    p13.y = (p3.y - p1.y);
    p13.z = (p3.z - p1.z);

    Plane plane;
    plane.a = p12.y * p13.z - p13.y * p12.z;
    plane.b = p13.x * p12.z - p12.x * p13.z;
    plane.c = p12.x * p13.y - p13.x * p12.y;
    plane.d = -plane.a * p1.x - plane.b * p1.y - plane.c * p1.z;

    // degenerate triangle
    if (plane.a == 0 && plane.b == 0 && plane.c == 0) {
        if (p1.x > p2.x || p1.y > p2.y || p1.z > p2.z) {
            swap(&p1.x, &p2.x);
            swap(&p1.y, &p2.y);
            swap(&p1.z, &p2.z);
        }
        if (p1.x > p3.x || p1.y > p3.y || p1.z > p3.z) {
            swap(&p1.x, &p3.x);
            swap(&p1.y, &p3.y);
            swap(&p1.z, &p3.z);
        }
        if (p2.x > p3.x || p2.y > p3.y || p2.z > p3.z) {
            swap(&p2.x, &p3.x);
            swap(&p2.y, &p3.y);
            swap(&p2.z, &p3.z);
        }
        m_dssCreator.voxelizeSegment(p1, p3, ocTree);
        return;
    }

    // finding the projected plane with maximum area
    int maxDim = (int)std::max({std::abs(plane.a), std::abs(plane.b), std::abs(plane.c)});
    Point2i _p1, _p2, _p3;
    Plane _plane;
    char axis;
    if (maxDim == std::abs(plane.a)) {
        _p1 = {p1.y, p1.z}; _p2 = {p2.y, p2.z}; _p3 = {p3.y, p3.z};
        _plane = {plane.b, plane.c, plane.d, plane.a};
        axis = 1;
    } else if (maxDim == std::abs(plane.b)) {
        _p1 = {p1.x, p1.z}; _p2 = {p2.x, p2.z}; _p3 = {p3.x, p3.z};
        _plane = {plane.a, plane.c, plane.d, plane.b};
        axis = 2;
    } else {
        _p1 = {p1.x, p1.y}; _p2 = {p2.x, p2.y}; _p3 = {p3.x, p3.y};
        _plane = {plane.a, plane.b, plane.d, plane.c};
        axis = 3;
    }

    std::vector<Point2i> boundaryPixels = computeTriEdgePixels(_p1, _p2, _p3);

    Point2i yBound;
    yBound.x = std::min({_p1.y, _p2.y, _p3.y}); // min y
    yBound.y = std::max({_p1.y, _p2.y, _p3.y}); // max y

    for (int y = 0; y <= yBound.y - yBound.x; y++) {
        for (int x = boundaryPixels[y].x; x <= boundaryPixels[y].y; x++) {
            // float z = (float)(std::abs(_plane.d)) / 2.0f;
            // z -= (float)_plane.a * (float)x;
            // z -= (float)_plane.b * (float)(y + yBound.x);
            // z -= (float)_plane.c;
            // z /= (float)_plane.d;
            float z = ((float)maxDim / 2.0 - (float)_plane.a * (float)x - (float)_plane.b * (float)(y + yBound.x) - (float)_plane.c) / (float)_plane.d;
            int i, j, k, zz = _plane.d < 0 ? (int)std::ceil(z) : (int)std::floor(z);
            if (axis == 1) { i = zz; j = x; k = y + yBound.x; }
            else if (axis == 2) { i = x; j = zz; k = y + yBound.x; }
            else { i = x; j = y + yBound.x; k = zz; }
            ocTree->insert(Point3i(i, j, k));
        }
    }
}

std::vector<Point2i> TriangleVoxelizer::computeTriEdgePixels(Point2i p1, Point2i p2, Point2i p3) {
    Point2i yBound;
    yBound.x = std::min({p1.y, p2.y, p3.y}); // min y
    yBound.y = std::max({p1.y, p2.y, p3.y}); // max y
    std::vector<Point2i> boundaryPixels(yBound.y - yBound.x + 1);
    for (int i = 0; i <= yBound.y - yBound.x; i++) {
        boundaryPixels[i].x = INT_MAX;
        boundaryPixels[i].y = INT_MIN;
    }

    auto updateMaxMin = [&] (std::vector<Point2i>& points) {
        for (size_t i = 0; i < points.size(); i++) {
            int temp = points[i].y - yBound.x;
            if (temp >= 0 && temp < boundaryPixels.size()) {
                if (boundaryPixels[temp].x > points[i].x) boundaryPixels[temp].x = points[i].x;
                if (boundaryPixels[temp].y < points[i].x) boundaryPixels[temp].y = points[i].x;
            }
        }
    };

    auto points = m_dssCreator.rasterizeSegment(p1, p2);
    updateMaxMin(points);
    points = m_dssCreator.rasterizeSegment(p2, p3);
    updateMaxMin(points);
    points = m_dssCreator.rasterizeSegment(p1, p3);
    updateMaxMin(points);

    return boundaryPixels;
}