#include "DSSCreator.h"
#include "../../../Helper/HelperFunctions.h"
#include <algorithm>
#include <cmath>
#include <cassert>

std::vector<Point2i> DSSCreator::rasterizeDSS(Point2i p1, Point2i p2) {
    int q = std::abs(p2.x - p1.x);
    int p = std::abs(p2.y - p1.y);
    Point2i _p1, _p2;
    Values values;
    Flags flags;
    if (p <= q) { // > 0 & <= 45 && > 180 & <= 225
        bool flag = false;
        if (p1.x < p2.x && p1.y < p2.y) { _p1 = p1; _p2 = p2; flag = true; }
        else if (p1.x > p2.x && p1.y > p2.y) { _p1 = p2; _p2 = p1; flag = true; }
        if(flag) {
            values = {1, 1};
            flags = {0, 0, -1};
        }
    }
    if (p > q) { // > 45 & <= 90 && > 225 & <= 270
        bool flag = false;
        if (p1.y < p2.y && p1.x <= p2.x) { _p1 = {p2.y, p2.x}; _p2 = {p1.y, p1.x}; flag = true; }
        else if (p1.y > p2.y && p1.x >= p2.x) { _p1 = {p1.y, p1.x}; _p2 = {p2.y, p2.x}; flag = true; }
        if(flag) {
            values = {1, 1};
            flags = {1, 1, 2};
        }
    }
    if (p >= q) { // > 90 & <= 135 && > 270 & <= 315
        bool flag = false;
        if (p1.y < p2.y && p1.x > p2.x) { _p1 = {p1.y, p2.x}; _p2 = {p2.y, p1.x}; flag = true; }
        else if (p1.y > p2.y && p1.x < p2.x) { _p1 = {p2.y, p1.x}; _p2 = {p1.y, p2.x}; flag = true; }
        if(flag) {
            values = {1, -1};
            flags = {0, 1, 1};
        }
    }
    if (p < q) { // > 135 & <= 180 && > 315 & <= 360
        bool flag = false;
        if (p1.x > p2.x && p1.y <= p2.y) { _p1 = {p1.x, p2.y}; _p2 = {p2.x, p1.y}; flag = true; }
        else if (p1.x < p2.x && p1.y >= p2.y) { _p1 = {p2.x, p1.y}; _p2 = {p1.x, p2.y}; flag = true; }
        if(flag) {
            values = {1, -1};
            flags = {1, 0, 0};
        }
    }
    // return std::vector<Point2i> {};
    return bresenhamLineDrawing(_p1, _p2, values, flags);
}

void DSSCreator::voxelizeDSS(Point3i p1, Point3i p2, OcTree* ocTree) {
    Point3i absPoint;
    absPoint.x = std::abs(p2.x - p1.x);
    absPoint.y = std::abs(p2.y - p1.y);
    absPoint.z = std::abs(p2.z - p1.z);
    int max = std::max({absPoint.x, absPoint.y, absPoint.z});
    // std::cout << "voxelizeDSS: max = " << max << std::endl;
    if (max == 0) {
        ocTree->insert(p1);
        return;
    }
    std::vector<Point3i> voxels;
    voxels.resize(max + 1);
    if (max == absPoint.x) { // x coordinates with highest difference
        auto pixels = rasterizeDSS(Point2i{p1.x, p1.y}, Point2i{p2.x, p2.y});
        for(int i = 0; i < pixels.size(); i++) {
            voxels[i].x = pixels[i].x;
            voxels[i].y = pixels[i].y;
        }
        int size1 = pixels.size();
        pixels = rasterizeDSS(Point2i{p1.x, p1.z}, Point2i{p2.x, p2.z});
        int size2 = pixels.size();
        assert(size1 == size2 && "rasterizeDSS should return the same number of points for both projections");
        for (int i = 0; i < size1; i++) voxels[i].z = pixels[i].y;
    } else if (max == absPoint.y) { // y coordinates with highest difference
        auto pixels = rasterizeDSS(Point2i{p1.x, p1.y}, Point2i{p2.x, p2.y});
        for(int i = 0; i < pixels.size(); i++) {
            voxels[i].x = pixels[i].x;
            voxels[i].y = pixels[i].y;
        }
        int size1 = pixels.size();
        pixels = rasterizeDSS(Point2i{p1.y, p1.z}, Point2i{p2.y, p2.z});
        int size2 = pixels.size();
        assert(size1 == size2 && "rasterizeDSS should return the same number of points for both projections");
        for (int i = 0; i < size1; i++) voxels[i].z = pixels[i].y;
    } else { // z coordinates with highest difference
        auto pixels = rasterizeDSS(Point2i{p1.x, p1.z}, Point2i{p2.x, p2.z});
        for(int i = 0; i < pixels.size(); i++) {
            voxels[i].x = pixels[i].x;
            voxels[i].z = pixels[i].y;
        }
        int size1 = pixels.size();
        pixels = rasterizeDSS(Point2i{p1.y, p1.z}, Point2i{p2.y, p2.z});
        int size2 = pixels.size();
        assert(size1 == size2 && "rasterizeDSS should return the same number of points for both projections");
        for (int i = 0; i < size1; i++) voxels[i].y = pixels[i].x;
    }
    for (const auto& voxel : voxels)
        ocTree->insert(voxel);
}

std::vector<Point2i> DSSCreator::bresenhamLineDrawing(Point2i p1, Point2i p2, Values values, Flags flags) {
    int p = (p2.y - p1.y);
    int q = (p1.x - p2.x);
    if (flags.flag3 == 0) swap(&p1.x, &p2.x);
    if (flags.flag3 == 1) swap(&p1.y, &p2.y);
    if (flags.flag3 == 2) {
        swap(&p1.x, &p2.x);
        swap(&p1.y, &p2.y);
    }
    int f = 2 * p + q;
    int d = 2 * p;
    int dd = 2 * (p + q);

    std::vector<Point2i> pixels;

    // m_pixels.reserve(std::abs(p2.x - p1.x) + 1);

    while (p1.x <= p2.x) {
        if (flags.flag2 == 0) { // x and y in original form
            pixels.emplace_back(p1.x, p1.y);
        } else { // x and y in swapped form
            pixels.emplace_back(p1.y, p1.x);
        }
        if (f <= 0) {
            if (flags.flag1 == 0) f += d;
            else {
                f += dd;
                p1.y += values.val2;
            }
        } else {
            if (flags.flag1 == 0) {
                f += dd;
                p1.y += values.val2;
            } else f += d;
        }
        p1.x += values.val1;
    }

    return pixels;
}