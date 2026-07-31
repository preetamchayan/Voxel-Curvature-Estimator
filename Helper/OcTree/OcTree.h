#pragma once

#include "../GeometryTypes.h"
#include <vector>

#define INSERT 0
#define DELETE 1
#define SEARCH 2

class OcTree {
public:
    OcTree(const BBox3i& bounds);
    OcTree(const std::vector<unsigned char>& voxels, const BBox3i& bounds);
    OcTree(const std::vector<Point3i>& voxels, const BBox3i& bounds);
    bool search(Point3i p);
    bool insert(Point3i p);
    bool remove(Point3i p);
    int getVoxelCount() const;
    void getVoxels(std::vector<Point3i>& occupiedVoxels) const;

private:
    void build(const std::vector<unsigned char>& voxels);
    bool traverse(Point3i p, unsigned char flag);
    bool inRange(Point3i p, const BBox3i& bounds);
    void computeChildBounds(const BBox3i& parentBounds, const Point3i& mid,int childIndex, BBox3i& childBounds) const;
    bool isVoxel(const BBox3i& bounds) const;
    Point3i computeMid(const BBox3i& bounds) const;

private:
    std::vector<int> m_nodes;
    int m_voxelCount;
    BBox3i m_bounds;
};