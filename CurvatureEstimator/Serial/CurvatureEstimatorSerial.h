#pragma once

#include "../../Helper/GeometryTypes.h"
#include "../CurvatureEstimatorBase.h"
#include <vector>
#include <array>
#include <cmath>
#include <limits>

class CurvatureEstimatorSerial : public CurvatureEstimatorBase {
public:
    explicit CurvatureEstimatorSerial(const BBox3i& bounds);
    void preprocessVoxels(std::vector<unsigned char>& voxels,
                          const Dimensions3i& dims) override;
    void estimateCurvature(int curveLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;
private:
    const std::vector<unsigned char>* m_voxels;
    BBox3i m_bounds;
    int m_curveLength;
    Dimensions3i m_dims;
private:
    void computeInnerSpaceAndFrontierVoxels(std::vector<unsigned char>& voxels);
    void computeInnerSpaceVoxels(
        std::vector<unsigned char>& voxels,
        int R,
        int C,
        int D,
        int plane
    );
    void markInteriorVoxels(std::vector<unsigned char>& voxels);
    void computeFrontierVoxels(std::vector<unsigned char>& voxels);
    int computeCurvatureAtVoxel(Point3i voxel);
    size_t getVoxelID(int x, int y, int z) const;
    bool inGrid(int x, int y, int z) const;
    bool isSurface(const Point3i& voxel) const;
    bool isInterior(const Point3i& voxel) const;
    int computeCurvatureOfDigitalCurve3D(Point3i voxel, int plane);
    void getNeighborsInPlane(Point3i voxel, int plane, std::vector<std::pair<Point3i, uint8_t>>& neighbors);
    bool computeCurvatureOfDigitalCurve2D(
        int plane,
        Point3i voxel,
        std::vector<std::pair<Point3i, uint8_t>>& trailCurve,
        std::vector<std::pair<Point3i, uint8_t>>& leadCurve,
        std::vector<Point3i>& curve3D,
        int& curvature
    );
    bool findNextLevelVoxels(
        int i,
        int plane,
        std::vector<std::pair<Point3i, uint8_t>>& curve,
        const std::vector<Point3i>& curve3D
    );
    void addNeighbor(Point3i voxel, Point3i neighbor, int neighborIndex, int plane, std::vector<std::pair<Point3i, uint8_t>>& neighbors);
    uint8_t getChainCode(Point2i p1, Point2i p2);
};