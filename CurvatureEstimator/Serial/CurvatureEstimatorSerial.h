#pragma once

#include "../../Helper/GeometryTypes.h"
#include "../CurvatureEstimatorBase.h"
#include <vector>
#include <array>
#include <cmath>
#include <limits>

class CurvatureEstimatorSerial : public CurvatureEstimatorBase {
private:
    static constexpr int MAX_CURVE_LENGTH = 32;
    static constexpr int INVALID_VOXEL_ID = -1;
    static constexpr uint8_t INVALID_CHAIN_CODE = 255;

    struct Candidate {
        int voxelID;
        uint8_t chainCode;
    };

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
    int computeCurvatureAtVoxel(int voxelID);
    size_t getVoxelID(int x, int y, int z) const;
    Point3i getCoordinates(int voxelID) const;
    bool inGrid(int x, int y, int z) const;
    bool isSurface(const Point3i& voxel) const;
    bool isInterior(const Point3i& voxel) const;
    Candidate invalidCandidate() const;
    bool isInvalidCandidate(const Candidate& candidate) const;
    int computeCurvatureOfDigitalCurve3D(Point3i voxel, int plane);
    void getNeighborsInPlane(Point3i voxel, int plane, std::array<Candidate, 8>& neighbors);
    bool computeCurvatureOfDigitalCurve2D(
        int plane,
        Point3i voxel,
        std::array<Candidate, 8>& trailCurve,
        std::array<Candidate, 8>& leadCurve,
        std::array<int, 2 * MAX_CURVE_LENGTH + 1>& curve3D,
        int& curvature
    );
    bool findNextLevelVoxels(
        int i,
        int plane,
        std::array<Candidate, 8>& curve,
        const std::array<int, 2 * MAX_CURVE_LENGTH + 1>& curve3D
    );
    void addNeighbor(Point3i voxel, Point3i neighbor, int neighborIndex, int plane, std::array<Candidate, 8>& neighbors);
    uint8_t getChainCode(Point2i p1, Point2i p2);
};