#pragma once

#include "../../Helper/GeometryTypes.h"
#include "../../Helper/OcTree/OcTree.h"
#include "../CurvatureEstimatorBaseEnv.h"
#include <vector>
#include <array>
#include <cmath>
#include <limits>

class CurvatureEstimatorSerialEnv : public CurvatureEstimatorBaseEnv {
public:
    CurvatureEstimatorSerialEnv(OcTree* ocTree, OcTree* ocTreeInnerVoxel);
    void estimateCurvature(int curveLength,
                           std::vector<Point3i> &voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i &dims) override;
private:
    OcTree* m_ocTree;
    OcTree* m_ocTreeInnerVoxel;
    std::vector<unsigned char> m_voxels;
    int m_curveLength;
    Dimensions3i m_dims;
private:
    int computeCurvatureAtVoxel(Point3i voxel);
    int computeCurvatureOfDigitalCurve3D(Point3i voxel, int plane);
    void getNeighborsInPlane(Point3i voxel, int plane, std::vector<std::pair<Point3i, uint8_t>>& neighbors);
    bool computeCurvatureOfDigitalCurve2D(
        int plane,
        Point3i voxel,
        std::vector<std::pair<Point3i, uint8_t>>& trailCurve,
        std::vector<std::pair<Point3i, uint8_t>>& leadCurve,
        std::vector<Point3i>& curve3D,
        uint8_t& prevTrailChainCode,
        uint8_t& prevLeadChainCode,
        int& curvatureSum,
        int& curvature
    );
    bool findNextLevelVoxels(
        int i,
        int plane,
        std::vector<std::pair<Point3i, uint8_t>>& curve,
        const std::vector<Point3i>& curve3D
    );
    void addNeighbor(Point3i voxel, Point3i neighbor, int plane, int i, std::vector<std::pair<Point3i, uint8_t>>& neighbors);
    uint8_t getChainCode(Point2i p1, Point2i p2);
};