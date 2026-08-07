#include "CurvatureEstimatorSerialEnv.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <iostream>
#include <stdexcept>

namespace {
    constexpr uint8_t INVALID_CHAIN_CODE = 255;
    const Point3i INVALID_POINT(-1, -1, -1);

    using CurveCandidate = std::pair<Point3i, uint8_t>;

    CurveCandidate invalidCandidate() {
        return {INVALID_POINT, INVALID_CHAIN_CODE};
    }

    bool isInvalidCandidate(const CurveCandidate& candidate) {
        return candidate.first == INVALID_POINT || candidate.second == INVALID_CHAIN_CODE;
    }
}

CurvatureEstimatorSerialEnv::CurvatureEstimatorSerialEnv(const BBox3i& bounds)
    : m_voxels(nullptr), m_bounds(bounds) {}

void CurvatureEstimatorSerialEnv::preprocessVoxels(std::vector<unsigned char>& voxels,
                                                   const Dimensions3i& dims) {
    m_dims = dims;
    computeInnerSpaceAndFrontierVoxels(voxels);
}

void CurvatureEstimatorSerialEnv::computeInnerSpaceAndFrontierVoxels(
    std::vector<unsigned char>& voxels) {
    std::cout << "Computing inner space via plane 2\n";
    computeInnerSpaceVoxels(voxels, m_dims.width, m_dims.height, m_dims.depth, 2);
    std::cout << "Computing inner space via plane 1\n";
    computeInnerSpaceVoxels(voxels, m_dims.depth, m_dims.width, m_dims.height, 1);
    std::cout << "Computing inner space via plane 0\n";
    computeInnerSpaceVoxels(voxels, m_dims.height, m_dims.depth, m_dims.width, 0);
    std::cout << "Marking interior voxels\n";
    markInteriorVoxels(voxels);
    std::cout << "Marking frontier voxels\n";
    computeFrontierVoxels(voxels);
}

void CurvatureEstimatorSerialEnv::computeInnerSpaceVoxels(
    std::vector<unsigned char>& voxels,
    int R,
    int C,
    int D,
    int plane) {
    auto getVoxelID = [&](int i, int j, int k) {
        int id = 0;
        int W = 0;
        switch (plane) {
            case 0: id = i * D + j * R * D; W = 1;     break;
            case 1: id = i * C * D + j;     W = C;     break;
            case 2: id = i + j * R;         W = R * C; break;
            default: throw std::runtime_error("invalid inner-space plane");
        }
        return static_cast<size_t>(id + k * W);
    };

    auto isVoxel = [&](int i, int j, int k) {
        return voxels[getVoxelID(i, j, k)] == 1;
    };

    auto markInterior = [&](int i, int j, int k) {
        voxels[getVoxelID(i, j, k)] += 2;
    };

    auto removeInterior = [&](int i, int j, int k) {
        voxels[getVoxelID(i, j, k)] -= 2;
    };

    for (int i = 0; i < R; ++i) {
        for (int j = 0; j < C; ++j) {
            int count = 0;
            int k = 0;
            for (; k < D; ++k) {
                if (isVoxel(i, j, k)) {
                    if (k == D - 1 || isVoxel(i, j, k + 1)) {
                        continue;
                    }
                    if (!isVoxel(i, j, k + 1)) {
                        count++;
                    }
                } else if (count % 2 == 1) {
                    markInterior(i, j, k);
                }
            }
            if (count % 2 == 1) {
                k--;
                while (k >= 0 && !isVoxel(i, j, k)) {
                    removeInterior(i, j, k);
                    k--;
                }
            }
        }
    }
}

void CurvatureEstimatorSerialEnv::markInteriorVoxels(std::vector<unsigned char>& voxels) {
    for (int z = 0; z < m_dims.depth; ++z) {
        for (int y = 0; y < m_dims.height; ++y) {
            for (int x = 0; x < m_dims.width; ++x) {
                const size_t id = static_cast<size_t>(x) +
                                  static_cast<size_t>(y) * m_dims.width +
                                  static_cast<size_t>(z) * m_dims.width * m_dims.height;
                if (voxels[id] == 2 || voxels[id] == 4) {
                    voxels[id] = 0;
                } else if (voxels[id] == 6) {
                    voxels[id] = 2;
                }
            }
        }
    }
}

void CurvatureEstimatorSerialEnv::computeFrontierVoxels(std::vector<unsigned char>& voxels) {
    for (int z = 0; z < m_dims.depth; ++z) {
        for (int y = 0; y < m_dims.height; ++y) {
            for (int x = 0; x < m_dims.width; ++x) {
                const size_t id = static_cast<size_t>(x) +
                                  static_cast<size_t>(y) * m_dims.width +
                                  static_cast<size_t>(z) * m_dims.width * m_dims.height;
                if (voxels[id] != 1) continue;
                if (x == 0 || voxels[id - 1] == 0) continue;
                if (x == m_dims.width - 1 || voxels[id + 1] == 0) continue;
                if (y == 0 || voxels[id - m_dims.width] == 0) continue;
                if (y == m_dims.height - 1 || voxels[id + m_dims.width] == 0) continue;
                if (z == 0 || voxels[id - m_dims.width * m_dims.height] == 0) continue;
                if (z == m_dims.depth - 1 || voxels[id + m_dims.width * m_dims.height] == 0) continue;
                voxels[id] = 2;
            }
        }
    }
}

void CurvatureEstimatorSerialEnv::estimateCurvature(int curveLength,
                                                    const std::vector<unsigned char>& voxels,
                                                    std::vector<int>& curvatures,
                                                    const Dimensions3i& dims) {
    std::cout << "Estimating curvature using Serial path..." << std::endl;
    m_curveLength = curveLength;
    m_dims = dims;
    m_voxels = &voxels;
    if (curvatures.size() != voxels.size()) {
        throw std::runtime_error("curvatures and voxels must have the same grid size");
    }

    int localMaxCurvature = std::numeric_limits<int>::min();
    int localMinCurvature = std::numeric_limits<int>::max();
    int count = 0;
    for (int z = 0; z < m_dims.depth; ++z) {
        for (int y = 0; y < m_dims.height; ++y) {
            for (int x = 0; x < m_dims.width; ++x) {
                const size_t id = getVoxelID(x, y, z);
                if (voxels[id] != 1) {
                    continue;
                }

                count++;
                // std::cout << "Total grid cells: " << voxels.size() << "; processing surface voxel: " << count << std::endl;
                int curvature = computeCurvatureAtVoxel(Point3i(x, y, z));
                curvatures[id] = curvature;
                if (curvature != std::numeric_limits<int>::max()) {
                    localMaxCurvature = std::max(localMaxCurvature, curvature);
                    localMinCurvature = std::min(localMinCurvature, curvature);
                }
            }
        }
    }
}

size_t CurvatureEstimatorSerialEnv::getVoxelID(int x, int y, int z) const {
    return static_cast<size_t>(x) +
           static_cast<size_t>(y) * m_dims.width +
           static_cast<size_t>(z) * m_dims.width * m_dims.height;
}

bool CurvatureEstimatorSerialEnv::inGrid(int x, int y, int z) const {
    return x >= 0 && x < m_dims.width &&
           y >= 0 && y < m_dims.height &&
           z >= 0 && z < m_dims.depth;
}

bool CurvatureEstimatorSerialEnv::isSurface(const Point3i& voxel) const {
    return m_voxels && inGrid(voxel.x, voxel.y, voxel.z) &&
           (*m_voxels)[getVoxelID(voxel.x, voxel.y, voxel.z)] == 1;
}

bool CurvatureEstimatorSerialEnv::isInterior(const Point3i& voxel) const {
    return m_voxels && inGrid(voxel.x, voxel.y, voxel.z) &&
           (*m_voxels)[getVoxelID(voxel.x, voxel.y, voxel.z)] == 2;
}

int CurvatureEstimatorSerialEnv::computeCurvatureAtVoxel(Point3i voxel) {
    int maxCurvature = std::numeric_limits<int>::min();
    int minCurvature = std::numeric_limits<int>::max();
    for (int plane = 0; plane < 9; ++plane) {
        int curvature = computeCurvatureOfDigitalCurve3D(voxel, plane);
        if (curvature == -1) continue;
        maxCurvature = std::max(maxCurvature, curvature);
        minCurvature = std::min(minCurvature, curvature);
    }
    if (maxCurvature == std::numeric_limits<int>::min() || minCurvature == std::numeric_limits<int>::max()) {
        return std::numeric_limits<int>::max(); // No valid curvature found
    }
    return maxCurvature + minCurvature;
}

int CurvatureEstimatorSerialEnv::computeCurvatureOfDigitalCurve3D(
    Point3i voxel, int plane
) {
    std::vector<Point3i> curve3D(2 * m_curveLength + 1);
    curve3D[m_curveLength] = voxel;
    int curvature = std::numeric_limits<int>::max();
    std::vector<std::pair<Point3i, uint8_t>> headCurve(8, invalidCandidate());
    std::vector<std::pair<Point3i, uint8_t>> trailCurve(8, invalidCandidate());
    std::vector<std::pair<Point3i, uint8_t>> leadCurve(8, invalidCandidate());

    getNeighborsInPlane(voxel, plane, headCurve);
    int count = 0;
    for (const auto& candidate : headCurve) {
        if (!isInvalidCandidate(candidate)) {
            count++;
        }
    }
    if (count == 0 || count > 3) return -1;

    // Extend the lead and the trail curves
    for (int i = 0; i < 7; ++i) {
        if (isInvalidCandidate(headCurve[i])) continue;

        for (int j = i + 1; j < 8; ++j) {
            if (isInvalidCandidate(headCurve[j])) continue;

            std::fill(trailCurve.begin(), trailCurve.end(), invalidCandidate());
            std::fill(leadCurve.begin(), leadCurve.end(), invalidCandidate());

            trailCurve[0] = headCurve[i];
            leadCurve[0] = headCurve[j];

            bool ok = computeCurvatureOfDigitalCurve2D(
                plane,
                voxel,
                trailCurve,
                leadCurve,
                curve3D,
                curvature
            );

            if (!ok) continue;
            if (curvature == 0) return 0;
        }
    }
    return (curvature == std::numeric_limits<int>::max()) ? -1 : curvature;
}

bool CurvatureEstimatorSerialEnv::computeCurvatureOfDigitalCurve2D(
    int plane,
    Point3i voxel,
    std::vector<std::pair<Point3i, uint8_t>>& trailCurve,
    std::vector<std::pair<Point3i, uint8_t>>& leadCurve,
    std::vector<Point3i>& curve3D,
    int& curvature
) {
    // Placeholder for the actual implementation of 2D curvature computation
    // This function should compute the curvature based on the lead and trail curves
    uint8_t prevTrailChainCode = INVALID_CHAIN_CODE;
    uint8_t prevLeadChainCode = INVALID_CHAIN_CODE;
    int curvatureSum = 0;
    int i = 1;
    for(; i <= m_curveLength; ++i) {
        uint8_t trailChainCode = INVALID_CHAIN_CODE, leadChainCode = INVALID_CHAIN_CODE;
        int minDiff = INVALID_CHAIN_CODE;
        for(int j = 0; j < 8; j++) {
            if(isInvalidCandidate(trailCurve[j])) continue;
            Point3i voxel1 = trailCurve[j].first;
            uint8_t trailChain = trailCurve[j].second;
            for(int k = 0; k < 8; k++) {
                if(isInvalidCandidate(leadCurve[k])) continue;
                Point3i voxel2 = leadCurve[k].first;
                uint8_t leadChain = leadCurve[k].second < 4 ?
                                    leadCurve[k].second + 4 :
                                    leadCurve[k].second - 4;
                int distX12 = std::max(std::max(
                    std::abs(voxel1.x-voxel2.x),
                    std::abs(voxel1.y-voxel2.y)),
                    std::abs(voxel1.z-voxel2.z));
                int distX10 = std::max(std::max(
                    std::abs(voxel1.x-voxel.x),
                    std::abs(voxel1.y-voxel.y)),
                    std::abs(voxel1.z-voxel.z));
                int distX20 = std::max(std::max(
                    std::abs(voxel.x-voxel2.x),
                    std::abs(voxel.y-voxel2.y)),
                    std::abs(voxel.z-voxel2.z));
                if((distX10 == 1 || distX20 == 1) && i != 1) continue;
                if((distX10 != 1 || distX20 != 1) && distX12 <= 1) continue;
                int diff = std::abs(static_cast<int>(leadChain) - static_cast<int>(trailChain));
                diff = std::min(diff, 8 - diff);
                if(diff < minDiff){
                    trailChainCode = trailChain;
                    leadChainCode = leadChain;
                    minDiff = diff;
                    curve3D[m_curveLength-i] = voxel1;
                    curve3D[m_curveLength+i] = voxel2;
                }
            }
        }
        if(trailChainCode == INVALID_CHAIN_CODE || leadChainCode == INVALID_CHAIN_CODE) break;
        if(prevTrailChainCode != INVALID_CHAIN_CODE && prevLeadChainCode != INVALID_CHAIN_CODE) {
            int tmp = std::abs(trailChainCode - leadChainCode);
            tmp = std::min(tmp, 8 - tmp);
            int tmp1 = std::abs(trailChainCode - prevLeadChainCode);
            tmp1 = std::min(tmp1, 8 - tmp1);
            tmp = std::min(tmp, tmp1);
            tmp1 = std::abs(prevTrailChainCode - leadChainCode);
            tmp1 = std::min(tmp1, 8 - tmp1);
            tmp = std::min(tmp, tmp1);
            tmp1 = std::abs(prevTrailChainCode - prevLeadChainCode);
            tmp1 = std::min(tmp1, 8 - tmp1);
            tmp = std::min(tmp, tmp1);
            curvatureSum += tmp;
        }
        prevLeadChainCode = leadChainCode;
        prevTrailChainCode = trailChainCode;
        
        if (i == m_curveLength) continue;
        
        //next trailing voxels at height k+1
        bool ok = findNextLevelVoxels(-i, plane, trailCurve, curve3D);
        if (!ok) break;

        //next leading voxels at height k+1
        ok = findNextLevelVoxels(i, plane, leadCurve, curve3D);
        if (!ok) break;
    }
    if (i != m_curveLength + 1) return false;
    curvature = std::min(curvatureSum, curvature);
    return true; // Return true if the curvature computation was successful, false otherwise
}

bool CurvatureEstimatorSerialEnv::findNextLevelVoxels(
    int i,
    int plane,
    std::vector<std::pair<Point3i, uint8_t>>& curve,
    const std::vector<Point3i>& curve3D
) {
    // Placeholder for the actual implementation of finding next level voxels
    // This function should populate the curve vector with the next level voxels based on the current voxel and plane
    Point3i voxel = curve3D[m_curveLength + i];
    i = std::abs(i);
    getNeighborsInPlane(voxel, plane, curve);
    int count = 0;
    for(int j = 0; j < 8; j++) {
        for(int k = m_curveLength - i; k <= m_curveLength + i; k++) {
            if(curve3D[k] == curve[j].first) {
                curve[j] = invalidCandidate();
                break;
            }
        }
        if (!isInvalidCandidate(curve[j])) {
            count++;
        }
        if (count > 3) break;
    }
    if (count == 0 || count > 3) return false;
    return true; // Return true if next level voxels are found, false otherwise
}

void CurvatureEstimatorSerialEnv::getNeighborsInPlane(
    Point3i voxel, int plane,
    std::vector<std::pair<Point3i, uint8_t>>& neighbors) {
    // Placeholder for the actual implementation of getting neighbors in a specific plane
    // This function should populate the neighbors vector with the neighboring voxels in the specified plane
    // The implementation will depend on how the planes are defined and how neighbors are determined
    std::fill(neighbors.begin(), neighbors.end(), invalidCandidate());
    std::vector<Point3i> voxels(8);
    int x = voxel.x, y = voxel.y, z = voxel.z;
    const int xmin = 0;
    const int xmax = m_dims.width - 1;
    const int ymin = 0;
    const int ymax = m_dims.height - 1;
    const int zmin = 0;
    const int zmax = m_dims.depth - 1;

    switch(plane) {
        case 0: // xy plane zero degree
            voxels[0] = Point3i(voxel.x+1, voxel.y, voxel.z);
            voxels[1] = Point3i(voxel.x-1, voxel.y, voxel.z);
            voxels[2] = Point3i(voxel.x, voxel.y+1, voxel.z);
            voxels[3] = Point3i(voxel.x, voxel.y-1, voxel.z);
            voxels[4] = Point3i(voxel.x+1, voxel.y+1, voxel.z);
            voxels[5] = Point3i(voxel.x+1, voxel.y-1, voxel.z);
            voxels[6] = Point3i(voxel.x-1, voxel.y+1, voxel.z);
            voxels[7] = Point3i(voxel.x-1, voxel.y-1, voxel.z);
            if(x != xmax && isSurface(voxels[0])) addNeighbor(voxel, voxels[0], 0, plane, neighbors);
            if(x != xmin && isSurface(voxels[1])) addNeighbor(voxel, voxels[1], 1, plane, neighbors);
            if(y != ymax && isSurface(voxels[2])) addNeighbor(voxel, voxels[2], 2, plane, neighbors);
            if(y != ymin && isSurface(voxels[3])) addNeighbor(voxel, voxels[3], 3, plane, neighbors);
            if(x != xmax && y != ymax && isSurface(voxels[4])) addNeighbor(voxel, voxels[4], 4, plane, neighbors);
            if(x != xmax && y != ymin && isSurface(voxels[5])) addNeighbor(voxel, voxels[5], 5, plane, neighbors);
            if(x != xmin && y != ymax && isSurface(voxels[6])) addNeighbor(voxel, voxels[6], 6, plane, neighbors);
            if(x != xmin && y != ymin && isSurface(voxels[7])) addNeighbor(voxel, voxels[7], 7, plane, neighbors);
        break;
        case 1: // xy plane 45 degree anti-clockwise
            voxels[0] = Point3i(voxel.x, voxel.y + 1, voxel.z);
            voxels[1] = Point3i(voxel.x, voxel.y - 1, voxel.z);
            voxels[2] = Point3i(voxel.x + 1, voxel.y, voxel.z - 1);
            voxels[3] = Point3i(voxel.x - 1, voxel.y, voxel.z + 1);
            voxels[4] = Point3i(voxel.x + 1, voxel.y + 1, voxel.z - 1);
            voxels[5] = Point3i(voxel.x - 1, voxel.y + 1, voxel.z + 1);
            voxels[6] = Point3i(voxel.x + 1, voxel.y - 1, voxel.z - 1);
            voxels[7] = Point3i(voxel.x - 1, voxel.y - 1, voxel.z + 1);
            if(y != ymax && (isSurface(voxels[0]) ||
            (x != xmin && isSurface(Point3i(x - 1, y + 1, z)) ||
            x != xmax && isSurface(Point3i(x + 1, y + 1, z))) &&
            (z != zmin && isSurface(Point3i(x, y + 1, z - 1)) ||
            z != zmax && isSurface(Point3i(x, y + 1, z + 1))) &&
            !isInterior(voxels[0])))
                addNeighbor(voxel, voxels[0], 0, plane, neighbors);
            if(y != ymin && (isSurface(voxels[1]) ||
            (x != xmin && isSurface(Point3i(x - 1, y - 1, z)) ||
            x != xmax && isSurface(Point3i(x + 1, y - 1, z))) &&
            (z != zmin && isSurface(Point3i(x, y - 1, z - 1)) ||
            z != zmax && isSurface(Point3i(x, y - 1, z + 1))) &&
            !isInterior(voxels[1])))
                addNeighbor(voxel, voxels[1], 1, plane, neighbors);
            if(voxel.x != xmax && voxel.z != zmin && isSurface(voxels[2]))
                addNeighbor(voxel, voxels[2], 2, plane, neighbors);
            if(voxel.x != xmin && voxel.z != zmax && isSurface(voxels[3]))
                addNeighbor(voxel, voxels[3], 3, plane, neighbors);
            if(voxel.x != xmax && voxel.y != ymax && voxel.z != zmin && (isSurface(voxels[4]) ||
            isSurface(Point3i(voxel.x, voxel.y + 1, voxel.z - 1)) && isSurface(Point3i(voxel.x + 1, voxel.y + 1, voxel.z)) &&
            !isInterior(voxels[4])))
                addNeighbor(voxel, voxels[4], 4, plane, neighbors);
            if(x != xmin && y != ymax && z != zmax && (isSurface(voxels[5]) ||
            isSurface(Point3i(x, y + 1, z + 1)) && isSurface(Point3i(x - 1, y + 1, z)) &&
            !isInterior(voxels[5])))
                addNeighbor(voxel, voxels[5], 5, plane, neighbors);
            if(x != xmax && y != ymin && z != zmin && (isSurface(voxels[6]) ||
            isSurface(Point3i(x, y - 1, z - 1)) && isSurface(Point3i(x + 1, y - 1, z)) &&
            !isInterior(voxels[6])))
                addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != xmin && y != ymin && z != zmax && (isSurface(voxels[7]) ||
			isSurface(Point3i(x, y - 1, z + 1)) && isSurface(Point3i(x - 1, y - 1, z)) &&
            !isInterior(voxels[7])))
				addNeighbor(voxel, voxels[7], 7, plane, neighbors);
		break;
		case 2: // xy plane 45 degree clockwise
            voxels[0] = Point3i(voxel.x, voxel.y + 1, voxel.z);
            voxels[1] = Point3i(voxel.x, voxel.y - 1, voxel.z);
            voxels[2] = Point3i(voxel.x + 1, voxel.y, voxel.z + 1);
            voxels[3] = Point3i(voxel.x - 1, voxel.y, voxel.z - 1);
            voxels[4] = Point3i(voxel.x + 1, voxel.y + 1, voxel.z + 1);
            voxels[5] = Point3i(voxel.x + 1, voxel.y - 1, voxel.z + 1);
            voxels[6] = Point3i(voxel.x - 1, voxel.y + 1, voxel.z - 1);
            voxels[7] = Point3i(voxel.x - 1, voxel.y - 1, voxel.z - 1);
			if(y != ymax && (isSurface(voxels[0]) ||
			(x != xmin && isSurface(Point3i(x - 1, y + 1, z)) || x != xmax && isSurface(Point3i(x + 1, y + 1, z))) &&
			(z != zmin && isSurface(Point3i(x, y + 1, z - 1)) || z != zmax && isSurface(Point3i(x, y + 1, z + 1))) &&
            !isInterior(voxels[0])))
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(y != ymin && (isSurface(voxels[1]) ||
			(x != xmin && isSurface(Point3i(x - 1, y - 1, z)) || x != xmax && isSurface(Point3i(x + 1, y - 1, z))) &&
			(z != zmin && isSurface(Point3i(x, y - 1, z - 1)) || z != zmax && isSurface(Point3i(x, y - 1, z + 1))) &&
            !isInterior(voxels[1])))
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(x != xmax && z != zmax && isSurface(voxels[2]))
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(x != xmin && z != zmin && isSurface(voxels[3]))
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != xmax && y != ymax && z != zmax && (isSurface(voxels[4]) ||
			isSurface(Point3i(x, y + 1, z + 1)) && isSurface(Point3i(x + 1, y + 1, z)) && !isInterior(voxels[4])))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != xmax && y != ymin && z != zmax && (isSurface(voxels[5]) ||
			isSurface(Point3i(x, y - 1, z + 1)) && isSurface(Point3i(x + 1, y - 1, z)) &&
            !isInterior(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != xmin && y != ymax && z != zmin && (isSurface(voxels[6]) ||
			isSurface(Point3i(x - 1, y + 1, z)) && isSurface(Point3i(x, y + 1, z - 1)) &&
            !isInterior(voxels[6])))
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != xmin && y != ymin && z != zmin && (isSurface(voxels[7]) ||
			isSurface(Point3i(x, y - 1, z - 1)) && isSurface(Point3i(x - 1, y - 1, z)) &&
            !isInterior(voxels[7])))
				addNeighbor(voxel, voxels[7], 7, plane, neighbors);
		break;
		case 3: // yz plane zero degree
            voxels[0] = Point3i(voxel.x, voxel.y + 1, voxel.z);
            voxels[1] = Point3i(voxel.x, voxel.y - 1, voxel.z);
            voxels[2] = Point3i(voxel.x, voxel.y, voxel.z + 1);
            voxels[3] = Point3i(voxel.x, voxel.y, voxel.z - 1);
            voxels[4] = Point3i(voxel.x, voxel.y + 1, voxel.z + 1);
            voxels[5] = Point3i(voxel.x, voxel.y + 1, voxel.z - 1);
            voxels[6] = Point3i(voxel.x, voxel.y - 1, voxel.z + 1);
            voxels[7] = Point3i(voxel.x, voxel.y - 1, voxel.z - 1);
			if(y != ymax && isSurface(voxels[0])) addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(y != ymin && isSurface(voxels[1])) addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(z != zmax && isSurface(voxels[2])) addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(z != zmin && isSurface(voxels[3])) addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(y != ymax && z != zmax && isSurface(voxels[4])) addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(y != ymax && z != zmin && isSurface(voxels[5])) addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(y != ymin && z != zmax && isSurface(voxels[6])) addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(y != ymin && z != zmin && isSurface(voxels[7])) addNeighbor(voxel, voxels[7], 7, plane, neighbors);
		break;
		case 4: // yz plane 45 degree anti-clockwise
            voxels[0] = Point3i(voxel.x, voxel.y, voxel.z + 1);
            voxels[1] = Point3i(voxel.x, voxel.y, voxel.z - 1);
            voxels[2] = Point3i(voxel.x - 1, voxel.y + 1, voxel.z);
            voxels[3] = Point3i(voxel.x + 1, voxel.y - 1, voxel.z);
            voxels[4] = Point3i(voxel.x - 1, voxel.y + 1, voxel.z + 1);
            voxels[5] = Point3i(voxel.x - 1, voxel.y + 1, voxel.z - 1);
            voxels[6] = Point3i(voxel.x + 1, voxel.y - 1, voxel.z + 1);
            voxels[7] = Point3i(voxel.x + 1, voxel.y - 1, voxel.z - 1);
			if(z != zmax && (isSurface(voxels[0]) ||
			(x != xmin && isSurface(Point3i(x-1, y, z+1))|| x != xmax && isSurface(Point3i(x+1, y, z+1))) &&
			(y != ymin && isSurface(Point3i(x, y-1, z+1)) || y != ymax && isSurface(Point3i(x, y+1, z+1))) &&
            !isInterior(voxels[0])))
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(z != zmin && (isSurface(voxels[1]) ||
			(x != xmin && isSurface(Point3i(x-1, y, z-1))|| x != xmax && isSurface(Point3i(x+1, y, z-1))) &&
			(y != ymin && isSurface(Point3i(x, y-1, z-1)) || y != ymax && isSurface(Point3i(x, y+1, z-1))) &&
            !isInterior(voxels[1])))
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(x != xmin && y != ymax && (isSurface(voxels[2]))) 
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(x != xmax && y != ymin && (isSurface(voxels[3])))
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != xmin && y != ymax && z != zmax && (isSurface(voxels[4]) ||
			isSurface(Point3i(x, y+1, z+1)) && isSurface(Point3i(x-1, y, z+1)) &&
            !isInterior(Point3i(x-1, y+1, z+1))))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != xmin && y != ymax && z != zmin && (isSurface(voxels[5]) ||
			isSurface(Point3i(x, y+1, z-1)) && isSurface(Point3i(x-1, y, z-1)) &&
            !isInterior(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != xmax && y != ymin && z != zmax && (isSurface(voxels[6]) ||
			isSurface(Point3i(x, y-1, z+1)) && isSurface(Point3i(x+1, y, z+1)) &&
            !isInterior(voxels[6])))
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != xmax && y != ymin && z != zmin && (isSurface(voxels[7]) ||
			isSurface(Point3i(x, y-1, z-1)) && isSurface(Point3i(x+1, y, z-1)) &&
            !isInterior(voxels[7])))
				addNeighbor(voxel, voxels[7], 7, plane, neighbors);
		break;
		case 5: // yz plane 45 degree clockwise
            voxels[0] = Point3i(voxel.x, voxel.y, voxel.z + 1);
            voxels[1] = Point3i(voxel.x, voxel.y, voxel.z - 1);
            voxels[2] = Point3i(voxel.x + 1, voxel.y + 1, voxel.z);
            voxels[3] = Point3i(voxel.x - 1, voxel.y - 1, voxel.z);
            voxels[4] = Point3i(voxel.x + 1, voxel.y + 1, voxel.z + 1);
            voxels[5] = Point3i(voxel.x + 1, voxel.y + 1, voxel.z - 1);
            voxels[6] = Point3i(voxel.x - 1, voxel.y - 1, voxel.z + 1);
            voxels[7] = Point3i(voxel.x - 1, voxel.y - 1, voxel.z - 1);
			if(z != zmax && (isSurface(voxels[0]) ||
			(x != xmin && isSurface(Point3i(x-1, y, z+1)) || x != xmax && isSurface(Point3i(x+1, y, z+1))) &&
			(y != ymin && isSurface(Point3i(x, y-1, z+1)) || y != ymax && isSurface(Point3i(x, y+1, z+1))) &&
            !isInterior(voxels[0]))) 
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(z != zmin && (isSurface(voxels[1]) ||
			(x != xmin && isSurface(Point3i(x-1, y, z-1)) || x != xmax && isSurface(Point3i(x+1, y, z-1))) &&
			(y != ymin && isSurface(Point3i(x, y-1, z-1)) || y != ymax && isSurface(Point3i(x, y+1, z-1))) &&
            !isInterior(voxels[1]))) 
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(x != xmax && y != ymax && isSurface(voxels[2]))
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(x != xmin && y != ymin && isSurface(voxels[3]))
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != xmax && y != ymax && z != zmax && (isSurface(voxels[4]) ||
			isSurface(Point3i(x, y+1, z+1)) && isSurface(Point3i(x+1, y, z+1)) &&
            !isInterior(voxels[4])))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != xmax && y != ymax && z != zmin && (isSurface(voxels[5]) ||
			isSurface(Point3i(x+1, y, z-1)) && isSurface(Point3i(x, y+1, z-1)) &&
            !isInterior(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != xmin && y != ymin && z != zmax && (isSurface(voxels[6]) ||
			isSurface(Point3i(x, y-1, z+1)) && isSurface(Point3i(x-1, y, z+1)) &&
            !isInterior(voxels[6]))) 
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != xmin && y != ymin && z != zmin && (isSurface(voxels[7]) ||
			isSurface(Point3i(x, y-1, z-1)) && isSurface(Point3i(x-1, y, z-1)) &&
            !isInterior(voxels[7]))) 
				addNeighbor(voxel, voxels[7], 7, plane, neighbors);
		break;
		case 6: // zx plane zero degree
            voxels[0] = Point3i(voxel.x, voxel.y, voxel.z + 1);
            voxels[1] = Point3i(voxel.x, voxel.y, voxel.z - 1);
            voxels[2] = Point3i(voxel.x + 1, voxel.y, voxel.z);
            voxels[3] = Point3i(voxel.x - 1, voxel.y, voxel.z);
            voxels[4] = Point3i(voxel.x + 1, voxel.y, voxel.z + 1);
            voxels[5] = Point3i(voxel.x - 1, voxel.y, voxel.z + 1);
            voxels[6] = Point3i(voxel.x + 1, voxel.y, voxel.z - 1);
            voxels[7] = Point3i(voxel.x - 1, voxel.y, voxel.z - 1);
			if(z != zmax && isSurface(voxels[0])) addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(z != zmin && isSurface(voxels[1])) addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(x != xmax && isSurface(voxels[2])) addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(x != xmin && isSurface(voxels[3])) addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != xmax && z != zmax && isSurface(voxels[4])) addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != xmin && z != zmax && isSurface(voxels[5])) addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != xmax && z != zmin && isSurface(voxels[6])) addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != xmin && z != zmin && isSurface(voxels[7])) addNeighbor(voxel, voxels[7], 7, plane, neighbors);
		break;
		case 7: // zx plane 45 degree anti-clockwise
            voxels[0] = Point3i(voxel.x + 1, voxel.y, voxel.z);
            voxels[1] = Point3i(voxel.x - 1, voxel.y, voxel.z);
            voxels[2] = Point3i(voxel.x, voxel.y - 1, voxel.z + 1);
            voxels[3] = Point3i(voxel.x, voxel.y + 1, voxel.z - 1);
            voxels[4] = Point3i(voxel.x + 1, voxel.y - 1, voxel.z + 1);
            voxels[5] = Point3i(voxel.x - 1, voxel.y - 1, voxel.z + 1);
            voxels[6] = Point3i(voxel.x + 1, voxel.y + 1, voxel.z - 1);
            voxels[7] = Point3i(voxel.x - 1, voxel.y + 1, voxel.z - 1);
			if(x != xmax && (isSurface(voxels[0]) ||
			(y != ymin && isSurface(Point3i(x+1, y-1, z)) || y != ymax && isSurface(Point3i(x+1, y+1, z))) && 
			(z != zmin && isSurface(Point3i(x+1, y, z-1)) || z != zmax && isSurface(Point3i(x+1, y, z+1))) &&
            !isInterior(voxels[0])))
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(x != xmin && (isSurface(voxels[1]) ||
			(y != ymin && isSurface(Point3i(x-1, y-1, z)) || y != ymax && isSurface(Point3i(x-1, y+1, z))) && 
			(z != zmin && isSurface(Point3i(x-1, y, z-1)) || z != zmax && isSurface(Point3i(x-1, y, z+1))) &&
            !isInterior(voxels[1])))
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(y != ymin && z != zmax && (isSurface(voxels[2])))
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(y != ymax && z != zmin && (isSurface(voxels[3])))
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != xmax && y != ymin && z != zmax && (isSurface(voxels[4]) ||
			isSurface(Point3i(x+1, y, z+1)) && isSurface(Point3i(x+1, y-1, z)) &&
            !isInterior(voxels[4])))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != xmin && y != ymin && z != zmax && (isSurface(voxels[5]) ||
			isSurface(Point3i(x-1, y, z+1)) && isSurface(Point3i(x-1, y-1, z)) &&
            !isInterior(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != xmax && y != ymax && z != zmin && (isSurface(voxels[6]) ||
			isSurface(Point3i(x+1, y, z-1)) && isSurface(Point3i(x+1, y+1, z)) &&
            !isInterior(voxels[6]))) 
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != xmin && y != ymax && z != zmin && (isSurface(voxels[7]) ||
			isSurface(Point3i(x-1, y, z-1)) && isSurface(Point3i(x-1, y+1, z)) &&
            !isInterior(voxels[7])))
				addNeighbor(voxel, voxels[7], 7, plane, neighbors);
		break;
		case 8: // zx plnae 45 degree clockwise
            voxels[0] = Point3i(voxel.x + 1, voxel.y, voxel.z);
            voxels[1] = Point3i(voxel.x - 1, voxel.y, voxel.z);
            voxels[2] = Point3i(voxel.x, voxel.y + 1, voxel.z + 1);
            voxels[3] = Point3i(voxel.x, voxel.y - 1, voxel.z - 1);
            voxels[4] = Point3i(voxel.x + 1, voxel.y + 1, voxel.z + 1);
            voxels[5] = Point3i(voxel.x + 1, voxel.y - 1, voxel.z - 1);
            voxels[6] = Point3i(voxel.x - 1, voxel.y + 1, voxel.z + 1);
            voxels[7] = Point3i(voxel.x - 1, voxel.y - 1, voxel.z - 1);
			if(x != xmax && (isSurface(voxels[0]) ||
			(y != ymin && isSurface(Point3i(x+1, y-1, z)) || y != ymax && isSurface(Point3i(x+1, y+1, z))) &&
			(z != zmin && isSurface(Point3i(x+1, y, z-1)) || z != zmax && isSurface(Point3i(x+1, y, z+1))) &&
            !isInterior(voxels[0])))
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(x != xmin && (isSurface(voxels[1]) ||
			(y != ymin && isSurface(Point3i(x-1, y-1, z)) || y != ymax && isSurface(Point3i(x-1, y+1, z))) &&
			(z != zmin && isSurface(Point3i(x-1, y, z-1)) || z != zmax && isSurface(Point3i(x-1, y, z+1))) &&
            !isInterior(voxels[1])))
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(y != ymax && z != zmax && isSurface(voxels[2])) 
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(y != ymin && z != zmin && isSurface(voxels[3])) 
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != xmax && y != ymax && z != zmax && (isSurface(voxels[4]) ||
			isSurface(Point3i(x+1, y, z+1)) && isSurface(Point3i(x+1, y+1, z)) &&
            !isInterior(voxels[4])))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != xmax && y != ymin && z != zmin && (isSurface(voxels[5]) ||
			isSurface(Point3i(x+1, y, z-1)) && isSurface(Point3i(x+1, y-1, z)) &&
            !isInterior(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != xmin && y != ymax && z != zmax && (isSurface(voxels[6]) ||
			isSurface(Point3i(x-1, y, z+1)) && isSurface(Point3i(x-1, y+1, z)) &&
            !isInterior(voxels[6])))
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != xmin && y != ymin && z != zmin && (isSurface(voxels[7]) ||
			isSurface(Point3i(x-1, y, z-1)) && isSurface(Point3i(x-1, y-1, z)) &&
            !isInterior(voxels[7])))
				addNeighbor(voxel, voxels[7], 7, plane, neighbors);
	}
}

void CurvatureEstimatorSerialEnv::addNeighbor(
    Point3i voxel, Point3i neighbor, int neighborId, int plane,
    std::vector<std::pair<Point3i, uint8_t>>& neighbors) {
    // Add the neighbor voxel and its corresponding chain code to the neighbors vector
    neighbors[neighborId].first = neighbor;
    switch(plane/3) {
        case 0: neighbors[neighborId].second = getChainCode(Point2i(neighbor.x, neighbor.y), Point2i(voxel.x, voxel.y)); break;
		case 1: neighbors[neighborId].second = getChainCode(Point2i(neighbor.y, neighbor.z), Point2i(voxel.y, voxel.z)); break;
		case 2: neighbors[neighborId].second = getChainCode(Point2i(neighbor.z, neighbor.x), Point2i(voxel.z, voxel.x)); break;
    }
}

uint8_t CurvatureEstimatorSerialEnv::getChainCode(Point2i neighbor, Point2i voxel) {
    // Calculate the chain code based on the relative position of the neighbor to the voxel
    Point2i dir = neighbor - voxel;
    if(dir.x == 1 && dir.y == 0) return 0; // Right
    if(dir.x == 1 && dir.y == 1) return 1; // Down-Right
    if(dir.x == 0 && dir.y == 1) return 2; // Down
    if(dir.x == -1 && dir.y == 1) return 3; // Down-Left
    if(dir.x == -1 && dir.y == 0) return 4; // Left
    if(dir.x == -1 && dir.y == -1) return 5; // Up-Left
    if(dir.x == 0 && dir.y == -1) return 6; // Up
    if(dir.x == 1 && dir.y == -1) return 7; // Up-Right
    return INVALID_CHAIN_CODE; // Invalid chain code
}