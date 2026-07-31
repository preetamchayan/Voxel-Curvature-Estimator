#include "CurvatureEstimatorSerialEnv.h"

#include <algorithm>

CurvatureEstimatorSerialEnv::CurvatureEstimatorSerialEnv(OcTree* ocTree, OcTree* ocTreeInnerVoxel) : m_ocTree(ocTree), m_ocTreeInnerVoxel(ocTreeInnerVoxel) {
    m_voxels.clear();
}

void CurvatureEstimatorSerialEnv::estimateCurvature(int curveLength,
                                                    std::vector<Point3i>& voxels,
                                                    std::vector<int>& curvatures,
                                                    const Dimensions3i& dims) {
    m_curveLength = curveLength;
    m_dims = dims;
    int localMaxCurvature = std::numeric_limits<int>::min();
    int localMinCurvature = std::numeric_limits<int>::max();
    for (const auto& voxel : voxels) {
        int curvature = computeCurvatureAtVoxel(voxel);
        curvatures.push_back(curvature);
        if (curvature != std::numeric_limits<int>::max()) { // Check for valid curvature
            localMaxCurvature = std::max(localMaxCurvature, curvature);
            localMinCurvature = std::min(localMinCurvature, curvature);
        }
    }
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
    uint8_t prevTrailChainCode, prevLeadChainCode;
    int curvature = std::numeric_limits<int>::max();
    int curvatureSum = 0, count = 0;
    std::vector<std::pair<Point3i, uint8_t>> headCurve(8, std::make_pair(Point3i(-1, -1, -1), -1));
    std::vector<std::pair<Point3i, uint8_t>> trailCurve(8, std::make_pair(Point3i(-1, -1, -1), -1));
    std::vector<std::pair<Point3i, uint8_t>> leadCurve(8, std::make_pair(Point3i(-1, -1, -1), -1));

    getNeighborsInPlane(voxel, plane, headCurve);
    count = std::count(headCurve.begin(), headCurve.end(), std::make_pair(Point3i(-1, -1, -1), -1));
    count = 8 - count;
    if (count == 0 || count > 3) return -1;

    // Extend the lead and the trail curves
    for (int i = 0; i < 7; ++i) {
        if (headCurve[i].first.x == -1) continue;
        for (int j = i + 1; j < 8; ++j) {
            if (headCurve[j].first.x == -1) continue;
            trailCurve[0] = headCurve[i];
            leadCurve[0] = headCurve[j];
            bool ok = computeCurvatureOfDigitalCurve2D(
                        plane,
                        voxel,
                        trailCurve,
                        leadCurve,
                        curve3D,
                        prevTrailChainCode,
                        prevLeadChainCode,
                        curvatureSum,
                        curvature
                      );
            if (!ok) continue;
            if (curvature == 0) return 0; // Early exit if curvature is zero
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
    uint8_t& prevTrailChainCode,
    uint8_t& prevLeadChainCode,
    int& curvatureSum,
    int& curvature
) {
    // Placeholder for the actual implementation of 2D curvature computation
    // This function should compute the curvature based on the lead and trail curves
    int i = 1;
    for(; i <= m_curveLength; ++i) {
        uint8_t trailChainCode = 255, leadChainCode = 255;
        uint8_t minDiff = 255;
        for(int j = 0; j < 8; j++) {
            if(trailCurve[j].first.x == -1) continue;
            Point3i voxel1 = trailCurve[j].first;
            uint8_t trailChain = trailCurve[j].second;
            for(int k = 0; k < 8; k++) {
                if(leadCurve[k].first.x == -1) {continue;}
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
                uint8_t diff = std::abs(leadChain-trailChain);
                diff = std::min<uint8_t>(diff, 8 - diff);
                if(diff < minDiff){
                    trailChainCode = trailChain;
                    leadChainCode = leadChain;
                    minDiff = diff;
                    curve3D[m_curveLength-i] = voxel1;
                    curve3D[m_curveLength+i] = voxel2;
                }
            }
        }
        if(trailChainCode == 255 || leadChainCode == 255) break;
        if(prevTrailChainCode != 255 && prevLeadChainCode != 255){
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
        
        if(i == m_curveLength) continue;
        
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
                curve[j].first = Point3i(-1, -1, -1);
                curve[j].second = 255;
                break;
            }
        }
        count = std::count(curve.begin(), curve.end(), std::make_pair(Point3i(-1, -1, -1), 255));
        count = 8 - count;
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
    std::vector<Point3i> voxels(8);
    int x = voxel.x, y = voxel.y, z = voxel.z;
    switch(plane){
        case 0: // xy plane zero degree
            voxels[0] = Point3i(voxel.x+1, voxel.y, voxel.z);
            voxels[1] = Point3i(voxel.x-1, voxel.y, voxel.z);
            voxels[2] = Point3i(voxel.x, voxel.y+1, voxel.z);
            voxels[3] = Point3i(voxel.x, voxel.y-1, voxel.z);
            voxels[4] = Point3i(voxel.x+1, voxel.y+1, voxel.z);
            voxels[5] = Point3i(voxel.x+1, voxel.y-1, voxel.z);
            voxels[6] = Point3i(voxel.x-1, voxel.y+1, voxel.z);
            voxels[7] = Point3i(voxel.x-1, voxel.y-1, voxel.z);
            if(x != m_dims.width - 1 && m_ocTree->search(voxels[0])) addNeighbor(voxel, voxels[0], 0, plane, neighbors);
            if(x != 0 && m_ocTree->search(voxels[1])) addNeighbor(voxel, voxels[1], 1, plane, neighbors);
            if(y != m_dims.height - 1 && m_ocTree->search(voxels[2])) addNeighbor(voxel, voxels[2], 2, plane, neighbors);
            if(y != 0 && m_ocTree->search(voxels[3])) addNeighbor(voxel, voxels[3], 3, plane, neighbors);
            if(x != m_dims.width - 1 && y != m_dims.height - 1 && m_ocTree->search(voxels[4])) addNeighbor(voxel, voxels[4], 4, plane, neighbors);
            if(x != m_dims.width - 1 && y != 0 && m_ocTree->search(voxels[5])) addNeighbor(voxel, voxels[5], 5, plane, neighbors);
            if(x != 0 && y != m_dims.height - 1 && m_ocTree->search(voxels[6])) addNeighbor(voxel, voxels[6], 6, plane, neighbors);
            if(x != 0 && y != 0 && m_ocTree->search(voxels[7])) addNeighbor(voxel, voxels[7], 7, plane, neighbors);
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
            if(y != m_dims.height - 1 && (m_ocTree->search(voxels[0]) ||
            (x != 0 && m_ocTree->search(Point3i(x - 1, y + 1, z)) ||
            x != m_dims.width - 1 && m_ocTree->search(Point3i(x + 1, y + 1, z))) &&
            (z != 0 && m_ocTree->search(Point3i(x, y + 1, z - 1)) ||
            z != m_dims.depth - 1 && m_ocTree->search(Point3i(x, y + 1, z + 1))) &&
            !m_ocTreeInnerVoxel->search(voxels[0])))
                addNeighbor(voxel, voxels[0], 0, plane, neighbors);
            if(y != 0 && (m_ocTree->search(voxels[1]) ||
            (x != 0 && m_ocTree->search(Point3i(x - 1, y - 1, z)) ||
            x != m_dims.width - 1 && m_ocTree->search(Point3i(x + 1, y - 1, z))) &&
            (z != 0 && m_ocTree->search(Point3i(x, y - 1, z - 1)) ||
            z != m_dims.depth - 1 && m_ocTree->search(Point3i(x, y - 1, z + 1))) &&
            !m_ocTreeInnerVoxel->search(voxels[1])))
                addNeighbor(voxel, voxels[1], 1, plane, neighbors);
            if(voxel.x != m_dims.width - 1 && voxel.z != 0 && m_ocTree->search(voxels[2]))
                addNeighbor(voxel, voxels[2], 2, plane, neighbors);
            if(voxel.x != 0 && voxel.z != m_dims.depth - 1 && m_ocTree->search(voxels[3]))
                addNeighbor(voxel, voxels[3], 3, plane, neighbors);
            if(voxel.x != m_dims.width - 1 && voxel.y != m_dims.height - 1 && voxel.z != 0 && (m_ocTree->search(voxels[4]) ||
            m_ocTree->search(Point3i(voxel.x, voxel.y + 1, voxel.z - 1)) && m_ocTree->search(Point3i(voxel.x + 1, voxel.y + 1, voxel.z)) &&
            !m_ocTreeInnerVoxel->search(voxels[4])))
                addNeighbor(voxel, voxels[4], 4, plane, neighbors);
            if(x != 0 && y != m_dims.height - 1 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[5]) ||
            m_ocTree->search(Point3i(x, y + 1, z + 1)) && m_ocTree->search(Point3i(x - 1, y + 1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[5])))
                addNeighbor(voxel, voxels[5], 5, plane, neighbors);
            if(x != m_dims.width - 1 && y != 0 && z != 0 && (m_ocTree->search(voxels[6]) ||
            m_ocTree->search(Point3i(x, y - 1, z - 1)) && m_ocTree->search(Point3i(x + 1, y - 1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[6])))
                addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != 0 && y != 0 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[7]) ||
			m_ocTree->search(Point3i(x, y - 1, z + 1)) && m_ocTree->search(Point3i(x - 1, y - 1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[7])))
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
			if(y != m_dims.height - 1 && (m_ocTree->search(voxels[0]) ||
			(x != 0 && m_ocTree->search(Point3i(x - 1, y + 1, z)) || x != m_dims.width - 1 && m_ocTree->search(Point3i(x + 1, y + 1, z))) &&
			(z != 0 && m_ocTree->search(Point3i(x, y + 1, z - 1)) || z != m_dims.depth - 1 && m_ocTree->search(Point3i(x, y + 1, z + 1))) &&
            !m_ocTreeInnerVoxel->search(voxels[0])))
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(y != 0 && (m_ocTree->search(voxels[1]) ||
			(x != 0 && m_ocTree->search(Point3i(x - 1, y - 1, z)) || x != m_dims.width - 1 && m_ocTree->search(Point3i(x + 1, y - 1, z))) &&
			(z != 0 && m_ocTree->search(Point3i(x, y - 1, z - 1)) || z != m_dims.depth - 1 && m_ocTree->search(Point3i(x, y - 1, z + 1))) &&
            !m_ocTreeInnerVoxel->search(voxels[1])))
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(x != m_dims.width - 1 && z != m_dims.depth - 1 && m_ocTree->search(voxels[2]))
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(x != 0 && z != 0 && m_ocTree->search(voxels[3]))
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != m_dims.width - 1 && y != m_dims.height - 1 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[4]) ||
			m_ocTree->search(Point3i(x, y + 1, z + 1)) && m_ocTree->search(Point3i(x + 1, y + 1, z)) && !m_ocTreeInnerVoxel->search(voxels[4])))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != m_dims.width - 1 && y != 0 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[5]) ||
			m_ocTree->search(Point3i(x, y - 1, z + 1)) && m_ocTree->search(Point3i(x + 1, y - 1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != 0 && y != m_dims.height - 1 && z != 0 && (m_ocTree->search(voxels[6]) ||
			m_ocTree->search(Point3i(x - 1, y + 1, z - 1)) && m_ocTree->search(Point3i(x, y + 1, z - 1)) &&
            !m_ocTreeInnerVoxel->search(voxels[6])))
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != 0 && y != 0 && z != 0 && (m_ocTree->search(voxels[7]) ||
			m_ocTree->search(Point3i(x, y - 1, z - 1)) && m_ocTree->search(Point3i(x - 1, y - 1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[7])))
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
			if(y != m_dims.height - 1 && m_ocTree->search(voxels[0])) addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(y != 0 && m_ocTree->search(voxels[1])) addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(z != m_dims.depth - 1 && m_ocTree->search(voxels[2])) addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(z != 0 && m_ocTree->search(voxels[3])) addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(y != m_dims.height - 1 && z != m_dims.depth - 1 && m_ocTree->search(voxels[4])) addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(y != m_dims.height - 1 && z != 0 && m_ocTree->search(voxels[5])) addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(y != 0 && z != m_dims.depth - 1 && m_ocTree->search(voxels[6])) addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(y != 0 && z != 0 && m_ocTree->search(voxels[7])) addNeighbor(voxel, voxels[7], 7, plane, neighbors);
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
			if(z != m_dims.depth - 1 && (m_ocTree->search(voxels[0]) ||
			(x != 0 && m_ocTree->search(Point3i(x-1, y, z+1))|| x != m_dims.width - 1 && m_ocTree->search(Point3i(x+1, y, z+1))) &&
			(y != 0 && m_ocTree->search(Point3i(x, y-1, z+1)) || y != m_dims.height - 1 && m_ocTree->search(Point3i(x, y+1, z+1))) &&
            !m_ocTreeInnerVoxel->search(voxels[0])))
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(z != 0 && (m_ocTree->search(voxels[1]) ||
			(x != 0 && m_ocTree->search(Point3i(x-1, y, z-1))|| x != m_dims.width - 1 && m_ocTree->search(Point3i(x+1, y, z-1))) &&
			(y != 0 && m_ocTree->search(Point3i(x, y-1, z-1)) || y != m_dims.height - 1 && m_ocTree->search(Point3i(x, y+1, z-1))) &&
            !m_ocTreeInnerVoxel->search(voxels[1])))
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(x != 0 && y != m_dims.height - 1 && (m_ocTree->search(voxels[2]))) 
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(x != m_dims.width - 1 && y != 0 && (m_ocTree->search(voxels[3])))
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != 0 && y != m_dims.height - 1 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[4]) ||
			m_ocTree->search(Point3i(x, y+1, z+1)) && m_ocTree->search(Point3i(x-1, y, z+1)) &&
            !m_ocTreeInnerVoxel->search(Point3i(x-1, y+1, z+1))))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != 0 && y != m_dims.height - 1 && z != 0 && (m_ocTree->search(voxels[5]) ||
			m_ocTree->search(Point3i(x, y+1, z-1)) && m_ocTree->search(Point3i(x-1, y, z-1)) &&
            !m_ocTreeInnerVoxel->search(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != m_dims.width - 1 && y != 0 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[6]) ||
			m_ocTree->search(Point3i(x+1, y-1, z+1)) && m_ocTree->search(Point3i(x, y-1, z+1)) &&
            !m_ocTreeInnerVoxel->search(voxels[6])))
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != m_dims.width - 1 && y != 0 && z != 0 && (m_ocTree->search(voxels[7]) ||
			m_ocTree->search(Point3i(x, y-1, z-1)) && m_ocTree->search(Point3i(x+1, y, z-1)) &&
            !m_ocTreeInnerVoxel->search(voxels[7])))
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
			if(z != m_dims.depth - 1 && (m_ocTree->search(voxels[0]) == 1 ||
			(x != 0 && m_ocTree->search(Point3i(x-1, y, z+1)) || x != m_dims.width - 1 && m_ocTree->search(Point3i(x+1, y, z+1))) &&
			(y != 0 && m_ocTree->search(Point3i(x, y-1, z+1)) || y != m_dims.height - 1 && m_ocTree->search(Point3i(x, y+1, z+1))) &&
            !m_ocTreeInnerVoxel->search(voxels[0]))) 
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(z != 0 && (m_ocTree->search(voxels[1]) ||
			(x != 0 && m_ocTree->search(Point3i(x-1, y, z-1)) || x != m_dims.width - 1 && m_ocTree->search(Point3i(x+1, y, z-1))) &&
			(y != 0 && m_ocTree->search(Point3i(x, y-1, z-1)) || y != m_dims.height - 1 && m_ocTree->search(Point3i(x, y+1, z-1))) &&
            !m_ocTreeInnerVoxel->search(voxels[1]))) 
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(x != m_dims.width - 1 && y != m_dims.height - 1 && m_ocTree->search(voxels[2]))
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(x != 0 && y != 0 && m_ocTree->search(voxels[3]))
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != m_dims.width - 1 && y != m_dims.height - 1 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[4]) ||
			m_ocTree->search(Point3i(x, y+1, z+1)) && m_ocTree->search(Point3i(x+1, y, z+1)) &&
            !m_ocTreeInnerVoxel->search(voxels[4])))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != m_dims.width - 1 && y != m_dims.height - 1 && z != 0 && (m_ocTree->search(voxels[5]) ||
			m_ocTree->search(Point3i(x+1, y, z-1)) && m_ocTree->search(Point3i(x, y+1, z-1)) &&
            !m_ocTreeInnerVoxel->search(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != 0 && y != 0 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[6]) ||
			m_ocTree->search(Point3i(x, y-1, z+1)) && m_ocTree->search(Point3i(x-1, y, z+1)) &&
            !m_ocTreeInnerVoxel->search(voxels[6]))) 
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != 0 && y != 0 && z != 0 && (m_ocTree->search(voxels[7]) ||
			m_ocTree->search(Point3i(x, y-1, z-1)) && m_ocTree->search(Point3i(x-1, y, z-1)) &&
            !m_ocTreeInnerVoxel->search(voxels[7]))) 
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
			if(z != m_dims.depth - 1 && m_ocTree->search(voxels[0])) addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(z != 0 && m_ocTree->search(voxels[1])) addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(x != m_dims.width - 1 && m_ocTree->search(voxels[2])) addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(x != 0 && m_ocTree->search(voxels[3])) addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != m_dims.width - 1 && z != m_dims.depth - 1 && m_ocTree->search(voxels[4])) addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != 0 && z != m_dims.depth - 1 && m_ocTree->search(voxels[5])) addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != m_dims.width - 1 && z != 0 && m_ocTree->search(voxels[6])) addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != 0 && z != 0 && m_ocTree->search(voxels[7])) addNeighbor(voxel, voxels[7], 7, plane, neighbors);
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
			if(x != m_dims.width - 1 && (m_ocTree->search(voxels[0]) ||
			(y != 0 && m_ocTree->search(Point3i(x+1, y-1, z)) || y != m_dims.height - 1 && m_ocTree->search(Point3i(x+1, y+1, z))) && 
			(z != 0 && m_ocTree->search(Point3i(x+1, y, z-1)) || z != m_dims.depth - 1 && m_ocTree->search(Point3i(x+1, y, z+1))) &&
            !m_ocTreeInnerVoxel->search(voxels[0])))
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(x != 0 && (m_ocTree->search(voxels[1]) ||
			(y != 0 && m_ocTree->search(Point3i(x-1, y-1, z)) || y != m_dims.height - 1 && m_ocTree->search(Point3i(x-1, y+1, z))) && 
			(z != 0 && m_ocTree->search(Point3i(x-1, y, z-1)) || z != m_dims.depth - 1 && m_ocTree->search(Point3i(x-1, y, z+1))) &&
            !m_ocTreeInnerVoxel->search(voxels[1])))
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(y != 0 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[2])))
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(y != m_dims.height - 1 && z != 0 && (m_ocTree->search(voxels[3])))
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != m_dims.width - 1 && y != 0 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[4]) ||
			m_ocTree->search(Point3i(x+1, y, z+1)) && m_ocTree->search(Point3i(x+1, y-1, z)) == 1 &&
            !m_ocTreeInnerVoxel->search(voxels[4])))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != 0 && y != 0 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[5]) ||
			m_ocTree->search(Point3i(x-1, y, z+1)) && m_ocTree->search(Point3i(x-1, y-1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != m_dims.width - 1 && y != m_dims.height - 1 && z != 0 && (m_ocTree->search(voxels[6]) ||
			m_ocTree->search(Point3i(x+1, y, z-1)) && m_ocTree->search(Point3i(x+1, y+1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[6]))) 
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != 0 && y != m_dims.height - 1 && z != 0 && (m_ocTree->search(voxels[7]) ||
			m_ocTree->search(Point3i(x-1, y+1, z-1)) && m_ocTree->search(Point3i(x-1, y, z-1)) &&
            !m_ocTreeInnerVoxel->search(voxels[7])))
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
			if(x != m_dims.width - 1 && (m_ocTree->search(voxels[0]) ||
			(y != 0 && m_ocTree->search(Point3i(x+1, y-1, z)) || y != m_dims.height - 1 && m_ocTree->search(Point3i(x+1, y+1, z))) &&
			(z != 0 && m_ocTree->search(Point3i(x+1, y, z-1)) || z != m_dims.depth - 1 && m_ocTree->search(Point3i(x+1, y, z+1))) &&
            !m_ocTreeInnerVoxel->search(voxels[0])))
				addNeighbor(voxel, voxels[0], 0, plane, neighbors);
			if(x != 0 && (m_ocTree->search(voxels[1]) ||
			(y != 0 && m_ocTree->search(Point3i(x-1, y-1, z)) || y != m_dims.height - 1 && m_ocTree->search(Point3i(x-1, y+1, z))) &&
			(z != 0 && m_ocTree->search(Point3i(x-1, y, z-1)) || z != m_dims.depth - 1 && m_ocTree->search(Point3i(x-1, y, z+1))) &&
            !m_ocTreeInnerVoxel->search(voxels[1])))
				addNeighbor(voxel, voxels[1], 1, plane, neighbors);
			if(y != m_dims.height - 1 && z != m_dims.depth - 1 && m_ocTree->search(voxels[2])) 
				addNeighbor(voxel, voxels[2], 2, plane, neighbors);
			if(y != 0 && z != 0 && m_ocTree->search(voxels[3])) 
				addNeighbor(voxel, voxels[3], 3, plane, neighbors);
			if(x != m_dims.width - 1 && y != m_dims.height - 1 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[4]) ||
			m_ocTree->search(Point3i(x+1, y, z+1)) && m_ocTree->search(Point3i(x+1, y+1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[4])))
				addNeighbor(voxel, voxels[4], 4, plane, neighbors);
			if(x != m_dims.width - 1 && y != 0 && z != 0 && (m_ocTree->search(voxels[5]) ||
			m_ocTree->search(Point3i(x+1, y, z-1)) && m_ocTree->search(Point3i(x+1, y-1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[5])))
				addNeighbor(voxel, voxels[5], 5, plane, neighbors);
			if(x != 0 && y != m_dims.height - 1 && z != m_dims.depth - 1 && (m_ocTree->search(voxels[6]) ||
			m_ocTree->search(Point3i(x-1, y+1, z+1)) && m_ocTree->search(Point3i(x-1, y, z+1)) &&
            !m_ocTreeInnerVoxel->search(voxels[6])))
				addNeighbor(voxel, voxels[6], 6, plane, neighbors);
			if(x != 0 && y != 0 && z != 0 && (m_ocTree->search(voxels[7]) ||
			m_ocTree->search(Point3i(x-1, y, z-1)) && m_ocTree->search(Point3i(x-1, y-1, z)) &&
            !m_ocTreeInnerVoxel->search(voxels[7])))
				addNeighbor(voxel, voxels[7], 7, plane, neighbors);
	}
}

void CurvatureEstimatorSerialEnv::addNeighbor(
    Point3i voxel, Point3i neighbor, int plane, int i,
    std::vector<std::pair<Point3i, uint8_t>>& neighbors) {
    // Add the neighbor voxel and its corresponding chain code to the neighbors vector
    neighbors[i].first = neighbor;
    switch(plane/3) {
        case 0: neighbors[i].second = getChainCode(Point2i(neighbor.x, neighbor.y), Point2i(voxel.x, voxel.y)); break;
		case 1: neighbors[i].second = getChainCode(Point2i(neighbor.y, neighbor.z), Point2i(voxel.y, voxel.z)); break;
		case 2: neighbors[i].second = getChainCode(Point2i(neighbor.z, neighbor.x), Point2i(voxel.z, voxel.x)); break;
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
    return 255; // Invalid chain code
}