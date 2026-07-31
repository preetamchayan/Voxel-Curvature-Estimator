#include "CurvatureEstimator.h"

// Include only the selected parallel environment header to avoid optional dependency
#if CURVATURE_ESTIMATOR_MODE == PARALLEL_OPENCL
#include "OpenCL/CurvatureEstimatorOclEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_VULKAN
#include "Vulkan/CurvatureEstimatorVulkanEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_CUDA
#include "Cuda/CurvatureEstimatorCudaEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_DIRECTX
#include "DirectX/CurvatureEstimatorDirectxEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_METAL
#include "Metal/CurvatureEstimatorMetalEnv.h"
#else
#include "Serial/CurvatureEstimatorSerialEnv.h"
#endif

#include "../Helper/HelperFunctions.h"

#include <algorithm>
#include <climits>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cassert>

CurvatureEstimator::CurvatureEstimator(
    std::vector<unsigned char>& voxels,
    OcTree* ocTree,
    const BBox3i& bounds,
    const Dimensions3i& dims)
    : m_bounds(bounds), m_dims(dims), m_ocTree(ocTree) {
    m_voxels.clear();
    // inner space should be computed before populating the voxels.
    // TODO: Consider computing inner space and frontier voxels here.
    computeInnerSpaceAndFrontierVoxels(voxels);
    if (m_ocTree != nullptr) {
        m_ocTree->getVoxels(m_voxels);
    } else {
        assert(voxels.size() == static_cast<size_t>(dims.width * dims.height * dims.depth));
        for (size_t id = 0; id < voxels.size(); ++id) {
            if (voxels[id] == 1) {
                int z = id / (dims.width * dims.height);
                int y = (id % (dims.width * dims.height)) / dims.width;
                int x = id % dims.width;
                m_voxels.emplace_back(Point3i(x, y, z));
            }
        }
        assert(m_voxels.size() > 0);
    }
    m_baseEnv = nullptr;
    m_ocTreeInnerVoxel = new OcTree(m_bounds);
}

CurvatureEstimator::~CurvatureEstimator() {
    if (m_ocTree) {
        delete m_ocTree;
    }
    if (m_baseEnv) {
        delete m_baseEnv;
    }
}

void CurvatureEstimator::estimateCurvature(int curveLength) {
#if CURVATURE_ESTIMATOR_MODE == SERIAL
    if (m_ocTree == nullptr) {
        m_ocTree = new OcTree(m_voxels, m_bounds);
    }
    assert(m_ocTree != nullptr);
    m_baseEnv = new CurvatureEstimatorSerialEnv(m_ocTree, m_ocTreeInnerVoxel);
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_OPENCL
    m_baseEnv = new CurvatureEstimatorOclEnv();
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_VULKAN
    m_baseEnv = new CurvatureEstimatorVulkanEnv();
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_CUDA
    m_baseEnv = new CurvatureEstimatorCudaEnv();
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_DIRECTX
    m_baseEnv = new CurvatureEstimatorDirectxEnv();
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_METAL
    m_baseEnv = new CurvatureEstimatorMetalEnv();
#else
    #error "Invalid CURVATURE_ESTIMATOR_MODE defined. Please define it as SERIAL, PARALLEL_OPENCL, PARALLEL_VULKAN, PARALLEL_CUDA, PARALLEL_DIRECTX or PARALLEL_METAL." 
#endif
    if (m_baseEnv) {
        m_baseEnv->estimateCurvature(curveLength, m_voxels, m_curvatures, m_dims);
        assert(m_voxels.size() == m_curvatures.size());
        averageCurvature();
    }
}

const std::vector<Point3i>& CurvatureEstimator::getVoxels() const {
    return m_voxels;
}

BBox3i CurvatureEstimator::getSceneBounds() const {
    return m_bounds;
}

void CurvatureEstimator::computeInnerSpaceAndFrontierVoxels(
    std::vector<unsigned char>& voxels
) {
    std::vector<OcTree> ocTrees;
    for (int i = 0; i < 3; i++)
        ocTrees.emplace_back(m_bounds);

    computeInnerSpaceVoxels(voxels, ocTrees, m_dims.width, m_dims.height, m_dims.depth, 2);
    computeInnerSpaceVoxels(voxels, ocTrees, m_dims.depth, m_dims.width, m_dims.height, 1);
    computeInnerSpaceVoxels(voxels, ocTrees, m_dims.height, m_dims.depth, m_dims.width, 0);
    markInteriorVoxels(voxels, ocTrees);
    computeFrontierVoxels(voxels);
}

void CurvatureEstimator::computeInnerSpaceVoxels(
    std::vector<unsigned char>& voxels,
    std::vector<OcTree>& ocTrees,
    int R, int C, int D, int plane
) {
    auto get3DPoint = [&](int i, int j, int k) {
        switch(plane) {
            case 0: return Point3i(j, k, i);
            case 1: return Point3i(k, i, j);
            case 2: return Point3i(i, j, k);
        }
    };

    auto getVoxelID = [&](int i, int j, int k) {
        int id, W;
        switch(plane) {
            case 0: id = i*D + j*R*D; W = 1;   break;
            case 1: id = i*C*D + j;   W = C;   break;
            case 2: id = i + j*R;     W = R*C; break;
        }
        return id + k*W;
    };

    auto isVoxel = [&](int i, int j, int k) {
        if (m_ocTree != nullptr)
            return m_ocTree->search(get3DPoint(i, j, k));
        else {
            assert(voxels.size() != 0);
            return voxels[getVoxelID(i, j, k)] == 1;
        }
    };

    auto markInterior = [&](int i, int j, int k) {
        if (m_ocTree != nullptr)
            ocTrees[plane].insert(get3DPoint(i, j, k));
        else {
            assert(voxels.size() != 0);
            voxels[getVoxelID(i, j, k)] += 2;
        }
    };

    auto removeInterior = [&](int i, int j, int k) {
        if (m_ocTree != nullptr)
            ocTrees[plane].remove(get3DPoint(i, j, k));
        else {
            assert(voxels.size() != 0);
            voxels[getVoxelID(i, j, k)] -= 2;
        }
    };

    for (int i = 0; i < R; ++i) {
        for (int j = 0; j < C; ++j) {
            int count = 0, k = 0;
            for (; k < D; ++k) {
                if (isVoxel(i, j, k)) {
                    if(k == D-1 || isVoxel(i, j, k+1)) continue;
                    else if (!isVoxel(i, j, k+1)) count++;
                }
                else if (count%2 == 1) markInterior(i, j, k);
            }
            if (count%2 == 1) {
                k--;
                while (!isVoxel(i, j, k)) {
                    removeInterior(i, j, k);
                    k--;
                }
            }
        }
    }
}

void CurvatureEstimator::markInteriorVoxels(
    std::vector<unsigned char>& voxels,
    std::vector<OcTree>& ocTrees
) {
    for (int i = 0; i < m_dims.width; i++) {
        for (int j = 0; j < m_dims.height; j++) {
            for (int k = 0; k < m_dims.depth; k++) {
                if (m_ocTree != nullptr) {
                    Point3i p = Point3i(i, j, k);
                    if (ocTrees[0].search(p)
                        && ocTrees[1].search(p)
                        && ocTrees[2].search(p))
                        m_ocTreeInnerVoxel->insert(p);
                }
                else {
                    assert(voxels.size() != 0);
                    int id = i + j*m_dims.width + k*m_dims.width*m_dims.height;
                    if (voxels[id] == 2 || voxels[id] == 4) voxels[id] = 0;
                    else if (voxels[id] == 6) voxels[id] = 2;
                }
            }
        }
    }
}

void CurvatureEstimator::computeFrontierVoxels(
    std::vector<unsigned char>& voxels
) {
    if (m_ocTree != nullptr) {
        std::vector<Point3i> points;
        m_ocTree->getVoxels(points);

        for (const auto& point : points) {
            if (point.x == 0
                || !m_ocTree->search(Point3i(point.x - 1, point.y, point.z))) continue;
            else if (point.x == m_dims.width - 1
                     || !m_ocTree->search(Point3i(point.x + 1, point.y, point.z))) continue;
            else if (point.y == 0
                     || !m_ocTree->search(Point3i(point.x, point.y - 1, point.z))) continue;
            else if (point.y == m_dims.height - 1
                     || !m_ocTree->search(Point3i(point.x, point.y + 1, point.z))) continue;
            else if (point.z == 0
                     || !m_ocTree->search(Point3i(point.x, point.y, point.z - 1))) continue;
            else if (point.z == m_dims.depth - 1
                     || !m_ocTree->search(Point3i(point.x, point.y, point.z + 1))) continue;
            m_ocTree->remove(point);
            m_ocTreeInnerVoxel->insert(point);
        }
    }
    else {
        for (int i = 0; i < m_dims.width; i++) {
            for (int j = 0; j < m_dims.height; j++) {
                for (int k = 0; k < m_dims.depth; k++) {
                    int id = i + j * m_dims.width + k * m_dims.width * m_dims.height;
                    if (voxels[id] != 1) continue;
                    else if (i == 0 || voxels[id - 1] == 0) continue;
                    else if (i == m_dims.width - 1 || voxels[id + 1] == 0) continue;
                    else if (j == 0 || voxels[id - m_dims.width] == 0) continue;
                    else if (j == m_dims.height - 1 || voxels[id + m_dims.width] == 0) continue;
                    else if (k == 0 || voxels[id - m_dims.width * m_dims.height] == 0) continue;
                    else if (k == m_dims.depth - 1 || voxels[id + m_dims.width * m_dims.height] == 0) continue;
                    voxels[id] = 2;
                }
            }
        }
    }
}

void CurvatureEstimator::averageCurvature() {
    int curvatureGrid[m_dims.width][m_dims.height][m_dims.depth];
    for (size_t i = 0; i < m_voxels.size(); ++i) {
        const auto& voxel = m_voxels[i];
        int curvature = m_curvatures[i];
        assert(curvature > 0 && curvature < 256);
        curvatureGrid[voxel.x][voxel.y][voxel.z] = curvature;
    }

    int maxCurvature = std::numeric_limits<int>::min();

    for (size_t v = 0; v < m_voxels.size(); ++v) {
        const auto& voxel = m_voxels[v];
        int x = m_voxels[v].x, y = m_voxels[v].y, z = m_voxels[v].z;
        int R = m_dims.width, C = m_dims.height, D = m_dims.depth;
        int curvatureValue = 0, count = 0;
        for (int i = -2; i <= 2; ++i) {
            for (int j = -2; j <= 2; ++j) {
                for (int k = -2; k <= 2; ++k) {
                    if (i == 0 && j == 0 && k == 0
                        || x+i < 0 || x+i >= R || y+j < 0 || y+j >= C || z+k < 0 || z+k >= D) 
                        continue;
                    if (m_ocTree->search(Point3i(x+i, y+j, z+k)) && curvatureGrid[x+i][y+j][z+k] != std::numeric_limits<int>::max()) {
                        curvatureValue += curvatureGrid[x+i][y+j][z+k];
                        count++;
                    }
                }
            }
        }
        if (count == 0) m_curvatures[v] = std::numeric_limits<int>::max();
        else {
            m_curvatures[v] = curvatureValue / count;
            maxCurvature = std::max(m_curvatures[v], maxCurvature);
        }
    }

    std::ofstream fpMTL("curvatureMaterial.mtl");
    m_colors = writeMaterialFile(maxCurvature, fpMTL);
}

std::vector<Color>& CurvatureEstimator::writeMaterialFile(
    int maxCurvature,
    std::ofstream& fpMTL
) {
    std::vector<Color> colors;
    colors.push_back(Color(128, 128, 128));
	for(int i = 1; i <= maxCurvature + 1; i++){
		Color hsv;
		hsv.r = 240.0-((240.0)*((float)(i-1)))/(float)(maxCurvature);
		hsv.g = 1.0; hsv.b = 1.0;
		const auto& color = hsv2rgb(hsv);
		fpMTL << "newmtl " << i << "\nKd " << color.r << " " << color.g << " " << color.b << "\nillum 1\n\n";
		colors.push_back(color);
	}
    return colors;
}

void CurvatureEstimator::exportCurvatureOBJ(const std::string& filename) const {
    std::ofstream fp(filename);
    for (size_t i = 0; i < m_voxels.size(); ++i) {
        const auto& voxel = m_voxels[i];
        const auto& curvature = m_curvatures[i];
        fp << "usemtl " << curvature + 1 << "\n\n";
        writePointToVoxel(voxel, fp);
    }
} 