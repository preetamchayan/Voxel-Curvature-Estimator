#include "CurvatureEstimator.h"

// Include only the selected parallel environment header to avoid optional dependency
#if CURVATURE_ESTIMATOR_MODE == PARALLEL_OPENCL
#include "Parallel/OpenCL/CurvatureEstimatorOclEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_VULKAN
#include "Parallel/Vulkan/CurvatureEstimatorVulkanEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_CUDA
#include "Parallel/Cuda/CurvatureEstimatorCudaEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_DIRECTX
#include "Parallel/DirectX/CurvatureEstimatorDirectXEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_METAL
#include "Parallel/Metal/CurvatureEstimatorMetalEnv.h"
#else
#include "Serial/CurvatureEstimatorSerialEnv.h"
#endif

#include "../Helper/HelperFunctions.h"

#include <algorithm>
#include <climits>
#include <iostream>
#include <sstream>
#include <cassert>
#include <limits>
#include <filesystem>

CurvatureEstimator::CurvatureEstimator(
    std::vector<unsigned char>& voxels,
    const BBox3i& bounds,
    const Dimensions3i& dims)
    : m_voxels(voxels), m_bounds(bounds), m_dims(dims), m_baseEnv(nullptr) {
    const size_t voxelGridSize = static_cast<size_t>(dims.width) * dims.height * dims.depth;
    assert(m_voxels.size() == voxelGridSize);

#if CURVATURE_ESTIMATOR_MODE == SERIAL
    m_baseEnv = new CurvatureEstimatorSerialEnv(m_bounds);
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_OPENCL
    m_baseEnv = new CurvatureEstimatorOclEnv();
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_VULKAN
    m_baseEnv = new CurvatureEstimatorVulkanEnv();
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_CUDA
    m_baseEnv = new CurvatureEstimatorCudaEnv();
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_DIRECTX
    m_baseEnv = new CurvatureEstimatorDirectXEnv();
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_METAL
    m_baseEnv = new CurvatureEstimatorMetalEnv();
#else
    #error "Invalid CURVATURE_ESTIMATOR_MODE defined. Please define it as SERIAL, PARALLEL_OPENCL, PARALLEL_VULKAN, PARALLEL_CUDA, PARALLEL_DIRECTX or PARALLEL_METAL." 
#endif

    if (m_baseEnv) {
        m_baseEnv->preprocessVoxels(m_voxels, m_dims);
    }
}

CurvatureEstimator::~CurvatureEstimator() {
    if (m_baseEnv) {
        delete m_baseEnv;
        m_baseEnv = nullptr;
    }
}

void CurvatureEstimator::estimateCurvature(int curveLength) {
    m_curveLength = curveLength;
    m_curvatures.clear();

    if (m_baseEnv) {
        m_curvatures.assign(m_voxels.size(), std::numeric_limits<int>::max());
        m_baseEnv->estimateCurvature(curveLength, m_voxels, m_curvatures, m_dims);
        assert(m_voxels.size() == m_curvatures.size());
        averageCurvature();
    }
}

const std::vector<unsigned char>& CurvatureEstimator::getVoxels() const {
    return m_voxels;
}

BBox3i CurvatureEstimator::getSceneBounds() const {
    return m_bounds;
}

void CurvatureEstimator::averageCurvature() {
    const int R = m_dims.width;
    const int C = m_dims.height;
    const int D = m_dims.depth;
    const int theoreticalMaxCurvature = std::max(0, 4 * (m_curveLength - 1));

    auto inGrid = [&](int gx, int gy, int gz) {
        return gx >= 0 && gx < R &&
               gy >= 0 && gy < C &&
               gz >= 0 && gz < D;
    };

    auto index = [&](int gx, int gy, int gz) {
        return static_cast<size_t>(gx)
             + static_cast<size_t>(gy) * R
             + static_cast<size_t>(gz) * R * C;
    };

    std::vector<int> averagedCurvatures(m_curvatures.size(), std::numeric_limits<int>::max());
    int maxCurvature = std::numeric_limits<int>::min();

    for (int z = 0; z < D; ++z) {
        for (int y = 0; y < C; ++y) {
            for (int x = 0; x < R; ++x) {
                const size_t id = index(x, y, z);
                if (m_voxels[id] != 1) {
                    continue;
                }

                int curvatureValue = 0, count = 0;
                for (int dx = -2; dx <= 2; ++dx) {
                    for (int dy = -2; dy <= 2; ++dy) {
                        for (int dz = -2; dz <= 2; ++dz) {
                            const int nx = x + dx;
                            const int ny = y + dy;
                            const int nz = z + dz;

                            if (!inGrid(nx, ny, nz)) {
                                continue;
                            }

                            const size_t neighborId = index(nx, ny, nz);
                            if (m_voxels[neighborId] == 1 &&
                                m_curvatures[neighborId] != std::numeric_limits<int>::max()) {
                                curvatureValue += m_curvatures[neighborId];
                                count++;
                            }
                        }
                    }
                }

                if (count != 0) {
                    int averagedCurvature = curvatureValue / count;
                    averagedCurvatures[id] = std::min(theoreticalMaxCurvature, averagedCurvature);
                    maxCurvature = std::max(maxCurvature, averagedCurvatures[id]);
                }
            }
        }
    }

    m_curvatures.swap(averagedCurvatures);

    std::filesystem::create_directories("output");
    std::ofstream fpMTL("output/curvatureMaterial.mtl");
    writeMaterialFile(maxCurvature, fpMTL, m_colors);
}

void CurvatureEstimator::writeMaterialFile(
    int maxCurvature,
    std::ofstream& fpMTL,
    std::vector<Color>& colors
) {
    colors.clear();

    fpMTL << "newmtl 0\n";
    fpMTL << "Kd 0.5 0.5 0.5\n";
    fpMTL << "illum 1\n\n";
    colors.push_back(Color(0.5f, 0.5f, 0.5f));

    if (maxCurvature <= 0) {
        fpMTL << "newmtl 1\n";
        fpMTL << "Kd 0.0 0.0 1.0\n";
        fpMTL << "illum 1\n\n";
        colors.push_back(Color(0.0f, 0.0f, 1.0f));
        return;
    }

	for(int i = 1; i <= maxCurvature + 1; i++) {
		Color hsv;
		hsv.r = 240.0-((240.0)*((float)(i-1)))/(float)(maxCurvature);
		hsv.g = 1.0; hsv.b = 1.0;
		const auto& color = hsv2rgb(hsv);
		fpMTL << "newmtl " << i << "\nKd " << color.r << " " << color.g << " " << color.b << "\nillum 1\n\n";
		colors.push_back(color);
	}
}

void CurvatureEstimator::exportCurvatureOBJ(const std::string& filename) const {
    std::ofstream fp(filename);
    fp << "mtllib curvatureMaterial.mtl\n\n";

        for (int z = 0; z < m_dims.depth; ++z) {
        for (int y = 0; y < m_dims.height; ++y) {
            for (int x = 0; x < m_dims.width; ++x) {
                const size_t id = static_cast<size_t>(x) +
                                  static_cast<size_t>(y) * m_dims.width +
                                  static_cast<size_t>(z) * m_dims.width * m_dims.height;
                if (m_voxels[id] != 1) {
                    continue;
                }

                const auto& curvature = m_curvatures[id];
                if (curvature == std::numeric_limits<int>::max())
                    fp << "usemtl 0\n\n";
                else
                    fp << "usemtl " << curvature + 1 << "\n\n";

                writePointToVoxel(
                    Point3i(
                        x + m_bounds.xmin,
                        y + m_bounds.ymin,
                        z + m_bounds.zmin
                    ),
                    fp
                );
            }
        }
    }
}