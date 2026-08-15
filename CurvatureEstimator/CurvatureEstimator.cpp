#include "CurvatureEstimator.h"

// Include only the selected backend header to avoid optional dependencies.
#if CURVATURE_ESTIMATOR == OPENCL
#include "Parallel/OpenCL/CurvatureEstimatorOpenCL.h"
#elif CURVATURE_ESTIMATOR == VULKAN
#include "Parallel/Vulkan/CurvatureEstimatorVulkan.h"
#elif CURVATURE_ESTIMATOR == CUDA
#include "Parallel/Cuda/CurvatureEstimatorCuda.h"
#elif CURVATURE_ESTIMATOR == DIRECTX
#include "Parallel/DirectX/CurvatureEstimatorDirectX.h"
#elif CURVATURE_ESTIMATOR == METAL
#include "Parallel/Metal/CurvatureEstimatorMetal.h"
#else
#include "Serial/CurvatureEstimatorSerial.h"
#endif

#include "../Helper/HelperFunctions.h"

#include <algorithm>
#include <climits>
#include <iostream>
#include <sstream>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <limits>
#include <filesystem>

CurvatureEstimator::CurvatureEstimator(
    std::vector<unsigned char>& voxels,
    const BBox3i& bounds,
    const Dimensions3i& dims)
    : m_voxels(voxels), m_bounds(bounds), m_dims(dims), m_curveLength(0),
      m_maxCurvature(std::numeric_limits<int>::min()), m_base(nullptr) {
    const size_t voxelGridSize = static_cast<size_t>(dims.width) * dims.height * dims.depth;
    assert(m_voxels.size() == voxelGridSize);

#if CURVATURE_ESTIMATOR == SERIAL
    m_base = new CurvatureEstimatorSerial(m_bounds);
#elif CURVATURE_ESTIMATOR == OPENCL
    m_base = new CurvatureEstimatorOpenCL();
#elif CURVATURE_ESTIMATOR == VULKAN
    m_base = new CurvatureEstimatorVulkan();
#elif CURVATURE_ESTIMATOR == CUDA
    m_base = new CurvatureEstimatorCuda();
#elif CURVATURE_ESTIMATOR == DIRECTX
    m_base = new CurvatureEstimatorDirectX();
#elif CURVATURE_ESTIMATOR == METAL
    m_base = new CurvatureEstimatorMetal();
#else
    #error "Invalid CURVATURE_ESTIMATOR defined. Please define it as SERIAL, OPENCL, VULKAN, CUDA, DIRECTX, or METAL."
#endif

    if (m_base) {
        m_base->preprocessVoxels(m_voxels, m_dims);
    }
}

CurvatureEstimator::~CurvatureEstimator() {
    if (m_base) {
        delete m_base;
        m_base = nullptr;
    }
}

void CurvatureEstimator::estimateCurvature(int curveLength) {
    m_curveLength = curveLength;
    m_curvatures.clear();

    if (m_base) {
        m_curvatures.assign(m_voxels.size(), std::numeric_limits<int>::max());
        const auto backendStart = std::chrono::steady_clock::now();
        m_base->estimateCurvature(curveLength, m_voxels, m_curvatures, m_dims);
        const auto backendEnd = std::chrono::steady_clock::now();
        std::cout << "CURVATURE_PROFILE_BACKEND_MS="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         backendEnd - backendStart).count() << std::endl;
        assert(m_voxels.size() == m_curvatures.size());

        constexpr std::uint64_t fnvOffsetBasis = 14695981039346656037ull;
        constexpr std::uint64_t fnvPrime = 1099511628211ull;
        const int invalidCurvature = std::numeric_limits<int>::max();
        size_t surfaceVoxelCount = 0;
        size_t rawValidCurvatureCount = 0;
        std::uint64_t rawCurvatureChecksum = fnvOffsetBasis;

        for (size_t id = 0; id < m_voxels.size(); ++id) {
            if (m_voxels[id] != 1) {
                continue;
            }

            ++surfaceVoxelCount;
            const int curvature = m_curvatures[id];
            if (curvature != invalidCurvature) {
                ++rawValidCurvatureCount;
            }

            const std::uint64_t value = static_cast<std::uint32_t>(curvature);
            rawCurvatureChecksum ^= static_cast<std::uint64_t>(id);
            rawCurvatureChecksum *= fnvPrime;
            rawCurvatureChecksum ^= value;
            rawCurvatureChecksum *= fnvPrime;
        }

        std::cout << "CURVATURE_RAW_SURFACE_VOXELS=" << surfaceVoxelCount << '\n'
                  << "CURVATURE_RAW_VALID_VOXELS=" << rawValidCurvatureCount << '\n'
                  << "CURVATURE_RAW_INVALID_VOXELS=" << (surfaceVoxelCount - rawValidCurvatureCount) << '\n'
                  << "CURVATURE_RAW_CHECKSUM=" << rawCurvatureChecksum << std::endl;

        const auto averagingStart = std::chrono::steady_clock::now();
        averageCurvature();
        const auto averagingEnd = std::chrono::steady_clock::now();
        std::cout << "CURVATURE_PROFILE_AVERAGING_MS="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         averagingEnd - averagingStart).count() << std::endl;
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
    m_maxCurvature = maxCurvature;
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

void CurvatureEstimator::exportCurvatureLog(const std::string& filename) const {
    std::ofstream logFile(filename);
    if (!logFile) {
        throw std::runtime_error("Unable to open curvature log file: " + filename);
    }

    for (int z = 0; z < m_dims.depth; ++z) {
        for (int y = 0; y < m_dims.height; ++y) {
            for (int x = 0; x < m_dims.width; ++x) {
                const size_t id = static_cast<size_t>(x) +
                                  static_cast<size_t>(y) * m_dims.width +
                                  static_cast<size_t>(z) * m_dims.width * m_dims.height;
                if (m_voxels[id] != 1) {
                    continue;
                }

                logFile << (x + m_bounds.xmin) << ' '
                        << (y + m_bounds.ymin) << ' '
                        << (z + m_bounds.zmin) << ' '
                        << m_curvatures[id] << '\n';
            }
        }
    }

    std::cout << "Exported curvature log to " << filename << std::endl;
}

void CurvatureEstimator::exportCurvatureOBJ(const std::string& filename,
                                            const std::string& materialFilename) {
    std::ofstream materialFile(materialFilename);
    if (!materialFile) {
        throw std::runtime_error("Unable to open curvature material file: " + materialFilename);
    }
    writeMaterialFile(m_maxCurvature, materialFile, m_colors);

    std::ofstream fp(filename);
    if (!fp) {
        throw std::runtime_error("Unable to open curvature OBJ file: " + filename);
    }
    fp << "mtllib " << std::filesystem::path(materialFilename).filename().string() << "\n\n";

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
    std::cout << "Exported voxel curvature map to " << filename << std::endl;
}