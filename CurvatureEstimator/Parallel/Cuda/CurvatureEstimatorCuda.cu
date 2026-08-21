#include "CurvatureEstimatorCuda.h"

#include "CurvatureEstimatorKernel.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

CurvatureEstimatorCuda::CurvatureEstimatorCuda() {
    std::cout << "Initializing Cuda curvature environment..." << std::endl;
    checkCudaError(cudaSetDevice(0), "Failed to set Cuda device");
}

CurvatureEstimatorCuda::~CurvatureEstimatorCuda() {
    std::cout << "Cleaning up Cuda curvature environment..." << std::endl;
}

void CurvatureEstimatorCuda::checkCudaError(cudaError_t error, const char* message) {
    if (error != cudaSuccess) {
        throw std::runtime_error(std::string("Cuda error at ") + message + ": " + cudaGetErrorString(error));
    }
}

void CurvatureEstimatorCuda::preprocessVoxels(std::vector<unsigned char>& voxels,
                                              const Dimensions3i& dims) {
    std::cout << "Preprocessing curvature voxels using Cuda..." << std::endl;

    unsigned char* voxelsBuffer = nullptr;
    checkCudaError(cudaMalloc(reinterpret_cast<void**>(&voxelsBuffer), sizeof(unsigned char) * voxels.size()),
                   "Failed to allocate curvature preprocess voxel buffer on device");
    checkCudaError(cudaMemcpy(voxelsBuffer, voxels.data(), sizeof(unsigned char) * voxels.size(), cudaMemcpyHostToDevice),
                   "Failed to copy curvature preprocess voxels to device");

    constexpr int dim = 8;
    dim3 block2D(dim, dim);
    dim3 grid2D((dims.width + dim - 1) / dim, (dims.height + dim - 1) / dim);
    computeInnerSpaceVoxels<<<grid2D, block2D>>>(voxelsBuffer, dims.width, dims.height, dims.depth, 2);
    checkCudaError(cudaGetLastError(), "Failed to launch Cuda computeInnerSpaceVoxels z-axis kernel");
    checkCudaError(cudaDeviceSynchronize(), "Failed to synchronize Cuda computeInnerSpaceVoxels z-axis kernel");

    grid2D = dim3((dims.depth + dim - 1) / dim, (dims.width + dim - 1) / dim);
    computeInnerSpaceVoxels<<<grid2D, block2D>>>(voxelsBuffer, dims.depth, dims.width, dims.height, 1);
    checkCudaError(cudaGetLastError(), "Failed to launch Cuda computeInnerSpaceVoxels y-axis kernel");
    checkCudaError(cudaDeviceSynchronize(), "Failed to synchronize Cuda computeInnerSpaceVoxels y-axis kernel");

    grid2D = dim3((dims.height + dim - 1) / dim, (dims.depth + dim - 1) / dim);
    computeInnerSpaceVoxels<<<grid2D, block2D>>>(voxelsBuffer, dims.height, dims.depth, dims.width, 0);
    checkCudaError(cudaGetLastError(), "Failed to launch Cuda computeInnerSpaceVoxels x-axis kernel");
    checkCudaError(cudaDeviceSynchronize(), "Failed to synchronize Cuda computeInnerSpaceVoxels x-axis kernel");

    dim3 block3D(dim, dim, dim);
    dim3 grid3D((dims.width + dim - 1) / dim,
                (dims.height + dim - 1) / dim,
                (dims.depth + dim - 1) / dim);
    markInteriorVoxels<<<grid3D, block3D>>>(voxelsBuffer, dims.width, dims.height, dims.depth);
    checkCudaError(cudaGetLastError(), "Failed to launch Cuda markInteriorVoxels kernel");
    checkCudaError(cudaDeviceSynchronize(), "Failed to synchronize Cuda markInteriorVoxels kernel");

    computeFrontierVoxels<<<grid3D, block3D>>>(voxelsBuffer, dims.width, dims.height, dims.depth);
    checkCudaError(cudaGetLastError(), "Failed to launch Cuda computeFrontierVoxels kernel");
    checkCudaError(cudaDeviceSynchronize(), "Failed to synchronize Cuda computeFrontierVoxels kernel");

    checkCudaError(cudaMemcpy(voxels.data(), voxelsBuffer, sizeof(unsigned char) * voxels.size(), cudaMemcpyDeviceToHost),
                   "Failed to copy curvature preprocess voxels back to host");
    cudaFree(voxelsBuffer);
}

void CurvatureEstimatorCuda::estimateCurvature(int curvLength,
                                               const std::vector<unsigned char>& voxels,
                                               std::vector<int>& curvatures,
                                               const Dimensions3i& dims) {
    std::cout << "Estimating curvature using Cuda..." << std::endl;

    if (curvLength < 1) {
        throw std::runtime_error("curve length must be positive");
    }
    if (curvLength > MAX_CURVE_LENGTH) {
        throw std::runtime_error("Cuda curvature curve length exceeds MAX_CURVE_LENGTH");
    }
    if (curvatures.size() != voxels.size()) {
        throw std::runtime_error("curvatures and voxels must have the same grid size");
    }

    const size_t voxelCount = voxels.size();
    if (voxelCount > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("Cuda curvature supports at most INT_MAX voxels");
    }

    std::vector<int> surfaceVoxelIds;
    surfaceVoxelIds.reserve(voxelCount);
    for (size_t id = 0; id < voxelCount; ++id) {
        if (voxels[id] == 1) {
            surfaceVoxelIds.push_back(static_cast<int>(id));
        }
    }
    std::cout << "Cuda curvature dispatch: " << surfaceVoxelIds.size()
              << " surface voxels out of " << voxelCount << " grid voxels" << std::endl;

    unsigned char* voxelsBuffer = nullptr;
    int* curvaturesBuffer = nullptr;
    int* surfaceVoxelIdsBuffer = nullptr;

    std::fill(curvatures.begin(), curvatures.end(), std::numeric_limits<int>::max());

    checkCudaError(cudaMalloc(reinterpret_cast<void**>(&voxelsBuffer), sizeof(unsigned char) * voxels.size()),
                   "Failed to allocate curvature voxel buffer on device");
    checkCudaError(cudaMalloc(reinterpret_cast<void**>(&curvaturesBuffer), sizeof(int) * curvatures.size()),
                   "Failed to allocate curvature output buffer on device");
    checkCudaError(cudaMemcpy(voxelsBuffer, voxels.data(), sizeof(unsigned char) * voxels.size(), cudaMemcpyHostToDevice),
                   "Failed to copy curvature voxels to device");
    checkCudaError(cudaMemcpy(curvaturesBuffer, curvatures.data(), sizeof(int) * curvatures.size(), cudaMemcpyHostToDevice),
                   "Failed to copy curvature output to device");

    if (!surfaceVoxelIds.empty()) {
        checkCudaError(cudaMalloc(reinterpret_cast<void**>(&surfaceVoxelIdsBuffer), sizeof(int) * surfaceVoxelIds.size()),
                       "Failed to allocate curvature surface voxel ID buffer on device");
        checkCudaError(cudaMemcpy(surfaceVoxelIdsBuffer, surfaceVoxelIds.data(), sizeof(int) * surfaceVoxelIds.size(), cudaMemcpyHostToDevice),
                       "Failed to copy curvature surface voxel IDs to device");

        constexpr int dim = 256;
        dim3 block(dim);
        dim3 grid((static_cast<unsigned int>(surfaceVoxelIds.size()) + dim - 1) / dim);
        const int surfaceVoxelCount = static_cast<int>(surfaceVoxelIds.size());

        ::estimateCurvature<<<grid, block>>>(voxelsBuffer, curvaturesBuffer, surfaceVoxelIdsBuffer,
                                             surfaceVoxelCount, curvLength,
                                             dims.width, dims.height, dims.depth);
        checkCudaError(cudaGetLastError(), "Failed to launch Cuda estimateCurvature kernel");
        checkCudaError(cudaDeviceSynchronize(), "Failed to synchronize Cuda estimateCurvature kernel");
    }

    checkCudaError(cudaMemcpy(curvatures.data(), curvaturesBuffer, sizeof(int) * curvatures.size(), cudaMemcpyDeviceToHost),
                   "Failed to copy curvatures back to host");

    if (surfaceVoxelIdsBuffer) cudaFree(surfaceVoxelIdsBuffer);
    cudaFree(curvaturesBuffer);
    cudaFree(voxelsBuffer);
}
