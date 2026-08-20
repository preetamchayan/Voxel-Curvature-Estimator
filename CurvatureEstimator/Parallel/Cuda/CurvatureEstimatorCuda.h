#pragma once

#include "../../CurvatureEstimatorBase.h"
#include <cuda_runtime.h>

class CurvatureEstimatorCuda : public CurvatureEstimatorBase {
public:
    CurvatureEstimatorCuda();
    ~CurvatureEstimatorCuda();

    void preprocessVoxels(std::vector<unsigned char>& voxels,
                          const Dimensions3i& dims) override;

    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;

private:
    void checkCudaError(cudaError_t error, const char* message);
};