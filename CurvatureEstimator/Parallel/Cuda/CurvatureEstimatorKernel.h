#pragma once

#include <cuda_runtime.h>

constexpr int MAX_CURVE_LENGTH = 32;

extern __global__ void computeInnerSpaceVoxels(unsigned char* voxels, int R, int C, int D, int plane);
extern __global__ void markInteriorVoxels(unsigned char* voxels, int R, int C, int D);
extern __global__ void computeFrontierVoxels(unsigned char* voxels, int R, int C, int D);
extern __global__ void estimateCurvature(const unsigned char* voxels,
                                         int* curvatures,
                                         const int* surfaceVoxelIds,
                                         int surfaceVoxelCount,
                                         int curveLength,
                                         int R,
                                         int C,
                                         int D);