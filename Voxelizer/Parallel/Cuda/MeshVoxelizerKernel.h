#pragma once
#include <cuda_runtime.h>

#define MAX_ARRAY_SIZE 1024

extern __global__ void voxelizeFace(const unsigned int* faces, int numFaces,
                                     const int* intVertices,
                                     unsigned char* voxels, int R, int C, int D, int xmin, int ymin, int zmin,
                                     unsigned int *totalSize);
