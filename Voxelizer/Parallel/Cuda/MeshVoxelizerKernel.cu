#include "cuda_runtime.h"
#include "MeshVoxelizerKernel.h"
#include "TriangleVoxelizer.cu"
#include <cmath>

__global__ void voxelizeFace(const unsigned int* faces, int numFaces,
                             const int* intVertices,
                             unsigned char* voxels, int R, int C, int D, int xmin, int ymin, int zmin,
                             unsigned int *totalSize) {
    int faceId = threadIdx.x + blockIdx.x * blockDim.x;
    if (faceId < 0 || faceId >= numFaces) return;
    atomicAdd(totalSize, 1u);

    int v1 = faces[faceId * 3];
    int v2 = faces[faceId * 3 + 1];
    int v3 = faces[faceId * 3 + 2];

    int3 p1, p2, p3;
    p1.x = intVertices[v1 * 3];
    p1.y = intVertices[v1 * 3 + 1];
    p1.z = intVertices[v1 * 3 + 2];
    
    p2.x = intVertices[v2 * 3];
    p2.y = intVertices[v2 * 3 + 1];
    p2.z = intVertices[v2 * 3 + 2];
    
    p3.x = intVertices[v3 * 3];
    p3.y = intVertices[v3 * 3 + 1];
    p3.z = intVertices[v3 * 3 + 2];

    int3 dim = {R, C, D};
    int3 minBound = {xmin, ymin, zmin};

    voxelizeTriangle(p1, p2, p3, voxels, dim, minBound);
}