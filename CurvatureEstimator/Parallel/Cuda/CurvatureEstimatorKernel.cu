#include "cuda_runtime.h"
#include "CurvatureEstimatorKernel.h"

#include <climits>

namespace {
constexpr int INT_MAX_VALUE = 2147483647;
constexpr int INT_MIN_VALUE = (-2147483647 - 1);
constexpr unsigned char INVALID_CHAIN_CODE = static_cast<unsigned char>(255);
constexpr unsigned int INVALID_VOXEL_ID = UINT_MAX;
}

struct Candidate {
    unsigned int voxelID;
    unsigned char chainCode;
};

__device__ inline int getVoxelID(int3 point, int R, int C) {
    return point.x + point.y * R + point.z * R * C;
}

__device__ inline int3 getCoordinates(unsigned int voxelID, int R, int C) {
    int t = voxelID % (R * C);
    return make_int3(t % R, t / R, voxelID / (R * C));
}

__device__ inline int inGrid(int x, int y, int z, int R, int C, int D) {
    return x >= 0 && x < R && y >= 0 && y < C && z >= 0 && z < D;
}

__device__ inline int pointEqual(int3 a, int3 b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

__device__ inline Candidate invalidCandidate(void) {
    Candidate c;
    c.voxelID = INVALID_VOXEL_ID;
    c.chainCode = INVALID_CHAIN_CODE;
    return c;
}

__device__ inline int isInvalidCandidate(Candidate c) {
    return c.chainCode == INVALID_CHAIN_CODE || c.voxelID == INVALID_VOXEL_ID;
}

__device__ inline int isSurface(const unsigned char* voxels, int3 point, int R, int C, int D) {
    return inGrid(point.x, point.y, point.z, R, C, D) &&
           voxels[getVoxelID(point, R, C)] == 1;
}

__device__ inline int isInterior(const unsigned char* voxels, int3 point, int R, int C, int D) {
    return inGrid(point.x, point.y, point.z, R, C, D) &&
           voxels[getVoxelID(point, R, C)] == 2;
}

__device__ inline unsigned char getChainCode(int2 neighbor, int2 voxel) {
    const int2 dir = make_int2(neighbor.x - voxel.x, neighbor.y - voxel.y);
    if (dir.x == 1 && dir.y == 0) return 0;
    if (dir.x == 1 && dir.y == 1) return 1;
    if (dir.x == 0 && dir.y == 1) return 2;
    if (dir.x == -1 && dir.y == 1) return 3;
    if (dir.x == -1 && dir.y == 0) return 4;
    if (dir.x == -1 && dir.y == -1) return 5;
    if (dir.x == 0 && dir.y == -1) return 6;
    if (dir.x == 1 && dir.y == -1) return 7;
    return INVALID_CHAIN_CODE;
}

__device__ inline void addNeighbor(int3 voxel, int3 neighbor, int neighborIndex, int plane, Candidate neighbors[8], int R, int C) {
    neighbors[neighborIndex].voxelID = getVoxelID(neighbor, R, C);
    switch (plane / 3) {
        case 0:
            neighbors[neighborIndex].chainCode = getChainCode(make_int2(neighbor.x, neighbor.y), make_int2(voxel.x, voxel.y));
            break;
        case 1:
            neighbors[neighborIndex].chainCode = getChainCode(make_int2(neighbor.y, neighbor.z), make_int2(voxel.y, voxel.z));
            break;
        case 2:
            neighbors[neighborIndex].chainCode = getChainCode(make_int2(neighbor.z, neighbor.x), make_int2(voxel.z, voxel.x));
            break;
    }
}

__device__ void getNeighborsInPlane(int3 voxel, int plane, const unsigned char* voxels, int R, int C, int D, Candidate neighbors[8]) {
    for (int n = 0; n < 8; ++n) neighbors[n] = invalidCandidate();

    const int x = voxel.x;
    const int y = voxel.y;
    const int z = voxel.z;
    const int xmin = 0, xmax = R - 1;
    const int ymin = 0, ymax = C - 1;
    const int zmin = 0, zmax = D - 1;

    switch (plane) {
        case 0:
            if (x != xmax && isSurface(voxels, make_int3(x + 1, y, z), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y, z), 0, plane, neighbors, R, C);
            if (x != xmin && isSurface(voxels, make_int3(x - 1, y, z), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y, z), 1, plane, neighbors, R, C);
            if (y != ymax && isSurface(voxels, make_int3(x, y + 1, z), R, C, D)) addNeighbor(voxel, make_int3(x, y + 1, z), 2, plane, neighbors, R, C);
            if (y != ymin && isSurface(voxels, make_int3(x, y - 1, z), R, C, D)) addNeighbor(voxel, make_int3(x, y - 1, z), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y + 1, z), 4, plane, neighbors, R, C);
            if (x != xmax && y != ymin && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y - 1, z), 5, plane, neighbors, R, C);
            if (x != xmin && y != ymax && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y + 1, z), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y - 1, z), 7, plane, neighbors, R, C);
            break;
        case 1:
            if (y != ymax && (isSurface(voxels, make_int3(x, y + 1, z), R, C, D) || ((x != xmin && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D)) || (x != xmax && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x, y + 1, z), R, C, D))) addNeighbor(voxel, make_int3(x, y + 1, z), 0, plane, neighbors, R, C);
            if (y != ymin && (isSurface(voxels, make_int3(x, y - 1, z), R, C, D) || ((x != xmin && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D)) || (x != xmax && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x, y - 1, z), R, C, D))) addNeighbor(voxel, make_int3(x, y - 1, z), 1, plane, neighbors, R, C);
            if (x != xmax && z != zmin && isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y, z - 1), 2, plane, neighbors, R, C);
            if (x != xmin && z != zmax && isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y, z + 1), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, make_int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, make_int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y + 1, z - 1), 4, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, make_int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, make_int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y + 1, z + 1), 5, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, make_int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, make_int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y - 1, z - 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, make_int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, make_int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y - 1, z + 1), 7, plane, neighbors, R, C);
            break;
        case 2:
            if (y != ymax && (isSurface(voxels, make_int3(x, y + 1, z), R, C, D) || ((x != xmin && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D)) || (x != xmax && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x, y + 1, z), R, C, D))) addNeighbor(voxel, make_int3(x, y + 1, z), 0, plane, neighbors, R, C);
            if (y != ymin && (isSurface(voxels, make_int3(x, y - 1, z), R, C, D) || ((x != xmin && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D)) || (x != xmax && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x, y - 1, z), R, C, D))) addNeighbor(voxel, make_int3(x, y - 1, z), 1, plane, neighbors, R, C);
            if (x != xmax && z != zmax && isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y, z + 1), 2, plane, neighbors, R, C);
            if (x != xmin && z != zmin && isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y, z - 1), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, make_int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, make_int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, make_int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, make_int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y - 1, z + 1), 5, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, make_int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D) && isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D) && !isInterior(voxels, make_int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y + 1, z - 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, make_int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, make_int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 3:
            if (y != ymax && isSurface(voxels, make_int3(x, y + 1, z), R, C, D)) addNeighbor(voxel, make_int3(x, y + 1, z), 0, plane, neighbors, R, C);
            if (y != ymin && isSurface(voxels, make_int3(x, y - 1, z), R, C, D)) addNeighbor(voxel, make_int3(x, y - 1, z), 1, plane, neighbors, R, C);
            if (z != zmax && isSurface(voxels, make_int3(x, y, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x, y, z + 1), 2, plane, neighbors, R, C);
            if (z != zmin && isSurface(voxels, make_int3(x, y, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x, y, z - 1), 3, plane, neighbors, R, C);
            if (y != ymax && z != zmax && isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (y != ymax && z != zmin && isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x, y + 1, z - 1), 5, plane, neighbors, R, C);
            if (y != ymin && z != zmax && isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x, y - 1, z + 1), 6, plane, neighbors, R, C);
            if (y != ymin && z != zmin && isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 4:
            if (z != zmax && (isSurface(voxels, make_int3(x, y, z + 1), R, C, D) || ((x != xmin && isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D)) || (x != xmax && isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D))) && ((y != ymin && isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D)) || (y != ymax && isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x, y, z + 1), R, C, D))) addNeighbor(voxel, make_int3(x, y, z + 1), 0, plane, neighbors, R, C);
            if (z != zmin && (isSurface(voxels, make_int3(x, y, z - 1), R, C, D) || ((x != xmin && isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D)) || (x != xmax && isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D))) && ((y != ymin && isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D)) || (y != ymax && isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D))) && !isInterior(voxels, make_int3(x, y, z - 1), R, C, D))) addNeighbor(voxel, make_int3(x, y, z - 1), 1, plane, neighbors, R, C);
            if (x != xmin && y != ymax && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y + 1, z), 2, plane, neighbors, R, C);
            if (x != xmax && y != ymin && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y - 1, z), 3, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, make_int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D) && !isInterior(voxels, make_int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, make_int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D) && !isInterior(voxels, make_int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y + 1, z - 1), 5, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, make_int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D) && !isInterior(voxels, make_int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y - 1, z + 1), 6, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, make_int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D) && !isInterior(voxels, make_int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 5:
            if (z != zmax && (isSurface(voxels, make_int3(x, y, z + 1), R, C, D) || ((x != xmin && isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D)) || (x != xmax && isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D))) && ((y != ymin && isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D)) || (y != ymax && isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x, y, z + 1), R, C, D))) addNeighbor(voxel, make_int3(x, y, z + 1), 0, plane, neighbors, R, C);
            if (z != zmin && (isSurface(voxels, make_int3(x, y, z - 1), R, C, D) || ((x != xmin && isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D)) || (x != xmax && isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D))) && ((y != ymin && isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D)) || (y != ymax && isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D))) && !isInterior(voxels, make_int3(x, y, z - 1), R, C, D))) addNeighbor(voxel, make_int3(x, y, z - 1), 1, plane, neighbors, R, C);
            if (x != xmax && y != ymax && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y + 1, z), 2, plane, neighbors, R, C);
            if (x != xmin && y != ymin && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y - 1, z), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, make_int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D) && !isInterior(voxels, make_int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, make_int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D) && isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D) && !isInterior(voxels, make_int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y + 1, z - 1), 5, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, make_int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D) && !isInterior(voxels, make_int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y - 1, z + 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, make_int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D) && !isInterior(voxels, make_int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 6:
            if (z != zmax && isSurface(voxels, make_int3(x, y, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x, y, z + 1), 0, plane, neighbors, R, C);
            if (z != zmin && isSurface(voxels, make_int3(x, y, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x, y, z - 1), 1, plane, neighbors, R, C);
            if (x != xmax && isSurface(voxels, make_int3(x + 1, y, z), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y, z), 2, plane, neighbors, R, C);
            if (x != xmin && isSurface(voxels, make_int3(x - 1, y, z), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y, z), 3, plane, neighbors, R, C);
            if (x != xmax && z != zmax && isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y, z + 1), 4, plane, neighbors, R, C);
            if (x != xmin && z != zmax && isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y, z + 1), 5, plane, neighbors, R, C);
            if (x != xmax && z != zmin && isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x + 1, y, z - 1), 6, plane, neighbors, R, C);
            if (x != xmin && z != zmin && isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x - 1, y, z - 1), 7, plane, neighbors, R, C);
            break;
        case 7:
            if (x != xmax && (isSurface(voxels, make_int3(x + 1, y, z), R, C, D) || ((y != ymin && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x + 1, y, z), R, C, D))) addNeighbor(voxel, make_int3(x + 1, y, z), 0, plane, neighbors, R, C);
            if (x != xmin && (isSurface(voxels, make_int3(x - 1, y, z), R, C, D) || ((y != ymin && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x - 1, y, z), R, C, D))) addNeighbor(voxel, make_int3(x - 1, y, z), 1, plane, neighbors, R, C);
            if (y != ymin && z != zmax && isSurface(voxels, make_int3(x, y - 1, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x, y - 1, z + 1), 2, plane, neighbors, R, C);
            if (y != ymax && z != zmin && isSurface(voxels, make_int3(x, y + 1, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x, y + 1, z - 1), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, make_int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, make_int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y - 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, make_int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, make_int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y - 1, z + 1), 5, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, make_int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, make_int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y + 1, z - 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, make_int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, make_int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y + 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 8:
            if (x != xmax && (isSurface(voxels, make_int3(x + 1, y, z), R, C, D) || ((y != ymin && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x + 1, y, z), R, C, D))) addNeighbor(voxel, make_int3(x + 1, y, z), 0, plane, neighbors, R, C);
            if (x != xmin && (isSurface(voxels, make_int3(x - 1, y, z), R, C, D) || ((y != ymin && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D))) && !isInterior(voxels, make_int3(x - 1, y, z), R, C, D))) addNeighbor(voxel, make_int3(x - 1, y, z), 1, plane, neighbors, R, C);
            if (y != ymax && z != zmax && isSurface(voxels, make_int3(x, y + 1, z + 1), R, C, D)) addNeighbor(voxel, make_int3(x, y + 1, z + 1), 2, plane, neighbors, R, C);
            if (y != ymin && z != zmin && isSurface(voxels, make_int3(x, y - 1, z - 1), R, C, D)) addNeighbor(voxel, make_int3(x, y - 1, z - 1), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, make_int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x + 1, y, z + 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, make_int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, make_int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x + 1, y, z - 1), R, C, D) && isSurface(voxels, make_int3(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, make_int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x + 1, y - 1, z - 1), 5, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, make_int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, make_int3(x - 1, y, z + 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, make_int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y + 1, z + 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, make_int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, make_int3(x - 1, y, z - 1), R, C, D) && isSurface(voxels, make_int3(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, make_int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, make_int3(x - 1, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
    }
}

__device__ int findNextLevelVoxels(int step, int plane, Candidate curve[8], const unsigned int curve3D[2 * MAX_CURVE_LENGTH + 1], int curveLength, const unsigned char* voxels, int R, int C, int D) {
    int3 voxel = getCoordinates(curve3D[curveLength + step], R, C);
    int absoluteStep = abs(step);
    getNeighborsInPlane(voxel, plane, voxels, R, C, D, curve);

    int count = 0;
    for (int j = 0; j < 8; ++j) {
        for (int k = curveLength - absoluteStep; k <= curveLength + absoluteStep; ++k) {
            if (curve3D[k] == curve[j].voxelID) {
                curve[j] = invalidCandidate();
                break;
            }
        }
        if (!isInvalidCandidate(curve[j])) ++count;
        if (count > 3) break;
    }
    return !(count == 0 || count > 3);
}

__device__ int computeCurvatureOfDigitalCurve2D(int plane, int3 voxel, Candidate trailCurve[8], Candidate leadCurve[8], unsigned int curve3D[2 * MAX_CURVE_LENGTH + 1], int curveLength, const unsigned char* voxels, int R, int C, int D, int* curvature) {
    unsigned char prevTrailChainCode = INVALID_CHAIN_CODE;
    unsigned char prevLeadChainCode = INVALID_CHAIN_CODE;
    int curvatureSum = 0;
    int i = 1;

    for (; i <= curveLength; ++i) {
        unsigned char trailChainCode = INVALID_CHAIN_CODE;
        unsigned char leadChainCode = INVALID_CHAIN_CODE;
        int minDiff = INVALID_CHAIN_CODE;

        for (int j = 0; j < 8; ++j) {
            if (isInvalidCandidate(trailCurve[j])) continue;
            int3 voxel1 = getCoordinates(trailCurve[j].voxelID, R, C);
            unsigned char trailChain = trailCurve[j].chainCode;

            for (int k = 0; k < 8; ++k) {
                if (isInvalidCandidate(leadCurve[k])) continue;
                int3 voxel2 = getCoordinates(leadCurve[k].voxelID, R, C);
                unsigned char leadChain = leadCurve[k].chainCode < 4 ? leadCurve[k].chainCode + 4 : leadCurve[k].chainCode - 4;

                int distX12 = max(max(abs(voxel1.x - voxel2.x), abs(voxel1.y - voxel2.y)), abs(voxel1.z - voxel2.z));
                int distX10 = max(max(abs(voxel1.x - voxel.x), abs(voxel1.y - voxel.y)), abs(voxel1.z - voxel.z));
                int distX20 = max(max(abs(voxel.x - voxel2.x), abs(voxel.y - voxel2.y)), abs(voxel.z - voxel2.z));
                if ((distX10 == 1 || distX20 == 1) && i != 1) continue;
                if ((distX10 != 1 || distX20 != 1) && distX12 <= 1) continue;

                int diff = abs(static_cast<int>(leadChain) - static_cast<int>(trailChain));
                diff = min(diff, 8 - diff);
                if (diff < minDiff) {
                    trailChainCode = trailChain;
                    leadChainCode = leadChain;
                    minDiff = diff;
                    curve3D[curveLength - i] = getVoxelID(voxel1, R, C);
                    curve3D[curveLength + i] = getVoxelID(voxel2, R, C);
                }
            }
        }

        if (trailChainCode == INVALID_CHAIN_CODE || leadChainCode == INVALID_CHAIN_CODE) break;

        if (prevTrailChainCode != INVALID_CHAIN_CODE && prevLeadChainCode != INVALID_CHAIN_CODE) {
            int tmp = abs(static_cast<int>(trailChainCode) - static_cast<int>(leadChainCode));
            tmp = min(tmp, 8 - tmp);
            int tmp1 = abs(static_cast<int>(trailChainCode) - static_cast<int>(prevLeadChainCode));
            tmp1 = min(tmp1, 8 - tmp1);
            tmp = min(tmp, tmp1);
            tmp1 = abs(static_cast<int>(prevTrailChainCode) - static_cast<int>(leadChainCode));
            tmp1 = min(tmp1, 8 - tmp1);
            tmp = min(tmp, tmp1);
            tmp1 = abs(static_cast<int>(prevTrailChainCode) - static_cast<int>(prevLeadChainCode));
            tmp1 = min(tmp1, 8 - tmp1);
            tmp = min(tmp, tmp1);
            curvatureSum += tmp;
        }

        prevLeadChainCode = leadChainCode;
        prevTrailChainCode = trailChainCode;

        if (i == curveLength) continue;
        if (!findNextLevelVoxels(-i, plane, trailCurve, curve3D, curveLength, voxels, R, C, D)) break;
        if (!findNextLevelVoxels(i, plane, leadCurve, curve3D, curveLength, voxels, R, C, D)) break;
    }

    if (i != curveLength + 1) return 0;
    *curvature = min(curvatureSum, *curvature);
    return 1;
}

__device__ int computeCurvatureOfDigitalCurve3D(int3 voxel, int plane, int curveLength, const unsigned char* voxels, int R, int C, int D) {
    unsigned int curve3D[2 * MAX_CURVE_LENGTH + 1];
    curve3D[curveLength] = getVoxelID(voxel, R, C);

    int curvature = INT_MAX_VALUE;
    Candidate headCurve[8];
    Candidate trailCurve[8];
    Candidate leadCurve[8];

    getNeighborsInPlane(voxel, plane, voxels, R, C, D, headCurve);
    int count = 0;
    for (int i = 0; i < 8; ++i) {
        if (!isInvalidCandidate(headCurve[i])) ++count;
    }
    if (count == 0 || count > 3) return -1;

    for (int i = 0; i < 7; ++i) {
        if (isInvalidCandidate(headCurve[i])) continue;
        for (int j = i + 1; j < 8; ++j) {
            if (isInvalidCandidate(headCurve[j])) continue;

            for (int k = 0; k < 8; ++k) {
                trailCurve[k] = invalidCandidate();
                leadCurve[k] = invalidCandidate();
            }
            trailCurve[0] = headCurve[i];
            leadCurve[0] = headCurve[j];

            if (!computeCurvatureOfDigitalCurve2D(plane, voxel, trailCurve, leadCurve, curve3D, curveLength, voxels, R, C, D, &curvature)) continue;
            if (curvature == 0) return 0;
        }
    }

    return curvature == INT_MAX_VALUE ? -1 : curvature;
}

__device__ int computeCurvatureAtVoxel(int3 voxel, int curveLength, const unsigned char* voxels, int R, int C, int D) {
    int maxCurvature = INT_MIN_VALUE;
    int minCurvature = INT_MAX_VALUE;

    for (int plane = 0; plane < 9; ++plane) {
        int curvature = computeCurvatureOfDigitalCurve3D(voxel, plane, curveLength, voxels, R, C, D);
        if (curvature == -1) continue;
        maxCurvature = max(maxCurvature, curvature);
        minCurvature = min(minCurvature, curvature);
    }

    if (maxCurvature == INT_MIN_VALUE || minCurvature == INT_MAX_VALUE) return INT_MAX_VALUE;
    return maxCurvature + minCurvature;
}

__device__ inline int innerSpaceVoxelID(int i, int j, int k, int R, int C, int D, int plane) {
    int id = 0;
    int W = 0;
    switch (plane) {
        case 0: id = i * D + j * R * D; W = 1;     break;
        case 1: id = i * C * D + j;     W = C;     break;
        default: id = i + j * R;        W = R * C; break;
    }
    return id + k * W;
}

__global__ void computeInnerSpaceVoxels(unsigned char* voxels, int R, int C, int D, int plane) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= R || y >= C) return;

    int count = 0;
    int z = 0;

    for (; z < D; ++z) {
        const int id = innerSpaceVoxelID(x, y, z, R, C, D, plane);
        if (voxels[id] == 1) {
            const int nextIsVoxel = (z != D - 1) && voxels[innerSpaceVoxelID(x, y, z + 1, R, C, D, plane)] == 1;
            if (z == D - 1 || nextIsVoxel) continue;
            if (!nextIsVoxel) ++count;
        } else if ((count & 1) == 1) {
            voxels[id] = (unsigned char)(voxels[id] + 2);
        }
    }

    if ((count & 1) == 1) {
        --z;
        while (z >= 0 && voxels[innerSpaceVoxelID(x, y, z, R, C, D, plane)] != 1) {
            const int id = innerSpaceVoxelID(x, y, z, R, C, D, plane);
            voxels[id] = (unsigned char)(voxels[id] - 2);
            --z;
        }
    }
}

__global__ void markInteriorVoxels(unsigned char* voxels, int R, int C, int D) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    const int z = blockIdx.z * blockDim.z + threadIdx.z;
    if (x >= R || y >= C || z >= D) return;

    const int id = getVoxelID(make_int3(x, y, z), R, C);
    if (voxels[id] == 2 || voxels[id] == 4) voxels[id] = 0;
    else if (voxels[id] == 6) voxels[id] = 2;
}

__global__ void computeFrontierVoxels(unsigned char* voxels, int R, int C, int D) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    const int z = blockIdx.z * blockDim.z + threadIdx.z;
    if (x >= R || y >= C || z >= D) return;

    const int id = getVoxelID(make_int3(x, y, z), R, C);
    if (voxels[id] != 1) return;

    const int slice = R * C;
    if (x == 0 || voxels[id - 1] == 0) return;
    if (x == R - 1 || voxels[id + 1] == 0) return;
    if (y == 0 || voxels[id - R] == 0) return;
    if (y == C - 1 || voxels[id + R] == 0) return;
    if (z == 0 || voxels[id - slice] == 0) return;
    if (z == D - 1 || voxels[id + slice] == 0) return;
    voxels[id] = 2;
}


__global__ void estimateCurvature(const unsigned char* voxels,
                                  int* curvatures,
                                  const int* surfaceVoxelIds,
                                  int surfaceVoxelCount,
                                  int curveLength,
                                  int R,
                                  int C,
                                  int D) {
    const int surfaceIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (surfaceIndex >= surfaceVoxelCount) return;
    const int voxelID = surfaceVoxelIds[surfaceIndex];

    if (curveLength > MAX_CURVE_LENGTH || voxels[voxelID] != 1) {
        curvatures[voxelID] = INT_MAX_VALUE;
        return;
    }

    const int3 voxel = getCoordinates(voxelID, R, C);
    curvatures[voxelID] = computeCurvatureAtVoxel(voxel, curveLength, voxels, R, C, D);
}
