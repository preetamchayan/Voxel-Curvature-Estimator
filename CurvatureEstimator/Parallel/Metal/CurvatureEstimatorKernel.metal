#include <metal_stdlib>

using namespace metal;

constant int MAX_CURVE_LENGTH = 32;
constant unsigned char INVALID_CHAIN_CODE = static_cast<unsigned char>(255);
constant uint INVALID_VOXEL_ID = UINT_MAX;

struct Candidate {
    uint voxelID;
    uchar chainCode;
};

inline int getVoxelID(int3 point, int R, int C) {
    return point.x + point.y * R + point.z * R * C;
}

inline int3 getCoordinates(uint voxelID, int R, int C) {
    int t = int(voxelID) % (R * C);
    return int3(t % R, t / R, int(voxelID) / (R * C));
}

inline int inGrid(int x, int y, int z, int R, int C, int D) {
    return x >= 0 && x < R && y >= 0 && y < C && z >= 0 && z < D;
}

inline Candidate invalidCandidate() {
    Candidate c;
    c.voxelID = INVALID_VOXEL_ID;
    c.chainCode = INVALID_CHAIN_CODE;
    return c;
}

inline int isInvalidCandidate(Candidate c) {
    return c.chainCode == INVALID_CHAIN_CODE || c.voxelID == INVALID_VOXEL_ID;
}

inline int isSurface(const device uchar* voxels, int3 point, int R, int C, int D) {
    return inGrid(point.x, point.y, point.z, R, C, D) &&
           voxels[getVoxelID(point, R, C)] == 1;
}

inline int isInterior(const device uchar* voxels, int3 point, int R, int C, int D) {
    return inGrid(point.x, point.y, point.z, R, C, D) &&
           voxels[getVoxelID(point, R, C)] == 2;
}

inline uchar getChainCode(int2 neighbor, int2 voxel) {
    const int2 dir = int2(neighbor.x - voxel.x, neighbor.y - voxel.y);
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

inline void addNeighbor(int3 voxel, int3 neighbor, int neighborIndex, int plane, Candidate neighbors[8], int R, int C) {
    neighbors[neighborIndex].voxelID = getVoxelID(neighbor, R, C);
    switch (plane / 3) {
        case 0:
            neighbors[neighborIndex].chainCode = getChainCode(int2(neighbor.x, neighbor.y), int2(voxel.x, voxel.y));
            break;
        case 1:
            neighbors[neighborIndex].chainCode = getChainCode(int2(neighbor.y, neighbor.z), int2(voxel.y, voxel.z));
            break;
        case 2:
            neighbors[neighborIndex].chainCode = getChainCode(int2(neighbor.z, neighbor.x), int2(voxel.z, voxel.x));
            break;
    }
}

void getNeighborsInPlane(int3 voxel, int plane, const device uchar* voxels, int R, int C, int D, Candidate neighbors[8]) {
    for (int n = 0; n < 8; ++n) neighbors[n] = invalidCandidate();

    const int x = voxel.x;
    const int y = voxel.y;
    const int z = voxel.z;
    const int xmin = 0, xmax = R - 1;
    const int ymin = 0, ymax = C - 1;
    const int zmin = 0, zmax = D - 1;

    switch (plane) {
        case 0:
            if (x != xmax && isSurface(voxels, int3(x + 1, y, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z), 0, plane, neighbors, R, C);
            if (x != xmin && isSurface(voxels, int3(x - 1, y, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z), 1, plane, neighbors, R, C);
            if (y != ymax && isSurface(voxels, int3(x, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z), 2, plane, neighbors, R, C);
            if (y != ymin && isSurface(voxels, int3(x, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y + 1, z), 4, plane, neighbors, R, C);
            if (x != xmax && y != ymin && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y - 1, z), 5, plane, neighbors, R, C);
            if (x != xmin && y != ymax && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y + 1, z), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y - 1, z), 7, plane, neighbors, R, C);
            break;
        case 1:
            if (y != ymax && (isSurface(voxels, int3(x, y + 1, z), R, C, D) || ((x != xmin && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D)) || (x != xmax && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, int3(x, y + 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, int3(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, int3(x, y + 1, z), R, C, D))) addNeighbor(voxel, int3(x, y + 1, z), 0, plane, neighbors, R, C);
            if (y != ymin && (isSurface(voxels, int3(x, y - 1, z), R, C, D) || ((x != xmin && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D)) || (x != xmax && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, int3(x, y - 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, int3(x, y - 1, z + 1), R, C, D))) && !isInterior(voxels, int3(x, y - 1, z), R, C, D))) addNeighbor(voxel, int3(x, y - 1, z), 1, plane, neighbors, R, C);
            if (x != xmax && z != zmin && isSurface(voxels, int3(x + 1, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z - 1), 2, plane, neighbors, R, C);
            if (x != xmin && z != zmax && isSurface(voxels, int3(x - 1, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z + 1), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, int3(x, y + 1, z - 1), R, C, D) && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z - 1), 4, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, int3(x, y + 1, z + 1), R, C, D) && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z + 1), 5, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, int3(x, y - 1, z - 1), R, C, D) && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z - 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, int3(x, y - 1, z + 1), R, C, D) && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z + 1), 7, plane, neighbors, R, C);
            break;
        case 2:
            if (y != ymax && (isSurface(voxels, int3(x, y + 1, z), R, C, D) || ((x != xmin && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D)) || (x != xmax && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, int3(x, y + 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, int3(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, int3(x, y + 1, z), R, C, D))) addNeighbor(voxel, int3(x, y + 1, z), 0, plane, neighbors, R, C);
            if (y != ymin && (isSurface(voxels, int3(x, y - 1, z), R, C, D) || ((x != xmin && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D)) || (x != xmax && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, int3(x, y - 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, int3(x, y - 1, z + 1), R, C, D))) && !isInterior(voxels, int3(x, y - 1, z), R, C, D))) addNeighbor(voxel, int3(x, y - 1, z), 1, plane, neighbors, R, C);
            if (x != xmax && z != zmax && isSurface(voxels, int3(x + 1, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z + 1), 2, plane, neighbors, R, C);
            if (x != xmin && z != zmin && isSurface(voxels, int3(x - 1, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z - 1), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, int3(x, y + 1, z + 1), R, C, D) && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, int3(x, y - 1, z + 1), R, C, D) && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z + 1), 5, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, int3(x - 1, y + 1, z), R, C, D) && isSurface(voxels, int3(x, y + 1, z - 1), R, C, D) && !isInterior(voxels, int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z - 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, int3(x, y - 1, z - 1), R, C, D) && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 3:
            if (y != ymax && isSurface(voxels, int3(x, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z), 0, plane, neighbors, R, C);
            if (y != ymin && isSurface(voxels, int3(x, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z), 1, plane, neighbors, R, C);
            if (z != zmax && isSurface(voxels, int3(x, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y, z + 1), 2, plane, neighbors, R, C);
            if (z != zmin && isSurface(voxels, int3(x, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y, z - 1), 3, plane, neighbors, R, C);
            if (y != ymax && z != zmax && isSurface(voxels, int3(x, y + 1, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (y != ymax && z != zmin && isSurface(voxels, int3(x, y + 1, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z - 1), 5, plane, neighbors, R, C);
            if (y != ymin && z != zmax && isSurface(voxels, int3(x, y - 1, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z + 1), 6, plane, neighbors, R, C);
            if (y != ymin && z != zmin && isSurface(voxels, int3(x, y - 1, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 4:
            if (z != zmax && (isSurface(voxels, int3(x, y, z + 1), R, C, D) || ((x != xmin && isSurface(voxels, int3(x - 1, y, z + 1), R, C, D)) || (x != xmax && isSurface(voxels, int3(x + 1, y, z + 1), R, C, D))) && ((y != ymin && isSurface(voxels, int3(x, y - 1, z + 1), R, C, D)) || (y != ymax && isSurface(voxels, int3(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, int3(x, y, z + 1), R, C, D))) addNeighbor(voxel, int3(x, y, z + 1), 0, plane, neighbors, R, C);
            if (z != zmin && (isSurface(voxels, int3(x, y, z - 1), R, C, D) || ((x != xmin && isSurface(voxels, int3(x - 1, y, z - 1), R, C, D)) || (x != xmax && isSurface(voxels, int3(x + 1, y, z - 1), R, C, D))) && ((y != ymin && isSurface(voxels, int3(x, y - 1, z - 1), R, C, D)) || (y != ymax && isSurface(voxels, int3(x, y + 1, z - 1), R, C, D))) && !isInterior(voxels, int3(x, y, z - 1), R, C, D))) addNeighbor(voxel, int3(x, y, z - 1), 1, plane, neighbors, R, C);
            if (x != xmin && y != ymax && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y + 1, z), 2, plane, neighbors, R, C);
            if (x != xmax && y != ymin && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y - 1, z), 3, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, int3(x, y + 1, z + 1), R, C, D) && isSurface(voxels, int3(x - 1, y, z + 1), R, C, D) && !isInterior(voxels, int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, int3(x, y + 1, z - 1), R, C, D) && isSurface(voxels, int3(x - 1, y, z - 1), R, C, D) && !isInterior(voxels, int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z - 1), 5, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, int3(x, y - 1, z + 1), R, C, D) && isSurface(voxels, int3(x + 1, y, z + 1), R, C, D) && !isInterior(voxels, int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z + 1), 6, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, int3(x, y - 1, z - 1), R, C, D) && isSurface(voxels, int3(x + 1, y, z - 1), R, C, D) && !isInterior(voxels, int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 5:
            if (z != zmax && (isSurface(voxels, int3(x, y, z + 1), R, C, D) || ((x != xmin && isSurface(voxels, int3(x - 1, y, z + 1), R, C, D)) || (x != xmax && isSurface(voxels, int3(x + 1, y, z + 1), R, C, D))) && ((y != ymin && isSurface(voxels, int3(x, y - 1, z + 1), R, C, D)) || (y != ymax && isSurface(voxels, int3(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, int3(x, y, z + 1), R, C, D))) addNeighbor(voxel, int3(x, y, z + 1), 0, plane, neighbors, R, C);
            if (z != zmin && (isSurface(voxels, int3(x, y, z - 1), R, C, D) || ((x != xmin && isSurface(voxels, int3(x - 1, y, z - 1), R, C, D)) || (x != xmax && isSurface(voxels, int3(x + 1, y, z - 1), R, C, D))) && ((y != ymin && isSurface(voxels, int3(x, y - 1, z - 1), R, C, D)) || (y != ymax && isSurface(voxels, int3(x, y + 1, z - 1), R, C, D))) && !isInterior(voxels, int3(x, y, z - 1), R, C, D))) addNeighbor(voxel, int3(x, y, z - 1), 1, plane, neighbors, R, C);
            if (x != xmax && y != ymax && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y + 1, z), 2, plane, neighbors, R, C);
            if (x != xmin && y != ymin && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y - 1, z), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, int3(x, y + 1, z + 1), R, C, D) && isSurface(voxels, int3(x + 1, y, z + 1), R, C, D) && !isInterior(voxels, int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, int3(x + 1, y, z - 1), R, C, D) && isSurface(voxels, int3(x, y + 1, z - 1), R, C, D) && !isInterior(voxels, int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z - 1), 5, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, int3(x, y - 1, z + 1), R, C, D) && isSurface(voxels, int3(x - 1, y, z + 1), R, C, D) && !isInterior(voxels, int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z + 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, int3(x, y - 1, z - 1), R, C, D) && isSurface(voxels, int3(x - 1, y, z - 1), R, C, D) && !isInterior(voxels, int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 6:
            if (z != zmax && isSurface(voxels, int3(x, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y, z + 1), 0, plane, neighbors, R, C);
            if (z != zmin && isSurface(voxels, int3(x, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y, z - 1), 1, plane, neighbors, R, C);
            if (x != xmax && isSurface(voxels, int3(x + 1, y, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z), 2, plane, neighbors, R, C);
            if (x != xmin && isSurface(voxels, int3(x - 1, y, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z), 3, plane, neighbors, R, C);
            if (x != xmax && z != zmax && isSurface(voxels, int3(x + 1, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z + 1), 4, plane, neighbors, R, C);
            if (x != xmin && z != zmax && isSurface(voxels, int3(x - 1, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z + 1), 5, plane, neighbors, R, C);
            if (x != xmax && z != zmin && isSurface(voxels, int3(x + 1, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z - 1), 6, plane, neighbors, R, C);
            if (x != xmin && z != zmin && isSurface(voxels, int3(x - 1, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z - 1), 7, plane, neighbors, R, C);
            break;
        case 7:
            if (x != xmax && (isSurface(voxels, int3(x + 1, y, z), R, C, D) || ((y != ymin && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, int3(x + 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, int3(x + 1, y, z + 1), R, C, D))) && !isInterior(voxels, int3(x + 1, y, z), R, C, D))) addNeighbor(voxel, int3(x + 1, y, z), 0, plane, neighbors, R, C);
            if (x != xmin && (isSurface(voxels, int3(x - 1, y, z), R, C, D) || ((y != ymin && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, int3(x - 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, int3(x - 1, y, z + 1), R, C, D))) && !isInterior(voxels, int3(x - 1, y, z), R, C, D))) addNeighbor(voxel, int3(x - 1, y, z), 1, plane, neighbors, R, C);
            if (y != ymin && z != zmax && isSurface(voxels, int3(x, y - 1, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z + 1), 2, plane, neighbors, R, C);
            if (y != ymax && z != zmin && isSurface(voxels, int3(x, y + 1, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z - 1), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, int3(x + 1, y, z + 1), R, C, D) && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(voxels, int3(x - 1, y, z + 1), R, C, D) && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z + 1), 5, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, int3(x + 1, y, z - 1), R, C, D) && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z - 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(voxels, int3(x - 1, y, z - 1), R, C, D) && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z - 1), 7, plane, neighbors, R, C);
            break;
        case 8:
            if (x != xmax && (isSurface(voxels, int3(x + 1, y, z), R, C, D) || ((y != ymin && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, int3(x + 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, int3(x + 1, y, z + 1), R, C, D))) && !isInterior(voxels, int3(x + 1, y, z), R, C, D))) addNeighbor(voxel, int3(x + 1, y, z), 0, plane, neighbors, R, C);
            if (x != xmin && (isSurface(voxels, int3(x - 1, y, z), R, C, D) || ((y != ymin && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, int3(x - 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, int3(x - 1, y, z + 1), R, C, D))) && !isInterior(voxels, int3(x - 1, y, z), R, C, D))) addNeighbor(voxel, int3(x - 1, y, z), 1, plane, neighbors, R, C);
            if (y != ymax && z != zmax && isSurface(voxels, int3(x, y + 1, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z + 1), 2, plane, neighbors, R, C);
            if (y != ymin && z != zmin && isSurface(voxels, int3(x, y - 1, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z - 1), 3, plane, neighbors, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, int3(x + 1, y, z + 1), R, C, D) && isSurface(voxels, int3(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z + 1), 4, plane, neighbors, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, int3(x + 1, y, z - 1), R, C, D) && isSurface(voxels, int3(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z - 1), 5, plane, neighbors, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(voxels, int3(x - 1, y, z + 1), R, C, D) && isSurface(voxels, int3(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z + 1), 6, plane, neighbors, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(voxels, int3(x - 1, y, z - 1), R, C, D) && isSurface(voxels, int3(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z - 1), 7, plane, neighbors, R, C);
            break;
    }
}

int findNextLevelVoxels(int step, int plane, Candidate curve[8], const uint curve3D[2 * MAX_CURVE_LENGTH + 1], int curveLength, const device uchar* voxels, int R, int C, int D) {
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

int computeCurvatureOfDigitalCurve2D(int plane, int3 voxel, Candidate trailCurve[8], Candidate leadCurve[8], uint curve3D[2 * MAX_CURVE_LENGTH + 1], int curveLength, const device uchar* voxels, int R, int C, int D, thread int* curvature) {
    uchar prevTrailChainCode = INVALID_CHAIN_CODE;
    uchar prevLeadChainCode = INVALID_CHAIN_CODE;
    int curvatureSum = 0;
    int i = 1;

    for (; i <= curveLength; ++i) {
        uchar trailChainCode = INVALID_CHAIN_CODE;
        uchar leadChainCode = INVALID_CHAIN_CODE;
        int minDiff = INVALID_CHAIN_CODE;

        for (int j = 0; j < 8; ++j) {
            if (isInvalidCandidate(trailCurve[j])) continue;
            int3 voxel1 = getCoordinates(trailCurve[j].voxelID, R, C);
            uchar trailChain = trailCurve[j].chainCode;

            for (int k = 0; k < 8; ++k) {
                if (isInvalidCandidate(leadCurve[k])) continue;
                int3 voxel2 = getCoordinates(leadCurve[k].voxelID, R, C);
                uchar leadChain = leadCurve[k].chainCode < 4 ? leadCurve[k].chainCode + 4 : leadCurve[k].chainCode - 4;

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

int computeCurvatureOfDigitalCurve3D(int3 voxel, int plane, int curveLength, const device uchar* voxels, int R, int C, int D) {
    uint curve3D[2 * MAX_CURVE_LENGTH + 1];
    curve3D[curveLength] = getVoxelID(voxel, R, C);

    int curvature = 2147483647;
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

    return curvature == 2147483647 ? -1 : curvature;
}

int computeCurvatureAtVoxel(int3 voxel, int curveLength, const device uchar* voxels, int R, int C, int D) {
    int maxCurvature = -2147483647 - 1;
    int minCurvature = 2147483647;

    for (int plane = 0; plane < 9; ++plane) {
        int curvature = computeCurvatureOfDigitalCurve3D(voxel, plane, curveLength, voxels, R, C, D);
        if (curvature == -1) continue;
        maxCurvature = max(maxCurvature, curvature);
        minCurvature = min(minCurvature, curvature);
    }

    if (maxCurvature == -2147483647 - 1 || minCurvature == 2147483647) return 2147483647;
    return maxCurvature + minCurvature;
}

inline int innerSpaceVoxelID(int x, int y, int z, int R, int C, int D, int plane) {
    int id = 0;
    int W = 0;
    const int i = x;
    const int j = y;
    const int k = z;

    switch (plane) {
        case 0:
            id = i * D + j * R * D;
            W = 1;
            break;
        case 1:
            id = i * C * D + j;
            W = C;
            break;
        default:
            id = i + j * R;
            W = R * C;
            break;
    }
    return id + k * W;
}

kernel void computeInnerSpaceVoxels(device uchar* voxels [[buffer(0)]],
                                   constant int& R [[buffer(1)]],
                                   constant int& C [[buffer(2)]],
                                   constant int& D [[buffer(3)]],
                                   constant int& plane [[buffer(4)]],
                                   uint2 gid [[thread_position_in_grid]]) {
    const int x = int(gid.x);
    const int y = int(gid.y);
    if (x >= R || y >= C) {
        return;
    }

    int count = 0;
    int z = 0;

    for (; z < D; ++z) {
        const int id = innerSpaceVoxelID(x, y, z, R, C, D, plane);
        if (voxels[id] == 1) {
            const bool nextIsVoxel = (z != D - 1) && voxels[innerSpaceVoxelID(x, y, z + 1, R, C, D, plane)] == 1;
            if (z == D - 1 || nextIsVoxel) {
                continue;
            }
            ++count;
        } else if ((count & 1) == 1) {
            voxels[id] = uchar(voxels[id] + 2);
        }
    }

    if ((count & 1) == 1) {
        --z;
        while (z >= 0 && voxels[innerSpaceVoxelID(x, y, z, R, C, D, plane)] != 1) {
            const int id = innerSpaceVoxelID(x, y, z, R, C, D, plane);
            voxels[id] = uchar(voxels[id] - 2);
            --z;
        }
    }
}

kernel void markInteriorVoxels(device uchar* voxels [[buffer(0)]],
                              constant int& R [[buffer(1)]],
                              constant int& C [[buffer(2)]],
                              constant int& D [[buffer(3)]],
                              uint3 gid [[thread_position_in_grid]]) {
    const int x = int(gid.x);
    const int y = int(gid.y);
    const int z = int(gid.z);
    if (x >= R || y >= C || z >= D) {
        return;
    }

    const int id = getVoxelID(int3(x, y, z), R, C);
    if (voxels[id] == 2 || voxels[id] == 4) {
        voxels[id] = 0;
    } else if (voxels[id] == 6) {
        voxels[id] = 2;
    }
}

kernel void computeFrontierVoxels(device uchar* voxels [[buffer(0)]],
                                 constant int& R [[buffer(1)]],
                                 constant int& C [[buffer(2)]],
                                 constant int& D [[buffer(3)]],
                                 uint3 gid [[thread_position_in_grid]]) {
    const int x = int(gid.x);
    const int y = int(gid.y);
    const int z = int(gid.z);
    if (x >= R || y >= C || z >= D) {
        return;
    }

    const int id = getVoxelID(int3(x, y, z), R, C);
    if (voxels[id] != 1) {
        return;
    }

    const int slice = R * C;
    if (x == 0 || voxels[id - 1] == 0) return;
    if (x == R - 1 || voxels[id + 1] == 0) return;
    if (y == 0 || voxels[id - R] == 0) return;
    if (y == C - 1 || voxels[id + R] == 0) return;
    if (z == 0 || voxels[id - slice] == 0) return;
    if (z == D - 1 || voxels[id + slice] == 0) return;
    voxels[id] = 2;
}

kernel void estimateCurvature(device const uchar* voxels [[buffer(0)]],
                             device int* curvatures [[buffer(1)]],
                             device const int* surfaceVoxelIds [[buffer(2)]],
                             constant int& surfaceVoxelCount [[buffer(3)]],
                             constant int& curveLength [[buffer(4)]],
                             constant int& R [[buffer(5)]],
                             constant int& C [[buffer(6)]],
                             constant int& D [[buffer(7)]],
                             uint tid [[thread_position_in_grid]]) {
    if (tid >= uint(surfaceVoxelCount)) {
        return;
    }

    const int voxelID = surfaceVoxelIds[tid];
    if (curveLength > MAX_CURVE_LENGTH || voxels[voxelID] != 1) {
        curvatures[voxelID] = 2147483647;
        return;
    }

    const int3 voxel = getCoordinates(uint(voxelID), R, C);
    curvatures[voxelID] = computeCurvatureAtVoxel(voxel, curveLength, voxels, R, C, D);
}
