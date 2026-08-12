#define INT_MAX_VALUE 2147483647
#define INT_MIN_VALUE (-2147483647 - 1)
#define INVALID_CHAIN_CODE ((uchar)255)

typedef struct Candidate {
    int3 point;
    uchar chainCode;
} Candidate;

inline int getVoxelID(int x, int y, int z, int R, int C) {
    return x + y * R + z * R * C;
}

inline int inGrid(int x, int y, int z, int R, int C, int D) {
    return x >= 0 && x < R && y >= 0 && y < C && z >= 0 && z < D;
}

inline int pointEqual(int3 a, int3 b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline Candidate invalidCandidate(void) {
    Candidate c;
    c.point = (int3)(-1, -1, -1);
    c.chainCode = INVALID_CHAIN_CODE;
    return c;
}

inline int isInvalidCandidate(Candidate c) {
    return c.chainCode == INVALID_CHAIN_CODE || pointEqual(c.point, (int3)(-1, -1, -1));
}

inline int isSurface(__global const uchar* voxels, int3 point, int R, int C, int D) {
    return inGrid(point.x, point.y, point.z, R, C, D) &&
           voxels[getVoxelID(point.x, point.y, point.z, R, C)] == 1;
}

inline int isInterior(__global const uchar* voxels, int3 point, int R, int C, int D) {
    return inGrid(point.x, point.y, point.z, R, C, D) &&
           voxels[getVoxelID(point.x, point.y, point.z, R, C)] == 2;
}

inline uchar getChainCode(int2 neighbor, int2 voxel) {
    const int2 dir = neighbor - voxel;
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

inline void addNeighbor(int3 voxel, int3 neighbor, int neighborIndex, int plane, __private Candidate neighbors[8]) {
    neighbors[neighborIndex].point = neighbor;
    switch (plane / 3) {
        case 0:
            neighbors[neighborIndex].chainCode = getChainCode((int2)(neighbor.x, neighbor.y), (int2)(voxel.x, voxel.y));
            break;
        case 1:
            neighbors[neighborIndex].chainCode = getChainCode((int2)(neighbor.y, neighbor.z), (int2)(voxel.y, voxel.z));
            break;
        case 2:
            neighbors[neighborIndex].chainCode = getChainCode((int2)(neighbor.z, neighbor.x), (int2)(voxel.z, voxel.x));
            break;
    }
}

void getNeighborsInPlane(int3 voxel, int plane, __global const uchar* voxels, int R, int C, int D, __private Candidate neighbors[8]) {
    for (int n = 0; n < 8; ++n) neighbors[n] = invalidCandidate();

    int3 v[8];
    const int x = voxel.x;
    const int y = voxel.y;
    const int z = voxel.z;
    const int xmin = 0, xmax = R - 1;
    const int ymin = 0, ymax = C - 1;
    const int zmin = 0, zmax = D - 1;

    switch (plane) {
        case 0:
            v[0] = (int3)(x + 1, y, z);     v[1] = (int3)(x - 1, y, z);
            v[2] = (int3)(x, y + 1, z);     v[3] = (int3)(x, y - 1, z);
            v[4] = (int3)(x + 1, y + 1, z); v[5] = (int3)(x + 1, y - 1, z);
            v[6] = (int3)(x - 1, y + 1, z); v[7] = (int3)(x - 1, y - 1, z);
            if (x != xmax && isSurface(voxels, v[0], R, C, D)) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (x != xmin && isSurface(voxels, v[1], R, C, D)) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (y != ymax && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (y != ymin && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (x != xmax && y != ymax && isSurface(voxels, v[4], R, C, D)) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (x != xmax && y != ymin && isSurface(voxels, v[5], R, C, D)) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (x != xmin && y != ymax && isSurface(voxels, v[6], R, C, D)) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (x != xmin && y != ymin && isSurface(voxels, v[7], R, C, D)) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
        case 1:
            v[0] = (int3)(x, y + 1, z);         v[1] = (int3)(x, y - 1, z);
            v[2] = (int3)(x + 1, y, z - 1);     v[3] = (int3)(x - 1, y, z + 1);
            v[4] = (int3)(x + 1, y + 1, z - 1); v[5] = (int3)(x - 1, y + 1, z + 1);
            v[6] = (int3)(x + 1, y - 1, z - 1); v[7] = (int3)(x - 1, y - 1, z + 1);
            if (y != ymax && (isSurface(voxels, v[0], R, C, D) || ((x != xmin && isSurface(voxels, (int3)(x - 1, y + 1, z), R, C, D)) || (x != xmax && isSurface(voxels, (int3)(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, (int3)(x, y + 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, (int3)(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, v[0], R, C, D))) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (y != ymin && (isSurface(voxels, v[1], R, C, D) || ((x != xmin && isSurface(voxels, (int3)(x - 1, y - 1, z), R, C, D)) || (x != xmax && isSurface(voxels, (int3)(x + 1, y - 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, (int3)(x, y - 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, (int3)(x, y - 1, z + 1), R, C, D))) && !isInterior(voxels, v[1], R, C, D))) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (x != xmax && z != zmin && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (x != xmin && z != zmax && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, v[4], R, C, D) || (isSurface(voxels, (int3)(x, y + 1, z - 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, v[4], R, C, D)))) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, v[5], R, C, D) || (isSurface(voxels, (int3)(x, y + 1, z + 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, v[5], R, C, D)))) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, v[6], R, C, D) || (isSurface(voxels, (int3)(x, y - 1, z - 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, v[6], R, C, D)))) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, v[7], R, C, D) || (isSurface(voxels, (int3)(x, y - 1, z + 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, v[7], R, C, D)))) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
        case 2:
            v[0] = (int3)(x, y + 1, z);         v[1] = (int3)(x, y - 1, z);
            v[2] = (int3)(x + 1, y, z + 1);     v[3] = (int3)(x - 1, y, z - 1);
            v[4] = (int3)(x + 1, y + 1, z + 1); v[5] = (int3)(x + 1, y - 1, z + 1);
            v[6] = (int3)(x - 1, y + 1, z - 1); v[7] = (int3)(x - 1, y - 1, z - 1);
            if (y != ymax && (isSurface(voxels, v[0], R, C, D) || ((x != xmin && isSurface(voxels, (int3)(x - 1, y + 1, z), R, C, D)) || (x != xmax && isSurface(voxels, (int3)(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, (int3)(x, y + 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, (int3)(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, v[0], R, C, D))) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (y != ymin && (isSurface(voxels, v[1], R, C, D) || ((x != xmin && isSurface(voxels, (int3)(x - 1, y - 1, z), R, C, D)) || (x != xmax && isSurface(voxels, (int3)(x + 1, y - 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, (int3)(x, y - 1, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, (int3)(x, y - 1, z + 1), R, C, D))) && !isInterior(voxels, v[1], R, C, D))) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (x != xmax && z != zmax && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (x != xmin && z != zmin && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, v[4], R, C, D) || (isSurface(voxels, (int3)(x, y + 1, z + 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, v[4], R, C, D)))) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, v[5], R, C, D) || (isSurface(voxels, (int3)(x, y - 1, z + 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, v[5], R, C, D)))) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, v[6], R, C, D) || (isSurface(voxels, (int3)(x - 1, y + 1, z), R, C, D) && isSurface(voxels, (int3)(x, y + 1, z - 1), R, C, D) && !isInterior(voxels, v[6], R, C, D)))) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, v[7], R, C, D) || (isSurface(voxels, (int3)(x, y - 1, z - 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, v[7], R, C, D)))) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
        case 3:
            v[0] = (int3)(x, y + 1, z);     v[1] = (int3)(x, y - 1, z);
            v[2] = (int3)(x, y, z + 1);     v[3] = (int3)(x, y, z - 1);
            v[4] = (int3)(x, y + 1, z + 1); v[5] = (int3)(x, y + 1, z - 1);
            v[6] = (int3)(x, y - 1, z + 1); v[7] = (int3)(x, y - 1, z - 1);
            if (y != ymax && isSurface(voxels, v[0], R, C, D)) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (y != ymin && isSurface(voxels, v[1], R, C, D)) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (z != zmax && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (z != zmin && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (y != ymax && z != zmax && isSurface(voxels, v[4], R, C, D)) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (y != ymax && z != zmin && isSurface(voxels, v[5], R, C, D)) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (y != ymin && z != zmax && isSurface(voxels, v[6], R, C, D)) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (y != ymin && z != zmin && isSurface(voxels, v[7], R, C, D)) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
        case 4:
            v[0] = (int3)(x, y, z + 1);         v[1] = (int3)(x, y, z - 1);
            v[2] = (int3)(x - 1, y + 1, z);     v[3] = (int3)(x + 1, y - 1, z);
            v[4] = (int3)(x - 1, y + 1, z + 1); v[5] = (int3)(x - 1, y + 1, z - 1);
            v[6] = (int3)(x + 1, y - 1, z + 1); v[7] = (int3)(x + 1, y - 1, z - 1);
            if (z != zmax && (isSurface(voxels, v[0], R, C, D) || ((x != xmin && isSurface(voxels, (int3)(x - 1, y, z + 1), R, C, D)) || (x != xmax && isSurface(voxels, (int3)(x + 1, y, z + 1), R, C, D))) && ((y != ymin && isSurface(voxels, (int3)(x, y - 1, z + 1), R, C, D)) || (y != ymax && isSurface(voxels, (int3)(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, v[0], R, C, D))) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (z != zmin && (isSurface(voxels, v[1], R, C, D) || ((x != xmin && isSurface(voxels, (int3)(x - 1, y, z - 1), R, C, D)) || (x != xmax && isSurface(voxels, (int3)(x + 1, y, z - 1), R, C, D))) && ((y != ymin && isSurface(voxels, (int3)(x, y - 1, z - 1), R, C, D)) || (y != ymax && isSurface(voxels, (int3)(x, y + 1, z - 1), R, C, D))) && !isInterior(voxels, v[1], R, C, D))) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (x != xmin && y != ymax && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (x != xmax && y != ymin && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, v[4], R, C, D) || (isSurface(voxels, (int3)(x, y + 1, z + 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y, z + 1), R, C, D) && !isInterior(voxels, v[4], R, C, D)))) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, v[5], R, C, D) || (isSurface(voxels, (int3)(x, y + 1, z - 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y, z - 1), R, C, D) && !isInterior(voxels, v[5], R, C, D)))) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, v[6], R, C, D) || (isSurface(voxels, (int3)(x, y - 1, z + 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y, z + 1), R, C, D) && !isInterior(voxels, v[6], R, C, D)))) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, v[7], R, C, D) || (isSurface(voxels, (int3)(x, y - 1, z - 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y, z - 1), R, C, D) && !isInterior(voxels, v[7], R, C, D)))) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
        case 5:
            v[0] = (int3)(x, y, z + 1);         v[1] = (int3)(x, y, z - 1);
            v[2] = (int3)(x + 1, y + 1, z);     v[3] = (int3)(x - 1, y - 1, z);
            v[4] = (int3)(x + 1, y + 1, z + 1); v[5] = (int3)(x + 1, y + 1, z - 1);
            v[6] = (int3)(x - 1, y - 1, z + 1); v[7] = (int3)(x - 1, y - 1, z - 1);
            if (z != zmax && (isSurface(voxels, v[0], R, C, D) || ((x != xmin && isSurface(voxels, (int3)(x - 1, y, z + 1), R, C, D)) || (x != xmax && isSurface(voxels, (int3)(x + 1, y, z + 1), R, C, D))) && ((y != ymin && isSurface(voxels, (int3)(x, y - 1, z + 1), R, C, D)) || (y != ymax && isSurface(voxels, (int3)(x, y + 1, z + 1), R, C, D))) && !isInterior(voxels, v[0], R, C, D))) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (z != zmin && (isSurface(voxels, v[1], R, C, D) || ((x != xmin && isSurface(voxels, (int3)(x - 1, y, z - 1), R, C, D)) || (x != xmax && isSurface(voxels, (int3)(x + 1, y, z - 1), R, C, D))) && ((y != ymin && isSurface(voxels, (int3)(x, y - 1, z - 1), R, C, D)) || (y != ymax && isSurface(voxels, (int3)(x, y + 1, z - 1), R, C, D))) && !isInterior(voxels, v[1], R, C, D))) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (x != xmax && y != ymax && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (x != xmin && y != ymin && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, v[4], R, C, D) || (isSurface(voxels, (int3)(x, y + 1, z + 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y, z + 1), R, C, D) && !isInterior(voxels, v[4], R, C, D)))) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, v[5], R, C, D) || (isSurface(voxels, (int3)(x + 1, y, z - 1), R, C, D) && isSurface(voxels, (int3)(x, y + 1, z - 1), R, C, D) && !isInterior(voxels, v[5], R, C, D)))) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, v[6], R, C, D) || (isSurface(voxels, (int3)(x, y - 1, z + 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y, z + 1), R, C, D) && !isInterior(voxels, v[6], R, C, D)))) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, v[7], R, C, D) || (isSurface(voxels, (int3)(x, y - 1, z - 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y, z - 1), R, C, D) && !isInterior(voxels, v[7], R, C, D)))) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
        case 6:
            v[0] = (int3)(x, y, z + 1);     v[1] = (int3)(x, y, z - 1);
            v[2] = (int3)(x + 1, y, z);     v[3] = (int3)(x - 1, y, z);
            v[4] = (int3)(x + 1, y, z + 1); v[5] = (int3)(x - 1, y, z + 1);
            v[6] = (int3)(x + 1, y, z - 1); v[7] = (int3)(x - 1, y, z - 1);
            if (z != zmax && isSurface(voxels, v[0], R, C, D)) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (z != zmin && isSurface(voxels, v[1], R, C, D)) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (x != xmax && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (x != xmin && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (x != xmax && z != zmax && isSurface(voxels, v[4], R, C, D)) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (x != xmin && z != zmax && isSurface(voxels, v[5], R, C, D)) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (x != xmax && z != zmin && isSurface(voxels, v[6], R, C, D)) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (x != xmin && z != zmin && isSurface(voxels, v[7], R, C, D)) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
        case 7:
            v[0] = (int3)(x + 1, y, z);         v[1] = (int3)(x - 1, y, z);
            v[2] = (int3)(x, y - 1, z + 1);     v[3] = (int3)(x, y + 1, z - 1);
            v[4] = (int3)(x + 1, y - 1, z + 1); v[5] = (int3)(x - 1, y - 1, z + 1);
            v[6] = (int3)(x + 1, y + 1, z - 1); v[7] = (int3)(x - 1, y + 1, z - 1);
            if (x != xmax && (isSurface(voxels, v[0], R, C, D) || ((y != ymin && isSurface(voxels, (int3)(x + 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, (int3)(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, (int3)(x + 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, (int3)(x + 1, y, z + 1), R, C, D))) && !isInterior(voxels, v[0], R, C, D))) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (x != xmin && (isSurface(voxels, v[1], R, C, D) || ((y != ymin && isSurface(voxels, (int3)(x - 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, (int3)(x - 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, (int3)(x - 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, (int3)(x - 1, y, z + 1), R, C, D))) && !isInterior(voxels, v[1], R, C, D))) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (y != ymin && z != zmax && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (y != ymax && z != zmin && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (x != xmax && y != ymin && z != zmax && (isSurface(voxels, v[4], R, C, D) || (isSurface(voxels, (int3)(x + 1, y, z + 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, v[4], R, C, D)))) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (x != xmin && y != ymin && z != zmax && (isSurface(voxels, v[5], R, C, D) || (isSurface(voxels, (int3)(x - 1, y, z + 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, v[5], R, C, D)))) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (x != xmax && y != ymax && z != zmin && (isSurface(voxels, v[6], R, C, D) || (isSurface(voxels, (int3)(x + 1, y, z - 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, v[6], R, C, D)))) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (x != xmin && y != ymax && z != zmin && (isSurface(voxels, v[7], R, C, D) || (isSurface(voxels, (int3)(x - 1, y, z - 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, v[7], R, C, D)))) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
        case 8:
            v[0] = (int3)(x + 1, y, z);         v[1] = (int3)(x - 1, y, z);
            v[2] = (int3)(x, y + 1, z + 1);     v[3] = (int3)(x, y - 1, z - 1);
            v[4] = (int3)(x + 1, y + 1, z + 1); v[5] = (int3)(x + 1, y - 1, z - 1);
            v[6] = (int3)(x - 1, y + 1, z + 1); v[7] = (int3)(x - 1, y - 1, z - 1);
            if (x != xmax && (isSurface(voxels, v[0], R, C, D) || ((y != ymin && isSurface(voxels, (int3)(x + 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, (int3)(x + 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, (int3)(x + 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, (int3)(x + 1, y, z + 1), R, C, D))) && !isInterior(voxels, v[0], R, C, D))) addNeighbor(voxel, v[0], 0, plane, neighbors);
            if (x != xmin && (isSurface(voxels, v[1], R, C, D) || ((y != ymin && isSurface(voxels, (int3)(x - 1, y - 1, z), R, C, D)) || (y != ymax && isSurface(voxels, (int3)(x - 1, y + 1, z), R, C, D))) && ((z != zmin && isSurface(voxels, (int3)(x - 1, y, z - 1), R, C, D)) || (z != zmax && isSurface(voxels, (int3)(x - 1, y, z + 1), R, C, D))) && !isInterior(voxels, v[1], R, C, D))) addNeighbor(voxel, v[1], 1, plane, neighbors);
            if (y != ymax && z != zmax && isSurface(voxels, v[2], R, C, D)) addNeighbor(voxel, v[2], 2, plane, neighbors);
            if (y != ymin && z != zmin && isSurface(voxels, v[3], R, C, D)) addNeighbor(voxel, v[3], 3, plane, neighbors);
            if (x != xmax && y != ymax && z != zmax && (isSurface(voxels, v[4], R, C, D) || (isSurface(voxels, (int3)(x + 1, y, z + 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y + 1, z), R, C, D) && !isInterior(voxels, v[4], R, C, D)))) addNeighbor(voxel, v[4], 4, plane, neighbors);
            if (x != xmax && y != ymin && z != zmin && (isSurface(voxels, v[5], R, C, D) || (isSurface(voxels, (int3)(x + 1, y, z - 1), R, C, D) && isSurface(voxels, (int3)(x + 1, y - 1, z), R, C, D) && !isInterior(voxels, v[5], R, C, D)))) addNeighbor(voxel, v[5], 5, plane, neighbors);
            if (x != xmin && y != ymax && z != zmax && (isSurface(voxels, v[6], R, C, D) || (isSurface(voxels, (int3)(x - 1, y, z + 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y + 1, z), R, C, D) && !isInterior(voxels, v[6], R, C, D)))) addNeighbor(voxel, v[6], 6, plane, neighbors);
            if (x != xmin && y != ymin && z != zmin && (isSurface(voxels, v[7], R, C, D) || (isSurface(voxels, (int3)(x - 1, y, z - 1), R, C, D) && isSurface(voxels, (int3)(x - 1, y - 1, z), R, C, D) && !isInterior(voxels, v[7], R, C, D)))) addNeighbor(voxel, v[7], 7, plane, neighbors);
            break;
    }
}

int findNextLevelVoxels(int step, int plane, __private Candidate curve[8], __private const int3 curve3D[2 * MAX_CURVE_LENGTH + 1], int curveLength, __global const uchar* voxels, int R, int C, int D) {
    int3 voxel = curve3D[curveLength + step];
    int absoluteStep = abs(step);
    getNeighborsInPlane(voxel, plane, voxels, R, C, D, curve);

    int count = 0;
    for (int j = 0; j < 8; ++j) {
        for (int k = curveLength - absoluteStep; k <= curveLength + absoluteStep; ++k) {
            if (pointEqual(curve3D[k], curve[j].point)) {
                curve[j] = invalidCandidate();
                break;
            }
        }
        if (!isInvalidCandidate(curve[j])) ++count;
        if (count > 3) break;
    }
    return !(count == 0 || count > 3);
}

int computeCurvatureOfDigitalCurve2D(int plane, int3 voxel, __private Candidate trailCurve[8], __private Candidate leadCurve[8], __private int3 curve3D[2 * MAX_CURVE_LENGTH + 1], int curveLength, __global const uchar* voxels, int R, int C, int D, __private int* curvature) {
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
            int3 voxel1 = trailCurve[j].point;
            uchar trailChain = trailCurve[j].chainCode;

            for (int k = 0; k < 8; ++k) {
                if (isInvalidCandidate(leadCurve[k])) continue;
                int3 voxel2 = leadCurve[k].point;
                uchar leadChain = leadCurve[k].chainCode < 4 ? leadCurve[k].chainCode + 4 : leadCurve[k].chainCode - 4;

                int distX12 = max(max(abs(voxel1.x - voxel2.x), abs(voxel1.y - voxel2.y)), abs(voxel1.z - voxel2.z));
                int distX10 = max(max(abs(voxel1.x - voxel.x), abs(voxel1.y - voxel.y)), abs(voxel1.z - voxel.z));
                int distX20 = max(max(abs(voxel.x - voxel2.x), abs(voxel.y - voxel2.y)), abs(voxel.z - voxel2.z));
                if ((distX10 == 1 || distX20 == 1) && i != 1) continue;
                if ((distX10 != 1 || distX20 != 1) && distX12 <= 1) continue;

                int diff = abs((int)leadChain - (int)trailChain);
                diff = min(diff, 8 - diff);
                if (diff < minDiff) {
                    trailChainCode = trailChain;
                    leadChainCode = leadChain;
                    minDiff = diff;
                    curve3D[curveLength - i] = voxel1;
                    curve3D[curveLength + i] = voxel2;
                }
            }
        }

        if (trailChainCode == INVALID_CHAIN_CODE || leadChainCode == INVALID_CHAIN_CODE) break;

        if (prevTrailChainCode != INVALID_CHAIN_CODE && prevLeadChainCode != INVALID_CHAIN_CODE) {
            int tmp = abs((int)trailChainCode - (int)leadChainCode);
            tmp = min(tmp, 8 - tmp);
            int tmp1 = abs((int)trailChainCode - (int)prevLeadChainCode);
            tmp1 = min(tmp1, 8 - tmp1);
            tmp = min(tmp, tmp1);
            tmp1 = abs((int)prevTrailChainCode - (int)leadChainCode);
            tmp1 = min(tmp1, 8 - tmp1);
            tmp = min(tmp, tmp1);
            tmp1 = abs((int)prevTrailChainCode - (int)prevLeadChainCode);
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

int computeCurvatureOfDigitalCurve3D(int3 voxel, int plane, int curveLength, __global const uchar* voxels, int R, int C, int D) {
    int3 curve3D[2 * MAX_CURVE_LENGTH + 1];
    curve3D[curveLength] = voxel;

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

int computeCurvatureAtVoxel(int3 voxel, int curveLength, __global const uchar* voxels, int R, int C, int D) {
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

inline int innerSpaceVoxelID(int i, int j, int k, int R, int C, int D, int plane) {
    int id = 0;
    int W = 0;
    switch (plane) {
        case 0: id = i * D + j * R * D; W = 1;     break;
        case 1: id = i * C * D + j;     W = C;     break;
        default: id = i + j * R;        W = R * C; break;
    }
    return id + k * W;
}

__kernel void computeInnerSpaceVoxels(__global uchar* voxels, int R, int C, int D, int plane) {
    const int x = (int)get_global_id(0);
    const int y = (int)get_global_id(1);
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
            voxels[id] = (uchar)(voxels[id] + 2);
        }
    }

    if ((count & 1) == 1) {
        --z;
        while (z >= 0 && voxels[innerSpaceVoxelID(x, y, z, R, C, D, plane)] != 1) {
            const int id = innerSpaceVoxelID(x, y, z, R, C, D, plane);
            voxels[id] = (uchar)(voxels[id] - 2);
            --z;
        }
    }
}

__kernel void markInteriorVoxels(__global uchar* voxels, int R, int C, int D) {
    const int x = (int)get_global_id(0);
    const int y = (int)get_global_id(1);
    const int z = (int)get_global_id(2);
    if (x >= R || y >= C || z >= D) return;

    const int id = getVoxelID(x, y, z, R, C);
    if (voxels[id] == 2 || voxels[id] == 4) voxels[id] = 0;
    else if (voxels[id] == 6) voxels[id] = 2;
}

__kernel void computeFrontierVoxels(__global uchar* voxels, int R, int C, int D) {
    const int x = (int)get_global_id(0);
    const int y = (int)get_global_id(1);
    const int z = (int)get_global_id(2);
    if (x >= R || y >= C || z >= D) return;

    const int id = getVoxelID(x, y, z, R, C);
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


__kernel void estimateCurvature(__global const uchar* voxels,
                                __global int* curvatures,
                                __global const int* surfaceVoxelIds,
                                int curveLength,
                                int R,
                                int C,
                                int D) {
    const int surfaceIndex = (int)get_global_id(0);
    const int id = surfaceVoxelIds[surfaceIndex];

    if (curveLength > MAX_CURVE_LENGTH || voxels[id] != 1) {
        curvatures[id] = INT_MAX_VALUE;
        return;
    }

    const int slice = R * C;
    const int z = id / slice;
    const int rem = id - z * slice;
    const int y = rem / R;
    const int x = rem - y * R;
    curvatures[id] = computeCurvatureAtVoxel((int3)(x, y, z), curveLength, voxels, R, C, D);
}
