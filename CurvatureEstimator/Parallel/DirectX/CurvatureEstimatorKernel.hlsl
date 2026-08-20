#ifndef THREADS_PER_GROUP
#define THREADS_PER_GROUP 64
#endif

#ifndef MAX_CURVE_LENGTH
#define MAX_CURVE_LENGTH 32
#endif

#ifdef DX_CURVATURE_ESTIMATE_ONLY
#define EFFECTIVE_CURVE_LENGTH MAX_CURVE_LENGTH
#else
#define EFFECTIVE_CURVE_LENGTH curveLength
#endif

#define INT_MAX_VALUE 2147483647
#define INT_MIN_VALUE (-2147483647 - 1)
#define INVALID_CHAIN_CODE 255
#define INVALID_VOXEL_ID -1

cbuffer CurvatureParams : register(b0) {
    int operation;
    int width;
    int height;
    int depth;
    int plane;
    int curveLength;
    int surfaceVoxelCount;
    int padding;
};

RWByteAddressBuffer voxels : register(u0);
RWStructuredBuffer<int> curvatures : register(u1);
RWStructuredBuffer<int> planeCurvatures : register(u2);
StructuredBuffer<int> surfaceVoxelIds : register(t0);

#define VOXELS_PER_WORD 10u
#define VOXEL_BITS 3u
#define VOXEL_MASK 0x7u

uint loadVoxel(int voxelID) {
    const uint index = uint(voxelID);
    const uint wordIndex = index / VOXELS_PER_WORD;
    return (voxels.Load(wordIndex * 4u) >>
            ((index % VOXELS_PER_WORD) * VOXEL_BITS)) & VOXEL_MASK;
}

void storeVoxel(int voxelID, uint value) {
    const uint index = uint(voxelID);
    const uint wordIndex = index / VOXELS_PER_WORD;
    const uint byteOffset = wordIndex * 4u;
    const uint shift = (index % VOXELS_PER_WORD) * VOXEL_BITS;
    const uint mask = VOXEL_MASK << shift;
    uint expected = voxels.Load(byteOffset);
    uint observed;
    [allow_uav_condition]
    for (;;) {
        const uint replacement = (expected & ~mask) | ((value & VOXEL_MASK) << shift);
        voxels.InterlockedCompareExchange(byteOffset, expected, replacement, observed);
        if (observed == expected) return;
        expected = observed;
    }
}

#ifndef DX_CURVATURE_PREPROCESS_ONLY
// Candidate data is kept in parallel scalar arrays instead of struct/int2 arrays.
// This preserves the full 32-bit voxel ID range while avoiding FXC's expensive
// array-of-struct/vector indexable-temp lowering.
static int gCurve3D[2 * MAX_CURVE_LENGTH + 1];
#endif

int getVoxelID(int3 voxelCoord, int R, int C) {
    return voxelCoord.x + voxelCoord.y * R + voxelCoord.z * R * C;
}

int3 getCoordinates(int voxelID, int R, int C) {
    int t = voxelID % (R * C);
    return int3(t % R, t / R, voxelID / (R * C));
}

bool inGrid(int x, int y, int z, int R, int C, int D) {
    return x >= 0 && x < R && y >= 0 && y < C && z >= 0 && z < D;
}

#ifndef DX_CURVATURE_PREPROCESS_ONLY
void setInvalidCandidate(inout int voxelID, inout int chainCode) {
    voxelID = INVALID_VOXEL_ID;
    chainCode = INVALID_CHAIN_CODE;
}

bool isInvalidCandidate(int voxelID, int chainCode) {
    return chainCode == INVALID_CHAIN_CODE || voxelID == INVALID_VOXEL_ID;
}
#endif

bool isSurface(int3 voxelCoord, int R, int C, int D) {
    return inGrid(voxelCoord.x, voxelCoord.y, voxelCoord.z, R, C, D) &&
           loadVoxel(getVoxelID(voxelCoord, R, C)) == 1u;
}

bool isInterior(int3 voxelCoord, int R, int C, int D) {
    return inGrid(voxelCoord.x, voxelCoord.y, voxelCoord.z, R, C, D) &&
           loadVoxel(getVoxelID(voxelCoord, R, C)) == 2u;
}

#ifndef DX_CURVATURE_PREPROCESS_ONLY
int getChainCode(int2 neighbor, int2 voxel) {
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

void addNeighbor(int3 voxel, int3 neighbor, int neighborIndex, int planeIndex,
                 inout int neighborVoxelIds[8], inout int neighborChainCodes[8], int R, int C) {
    neighborVoxelIds[neighborIndex] = getVoxelID(neighbor, R, C);
    switch (planeIndex / 3) {
        case 0:
            neighborChainCodes[neighborIndex] = getChainCode(int2(neighbor.x, neighbor.y), int2(voxel.x, voxel.y));
            break;
        case 1:
            neighborChainCodes[neighborIndex] = getChainCode(int2(neighbor.y, neighbor.z), int2(voxel.y, voxel.z));
            break;
        case 2:
            neighborChainCodes[neighborIndex] = getChainCode(int2(neighbor.z, neighbor.x), int2(voxel.z, voxel.x));
            break;
    }
}

void getNeighborsInPlane(int3 voxel, int planeIndex, int R, int C, int D,
                         out int neighborVoxelIds[8], out int neighborChainCodes[8]) {
    [unroll]
    for (int n = 0; n < 8; ++n) setInvalidCandidate(neighborVoxelIds[n], neighborChainCodes[n]);

    const int x = voxel.x;
    const int y = voxel.y;
    const int z = voxel.z;
    const int xmin = 0, xmax = R - 1;
    const int ymin = 0, ymax = C - 1;
    const int zmin = 0, zmax = D - 1;

    switch (planeIndex) {
        case 0:
            if (x != xmax && isSurface(int3(x + 1, y, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && isSurface(int3(x - 1, y, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymax && isSurface(int3(x, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymin && isSurface(int3(x, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymax && isSurface(int3(x + 1, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y + 1, z), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymin && isSurface(int3(x + 1, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y - 1, z), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymax && isSurface(int3(x - 1, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y + 1, z), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymin && isSurface(int3(x - 1, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y - 1, z), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
        case 1:
            if (y != ymax && (isSurface(int3(x, y + 1, z), R, C, D) || (((x != xmin) && isSurface(int3(x - 1, y + 1, z), R, C, D)) || ((x != xmax) && isSurface(int3(x + 1, y + 1, z), R, C, D))) && (((z != zmin) && isSurface(int3(x, y + 1, z - 1), R, C, D)) || ((z != zmax) && isSurface(int3(x, y + 1, z + 1), R, C, D))) && !isInterior(int3(x, y + 1, z), R, C, D))) addNeighbor(voxel, int3(x, y + 1, z), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymin && (isSurface(int3(x, y - 1, z), R, C, D) || (((x != xmin) && isSurface(int3(x - 1, y - 1, z), R, C, D)) || ((x != xmax) && isSurface(int3(x + 1, y - 1, z), R, C, D))) && (((z != zmin) && isSurface(int3(x, y - 1, z - 1), R, C, D)) || ((z != zmax) && isSurface(int3(x, y - 1, z + 1), R, C, D))) && !isInterior(int3(x, y - 1, z), R, C, D))) addNeighbor(voxel, int3(x, y - 1, z), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && z != zmin && isSurface(int3(x + 1, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z - 1), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && z != zmax && isSurface(int3(x - 1, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z + 1), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(int3(x, y + 1, z - 1), R, C, D) && isSurface(int3(x + 1, y + 1, z), R, C, D) && !isInterior(int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z - 1), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(int3(x, y + 1, z + 1), R, C, D) && isSurface(int3(x - 1, y + 1, z), R, C, D) && !isInterior(int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z + 1), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(int3(x, y - 1, z - 1), R, C, D) && isSurface(int3(x + 1, y - 1, z), R, C, D) && !isInterior(int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z - 1), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(int3(x, y - 1, z + 1), R, C, D) && isSurface(int3(x - 1, y - 1, z), R, C, D) && !isInterior(int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z + 1), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
        case 2:
            if (y != ymax && (isSurface(int3(x, y + 1, z), R, C, D) || (((x != xmin) && isSurface(int3(x - 1, y + 1, z), R, C, D)) || ((x != xmax) && isSurface(int3(x + 1, y + 1, z), R, C, D))) && (((z != zmin) && isSurface(int3(x, y + 1, z - 1), R, C, D)) || ((z != zmax) && isSurface(int3(x, y + 1, z + 1), R, C, D))) && !isInterior(int3(x, y + 1, z), R, C, D))) addNeighbor(voxel, int3(x, y + 1, z), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymin && (isSurface(int3(x, y - 1, z), R, C, D) || (((x != xmin) && isSurface(int3(x - 1, y - 1, z), R, C, D)) || ((x != xmax) && isSurface(int3(x + 1, y - 1, z), R, C, D))) && (((z != zmin) && isSurface(int3(x, y - 1, z - 1), R, C, D)) || ((z != zmax) && isSurface(int3(x, y - 1, z + 1), R, C, D))) && !isInterior(int3(x, y - 1, z), R, C, D))) addNeighbor(voxel, int3(x, y - 1, z), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && z != zmax && isSurface(int3(x + 1, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z + 1), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && z != zmin && isSurface(int3(x - 1, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z - 1), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(int3(x, y + 1, z + 1), R, C, D) && isSurface(int3(x + 1, y + 1, z), R, C, D) && !isInterior(int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z + 1), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(int3(x, y - 1, z + 1), R, C, D) && isSurface(int3(x + 1, y - 1, z), R, C, D) && !isInterior(int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z + 1), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(int3(x - 1, y + 1, z), R, C, D) && isSurface(int3(x, y + 1, z - 1), R, C, D) && !isInterior(int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z - 1), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(int3(x, y - 1, z - 1), R, C, D) && isSurface(int3(x - 1, y - 1, z), R, C, D) && !isInterior(int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z - 1), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
        case 3:
            if (y != ymax && isSurface(int3(x, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymin && isSurface(int3(x, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (z != zmax && isSurface(int3(x, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y, z + 1), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (z != zmin && isSurface(int3(x, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y, z - 1), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymax && z != zmax && isSurface(int3(x, y + 1, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z + 1), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymax && z != zmin && isSurface(int3(x, y + 1, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z - 1), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymin && z != zmax && isSurface(int3(x, y - 1, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z + 1), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymin && z != zmin && isSurface(int3(x, y - 1, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z - 1), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
        case 4:
            if (z != zmax && (isSurface(int3(x, y, z + 1), R, C, D) || (((x != xmin) && isSurface(int3(x - 1, y, z + 1), R, C, D)) || ((x != xmax) && isSurface(int3(x + 1, y, z + 1), R, C, D))) && (((y != ymin) && isSurface(int3(x, y - 1, z + 1), R, C, D)) || ((y != ymax) && isSurface(int3(x, y + 1, z + 1), R, C, D))) && !isInterior(int3(x, y, z + 1), R, C, D))) addNeighbor(voxel, int3(x, y, z + 1), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (z != zmin && (isSurface(int3(x, y, z - 1), R, C, D) || (((x != xmin) && isSurface(int3(x - 1, y, z - 1), R, C, D)) || ((x != xmax) && isSurface(int3(x + 1, y, z - 1), R, C, D))) && (((y != ymin) && isSurface(int3(x, y - 1, z - 1), R, C, D)) || ((y != ymax) && isSurface(int3(x, y + 1, z - 1), R, C, D))) && !isInterior(int3(x, y, z - 1), R, C, D))) addNeighbor(voxel, int3(x, y, z - 1), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymax && isSurface(int3(x - 1, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y + 1, z), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymin && isSurface(int3(x + 1, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y - 1, z), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(int3(x, y + 1, z + 1), R, C, D) && isSurface(int3(x - 1, y, z + 1), R, C, D) && !isInterior(int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z + 1), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(int3(x, y + 1, z - 1), R, C, D) && isSurface(int3(x - 1, y, z - 1), R, C, D) && !isInterior(int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z - 1), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(int3(x, y - 1, z + 1), R, C, D) && isSurface(int3(x + 1, y, z + 1), R, C, D) && !isInterior(int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z + 1), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(int3(x, y - 1, z - 1), R, C, D) && isSurface(int3(x + 1, y, z - 1), R, C, D) && !isInterior(int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z - 1), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
        case 5:
            if (z != zmax && (isSurface(int3(x, y, z + 1), R, C, D) || (((x != xmin) && isSurface(int3(x - 1, y, z + 1), R, C, D)) || ((x != xmax) && isSurface(int3(x + 1, y, z + 1), R, C, D))) && (((y != ymin) && isSurface(int3(x, y - 1, z + 1), R, C, D)) || ((y != ymax) && isSurface(int3(x, y + 1, z + 1), R, C, D))) && !isInterior(int3(x, y, z + 1), R, C, D))) addNeighbor(voxel, int3(x, y, z + 1), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (z != zmin && (isSurface(int3(x, y, z - 1), R, C, D) || (((x != xmin) && isSurface(int3(x - 1, y, z - 1), R, C, D)) || ((x != xmax) && isSurface(int3(x + 1, y, z - 1), R, C, D))) && (((y != ymin) && isSurface(int3(x, y - 1, z - 1), R, C, D)) || ((y != ymax) && isSurface(int3(x, y + 1, z - 1), R, C, D))) && !isInterior(int3(x, y, z - 1), R, C, D))) addNeighbor(voxel, int3(x, y, z - 1), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymax && isSurface(int3(x + 1, y + 1, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y + 1, z), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymin && isSurface(int3(x - 1, y - 1, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y - 1, z), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(int3(x, y + 1, z + 1), R, C, D) && isSurface(int3(x + 1, y, z + 1), R, C, D) && !isInterior(int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z + 1), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(int3(x + 1, y, z - 1), R, C, D) && isSurface(int3(x, y + 1, z - 1), R, C, D) && !isInterior(int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z - 1), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(int3(x, y - 1, z + 1), R, C, D) && isSurface(int3(x - 1, y, z + 1), R, C, D) && !isInterior(int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z + 1), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(int3(x, y - 1, z - 1), R, C, D) && isSurface(int3(x - 1, y, z - 1), R, C, D) && !isInterior(int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z - 1), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
        case 6:
            if (z != zmax && isSurface(int3(x, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y, z + 1), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (z != zmin && isSurface(int3(x, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y, z - 1), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && isSurface(int3(x + 1, y, z), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && isSurface(int3(x - 1, y, z), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && z != zmax && isSurface(int3(x + 1, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z + 1), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && z != zmax && isSurface(int3(x - 1, y, z + 1), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z + 1), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && z != zmin && isSurface(int3(x + 1, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x + 1, y, z - 1), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && z != zmin && isSurface(int3(x - 1, y, z - 1), R, C, D)) addNeighbor(voxel, int3(x - 1, y, z - 1), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
        case 7:
            if (x != xmax && (isSurface(int3(x + 1, y, z), R, C, D) || (((y != ymin) && isSurface(int3(x + 1, y - 1, z), R, C, D)) || ((y != ymax) && isSurface(int3(x + 1, y + 1, z), R, C, D))) && (((z != zmin) && isSurface(int3(x + 1, y, z - 1), R, C, D)) || ((z != zmax) && isSurface(int3(x + 1, y, z + 1), R, C, D))) && !isInterior(int3(x + 1, y, z), R, C, D))) addNeighbor(voxel, int3(x + 1, y, z), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && (isSurface(int3(x - 1, y, z), R, C, D) || (((y != ymin) && isSurface(int3(x - 1, y - 1, z), R, C, D)) || ((y != ymax) && isSurface(int3(x - 1, y + 1, z), R, C, D))) && (((z != zmin) && isSurface(int3(x - 1, y, z - 1), R, C, D)) || ((z != zmax) && isSurface(int3(x - 1, y, z + 1), R, C, D))) && !isInterior(int3(x - 1, y, z), R, C, D))) addNeighbor(voxel, int3(x - 1, y, z), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymin && z != zmax && isSurface(int3(x, y - 1, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z + 1), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymax && z != zmin && isSurface(int3(x, y + 1, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z - 1), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymin && z != zmax && (isSurface(int3(x + 1, y - 1, z + 1), R, C, D) || (isSurface(int3(x + 1, y, z + 1), R, C, D) && isSurface(int3(x + 1, y - 1, z), R, C, D) && !isInterior(int3(x + 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z + 1), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymin && z != zmax && (isSurface(int3(x - 1, y - 1, z + 1), R, C, D) || (isSurface(int3(x - 1, y, z + 1), R, C, D) && isSurface(int3(x - 1, y - 1, z), R, C, D) && !isInterior(int3(x - 1, y - 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z + 1), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymax && z != zmin && (isSurface(int3(x + 1, y + 1, z - 1), R, C, D) || (isSurface(int3(x + 1, y, z - 1), R, C, D) && isSurface(int3(x + 1, y + 1, z), R, C, D) && !isInterior(int3(x + 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z - 1), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymax && z != zmin && (isSurface(int3(x - 1, y + 1, z - 1), R, C, D) || (isSurface(int3(x - 1, y, z - 1), R, C, D) && isSurface(int3(x - 1, y + 1, z), R, C, D) && !isInterior(int3(x - 1, y + 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z - 1), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
        case 8:
            if (x != xmax && (isSurface(int3(x + 1, y, z), R, C, D) || (((y != ymin) && isSurface(int3(x + 1, y - 1, z), R, C, D)) || ((y != ymax) && isSurface(int3(x + 1, y + 1, z), R, C, D))) && (((z != zmin) && isSurface(int3(x + 1, y, z - 1), R, C, D)) || ((z != zmax) && isSurface(int3(x + 1, y, z + 1), R, C, D))) && !isInterior(int3(x + 1, y, z), R, C, D))) addNeighbor(voxel, int3(x + 1, y, z), 0, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && (isSurface(int3(x - 1, y, z), R, C, D) || (((y != ymin) && isSurface(int3(x - 1, y - 1, z), R, C, D)) || ((y != ymax) && isSurface(int3(x - 1, y + 1, z), R, C, D))) && (((z != zmin) && isSurface(int3(x - 1, y, z - 1), R, C, D)) || ((z != zmax) && isSurface(int3(x - 1, y, z + 1), R, C, D))) && !isInterior(int3(x - 1, y, z), R, C, D))) addNeighbor(voxel, int3(x - 1, y, z), 1, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymax && z != zmax && isSurface(int3(x, y + 1, z + 1), R, C, D)) addNeighbor(voxel, int3(x, y + 1, z + 1), 2, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (y != ymin && z != zmin && isSurface(int3(x, y - 1, z - 1), R, C, D)) addNeighbor(voxel, int3(x, y - 1, z - 1), 3, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymax && z != zmax && (isSurface(int3(x + 1, y + 1, z + 1), R, C, D) || (isSurface(int3(x + 1, y, z + 1), R, C, D) && isSurface(int3(x + 1, y + 1, z), R, C, D) && !isInterior(int3(x + 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y + 1, z + 1), 4, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmax && y != ymin && z != zmin && (isSurface(int3(x + 1, y - 1, z - 1), R, C, D) || (isSurface(int3(x + 1, y, z - 1), R, C, D) && isSurface(int3(x + 1, y - 1, z), R, C, D) && !isInterior(int3(x + 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x + 1, y - 1, z - 1), 5, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymax && z != zmax && (isSurface(int3(x - 1, y + 1, z + 1), R, C, D) || (isSurface(int3(x - 1, y, z + 1), R, C, D) && isSurface(int3(x - 1, y + 1, z), R, C, D) && !isInterior(int3(x - 1, y + 1, z + 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y + 1, z + 1), 6, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            if (x != xmin && y != ymin && z != zmin && (isSurface(int3(x - 1, y - 1, z - 1), R, C, D) || (isSurface(int3(x - 1, y, z - 1), R, C, D) && isSurface(int3(x - 1, y - 1, z), R, C, D) && !isInterior(int3(x - 1, y - 1, z - 1), R, C, D)))) addNeighbor(voxel, int3(x - 1, y - 1, z - 1), 7, planeIndex, neighborVoxelIds, neighborChainCodes, R, C);
            break;
    }
}

bool findNextLevelVoxels(int step, int planeIndex,
                         inout int curveVoxelIds[8], inout int curveChainCodes[8],
                         int R, int C, int D) {
    int3 voxel = getCoordinates(gCurve3D[EFFECTIVE_CURVE_LENGTH + step], R, C);
    int absoluteStep = abs(step);
    getNeighborsInPlane(voxel, planeIndex, R, C, D, curveVoxelIds, curveChainCodes);

    int count = 0;
    [loop]
    for (int candidateIndex = 0; candidateIndex < 8; ++candidateIndex) {
        [loop]
        for (int curveIndex = EFFECTIVE_CURVE_LENGTH - absoluteStep; curveIndex <= EFFECTIVE_CURVE_LENGTH + absoluteStep; ++curveIndex) {
            if (gCurve3D[curveIndex] == curveVoxelIds[candidateIndex]) {
                setInvalidCandidate(curveVoxelIds[candidateIndex], curveChainCodes[candidateIndex]);
                break;
            }
        }
        if (!isInvalidCandidate(curveVoxelIds[candidateIndex], curveChainCodes[candidateIndex])) ++count;
        if (count > 3) break;
    }
    return !(count == 0 || count > 3);
}

bool computeCurvatureOfDigitalCurve2D(int planeIndex, int3 voxel,
                                      inout int trailVoxelIds[8], inout int trailChainCodes[8],
                                      inout int leadVoxelIds[8], inout int leadChainCodes[8],
                                      int R, int C, int D, inout int curvature) {
    int prevTrailChainCode = INVALID_CHAIN_CODE;
    int prevLeadChainCode = INVALID_CHAIN_CODE;
    int curvatureSum = 0;
    int i = 1;

    [loop]
    for (; i <= EFFECTIVE_CURVE_LENGTH; ++i) {
        int trailChainCode = INVALID_CHAIN_CODE;
        int leadChainCode = INVALID_CHAIN_CODE;
        int minDiff = INVALID_CHAIN_CODE;

        [loop]
        for (int j = 0; j < 8; ++j) {
            if (isInvalidCandidate(trailVoxelIds[j], trailChainCodes[j])) continue;
            int3 voxel1 = getCoordinates(trailVoxelIds[j], R, C);
            int trailChain = trailChainCodes[j];

            [loop]
            for (int k = 0; k < 8; ++k) {
                if (isInvalidCandidate(leadVoxelIds[k], leadChainCodes[k])) continue;
                int3 voxel2 = getCoordinates(leadVoxelIds[k], R, C);
                int leadChain = leadChainCodes[k] < 4 ? leadChainCodes[k] + 4 : leadChainCodes[k] - 4;

                int distX12 = max(max(abs(voxel1.x - voxel2.x), abs(voxel1.y - voxel2.y)), abs(voxel1.z - voxel2.z));
                int distX10 = max(max(abs(voxel1.x - voxel.x), abs(voxel1.y - voxel.y)), abs(voxel1.z - voxel.z));
                int distX20 = max(max(abs(voxel.x - voxel2.x), abs(voxel.y - voxel2.y)), abs(voxel.z - voxel2.z));
                if ((distX10 == 1 || distX20 == 1) && i != 1) continue;
                if ((distX10 != 1 || distX20 != 1) && distX12 <= 1) continue;

                int diff = abs(leadChain - trailChain);
                diff = min(diff, 8 - diff);
                if (diff < minDiff) {
                    trailChainCode = trailChain;
                    leadChainCode = leadChain;
                    minDiff = diff;
                    gCurve3D[EFFECTIVE_CURVE_LENGTH - i] = getVoxelID(voxel1, R, C);
                    gCurve3D[EFFECTIVE_CURVE_LENGTH + i] = getVoxelID(voxel2, R, C);
                }
            }
        }

        if (trailChainCode == INVALID_CHAIN_CODE || leadChainCode == INVALID_CHAIN_CODE) break;

        if (prevTrailChainCode != INVALID_CHAIN_CODE && prevLeadChainCode != INVALID_CHAIN_CODE) {
            int tmp = abs(trailChainCode - leadChainCode);
            tmp = min(tmp, 8 - tmp);
            int tmp1 = abs(trailChainCode - prevLeadChainCode);
            tmp1 = min(tmp1, 8 - tmp1);
            tmp = min(tmp, tmp1);
            tmp1 = abs(prevTrailChainCode - leadChainCode);
            tmp1 = min(tmp1, 8 - tmp1);
            tmp = min(tmp, tmp1);
            tmp1 = abs(prevTrailChainCode - prevLeadChainCode);
            tmp1 = min(tmp1, 8 - tmp1);
            tmp = min(tmp, tmp1);
            curvatureSum += tmp;
        }

        prevLeadChainCode = leadChainCode;
        prevTrailChainCode = trailChainCode;

        if (i == EFFECTIVE_CURVE_LENGTH) continue;
        if (!findNextLevelVoxels(-i, planeIndex, trailVoxelIds, trailChainCodes, R, C, D)) break;
        if (!findNextLevelVoxels(i, planeIndex, leadVoxelIds, leadChainCodes, R, C, D)) break;
    }

    if (i != EFFECTIVE_CURVE_LENGTH + 1) return false;
    curvature = min(curvatureSum, curvature);
    return true;
}

int computeCurvatureOfDigitalCurve3D(int3 voxel, int planeIndex, int R, int C, int D) {
    gCurve3D[EFFECTIVE_CURVE_LENGTH] = getVoxelID(voxel, R, C);

    int curvature = INT_MAX_VALUE;
    int headVoxelIds[8];
    int headChainCodes[8];
    int trailVoxelIds[8];
    int trailChainCodes[8];
    int leadVoxelIds[8];
    int leadChainCodes[8];

    getNeighborsInPlane(voxel, planeIndex, R, C, D, headVoxelIds, headChainCodes);
    int count = 0;
    [loop]
    for (int countIndex = 0; countIndex < 8; ++countIndex) {
        if (!isInvalidCandidate(headVoxelIds[countIndex], headChainCodes[countIndex])) ++count;
    }
    if (count == 0 || count > 3) return -1;

    [loop]
    for (int trailHeadIndex = 0; trailHeadIndex < 7; ++trailHeadIndex) {
        if (isInvalidCandidate(headVoxelIds[trailHeadIndex], headChainCodes[trailHeadIndex])) continue;
        [loop]
        for (int leadHeadIndex = trailHeadIndex + 1; leadHeadIndex < 8; ++leadHeadIndex) {
            if (isInvalidCandidate(headVoxelIds[leadHeadIndex], headChainCodes[leadHeadIndex])) continue;

            [unroll]
            for (int resetIndex = 0; resetIndex < 8; ++resetIndex) {
                setInvalidCandidate(trailVoxelIds[resetIndex], trailChainCodes[resetIndex]);
                setInvalidCandidate(leadVoxelIds[resetIndex], leadChainCodes[resetIndex]);
            }
            trailVoxelIds[0] = headVoxelIds[trailHeadIndex];
            trailChainCodes[0] = headChainCodes[trailHeadIndex];
            leadVoxelIds[0] = headVoxelIds[leadHeadIndex];
            leadChainCodes[0] = headChainCodes[leadHeadIndex];

            if (!computeCurvatureOfDigitalCurve2D(planeIndex, voxel,
                                                  trailVoxelIds, trailChainCodes,
                                                  leadVoxelIds, leadChainCodes,
                                                  R, C, D, curvature)) continue;
            if (curvature == 0) return 0;
        }
    }

    return curvature == INT_MAX_VALUE ? -1 : curvature;
}



#endif

#ifndef DX_CURVATURE_ESTIMATE_ONLY
int innerSpaceVoxelID(int i, int j, int k, int R, int C, int D, int planeIndex) {
    int id = 0;
    int W = 0;
    switch (planeIndex) {
        case 0: id = i * D + j * R * D; W = 1;     break;
        case 1: id = i * C * D + j;     W = C;     break;
        default: id = i + j * R;        W = R * C; break;
    }
    return id + k * W;
}

void computeInnerSpaceVoxels(int R, int C, int D, int planeIndex, uint3 dispatchThreadID) {
    const int x = int(dispatchThreadID.x);
    const int y = int(dispatchThreadID.y);
    if (x >= R || y >= C) return;

    int count = 0;
    int z = 0;

    for (; z < D; ++z) {
        const int id = innerSpaceVoxelID(x, y, z, R, C, D, planeIndex);
        if (loadVoxel(id) == 1u) {
            const bool nextIsVoxel = (z != D - 1) && loadVoxel(innerSpaceVoxelID(x, y, z + 1, R, C, D, planeIndex)) == 1u;
            if (z == D - 1 || nextIsVoxel) continue;
            if (!nextIsVoxel) ++count;
        } else if ((count & 1) == 1) {
            storeVoxel(id, loadVoxel(id) + 2u);
        }
    }

    if ((count & 1) == 1) {
        --z;
        while (z >= 0 && loadVoxel(innerSpaceVoxelID(x, y, z, R, C, D, planeIndex)) != 1u) {
            const int id = innerSpaceVoxelID(x, y, z, R, C, D, planeIndex);
            storeVoxel(id, loadVoxel(id) - 2u);
            --z;
        }
    }
}

void markInteriorVoxels(int R, int C, int D, uint3 dispatchThreadID) {
    const int x = int(dispatchThreadID.x);
    const int y = int(dispatchThreadID.y);
    const int z = int(dispatchThreadID.z);
    if (x >= R || y >= C || z >= D) return;

    const int id = getVoxelID(int3(x, y, z), R, C);
    const uint voxelValue = loadVoxel(id);
    if (voxelValue == 2u || voxelValue == 4u) storeVoxel(id, 0u);
    else if (voxelValue == 6u) storeVoxel(id, 2u);
}

void computeFrontierVoxels(int R, int C, int D, uint3 dispatchThreadID) {
    const int x = int(dispatchThreadID.x);
    const int y = int(dispatchThreadID.y);
    const int z = int(dispatchThreadID.z);
    if (x >= R || y >= C || z >= D) return;

    const int id = getVoxelID(int3(x, y, z), R, C);
    if (loadVoxel(id) != 1u) return;

    const int slice = R * C;
    if (x == 0 || loadVoxel(id - 1) == 0u) return;
    if (x == R - 1 || loadVoxel(id + 1) == 0u) return;
    if (y == 0 || loadVoxel(id - R) == 0u) return;
    if (y == C - 1 || loadVoxel(id + R) == 0u) return;
    if (z == 0 || loadVoxel(id - slice) == 0u) return;
    if (z == D - 1 || loadVoxel(id + slice) == 0u) return;
    storeVoxel(id, 2u);
}

#endif

#ifndef DX_CURVATURE_PREPROCESS_ONLY
void combineCurvature(uint3 dispatchThreadID) {
    const int surfaceIndex = int(dispatchThreadID.x);
    if (surfaceIndex >= surfaceVoxelCount) return;

    int minCurvature = INT_MAX_VALUE;
    int maxCurvature = INT_MIN_VALUE;
    const int planeBase = surfaceIndex * 9;
    [unroll]
    for (int planeIndex = 0; planeIndex < 9; ++planeIndex) {
        const int curvature = planeCurvatures[planeBase + planeIndex];
        if (curvature == -1) continue;
        minCurvature = min(minCurvature, curvature);
        maxCurvature = max(maxCurvature, curvature);
    }
    curvatures[surfaceIndex] = (minCurvature == INT_MAX_VALUE || maxCurvature == INT_MIN_VALUE)
        ? INT_MAX_VALUE
        : minCurvature + maxCurvature;
}

void estimateCurvature(uint3 dispatchThreadID) {
    const int surfaceIndex = int(dispatchThreadID.x);
    if (surfaceIndex >= surfaceVoxelCount) return;
    const int voxelID = surfaceVoxelIds[surfaceIndex];

    const int planeIndex = int(dispatchThreadID.y);
    if (planeIndex >= 9) return;

    int curvature = -1;
    if (curveLength == MAX_CURVE_LENGTH && loadVoxel(voxelID) == 1u) {
        const int3 voxel = getCoordinates(voxelID, width, height);
        curvature = computeCurvatureOfDigitalCurve3D(voxel, planeIndex, width, height, depth);
    }
    planeCurvatures[surfaceIndex * 9 + planeIndex] = curvature;
}

#endif

[numthreads(THREADS_PER_GROUP, 1, 1)]
void main(uint3 dispatchThreadID : SV_DispatchThreadID) {
#ifndef DX_CURVATURE_ESTIMATE_ONLY
    if (operation == 0) {
        if (plane == 2) computeInnerSpaceVoxels(width, height, depth, 2, dispatchThreadID);
        else if (plane == 1) computeInnerSpaceVoxels(depth, width, height, 1, dispatchThreadID);
        else computeInnerSpaceVoxels(height, depth, width, 0, dispatchThreadID);
    } else if (operation == 1) {
        markInteriorVoxels(width, height, depth, dispatchThreadID);
    } else if (operation == 2) {
        computeFrontierVoxels(width, height, depth, dispatchThreadID);
    }
#else
    if (operation == 3) {
#ifdef DX_CURVATURE_COMBINE_PASS
        combineCurvature(dispatchThreadID);
#else
        estimateCurvature(dispatchThreadID);
#endif
    }
#endif
#ifndef DX_CURVATURE_PREPROCESS_ONLY
#ifndef DX_CURVATURE_ESTIMATE_ONLY
    else if (operation == 3) {
        estimateCurvature(dispatchThreadID);
    }
#endif
#endif
}
