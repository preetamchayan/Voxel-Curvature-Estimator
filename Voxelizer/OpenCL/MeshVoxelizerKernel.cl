#include "TriangleVoxelizer.clh"

__kernel void voxelize_faces(__global const int* faces, int numFaces,
                             __global const int* intVertices,
                             __global uchar* voxels, int R, int C, int D, int xmin, int ymin, int zmin,
                             __global int *totalSize) {
    int faceId = get_global_id(0);
    if (faceId < 0 || faceId >= numFaces) return;
    atomic_inc(totalSize);

    int v1 = faces[faceId * 3];
    int v2 = faces[faceId * 3 + 1];
    int v3 = faces[faceId * 3 + 2];

    int3 p1 = (int3) (intVertices[v1 * 3], intVertices[v1 * 3 + 1], intVertices[v1 * 3 + 2]);
    int3 p2 = (int3) (intVertices[v2 * 3], intVertices[v2 * 3 + 1], intVertices[v2 * 3 + 2]);
    int3 p3 = (int3) (intVertices[v3 * 3], intVertices[v3 * 3 + 1], intVertices[v3 * 3 + 2]);

    int3 dim = (int3)(R, C, D);
    int3 minBound = (int3)(xmin, ymin, zmin);

    voxelizeTriangle(p1, p2, p3, voxels, dim, minBound);
}