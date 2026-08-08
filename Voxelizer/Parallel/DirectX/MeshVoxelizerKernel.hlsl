// Experimental DirectX entry shader. Compile this file to test the isolated
// Exp DSS/triangle implementation without changing the working shader set.

#define MAX_ARRAY_SIZE 128
#ifndef THREADS_PER_GROUP
#define THREADS_PER_GROUP 32
#endif

StructuredBuffer<int> facesBuf : register(t0);
StructuredBuffer<int> vertsBuf : register(t1);
RWStructuredBuffer<uint> voxelsBuf : register(u0);
RWStructuredBuffer<uint> totalBuf : register(u1);
RWStructuredBuffer<uint> debugBuf : register(u2);

cbuffer VoxelizerParams : register(b0) {
    int numFaces;
    int R;
    int C;
    int D;
    int xmin;
    int ymin;
    int zmin;
    int _padding;
};

void setVoxel(int voxelIndex) {
    const uint word = (uint)voxelIndex >> 5u;
    const uint bit = (uint)voxelIndex & 31u;
    uint previousValue;
    InterlockedOr(voxelsBuf[word], 1u << bit, previousValue);
}

#include "DSSCreator.hlsl"
#include "TriangleVoxelizer.hlsl"

[numthreads(THREADS_PER_GROUP, 1, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID) {
    const uint faceId = dispatchThreadId.x;
    if (faceId >= (uint)numFaces) return;

    uint previousTotal;
    InterlockedAdd(totalBuf[0], 1u, previousTotal);

    const int v1 = facesBuf[faceId * 3u];
    const int v2 = facesBuf[faceId * 3u + 1u];
    const int v3 = facesBuf[faceId * 3u + 2u];
    const int3 p1 = int3(vertsBuf[v1 * 3], vertsBuf[v1 * 3 + 1], vertsBuf[v1 * 3 + 2]);
    const int3 p2 = int3(vertsBuf[v2 * 3], vertsBuf[v2 * 3 + 1], vertsBuf[v2 * 3 + 2]);
    const int3 p3 = int3(vertsBuf[v3 * 3], vertsBuf[v3 * 3 + 1], vertsBuf[v3 * 3 + 2]);

    voxelizeTriangle(p1, p2, p3, faceId);
}