// Direct3D 11 compute shader for mesh voxelization.
// Mirrors the Vulkan/OpenCL/CUDA path: one thread processes one face.

#define MAX_ARRAY_SIZE 128

StructuredBuffer<int> facesBuf : register(t0);
StructuredBuffer<int> vertsBuf : register(t1);
RWStructuredBuffer<uint> voxelsBuf : register(u0);
RWStructuredBuffer<uint> totalBuf : register(u1);

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
    uint word = (uint)voxelIndex >> 5u;
    uint bit = (uint)voxelIndex & 31u;
    uint previousValue;
    InterlockedOr(voxelsBuf[word], 1u << bit, previousValue);
}

#include "DSSCreator.hlsl"
#include "TriangleVoxelizer.hlsl"

[numthreads(32, 1, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID) {
    uint gid = dispatchThreadId.x;
    if (gid >= (uint)numFaces) {
        return;
    }

    if (gid >= 16)
        return;

    uint previousTotal;
    InterlockedAdd(totalBuf[0], 1u, previousTotal);

    int faceId = (int)gid;
    int v1 = facesBuf[faceId * 3 + 0];
    int v2 = facesBuf[faceId * 3 + 1];
    int v3 = facesBuf[faceId * 3 + 2];

    int3 p1 = int3(vertsBuf[v1 * 3 + 0], vertsBuf[v1 * 3 + 1], vertsBuf[v1 * 3 + 2]);
    int3 p2 = int3(vertsBuf[v2 * 3 + 0], vertsBuf[v2 * 3 + 1], vertsBuf[v2 * 3 + 2]);
    int3 p3 = int3(vertsBuf[v3 * 3 + 0], vertsBuf[v3 * 3 + 1], vertsBuf[v3 * 3 + 2]);

    voxelizeTriangle(p1, p2, p3);
}