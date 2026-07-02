#include <metal_stdlib>
using namespace metal;

#define MAX_ARRAY_SIZE 1024

// Structs to match data layout
struct BBox3i {
    int xmin, ymin, zmin;
    int xmax, ymax, zmax;
};

struct Dimensions3i {
    int width, height, depth;
};

struct Face {
    int v0, v1, v2;
};

// Include shader modules
#include "DSSCreator.metal"
#include "TriangleVoxelizer.metal"

// Main compute kernel
kernel void voxelizeKernel(device const Face* faces [[buffer(0)]],
                           device const packed_int3* vertices [[buffer(1)]],
                           device atomic_uint* voxels [[buffer(2)]],
                           device const int* params [[buffer(3)]],
                           uint gid [[thread_position_in_grid]]) {
    // Extract parameters
    int3 minBound = int3(params[0], params[1], params[2]);
    int3 maxBound = int3(params[3], params[4], params[5]);
    int3 dim = int3(params[6], params[7], params[8]);
    
    uint faceId = gid;
    Face face = faces[faceId];
    int3 p1 = int3(vertices[face.v0]);
    int3 p2 = int3(vertices[face.v1]);
    int3 p3 = int3(vertices[face.v2]);
    
    voxelizeTriangle(p1, p2, p3, voxels, dim, minBound);
}
