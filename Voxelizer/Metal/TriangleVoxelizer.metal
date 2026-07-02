#include <metal_stdlib>
using namespace metal;

#define MAX_ARRAY_SIZE 1024

// Update min/max for triangle edge pixels
void updateMinMax(thread int2* points, int numPoints, int2 yBound, thread int2* minmax) {
    for (int i = 0; i < numPoints; i++) {
        int temp = points[i].y - yBound.x;
        if (temp >= 0 && temp <= yBound.y - yBound.x) {
            if (minmax[temp].x > points[i].x) minmax[temp].x = points[i].x;
            if (minmax[temp].y < points[i].x) minmax[temp].y = points[i].x;
        }
    }
}

// Compute triangle edge pixels
void computeTriEdgePixels(int2 p1, int2 p2, int2 p3, thread int2* minmax) {
    int2 yBound;
    yBound.x = min(min(p1.y, p2.y), p3.y);
    yBound.y = max(max(p1.y, p2.y), p3.y);
    
    for (int i = 0; i <= yBound.y - yBound.x; i++) {
        minmax[i] = int2(INT_MAX, INT_MIN);
    }
    
    thread int2 points[MAX_ARRAY_SIZE];
    int numPoints;

    rasterizeSegment(p1, p2, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    rasterizeSegment(p2, p3, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    rasterizeSegment(p1, p3, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);
}

// Main voxelize triangle function
void voxelizeTriangle(int3 p1, int3 p2, int3 p3, device atomic_uint* voxels, 
                      int3 dim, int3 minBound) {
    int3 p12 = int3(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    int3 p13 = int3(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    
    float4 plane;
    plane.x = (float)(p12.y * p13.z - p13.y * p12.z);
    plane.y = (float)(p13.x * p12.z - p12.x * p13.z);
    plane.z = (float)(p12.x * p13.y - p13.x * p12.y);
    plane.w = -(plane.x * (float)p1.x + plane.y * (float)p1.y + plane.z * (float)p1.z);
    
    // Degenerate triangle
    if (plane.x == 0 && plane.y == 0 && plane.z == 0) {
        if (p1.x > p2.x || p1.y > p2.y || p1.z > p2.z) swap3(p1, p2);
        if (p1.x > p3.x || p1.y > p3.y || p1.z > p3.z) swap3(p1, p3);
        if (p2.x > p3.x || p2.y > p3.y || p2.z > p3.z) swap3(p2, p3);
        voxelizeSegment(p1, p3, minBound, dim, voxels);
        return;
    }
    
    float maxDim = max(max(abs(plane.x), abs(plane.y)), abs(plane.z));
    int2 _p1, _p2, _p3;
    float4 _plane;
    int axis;
    
    if (maxDim == abs(plane.x)) {
        _p1 = p1.yz;
        _p2 = p2.yz;
        _p3 = p3.yz;
        _plane = float4(plane.y, plane.z, plane.w, plane.x);
        axis = 1;
    } else if (maxDim == abs(plane.y)) {
        _p1 = p1.xz;
        _p2 = p2.xz;
        _p3 = p3.xz;
        _plane = float4(plane.x, plane.z, plane.w, plane.y);
        axis = 2;
    } else {
        _p1 = p1.xy;
        _p2 = p2.xy;
        _p3 = p3.xy;
        _plane = float4(plane.x, plane.y, plane.w, plane.z);
        axis = 3;
    }
    
    thread int2 boundaryPixels[MAX_ARRAY_SIZE];
    computeTriEdgePixels(_p1, _p2, _p3, boundaryPixels);
    
    int2 yBound;
    yBound.x = min(min(_p1.y, _p2.y), _p3.y);
    yBound.y = max(max(_p1.y, _p2.y), _p3.y);
    
    for (int y = 0; y <= yBound.y - yBound.x; y++) {
        for (int x = boundaryPixels[y].x; x <= boundaryPixels[y].y; x++) {
            float z = ((float)maxDim / 2.0 - _plane.x * (float)x - 
                      _plane.y * (float)(y + yBound.x) - _plane.z) / _plane.w;
            int zz = _plane.w < 0 ? (int)ceil(z) : (int)floor(z);
            int i, j, k;
            
            if (axis == 1) { i = zz; j = x; k = y + yBound.x; }
            else if (axis == 2) { i = x; j = zz; k = y + yBound.x; }
            else { i = x; j = y + yBound.x; k = zz; }
            
            int voxelX = i - minBound.x;
            int voxelY = j - minBound.y;
            int voxelZ = k - minBound.z;
            
            if (voxelX >= 0 && voxelX < dim.x && voxelY >= 0 && voxelY < dim.y && 
                voxelZ >= 0 && voxelZ < dim.z) {
                int voxelIndex = voxelX + voxelY * dim.x + voxelZ * dim.x * dim.y;
                atomic_store_explicit(&voxels[voxelIndex], 1u, memory_order_relaxed);
            }
        }
    }
}
