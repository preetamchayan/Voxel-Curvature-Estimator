#include <metal_stdlib>
using namespace metal;

// Helper: Swap two int2 values
void swap2(thread int2& a, thread int2& b) {
    int2 tmp = a;
    a = b;
    b = tmp;
}

// Helper: Swap two int3 values
void swap3(thread int3& a, thread int3& b) {
    int3 tmp = a;
    a = b;
    b = tmp;
}

// Bresenham line drawing
void bresenhamLineDrawing(int2 p1, int2 p2, int2 val, int3 flags,
                          thread int2* points, thread int* numPoints) {
    int p = (p2.y - p1.y);
    int q = (p1.x - p2.x);
    
    if (flags.z == 0) {
        int tmp = p1.x; p1.x = p2.x; p2.x = tmp;
    }
    if (flags.z == 1) {
        int tmp = p1.y; p1.y = p2.y; p2.y = tmp;
    }
    if (flags.z == 2) {
        swap2(p1, p2);
    }
    
    *numPoints = abs(p2.x - p1.x) + 1;
    int f = 2 * p + q;
    int d = 2 * p;
    int dd = 2 * (p + q);
    int i = 0;
    
    while (p1.x <= p2.x) {
        if (flags.y == 0) {
            points[i] = p1;
        } else {
            points[i] = int2(p1.y, p1.x);
        }
        if (f <= 0) {
            if (flags.x == 0) f += d;
            else {
                f += dd;
                p1.y += val.y;
            }
        } else {
            if (flags.x == 0) {
                f += dd;
                p1.y += val.y;
            } else f += d;
        }
        p1.x += val.x;
        i++;
    }
}

// Rasterize a line segment
void rasterizeSegment(int2 p1, int2 p2, thread int2* points, thread int* numPoints) {
    int q = abs(p2.x - p1.x);
    int p = abs(p2.y - p1.y);
    int2 _p1, _p2;
    int2 val;
    int3 flags;
    
    if (p <= q) {
        bool flag = false;
        if (p1.x < p2.x && p1.y < p2.y) { _p1 = p1; _p2 = p2; flag = true; }
        else if (p1.x > p2.x && p1.y > p2.y) { _p1 = p2; _p2 = p1; flag = true; }
        if (flag) {
            val = int2(1, 1);
            flags = int3(0, 0, -1);
        }
    }
    
    if (p > q) {
        bool flag = false;
        if (p1.y < p2.y && p1.x <= p2.x) { _p1 = int2(p2.y, p2.x); _p2 = int2(p1.y, p1.x); flag = true; }
        else if (p1.y > p2.y && p1.x >= p2.x) { _p1 = int2(p1.y, p1.x); _p2 = int2(p2.y, p2.x); flag = true; }
        if (flag) {
            val = int2(1, 1);
            flags = int3(1, 1, 2);
        }
    }
    
    if (p >= q) {
        bool flag = false;
        if (p1.y < p2.y && p1.x > p2.x) { 
            _p1 = int2(p1.y, p2.x); _p2 = int2(p2.y, p1.x); flag = true; 
        }
        else if (p1.y > p2.y && p1.x < p2.x) { 
            _p1 = int2(p2.y, p1.x); _p2 = int2(p1.y, p2.x); flag = true; 
        }
        if (flag) {
            val = int2(1, -1);
            flags = int3(0, 1, 1);
        }
    }
    
    if (p < q) {
        bool flag = false;
        if (p1.x > p2.x && p1.y <= p2.y) { 
            _p1 = int2(p1.x, p2.y); _p2 = int2(p2.x, p1.y); flag = true; 
        }
        else if (p1.x < p2.x && p1.y >= p2.y) { 
            _p1 = int2(p2.x, p1.y); _p2 = int2(p1.x, p2.y); flag = true; 
        }
        if (flag) {
            val = int2(1, -1);
            flags = int3(1, 0, 0);
        }
    }
    
    bresenhamLineDrawing(_p1, _p2, val, flags, points, numPoints);
}

// Voxelize a segment (degenerate triangle)
void voxelizeSegment(int3 p1, int3 p2, int3 minBound, int3 dim, 
                     device atomic_uint* voxels) {
    int3 absPoint;
    absPoint.x = abs(p2.x - p1.x);
    absPoint.y = abs(p2.y - p1.y);
    absPoint.z = abs(p2.z - p1.z);
    int maxDim = max(max(absPoint.x, absPoint.y), absPoint.z);
    
    if (maxDim == 0) {
        int x = p1.x - minBound.x;
        int y = p1.y - minBound.y;
        int z = p1.z - minBound.z;
        if (x >= 0 && x < dim.x && y >= 0 && y < dim.y && z >= 0 && z < dim.z) {
            int voxelIndex = z * dim.x * dim.y + y * dim.x + x;
            atomic_store_explicit(&voxels[voxelIndex], 1u, memory_order_relaxed);
        }
        return;
    }
    
    thread int2 points[MAX_ARRAY_SIZE];
    thread int3 coordinates[MAX_ARRAY_SIZE];
    int numPoints;
    
    if (maxDim == absPoint.x) {
        int2 _p1 = p1.xy;
        int2 _p2 = p2.xy;
        rasterizeSegment(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].y = points[i].y;
        }
        _p1 = p1.xz;
        _p2 = p2.xz;
        rasterizeSegment(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) coordinates[i].z = points[i].y;
    } else if (maxDim == absPoint.y) {
        int2 _p1 = p1.xy;
        int2 _p2 = p2.xy;
        rasterizeSegment(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].y = points[i].y;
        }
        _p1 = p1.yz;
        _p2 = p2.yz;
        rasterizeSegment(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) coordinates[i].z = points[i].y;
    } else {
        int2 _p1 = p1.xz;
        int2 _p2 = p2.xz;
        rasterizeSegment(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].z = points[i].y;
        }
        _p1 = p1.yz;
        _p2 = p2.yz;
        rasterizeSegment(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) coordinates[i].y = points[i].x;
    }
    
    for (int i = 0; i < numPoints; i++) {
        int x = coordinates[i].x - minBound.x;
        int y = coordinates[i].y - minBound.y;
        int z = coordinates[i].z - minBound.z;
        if (x >= 0 && x < dim.x && y >= 0 && y < dim.y && z >= 0 && z < dim.z) {
            int voxelIndex = z * dim.x * dim.y + y * dim.x + x;
            atomic_store_explicit(&voxels[voxelIndex], 1u, memory_order_relaxed);
        }
    }
}
