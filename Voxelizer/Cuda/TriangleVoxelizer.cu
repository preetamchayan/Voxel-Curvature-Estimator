#include "DSSCreator.cu"

__device__ void swap3(int3* a, int3* b) {
    int3 tmp = *a;
    *a = *b;
    *b = tmp;
}

__device__ void updateMinMax(int2* points, int numPoints, int2 yBound, int2* minmax) {
    for (int i = 0; i < numPoints; i++) {
        int temp = points[i].y - yBound.x;
        if (temp >= 0 && temp <= yBound.y - yBound.x) {
            if (minmax[temp].x > points[i].x) minmax[temp].x = points[i].x;
            if (minmax[temp].y < points[i].x) minmax[temp].y = points[i].x;
        }
    }
}

__device__ void computeTriEdgePixels(int2 p1, int2 p2, int2 p3, int2* minmax) {
    int2 yBound;
    yBound.x = min(min(p1.y, p2.y), p3.y);
    yBound.y = max(max(p1.y, p2.y), p3.y);
    // int2 minmax[MAX_ARRAY_SIZE]; // assume max y range MAX_ARRAY_SIZE
    for (int i = 0; i <= yBound.y - yBound.x; i++) {
        minmax[i].x = INT_MAX;
        minmax[i].y = INT_MIN;
    }

    int2 points[MAX_ARRAY_SIZE]; // local array for points
    int numPoints;

    rasterizeDSS(p1, p2, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    rasterizeDSS(p2, p3, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    rasterizeDSS(p1, p3, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);
}

__device__ void voxelizeTriangle(int3 p1, int3 p2, int3 p3, unsigned char* voxels, int3 dim, int3 minBound) {
    int3 p12 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    int3 p13 = {p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

    int4 plane;
    plane.x = p12.y * p13.z - p13.y * p12.z;
    plane.y = p13.x * p12.z - p12.x * p13.z;
    plane.z = p12.x * p13.y - p13.x * p12.y;
    plane.w = -plane.x * p1.x - plane.y * p1.y - plane.z * p1.z;

    // degenerate triangle
    if (plane.x == 0 && plane.y == 0 && plane.z == 0) {
        if (p1.x > p2.x || p1.y > p2.y || p1.z > p2.z) {
            swap3(&p1, &p2);
        }
        if (p1.x > p3.x || p1.y > p3.y || p1.z > p3.z) {
            swap3(&p1, &p3);
        }
        if (p2.x > p3.x || p2.y > p3.y || p2.z > p3.z) {
            swap3(&p2, &p3);
        }
        voxelizeDSS(p1, p3, minBound, dim, voxels);
        return;
    }

    int maxDim = max(max(abs((int)plane.x), abs((int)plane.y)), abs((int)plane.z));
    int2 _p1, _p2, _p3;
    int4 _plane;
    char axis;
    if (maxDim == abs((int)plane.x)) {
        _p1.x = p1.y; _p1.y = p1.z; _p2.x = p2.y; _p2.y = p2.z; _p3.x = p3.y; _p3.y = p3.z;
        _plane = {plane.y, plane.z, plane.w, plane.x};
        axis = 1;
    } else if (maxDim == abs((int)plane.y)) {
        _p1.x = p1.x; _p1.y = p1.z; _p2.x = p2.x; _p2.y = p2.z; _p3.x = p3.x; _p3.y = p3.z;
        _plane = {plane.x, plane.z, plane.w, plane.y};
        axis = 2;
    } else {
        _p1.x = p1.x; _p1.y = p1.y; _p2.x = p2.x; _p2.y = p2.y; _p3.x = p3.x; _p3.y = p3.y;
        _plane = {plane.x, plane.y, plane.w, plane.z};
        axis = 3;
    }
    int2 boundaryPixels[MAX_ARRAY_SIZE];
    computeTriEdgePixels(_p1, _p2, _p3, boundaryPixels);

    int2 yBound;
    yBound.x = min(min(_p1.y, _p2.y), _p3.y);
    yBound.y = max(max(_p1.y, _p2.y), _p3.y);

    for (int y = 0; y <= yBound.y - yBound.x; y++) {
        for (int x = boundaryPixels[y].x; x <= boundaryPixels[y].y; x++) {
            float z = (float)maxDim / 2.0;
            z -= (float)_plane.x * (float)x;
            z -= (float)_plane.y * (float)(y + yBound.x);
            z -= (float)_plane.z;
            z /= (float)_plane.w;
            int zz = _plane.w < 0 ? (int)ceil(z) : (int)floor(z);
            int i, j, k;
            if (axis == 1) { i = zz; j = x; k = y + yBound.x; }
            else if (axis == 2) { i = x; j = zz; k = y + yBound.x; }
            else { i = x; j = y + yBound.x; k = zz; }
            int voxelX = i - minBound.x;
            int voxelY = j - minBound.y;
            int voxelZ = k - minBound.z;
            if (voxelX >= 0 && voxelX < dim.x && voxelY >= 0 && voxelY < dim.y && voxelZ >= 0 && voxelZ < dim.z) {
                int voxelIndex = voxelX + voxelY * dim.x + voxelZ * dim.x * dim.y;
                voxels[voxelIndex] = 1;
            }
        }
    }
}