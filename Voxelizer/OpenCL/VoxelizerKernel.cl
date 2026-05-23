#define MAX_ARRAY_SIZE 1024
#define SWAP_X(a,b) do { int _t = (a).x; (a).x = (b).x; (b).x = _t; } while(0)
#define SWAP_Y(a,b) do { int _t = (a).y; (a).y = (b).y; (b).y = _t; } while(0)
#define SWAP_Z(a,b) do { int _t = (a).z; (a).z = (b).z; (b).z = _t; } while(0)

inline void swap2(int2* a, int2* b) {
    int2 tmp = *a;
    *a = *b;
    *b = tmp;
}

inline void swap3(int3* a, int3* b) {
    int3 tmp = *a;
    *a = *b;
    *b = tmp;
}

void getPixelDSS2D(int2 p1, int2 p2, int2 val, int3 flag, int2* points, int *numPoints) {
    int p = (p2.y - p1.y);
    int q = (p1.x - p2.x);
    if (flag.z == 0) SWAP_X(p1, p2);
    if (flag.z == 1) SWAP_Y(p1, p2);
    if (flag.z == 2) {
        swap2(&p1, &p2);
    }
    // printf("(x1, y1)=(%d %d), (x2, y2)=(%d %d)\n", p1.x, p1.y, p2.x, p2.y);
    *numPoints = abs(p2.x - p1.x) + 1;
    // printf("numPoints = %d\n", *numPoints);
    int f = 2 * p + q;
    int d = 2 * p;
    int dd = 2 * (p + q);
    int i = 0;
    while (p1.x <= p2.x) {
        if (flag.y == 0) {
            points[i] = p1;
        } else {
            points[i].xy = p1.yx;
        }
        // printf("(%d %d) ", points[i].x, points[i].y);
        if (f <= 0) {
            if (flag.x == 0) f += d;
            else {
                f += dd;
                p1.y += val.y;
            }
        } else {
            if (flag.x == 0) {
                f += dd;
                p1.y += val.y;
            } else f += d;
        }
        p1.x += val.x;
        i++;
    }
    // printf("\n\n");
}

void DSS2D(int2 p1, int2 p2, int2* points, int* numPoints) {
    int q = abs(p2.x - p1.x);
    int p = abs(p2.y - p1.y);
    int2 _p1, _p2;
    // printf("p1=(%d %d), p2=(%d %d)\n", p1.x, p1.y, p2.x, p2.y);
    int2 val;
    int3 flags;
    if (p <= q) { // > 0 & <= 45 && > 180 & <= 225
        bool flag = false;
        if (p1.x < p2.x && p1.y < p2.y) { _p1 = p1; _p2 = p2; flag = true; }
        else if (p1.x > p2.x && p1.y > p2.y) { _p1 = p2; _p2 = p1; flag = true; }
        if(flag) {
            val = (int2)(1, 1);
            flags = (int3)(0, 0, -1);
        }
    }
    if (p > q) { // > 45 & <= 90 && > 225 & <= 270
        bool flag = false;
        if (p1.y < p2.y && p1.x <= p2.x) { _p1.xy = p2.yx;_p2.xy = p1.yx; flag = true; }
        else if (p1.y > p2.y && p1.x >= p2.x) { _p1.xy = p1.yx; _p2.xy = p2.yx; flag = true; }
        if(flag) {
            val = (int2)(1, 1);
            flags = (int3)(1, 1, 2);
        }
    }
    if (p >= q) { // > 90 & <= 135 && > 270 & <= 315
        bool flag = false;
        if (p1.y < p2.y && p1.x > p2.x) { _p1.x = p1.y; _p1.y = p2.x; _p2.x = p2.y; _p2.y = p1.x; flag = true; }
        else if (p1.y > p2.y && p1.x < p2.x) { _p1.x = p2.y; _p1.y = p1.x; _p2.x = p1.y; _p2.y = p2.x; flag = true; }
        if(flag) {
            val = (int2)(1, -1);
            flags = (int3)(0, 1, 1);
        }
    }
    if (p < q) { // > 135 & <= 180 && > 315 & <= 360
        bool flag = false;
        if (p1.x > p2.x && p1.y <= p2.y) { _p1.x = p1.x; _p1.y = p2.y; _p2.x = p2.x; _p2.y = p1.y; flag = true; }
        else if (p1.x < p2.x && p1.y >= p2.y) { _p1.x = p2.x; _p1.y = p1.y; _p2.x = p1.x; _p2.y = p2.y; flag = true; }
        if(flag) {
            val = (int2)(1, -1);
            flags = (int3)(1, 0, 0);
        }
    }
    // printf("_p1=(%d %d), _p2=(%d %d)\n", _p1.x, _p1.y, _p2.x, _p2.y);
    getPixelDSS2D(_p1, _p2, val, flags, points, numPoints);
}

void DSS3D(int3 p1, int3 p2, int3 minBound, int3 dim, __global uchar* voxels) {
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
            voxels[voxelIndex] = 1;
        }
        return;
    }
    int3 coordinates[MAX_ARRAY_SIZE];
    int2 points[MAX_ARRAY_SIZE];
    int numPoints;
    if (maxDim == absPoint.x) { // x coordinates with highest difference
        int2 _p1 = p1.xy;
        int2 _p2 = p2.xy;
        DSS2D(_p1, _p2, points, &numPoints);
        for(int i = 0; i < numPoints; i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].y = points[i].y;
        }
        _p1.xy = p1.xz;
        _p2.xy = p2.xz;
        DSS2D(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) coordinates[i].z = points[i].y;
    } else if (maxDim == absPoint.y) { // y coordinates with highest difference
        int2 _p1 = p1.xy;
        int2 _p2 = p2.xy;
        DSS2D(_p1, _p2, points, &numPoints);
        for(int i = 0; i < numPoints; i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].y = points[i].y;
        }
        _p1.xy = p1.yz;
        _p2.xy = p2.yz;
        DSS2D(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) coordinates[i].z = points[i].y;
    } else { // z coordinates with highest difference
        int2 _p1 = p1.xz;
        int2 _p2 = p2.xz;
        DSS2D(_p1, _p2, points, &numPoints);
        for(int i = 0; i < numPoints; i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].z = points[i].y;
        }
        _p1.xy = p1.yz;
        _p2.xy = p2.yz;
        DSS2D(_p1, _p2, points, &numPoints);
        for (int i = 0; i < numPoints; i++) coordinates[i].y = points[i].x;
    }
    for (int i = 0; i < numPoints; i++) {
        int x = coordinates[i].x - minBound.x;
        int y = coordinates[i].y - minBound.y;
        int z = coordinates[i].z - minBound.z;
        if (x >= 0 && x < dim.x && y >= 0 && y < dim.y && z >= 0 && z < dim.z) {
            int voxelIndex = z * dim.x * dim.y + y * dim.x + x;
            voxels[voxelIndex] = 1;
        }
    }
}

void updateMinMax(int2* points, int numPoints, int2 yBound, int2* minmax) {
    for (int i = 0; i < numPoints; i++) {
        int temp = points[i].y - yBound.x;
        if (temp >= 0 && temp <= yBound.y - yBound.x) {
            if (minmax[temp].x > points[i].x) minmax[temp].x = points[i].x;
            if (minmax[temp].y < points[i].x) minmax[temp].y = points[i].x;
        }
    }
}

void digitalTriangle2D(int2 p1, int2 p2, int2 p3, int4 plane, int maximum, char axis, __global uchar* voxels, int3 dim, int3 minBound) {
    int2 yBound;
    yBound.x = min(min(p1.y, p2.y), p3.y);
    yBound.y = max(max(p1.y, p2.y), p3.y);
    int2 minmax[MAX_ARRAY_SIZE]; // assume max y range MAX_ARRAY_SIZE
    for (int i = 0; i <= yBound.y - yBound.x; i++) {
        minmax[i].x = INT_MAX;
        minmax[i].y = INT_MIN;
    }

    int2 points[MAX_ARRAY_SIZE]; // local array for points
    int numPoints;

    DSS2D(p1, p2, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    DSS2D(p2, p3, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    DSS2D(p1, p3, points, &numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    for (int y = 0; y <= yBound.y - yBound.x; y++) {
        for (int x = minmax[y].x; x <= minmax[y].y; x++) {
            float z = ((float)maximum / 2.0 - (float)plane.x * (float)x - (float)plane.y * (float)(y + yBound.x) - (float)plane.z) / (float)plane.w;
            int zz = plane.w < 0 ? (int)ceil(z) : (int)floor(z);
            int i, j, k;
            if (axis == 1) { i = zz; j = x; k = y + yBound.x; }
            else if (axis == 2) { i = x; j = zz; k = y + yBound.x; }
            else { i = x; j = y + yBound.x; k = zz; }
            int voxelX = i - minBound.x;
            int voxelY = j - minBound.y;
            int voxelZ = k - minBound.z;
            if (voxelX >= 0 && voxelX < dim.x && voxelY >= 0 && voxelY < dim.y && voxelZ >= 0 && voxelZ < dim.z) {
                // printf("voxelX=%d, voxelY=%d, voxelZ=%d\n", voxelX, voxelY, voxelZ);
                int voxelIndex = voxelX + voxelY * dim.x + voxelZ * dim.x * dim.y;
                voxels[voxelIndex] = 1;
            }
        }
    }
}

void digitalTriangle3D(int3 p1, int3 p2, int3 p3, __global uchar* voxels, int3 dim, int3 minBound) {
    int3 p12 = (int3)(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    int3 p13 = (int3)(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

    int4 plane;
    plane.x = p12.y * p13.z - p13.y * p12.z;
    plane.y = p13.x * p12.z - p12.x * p13.z;
    plane.z = p12.x * p13.y - p13.x * p12.y;
    plane.w = -plane.x * p1.x - plane.y * p1.y - plane.z * p1.z;

    // degenerate triangle - simplified, just DSS3D but hard, skip for now
    if (plane.x == 0 && plane.y == 0 && plane.z == 0) {
        // handle degenerate triangles
        if (p1.x > p2.x || p1.y > p2.y || p1.z > p2.z) {
            swap3(&p1, &p2);
        }
        if (p1.x > p3.x || p1.y > p3.y || p1.z > p3.z) {
            swap3(&p1, &p3);
        }
        if (p2.x > p3.x || p2.y > p3.y || p2.z > p3.z) {
            swap3(&p2, &p3);
        }
        DSS3D(p1, p3, minBound, dim, voxels);
        return;
    }

    int maxDim = max(max(abs(plane.x), abs(plane.y)), abs(plane.z));
    int2 _p1, _p2, _p3;
    int4 _plane;
    char axis;
    if (maxDim == abs(plane.x)) {
        _p1.xy = p1.yz; _p2.xy = p2.yz; _p3.xy = p3.yz;
        _plane = (int4)(plane.y, plane.z, plane.w, plane.x);
        axis = 1;
    } else if (maxDim == abs(plane.y)) {
        _p1.xy = p1.xz; _p2.xy = p2.xz; _p3.xy = p3.xz;
        _plane = (int4)(plane.x, plane.z, plane.w, plane.y);
        axis = 2;
    } else {
        _p1.xy = p1.xy; _p2.xy = p2.xy; _p3.xy = p3.xy;
        _plane = (int4)(plane.x, plane.y, plane.w, plane.z);
        axis = 3;
    }
    if (p1.x == 0 && p1.y == 0 && p1.z == 20 && p2.x == 0 && p2.y == 10 && p2.z == 30 && p3.x == 0 && p3.y == 10 && p3.z == 20 ||
        p1.x == 0 && p1.y == 0 && p1.z == 20 && p2.x == 0 && p2.y == 10 && p2.z == 20 && p3.x == 0 && p3.y == 10 && p3.z == 30 ||
        p1.x == 0 && p1.y == 10 && p1.z == 30 && p2.x == 0 && p2.y == 0 && p2.z == 20 && p3.x == 0 && p3.y == 10 && p3.z == 20 ||
        p1.x == 0 && p1.y == 10 && p1.z == 30 && p2.x == 0 && p2.y == 10 && p2.z == 20 && p3.x == 0 && p3.y == 0 && p3.z == 20 ||
        p1.x == 0 && p1.y == 10 && p1.z == 20 && p2.x == 0 && p2.y == 0 && p2.z == 20 && p3.x == 0 && p3.y == 10 && p3.z == 30 ||
        p1.x == 0 && p1.y == 10 && p1.z == 20 && p2.x == 0 && p2.y == 10 && p2.z == 30 && p3.x == 0 && p3.y == 0 && p3.z == 20)
    {
        // printf("_p1 = (%d, %d), _p2 = (%d, %d), _p3 = (%d, %d)\n", _p1.x, _p1.y, _p2.x, _p2.y, _p3.x, _p3.y);
        // digitalTriangle2D(_p1, _p2, _p3, _plane, maxDim, axis, voxels, dim, minBound);
    }
    digitalTriangle2D(_p1, _p2, _p3, _plane, maxDim, axis, voxels, dim, minBound);
}

__kernel void voxelize_faces(__global const int* faces, int numFaces,
                             __global const int* intVertices,
                             __global uchar* voxels, int R, int C, int D, int xmin, int ymin, int zmin,
                             __global int *totalSize) {
    int faceId = get_global_id(0);
    if (faceId < 0 || faceId >= numFaces) return;
    // printf("Face ID: %d\n", faceId);
    atomic_inc(totalSize);

    int v1 = faces[faceId * 3];
    int v2 = faces[faceId * 3 + 1];
    int v3 = faces[faceId * 3 + 2];

    int3 p1 = (int3) (intVertices[v1 * 3], intVertices[v1 * 3 + 1], intVertices[v1 * 3 + 2]);
    int3 p2 = (int3) (intVertices[v2 * 3], intVertices[v2 * 3 + 1], intVertices[v2 * 3 + 2]);
    int3 p3 = (int3) (intVertices[v3 * 3], intVertices[v3 * 3 + 1], intVertices[v3 * 3 + 2]);

    int3 dim = (int3)(R, C, D);
    int3 minBound = (int3)(xmin, ymin, zmin);

    digitalTriangle3D(p1, p2, p3, voxels, dim, minBound);
}