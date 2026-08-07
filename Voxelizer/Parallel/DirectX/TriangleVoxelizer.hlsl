void updateMinMax(int2 points[MAX_ARRAY_SIZE], int numPoints, int2 yBound, inout int2 minmax[MAX_ARRAY_SIZE]) {
    for (int i = 0; i < numPoints; ++i) {
        int temp = points[i].y - yBound.x;
        if (temp >= 0 && temp <= yBound.y - yBound.x && temp < MAX_ARRAY_SIZE) {
            if (minmax[temp].x > points[i].x) minmax[temp].x = points[i].x;
            if (minmax[temp].y < points[i].x) minmax[temp].y = points[i].x;
        }
    }
}

void computeTriEdgePixels(int2 p1, int2 p2, int2 p3, inout int2 minmax[MAX_ARRAY_SIZE]) {
    int2 yBound;
    yBound.x = min(min(p1.y, p2.y), p3.y);
    yBound.y = max(max(p1.y, p2.y), p3.y);

    int rowCount = yBound.y - yBound.x + 1;
    if (rowCount > MAX_ARRAY_SIZE) {
        return;
    }

    for (int i = 0; i < rowCount; ++i) {
        minmax[i].x = 2147483647;
        minmax[i].y = -2147483647;
    }

    int2 points[MAX_ARRAY_SIZE];
    int numPoints;

    rasterizeSegment(p1, p2, points, numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    rasterizeSegment(p2, p3, points, numPoints);
    updateMinMax(points, numPoints, yBound, minmax);

    rasterizeSegment(p1, p3, points, numPoints);
    updateMinMax(points, numPoints, yBound, minmax);
}

void voxelizeTriangle(int3 p1, int3 p2, int3 p3) {
    int3 p12 = int3(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    int3 p13 = int3(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

    int4 plane;
    plane.x = p12.y * p13.z - p13.y * p12.z;
    plane.y = p13.x * p12.z - p12.x * p13.z;
    plane.z = p12.x * p13.y - p13.x * p12.y;
    plane.w = -plane.x * p1.x - plane.y * p1.y - plane.z * p1.z;

    if (plane.x == 0 && plane.y == 0 && plane.z == 0) {
        if (p1.x > p2.x || p1.y > p2.y || p1.z > p2.z) {
            int3 t = p1; p1 = p2; p2 = t;
        }
        if (p1.x > p3.x || p1.y > p3.y || p1.z > p3.z) {
            int3 t = p1; p1 = p3; p3 = t;
        }
        if (p2.x > p3.x || p2.y > p3.y || p2.z > p3.z) {
            int3 t = p2; p2 = p3; p3 = t;
        }
        voxelizeSegment(p1, p3);
        return;
    }

    int maxDim = max(max(abs(plane.x), abs(plane.y)), abs(plane.z));
    int2 _p1;
    int2 _p2;
    int2 _p3;
    int4 _plane;
    int axis;

    if (maxDim == abs(plane.x)) {
        _p1 = int2(p1.y, p1.z);
        _p2 = int2(p2.y, p2.z);
        _p3 = int2(p3.y, p3.z);
        _plane = int4(plane.y, plane.z, plane.w, plane.x);
        axis = 1;
    } else if (maxDim == abs(plane.y)) {
        _p1 = int2(p1.x, p1.z);
        _p2 = int2(p2.x, p2.z);
        _p3 = int2(p3.x, p3.z);
        _plane = int4(plane.x, plane.z, plane.w, plane.y);
        axis = 2;
    } else {
        _p1 = int2(p1.x, p1.y);
        _p2 = int2(p2.x, p2.y);
        _p3 = int2(p3.x, p3.y);
        _plane = int4(plane.x, plane.y, plane.w, plane.z);
        axis = 3;
    }

    int2 yBound;
    yBound.x = min(min(_p1.y, _p2.y), _p3.y);
    yBound.y = max(max(_p1.y, _p2.y), _p3.y);
    int rowCount = yBound.y - yBound.x + 1;
    if (rowCount > MAX_ARRAY_SIZE) {
        return;
    }

    int2 boundaryPixels[MAX_ARRAY_SIZE];
    computeTriEdgePixels(_p1, _p2, _p3, boundaryPixels);

    for (int y = 0; y < rowCount; ++y) {
        if (boundaryPixels[y].x == 2147483647 || boundaryPixels[y].y == -2147483647) {
            continue;
        }

        for (int x = boundaryPixels[y].x; x <= boundaryPixels[y].y; ++x) {
            float z = ((float)maxDim / 2.0f - (float)_plane.x * (float)x - (float)_plane.y * (float)(y + yBound.x) - (float)_plane.z) / (float)_plane.w;
            int zz = _plane.w < 0 ? (int)ceil(z) : (int)floor(z);

            int i;
            int j;
            int k;
            if (axis == 1) { i = zz; j = x; k = y + yBound.x; }
            else if (axis == 2) { i = x; j = zz; k = y + yBound.x; }
            else { i = x; j = y + yBound.x; k = zz; }

            int voxelX = i - xmin;
            int voxelY = j - ymin;
            int voxelZ = k - zmin;
            if (voxelX >= 0 && voxelX < R && voxelY >= 0 && voxelY < C && voxelZ >= 0 && voxelZ < D) {
                setVoxel(voxelX + voxelY * R + voxelZ * R * C);
            }
        }
    }
}
