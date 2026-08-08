// Experimental DirectX triangle module. It owns plane projection, triangle
// scan conversion, and voxel writes; DSS point generation stays in DSSCreator.

void updateMinMaxForSegment(int2 start, int2 end, int2 yBound, inout int2 minmax[MAX_ARRAY_SIZE]) {
    DSS2D dss = createRasterSegment(start, end);
    for (int pointCount = 0; hasPixel(dss) && pointCount < MAX_ARRAY_SIZE; ++pointCount) {
        const int2 rasterPoint = currentRasterSegment(dss);
        const int row = rasterPoint.y - yBound.x;
        if (row >= 0 && row < MAX_ARRAY_SIZE) {
            minmax[row].x = min(minmax[row].x, rasterPoint.x);
            minmax[row].y = max(minmax[row].y, rasterPoint.x);
        }
        advanceRasterSegment(dss);
    }
}

void voxelizeTriangle(int3 p1, int3 p2, int3 p3, uint faceId) {
    const int3 p12 = p2 - p1;
    const int3 p13 = p3 - p1;
    int4 plane;
    plane.x = p12.y * p13.z - p13.y * p12.z;
    plane.y = p13.x * p12.z - p12.x * p13.z;
    plane.z = p12.x * p13.y - p13.x * p12.y;
    plane.w = -plane.x * p1.x - plane.y * p1.y - plane.z * p1.z;

    if (plane.x == 0 && plane.y == 0 && plane.z == 0) {
        // Retain the serial endpoint ordering before creating the 3D DSS.
        if (p1.x > p2.x || p1.y > p2.y || p1.z > p2.z) { const int3 t = p1; p1 = p2; p2 = t; }
        if (p1.x > p3.x || p1.y > p3.y || p1.z > p3.z) { const int3 t = p1; p1 = p3; p3 = t; }
        if (p2.x > p3.x || p2.y > p3.y || p2.z > p3.z) { const int3 t = p2; p2 = p3; p3 = t; }
        voxelizeSegment(p1, p3);
        return;
    }

    const int maxDim = max(max(abs(plane.x), abs(plane.y)), abs(plane.z));
    int2 a;
    int2 b;
    int2 c;
    int4 projectedPlane;
    int axis;
    if (maxDim == abs(plane.x)) {
        a = int2(p1.y, p1.z); b = int2(p2.y, p2.z); c = int2(p3.y, p3.z);
        projectedPlane = int4(plane.y, plane.z, plane.w, plane.x); axis = 1;
    } else if (maxDim == abs(plane.y)) {
        a = int2(p1.x, p1.z); b = int2(p2.x, p2.z); c = int2(p3.x, p3.z);
        projectedPlane = int4(plane.x, plane.z, plane.w, plane.y); axis = 2;
    } else {
        a = int2(p1.x, p1.y); b = int2(p2.x, p2.y); c = int2(p3.x, p3.y);
        projectedPlane = int4(plane.x, plane.y, plane.w, plane.z); axis = 3;
    }

    const int2 yBounds = int2(min(min(a.y, b.y), c.y), max(max(a.y, b.y), c.y));
    const int rowCount = yBounds.y - yBounds.x + 1;
    if (rowCount > MAX_ARRAY_SIZE) return;

    int2 minmax[MAX_ARRAY_SIZE];
    for (int row = 0; row < rowCount; ++row)
        minmax[row] = int2(2147483647, -2147483647);

    updateMinMaxForSegment(a, b, yBounds, minmax);
    updateMinMaxForSegment(b, c, yBounds, minmax);
    updateMinMaxForSegment(a, c, yBounds, minmax);

    for (int scanline = 0; scanline < rowCount; ++scanline) {
        if (minmax[scanline].x == 2147483647 || minmax[scanline].y == -2147483647) continue;
        for (int x = minmax[scanline].x; x <= minmax[scanline].y; ++x) {
            const float z = ((float)maxDim / 2.0f - (float)projectedPlane.x * x - (float)projectedPlane.y * (scanline + yBounds.x) - (float)projectedPlane.z) / (float)projectedPlane.w;
            const int zz = projectedPlane.w < 0 ? (int)ceil(z) : (int)floor(z);
            const int3 voxelPoint = axis == 1 ? int3(zz, x, scanline + yBounds.x) : (axis == 2 ? int3(x, zz, scanline + yBounds.x) : int3(x, scanline + yBounds.x, zz));
            const int3 voxel = voxelPoint - int3(xmin, ymin, zmin);
            if (voxel.x >= 0 && voxel.x < R && voxel.y >= 0 && voxel.y < C && voxel.z >= 0 && voxel.z < D) {
                const uint index = (uint)(voxel.x + voxel.y * R + voxel.z * R * C);
                setVoxel(index);
            }
        }
    }
}