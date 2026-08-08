struct DSS2D {
    int2 cursor;
    int2 step;
    int decision;
    int straightStep;
    int diagonalStep;
    int flag1;
    int flag2;
    int remaining;
};

DSS2D createRasterSegment(int2 start, int2 end) {
    DSS2D result;
    result.cursor = start;
    result.step = int2(0, 0);
    result.decision = 0;
    result.straightStep = 0;
    result.diagonalStep = 0;
    result.flag1 = 0;
    result.flag2 = 0;
    result.remaining = 1;

    const int q = abs(end.x - start.x);
    const int p = abs(end.y - start.y);
    int2 a = start;
    int2 b = end;
    int flag3 = -1;
    bool classified = false;

    if (p <= q && start.x < end.x && start.y < end.y) { result.step = int2(1, 1); classified = true; }
    else if (p <= q && start.x > end.x && start.y > end.y) { a = end; b = start; result.step = int2(1, 1); classified = true; }
    else if (p > q && start.y < end.y && start.x <= end.x) { a = int2(end.y, end.x); b = int2(start.y, start.x); result.step = int2(1, 1); result.flag1 = 1; result.flag2 = 1; flag3 = 2; classified = true; }
    else if (p > q && start.y > end.y && start.x >= end.x) { a = int2(start.y, start.x); b = int2(end.y, end.x); result.step = int2(1, 1); result.flag1 = 1; result.flag2 = 1; flag3 = 2; classified = true; }
    else if (p >= q && start.y < end.y && start.x > end.x) { a = int2(start.y, end.x); b = int2(end.y, start.x); result.step = int2(1, -1); result.flag2 = 1; flag3 = 1; classified = true; }
    else if (p >= q && start.y > end.y && start.x < end.x) { a = int2(end.y, start.x); b = int2(start.y, end.x); result.step = int2(1, -1); result.flag2 = 1; flag3 = 1; classified = true; }
    else if (p < q && start.x > end.x && start.y <= end.y) { a = int2(start.x, end.y); b = int2(end.x, start.y); result.step = int2(1, -1); result.flag1 = 1; flag3 = 0; classified = true; }
    else if (p < q && start.x < end.x && start.y >= end.y) { a = int2(end.x, start.y); b = int2(start.x, end.y); result.step = int2(1, -1); result.flag1 = 1; flag3 = 0; classified = true; }

    if (!classified) return result;

    const int numerator = b.y - a.y;
    const int denominator = a.x - b.x;
    result.decision = 2 * numerator + denominator;
    result.straightStep = 2 * numerator;
    result.diagonalStep = 2 * (numerator + denominator);
    if (flag3 == 0) { const int t = a.x; a.x = b.x; b.x = t; }
    else if (flag3 == 1) { const int t = a.y; a.y = b.y; b.y = t; }
    else if (flag3 == 2) { const int2 t = a; a = b; b = t; }
    result.cursor = a;
    result.remaining = max(0, b.x - a.x + 1);
    return result;
}

bool hasPixel(DSS2D dss) {
    return dss.remaining > 0;
}

int2 currentRasterSegment(DSS2D dss) {
    return dss.flag2 == 0 ? dss.cursor : int2(dss.cursor.y, dss.cursor.x);
}

void advanceRasterSegment(inout DSS2D dss) {
    if (dss.decision <= 0) {
        if (dss.flag1 == 0) dss.decision += dss.straightStep;
        else { dss.decision += dss.diagonalStep; dss.cursor.y += dss.step.y; }
    } else if (dss.flag1 == 0) { dss.decision += dss.diagonalStep; dss.cursor.y += dss.step.y; }
    else dss.decision += dss.straightStep;
    dss.cursor.x += dss.step.x;
    dss.remaining -= 1;
}

// Exact working DirectX segment voxelizer, renamed for the experimental shader.
// This is intentionally not expressed through DSS3D yet; first we restore
// byte-for-byte behavior at the call boundary, then refactor safely.
void voxelizeSegment(int3 p1, int3 p2) {
    const int3 delta = abs(p2 - p1);
    const int maxv = max(max(delta.x, delta.y), delta.z);
    if (maxv == 0) {
        const int3 voxel = p1 - int3(xmin, ymin, zmin);
        if (voxel.x >= 0 && voxel.x < R
            && voxel.y >= 0 && voxel.y < C
            && voxel.z >= 0 && voxel.z < D)
            setVoxel(voxel.x + voxel.y * R + voxel.z * R * C);
        return;
    }

    DSS2D first;
    DSS2D second;
    int axis;
    if (maxv == delta.x) {
        first = createRasterSegment(int2(p1.x, p1.y), int2(p2.x, p2.y));
        second = createRasterSegment(int2(p1.x, p1.z), int2(p2.x, p2.z)); axis = 0;
    }
    else if (maxv == delta.y) {
        first = createRasterSegment(int2(p1.x, p1.y), int2(p2.x, p2.y));
        second = createRasterSegment(int2(p1.y, p1.z), int2(p2.y, p2.z)); axis = 1;
    }
    else {
        first = createRasterSegment(int2(p1.x, p1.z), int2(p2.x, p2.z));
        second = createRasterSegment(int2(p1.y, p1.z), int2(p2.y, p2.z)); axis = 2;
    }

    for (int index = 0; index <= maxv && index < MAX_ARRAY_SIZE; ++index) {
        const int2 a = currentRasterSegment(first);
        const int2 b = currentRasterSegment(second);
        const int3 coordinate = axis == 0 ? int3(a.x, a.y, b.y) : (axis == 1 ? int3(a.x, a.y, b.y) : int3(a.x, b.x, a.y));
        const int3 voxel = coordinate - int3(xmin, ymin, zmin);
        if (voxel.x >= 0 && voxel.x < R
            && voxel.y >= 0 && voxel.y < C
            && voxel.z >= 0 && voxel.z < D)
            setVoxel(voxel.x + voxel.y * R + voxel.z * R * C);

        advanceRasterSegment(first);
        advanceRasterSegment(second);
    }
}
