// Helper: integer-based Bresenham-like digital straight segment routines (DSS)

void bresenhamLineDrawing(int2 p1, int2 p2, int2 val, int3 flag, out int2 points[MAX_ARRAY_SIZE], out int numPoints) {
    int p = (p2.y - p1.y);
    int q = (p1.x - p2.x);

    if (flag.z == 0) { int t = p1.x; p1.x = p2.x; p2.x = t; }
    if (flag.z == 1) { int t = p1.y; p1.y = p2.y; p2.y = t; }
    if (flag.z == 2) {
        int tx = p1.x;
        int ty = p1.y;
        p1.x = p2.x;
        p1.y = p2.y;
        p2.x = tx;
        p2.y = ty;
    }

    numPoints = abs(p2.x - p1.x) + 1;
    int f = 2 * p + q;
    int d = 2 * p;
    int dd = 2 * (p + q);

    int i = 0;
    while (p1.x <= p2.x && i < MAX_ARRAY_SIZE) {
        points[i] = (flag.y == 0) ? p1 : int2(p1.y, p1.x);

        if (f <= 0) {
            if (flag.x == 0) {
                f += d;
            } else {
                f += dd;
                p1.y += val.y;
            }
        } else {
            if (flag.x == 0) {
                f += dd;
                p1.y += val.y;
            } else {
                f += d;
            }
        }

        p1.x += val.x;
        i++;
    }
}

void rasterizeSegment(int2 p1, int2 p2, out int2 points[MAX_ARRAY_SIZE], out int numPoints) {
    int q = abs(p2.x - p1.x);
    int p = abs(p2.y - p1.y);

    if (p == 0 && q == 0) {
        points[0] = p1;
        numPoints = 1;
    }

    int2 _p1;
    int2 _p2;
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
        if (p1.y < p2.y && p1.x > p2.x) { _p1 = int2(p1.y, p2.x); _p2 = int2(p2.y, p1.x); flag = true; }
        else if (p1.y > p2.y && p1.x < p2.x) { _p1 = int2(p2.y, p1.x); _p2 = int2(p1.y, p2.x); flag = true; }
        if (flag) {
            val = int2(1, -1);
            flags = int3(0, 1, 1);
        }
    }
    if (p < q) {
        bool flag = false;
        if (p1.x > p2.x && p1.y <= p2.y) { _p1 = int2(p1.x, p2.y); _p2 = int2(p2.x, p1.y); flag = true; }
        else if (p1.x < p2.x && p1.y >= p2.y) { _p1 = int2(p2.x, p1.y); _p2 = int2(p1.x, p2.y); flag = true; }
        if (flag) {
            val = int2(1, -1);
            flags = int3(1, 0, 0);
        }
    }

    bresenhamLineDrawing(_p1, _p2, val, flags, points, numPoints);
}

void voxelizeSegment(int3 p1, int3 p2) {
    int3 absPoint = int3(abs(p2.x - p1.x), abs(p2.y - p1.y), abs(p2.z - p1.z));
    int maxv = max(max(absPoint.x, absPoint.y), absPoint.z);

    if (maxv == 0) {
        int x = p1.x - xmin;
        int y = p1.y - ymin;
        int z = p1.z - zmin;
        if (x >= 0 && x < R && y >= 0 && y < C && z >= 0 && z < D) {
            setVoxel(x + y * R + z * R * C);
        }
        return;
    }

    int2 points[MAX_ARRAY_SIZE];
    int3 coordinates[MAX_ARRAY_SIZE];
    int numPoints;
    int2 _p1;
    int2 _p2;

    if (maxv == absPoint.x) {
        _p1 = int2(p1.x, p1.y);
        _p2 = int2(p2.x, p2.y);
        rasterizeSegment(_p1, _p2, points, numPoints);
        for (int i = 0; i < numPoints; ++i) { coordinates[i].x = points[i].x; coordinates[i].y = points[i].y; }

        _p1 = int2(p1.x, p1.z);
        _p2 = int2(p2.x, p2.z);
        rasterizeSegment(_p1, _p2, points, numPoints);
        for (int i = 0; i < numPoints; ++i) coordinates[i].z = points[i].y;
    } else if (maxv == absPoint.y) {
        _p1 = int2(p1.x, p1.y);
        _p2 = int2(p2.x, p2.y);
        rasterizeSegment(_p1, _p2, points, numPoints);
        for (int i = 0; i < numPoints; ++i) { coordinates[i].x = points[i].x; coordinates[i].y = points[i].y; }

        _p1 = int2(p1.y, p1.z);
        _p2 = int2(p2.y, p2.z);
        rasterizeSegment(_p1, _p2, points, numPoints);
        for (int i = 0; i < numPoints; ++i) coordinates[i].z = points[i].y;
    } else {
        _p1 = int2(p1.x, p1.z);
        _p2 = int2(p2.x, p2.z);
        rasterizeSegment(_p1, _p2, points, numPoints);
        for (int i = 0; i < numPoints; ++i) { coordinates[i].x = points[i].x; coordinates[i].z = points[i].y; }

        _p1 = int2(p1.y, p1.z);
        _p2 = int2(p2.y, p2.z);
        rasterizeSegment(_p1, _p2, points, numPoints);
        for (int i = 0; i < numPoints; ++i) coordinates[i].y = points[i].x;
    }

    for (int i = 0; i < numPoints; ++i) {
        int x = coordinates[i].x - xmin;
        int y = coordinates[i].y - ymin;
        int z = coordinates[i].z - zmin;
        if (x >= 0 && x < R && y >= 0 && y < C && z >= 0 && z < D) {
            setVoxel(x + y * R + z * R * C);
        }
    }
}
