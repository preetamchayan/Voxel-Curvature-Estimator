#include "Voxelizer.h"

// Include only the selected parallel environment header to avoid optional dependency
#if VOXELIZE_MODE == PARALLEL_OPENCL
#include "OpenCL/VoxelizerOclEnv.h"
#elif VOXELIZE_MODE == PARALLEL_VULKAN
#include "Vulkan/VoxelizerVulkanEnv.h"
#elif VOXELIZE_MODE == PARALLEL_CUDA
#include "Cuda/VoxelizerCudaEnv.h"
#elif VOXELIZE_MODE == PARALLEL_DIRECTX
#include "DirectX/VoxelizerDirectxEnv.h"
#else
#include "OpenCL/VoxelizerOclEnv.h"
#endif
#include <algorithm>
#include <climits>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cassert>

int maxSize = INT_MIN;

Voxelizer::Voxelizer(BBox3d bounds) : m_unscaledBounds(bounds) {
    m_ocTree = nullptr;
    m_baseEnv = nullptr;
}

Voxelizer::~Voxelizer() {
    if (m_ocTree) {
        delete m_ocTree;
    }
    if (m_baseEnv) {
        delete m_baseEnv;
    }
}

BBox3i Voxelizer::getSceneBounds() const {
    return m_scaledBounds;
}

void Voxelizer::swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

const std::vector<unsigned char>& Voxelizer::getVoxels() const {
    return m_voxels;
}

void Voxelizer::getVoxels(std::vector<Point3i>& voxels) const {
    m_ocTree->getOccupiedVoxels(voxels);
}

int Voxelizer::getVoxelCount() const {
#if VOXELIZE_MODE == SERIAL
    return m_ocTree->getVoxelCount();
#else
    return std::count(m_voxels.begin(), m_voxels.end(), 1);
#endif
}

void Voxelizer::getRecommendedScaleRange(int& s_low, int& s_high) const {
    double dx = m_unscaledBounds.xmax - m_unscaledBounds.xmin;
    double dy = m_unscaledBounds.ymax - m_unscaledBounds.ymin;
    double dz = m_unscaledBounds.zmax - m_unscaledBounds.zmin;
    double maximum = std::max({dx, dy, dz});
    if (maximum == 0) {
        s_low = 1;
        s_high = 1;
        return;
    }
    int p = 0;
    while ((int)(maximum * (float)p) < 1000) p += 50;
    s_high = p;
    s_low = p / 10;
}

bool Voxelizer::search(Point3i point) const {
    return m_ocTree->search(point);
}

bool Voxelizer::remove(Point3i point) const {
    return m_ocTree->remove(point);
}

std::vector<Point2i> Voxelizer::getPixelDSS2D(Point2i p1, Point2i p2, Values values, Flags flags) {
    int p = (p2.y - p1.y);
    int q = (p1.x - p2.x);
    if (flags.flag3 == 0) swap(&p1.x, &p2.x);
    if (flags.flag3 == 1) swap(&p1.y, &p2.y);
    if (flags.flag3 == 2) {
        swap(&p1.x, &p2.x);
        swap(&p1.y, &p2.y);
    }
    int f = 2 * p + q;
    int d = 2 * p;
    int dd = 2 * (p + q);

    std::vector<Point2i> points;

    while (p1.x <= p2.x) {
        if (flags.flag2 == 0) { // x and y in original form
            points.emplace_back(p1.x, p1.y);
        } else { // x and y in swapped form
            points.emplace_back(p1.y, p1.x);
        }
        if (f <= 0) {
            if (flags.flag1 == 0) f += d;
            else {
                f += dd;
                p1.y += values.val2;
            }
        } else {
            if (flags.flag1 == 0) {
                f += dd;
                p1.y += values.val2;
            } else f += d;
        }
        p1.x += values.val1;
    }

    return points;
}

std::vector<Point2i> Voxelizer::DSS2D(Point2i p1, Point2i p2) {
    int q = std::abs(p2.x - p1.x);
    int p = std::abs(p2.y - p1.y);
    Point2i _p1, _p2;
    Values values;
    Flags flags;
    if (p <= q) { // > 0 & <= 45 && > 180 & <= 225
        bool flag = false;
        if (p1.x < p2.x && p1.y < p2.y) { _p1 = p1; _p2 = p2; flag = true; }
        else if (p1.x > p2.x && p1.y > p2.y) { _p1 = p2; _p2 = p1; flag = true; }
        if(flag) {
            values = {1, 1};
            flags = {0, 0, -1};
        }
    }
    if (p > q) { // > 45 & <= 90 && > 225 & <= 270
        bool flag = false;
        if (p1.y < p2.y && p1.x <= p2.x) { _p1 = {p2.y, p2.x}; _p2 = {p1.y, p1.x}; flag = true; }
        else if (p1.y > p2.y && p1.x >= p2.x) { _p1 = {p1.y, p1.x}; _p2 = {p2.y, p2.x}; flag = true; }
        if(flag) {
            values = {1, 1};
            flags = {1, 1, 2};
        }
    }
    if (p >= q) { // > 90 & <= 135 && > 270 & <= 315
        bool flag = false;
        if (p1.y < p2.y && p1.x > p2.x) { _p1 = {p1.y, p2.x}; _p2 = {p2.y, p1.x}; flag = true; }
        else if (p1.y > p2.y && p1.x < p2.x) { _p1 = {p2.y, p1.x}; _p2 = {p1.y, p2.x}; flag = true; }
        if(flag) {
            values = {1, -1};
            flags = {0, 1, 1};
        }
    }
    if (p < q) { // > 135 & <= 180 && > 315 & <= 360
        bool flag = false;
        if (p1.x > p2.x && p1.y <= p2.y) { _p1 = {p1.x, p2.y}; _p2 = {p2.x, p1.y}; flag = true; }
        else if (p1.x < p2.x && p1.y >= p2.y) { _p1 = {p2.x, p1.y}; _p2 = {p1.x, p2.y}; flag = true; }
        if(flag) {
            values = {1, -1};
            flags = {1, 0, 0};
        }
    }
    return getPixelDSS2D(_p1, _p2, values, flags);
}

void Voxelizer::DSS3D(Point3i p1, Point3i p2) {
    Point3i absPoint;
    absPoint.x = std::abs(p2.x - p1.x);
    absPoint.y = std::abs(p2.y - p1.y);
    absPoint.z = std::abs(p2.z - p1.z);
    int max = std::max({absPoint.x, absPoint.y, absPoint.z});
    // std::cout << "DSS3D: max = " << max << std::endl;
    if (max > maxSize) {
        maxSize = max;
    }
    if (max == 0) {
        m_ocTree->insert(p1);
        return;
    }
    std::vector<Point2i> points;
    std::vector<Point3i> coordinates;
    coordinates.resize(max + 1);
    if (max == absPoint.x) { // x coordinates with highest difference
        auto points = DSS2D(Point2i{p1.x, p1.y}, Point2i{p2.x, p2.y});
        for(int i = 0; i < points.size(); i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].y = points[i].y;
        }
        int size1 = points.size();
        points.clear();
        points = DSS2D(Point2i{p1.x, p1.z}, Point2i{p2.x, p2.z});
        int size2 = points.size();
        assert(size1 == size2 && "DSS2D should return the same number of points for both projections");
        for (int i = 0; i < size1; i++) coordinates[i].z = points[i].y;
    } else if (max == absPoint.y) { // y coordinates with highest difference
        auto points = DSS2D(Point2i{p1.x, p1.y}, Point2i{p2.x, p2.y});
        for(int i = 0; i < points.size(); i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].y = points[i].y;
        }
        int size1 = points.size();
        points.clear();
        points = DSS2D(Point2i{p1.y, p1.z}, Point2i{p2.y, p2.z});
        int size2 = points.size();
        assert(size1 == size2 && "DSS2D should return the same number of points for both projections");
        for (int i = 0; i < size1; i++) coordinates[i].z = points[i].y;
    } else { // z coordinates with highest difference
        auto points = DSS2D(Point2i{p1.x, p1.z}, Point2i{p2.x, p2.z});
        for(int i = 0; i < points.size(); i++) {
            coordinates[i].x = points[i].x;
            coordinates[i].z = points[i].y;
        }
        int size1 = points.size();
        points.clear();
        points = DSS2D(Point2i{p1.y, p1.z}, Point2i{p2.y, p2.z});
        int size2 = points.size();
        assert(size1 == size2 && "DSS2D should return the same number of points for both projections");
        for (int i = 0; i < size1; i++) coordinates[i].y = points[i].x;
    }
    for (int i = 0; i < coordinates.size(); i++) {
        m_ocTree->insert(coordinates[i]);
    }
}

void Voxelizer::digitalTriangle2D(Point2i p1, Point2i p2, Point2i p3, Plane plane, char axis) {
    Point2i yBound;
    yBound.x = std::min({p1.y, p2.y, p3.y}); // min y
    yBound.y = std::max({p1.y, p2.y, p3.y}); // max y
    std::vector<Point2i> minmax(yBound.y - yBound.x + 1);
    // std::cout << "digitalTriangle2D: minmax size = " << minmax.size() << std::endl;
    for (int i = 0; i <= yBound.y - yBound.x; i++) {
        minmax[i].x = INT_MAX;
        minmax[i].y = INT_MIN;
    }

    auto updateMaxMin = [&] (std::vector<Point2i>& points) {
        for (size_t i = 0; i < points.size(); i++) {
            int temp = points[i].y - yBound.x;
            if (temp >= 0 && temp < minmax.size()) {
                if (minmax[temp].x > points[i].x) minmax[temp].x = points[i].x;
                if (minmax[temp].y < points[i].x) minmax[temp].y = points[i].x;
            }
        }
        points.clear();
    };

    std::vector<Point2i> points = DSS2D(p1, p2);
    // std::cout << "digitalTriangle2D: points size = " << points.size() << std::endl;
    if (maxSize < (int)points.size()) {
        maxSize = (int)points.size();
    }
    updateMaxMin(points);
    points = DSS2D(p2, p3);
    // std::cout << "digitalTriangle2D: points size = " << points.size() << std::endl;
    if (maxSize < (int)points.size()) {
        maxSize = (int)points.size();
    }
    updateMaxMin(points);
    points = DSS2D(p1, p3);
    // std::cout << "digitalTriangle2D: points size = " << points.size() << std::endl;
    if (maxSize < (int)points.size()) {
        maxSize = (int)points.size();
    }
    updateMaxMin(points);

    for (int y = 0; y <= yBound.y - yBound.x; y++) {
        for (int x = minmax[y].x; x <= minmax[y].y; x++) {
            float z = ((float)(std::abs(plane.d)) / 2.0 - (float)plane.a * (float)x - (float)plane.b * (float)(y + yBound.x) - (float)plane.c) / (float)plane.d;
            int i, j, k, zz = plane.d < 0 ? (int)std::ceil(z) : (int)std::floor(z);
            if (axis == 1) { i = zz; j = x; k = y + yBound.x; }
            else if (axis == 2) { i = x; j = zz; k = y + yBound.x; }
            else { i = x; j = y + yBound.x; k = zz; }
            m_ocTree->insert(Point3i(i, j, k));
        }
    }
}

void Voxelizer::digitalTriangle3D(Point3i p1, Point3i p2, Point3i p3) {
    Point3i p12, p13;

    // determining the co-efficients of the plane equation of the triangle
    p12.x = (p2.x - p1.x);
    p12.y = (p2.y - p1.y);
    p12.z = (p2.z - p1.z);
    p13.x = (p3.x - p1.x);
    p13.y = (p3.y - p1.y);
    p13.z = (p3.z - p1.z);

    Plane plane;
    plane.a = p12.y * p13.z - p13.y * p12.z;
    plane.b = p13.x * p12.z - p12.x * p13.z;
    plane.c = p12.x * p13.y - p13.x * p12.y;
    plane.d = -plane.a * p1.x - plane.b * p1.y - plane.c * p1.z;

    // degenerate triangle
    if (plane.a == 0 && plane.b == 0 && plane.c == 0) {
        if (p1.x > p2.x || p1.y > p2.y || p1.z > p2.z) {
            swap(&p1.x, &p2.x);
            swap(&p1.y, &p2.y);
            swap(&p1.z, &p2.z);
        }
        if (p1.x > p3.x || p1.y > p3.y || p1.z > p3.z) {
            swap(&p1.x, &p3.x);
            swap(&p1.y, &p3.y);
            swap(&p1.z, &p3.z);
        }
        if (p2.x > p3.x || p2.y > p3.y || p2.z > p3.z) {
            swap(&p2.x, &p3.x);
            swap(&p2.y, &p3.y);
            swap(&p2.z, &p3.z);
        }
        DSS3D(p1, p3);
        return;
    }

    // finding the projected plane with maximum area
    int maxDim = (int)std::max({std::abs(plane.a), std::abs(plane.b), std::abs(plane.c)});
    Point2i _p1, _p2, _p3;
    Plane _plane;
    char axis;
    if (maxDim == std::abs(plane.a)) {
        _p1 = {p1.y, p1.z}; _p2 = {p2.y, p2.z}; _p3 = {p3.y, p3.z};
        _plane = {plane.b, plane.c, plane.d, plane.a};
        axis = 1;
    } else if (maxDim == std::abs(plane.b)) {
        _p1 = {p1.x, p1.z}; _p2 = {p2.x, p2.z}; _p3 = {p3.x, p3.z};
        _plane = {plane.a, plane.c, plane.d, plane.b};
        axis = 2;
    } else {
        _p1 = {p1.x, p1.y}; _p2 = {p2.x, p2.y}; _p3 = {p3.x, p3.y};
        _plane = {plane.a, plane.b, plane.d, plane.c};
        axis = 3;
    }
    digitalTriangle2D(_p1, _p2, _p3, _plane, axis);
}

void Voxelizer::voxelize(const MeshLoader& mesh, float scale) {
    const auto& vertices = mesh.getVertices();
    const auto& faces = mesh.getFaces();

    // Scale vertices and find bounds
    std::vector<Point3i> intVertices(vertices.size());
    m_scaledBounds.xmax = INT_MIN, m_scaledBounds.xmin = INT_MAX, m_scaledBounds.ymax = INT_MIN, m_scaledBounds.ymin = INT_MAX, m_scaledBounds.zmax = INT_MIN, m_scaledBounds.zmin = INT_MAX;
    for (size_t i = 0; i < vertices.size(); ++i) {
        Point3f point3f;
        point3f.x = vertices[i].x * scale;
        point3f.y = vertices[i].y * scale;
        point3f.z = vertices[i].z * scale;

        Point3i point3i;
        point3i.x = static_cast<int>(std::round(point3f.x));
        point3i.y = static_cast<int>(std::round(point3f.y));
        point3i.z = static_cast<int>(std::round(point3f.z));

        intVertices[i] = point3i;
        m_scaledBounds.xmax = std::max(m_scaledBounds.xmax, point3i.x);
        m_scaledBounds.xmin = std::min(m_scaledBounds.xmin, point3i.x);
        m_scaledBounds.ymax = std::max(m_scaledBounds.ymax, point3i.y);
        m_scaledBounds.ymin = std::min(m_scaledBounds.ymin, point3i.y);
        m_scaledBounds.zmax = std::max(m_scaledBounds.zmax, point3i.z);
        m_scaledBounds.zmin = std::min(m_scaledBounds.zmin, point3i.z);
    }

    int R = m_scaledBounds.xmax - m_scaledBounds.xmin + 3;
    int C = m_scaledBounds.ymax - m_scaledBounds.ymin + 3;
    int D = m_scaledBounds.zmax - m_scaledBounds.zmin + 3;

    m_dims.width = R;
    m_dims.height = C;
    m_dims.depth = D;

#if VOXELIZE_MODE == SERIAL
    // Serial voxelization
    m_ocTree = new OcTree(m_scaledBounds);
    for (const auto& face : faces) {
        const auto& v1 = intVertices[face.v1];
        const auto& v2 = intVertices[face.v2];
        const auto& v3 = intVertices[face.v3];
        digitalTriangle3D(v1, v2, v3);
    }
    // std::cout << "Max size of 2D DSS: " << maxSize << std::endl;
    std::cout << "Total unique voxels inserted in octree: " << m_ocTree->getVoxelCount() << std::endl;
#else
    m_voxels.assign(R * C * D, 0);
    #if VOXELIZE_MODE == PARALLEL_OPENCL
        m_baseEnv = new VoxelizerOclEnv();
    #elif VOXELIZE_MODE == PARALLEL_VULKAN
        m_baseEnv = new VoxelizerVulkanEnv();
    #elif VOXELIZE_MODE == PARALLEL_CUDA
        m_baseEnv = new VoxelizerCudaEnv();
    #elif VOXELIZE_MODE == PARALLEL_DIRECTX
        m_baseEnv = new VoxelizerDirectxEnv();
    #else
        #error "Invalid VOXELIZE_MODE defined. Please define it as SERIAL, PARALLEL_OPENCL, PARALLEL_VULKAN, or PARALLEL_CUDA." 
    #endif
    if (m_baseEnv) {
        m_baseEnv->voxelize(m_voxels, intVertices, faces, m_scaledBounds, m_dims);
    }
#endif
}

void Voxelizer::exportVoxelsOBJ(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return;
    }

    std::ofstream logFile("output/voxel_export_log.txt");
    if (logFile.is_open()) {
        logFile << "Exporting voxel mesh to OBJ format\n";
        logFile << "Grid bounds: (" << m_scaledBounds.xmin << ", " << m_scaledBounds.ymin << ", " << m_scaledBounds.zmin << ") to (" << m_scaledBounds.xmax << ", " << m_scaledBounds.ymax << ", " << m_scaledBounds.zmax << ")\n";
        logFile << "Total voxels: " << getVoxelCount() << "\n";
    }

    file << "# Voxel mesh exported from Voxelizer\n";
    file << "# Grid bounds: (" << m_scaledBounds.xmin << ", " << m_scaledBounds.ymin << ", "
         << m_scaledBounds.zmin << ") to (" << m_scaledBounds.xmax << ", " << m_scaledBounds.ymax << ", "
         << m_scaledBounds.zmax << ")\n\n";

    auto writeToFile = [&] (int x, int y, int z) {
        if (logFile.is_open()) {
            logFile << "(" << x << ", " << y << ", " << z << ")\n";
        }

        file << "v " << x       << " " << y       << " " << z + 1   << "\n";  // v0: -8
        file << "v " << x + 1   << " " << y       << " " << z + 1   << "\n";  // v1: -7
        file << "v " << x + 1   << " " << y + 1   << " " << z + 1   << "\n";  // v2: -6
        file << "v " << x       << " " << y + 1   << " " << z + 1   << "\n";  // v3: -5
        file << "v " << x + 1   << " " << y + 1   << " " << z       << "\n";  // v4: -4
        file << "v " << x + 1   << " " << y       << " " << z       << "\n";  // v5: -3
        file << "v " << x       << " " << y       << " " << z       << "\n";  // v6: -2
        file << "v " << x       << " " << y + 1   << " " << z       << "\n";  // v7: -1

        file << "f -8 -7 -6 -5\n";
        file << "f -4 -3 -2 -1\n";
        file << "f -3 -4 -6 -7\n";
        file << "f -2 -8 -5 -1\n";
        file << "f -2 -3 -7 -8\n";
        file << "f -1 -5 -6 -4\n";
    };

#if VOXELIZE_MODE == SERIAL
    std::vector<Point3i> occupiedVoxels;
    m_ocTree->getOccupiedVoxels(occupiedVoxels);
    std::cout << "Exporting " << occupiedVoxels.size() << " occupied voxels to OBJ file." << std::endl;

    for (const auto& voxel : occupiedVoxels)
        writeToFile(voxel.x, voxel.y, voxel.z);
#else
    int R = m_scaledBounds.xmax - m_scaledBounds.xmin + 3;
    int C = m_scaledBounds.ymax - m_scaledBounds.ymin + 3;
    // Iterate through voxel grid
    for (int x = m_scaledBounds.xmin; x <= m_scaledBounds.xmax + 1; ++x) {
        for (int y = m_scaledBounds.ymin; y <= m_scaledBounds.ymax + 1; ++y) {
            for (int z = m_scaledBounds.zmin; z <= m_scaledBounds.zmax + 1; ++z) {
                int idx = (x - m_scaledBounds.xmin) + (y - m_scaledBounds.ymin) * R + (z - m_scaledBounds.zmin) * R * C;
                if (idx >= 0 && idx < m_voxels.size() && m_voxels[idx] == 1)
                    writeToFile(x, y, z);
            }
        }
    }
#endif

    logFile.close();
    file.close();
    std::cout << "Exported voxel mesh to " << filename << std::endl;
}