#pragma once

template <typename T>
struct Point2 {
    T x, y;
    Point2() = default;
    Point2(T x, T y) : x(x), y(y) {}
};

using Point2i = Point2<int>;
using Point2f = Point2<float>;
using Point2d = Point2<double>;

template <typename T>
struct Point3 {
    T x, y, z;
    Point3() = default;
    Point3(T x, T y, T z) : x(x), y(y), z(z) {}
};

using Point3i = Point3<int>;
using Point3f = Point3<float>;
using Point3d = Point3<double>;

struct Face {
    int v1, v2, v3;
};

template <typename T>
struct BBox2 {
    T xmin, xmax, ymin, ymax;
};

using BBox2i = BBox2<int>;
using BBox2f = BBox2<float>;
using BBox2d = BBox2<double>;

template <typename T>
struct BBox3 {
    T xmin, xmax, ymin, ymax, zmin, zmax;
};

using BBox3i = BBox3<int>;
using BBox3f = BBox3<float>;
using BBox3d = BBox3<double>;

template <typename T>
struct Dimensions3 {
    T width, height, depth;
};

using Dimensions3i = Dimensions3<int>;
using Dimensions3f = Dimensions3<float>;
using Dimensions3d = Dimensions3<double>;

template <typename T>
struct Dimensions2 {
    T width, height;
};

using Dimensions2i = Dimensions2<int>;
using Dimensions2f = Dimensions2<float>;
using Dimensions2d = Dimensions2<double>;

struct Plane {
    double a, b, c, d; // ax + by + cz + d = 0
};

struct Values {
    int val1, val2;
};

struct Flags {
    int flag1, flag2, flag3;
};