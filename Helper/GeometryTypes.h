#pragma once

#include <cmath>

template <typename T>
struct Point2 {
    T x, y;
    Point2() = default;
    Point2(T x, T y) : x(x), y(y) {}
    Point2<T> operator-(const Point2<T>& other) const {
        return Point2<T>(x - other.x, y - other.y);
    }
    Point2<T> operator+(const Point2<T>& other) const {
        return Point2<T>(x + other.x, y + other.y);
    }
    Point2<T> operator*(T scalar) const {
        return Point2<T>(x * scalar, y * scalar);
    }
    Point2<T> operator/(T scalar) const {
        return Point2<T>(x / scalar, y / scalar);
    }
    Point2<T> operator=(const Point2<T>& other) {
        x = other.x;
        y = other.y;
        return *this;
    }
    bool operator==(const Point2<T>& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point2<T>& other) const {
        return !(*this == other);
    }
    T dot(const Point2<T>& other) const {
        return x * other.x + y * other.y;
    }
    T cross(const Point2<T>& other) const {
        return x * other.y - y * other.x;
    }
    T length() const {
        return std::sqrt(x * x + y * y);
    }
    Point2<T> normalized() const {
        T len = length();
        if (len == 0) return Point2<T>(0, 0);
        return Point2<T>(x / len, y / len);
    }
};

using Point2i = Point2<int>;
using Point2f = Point2<float>;
using Point2d = Point2<double>;

template <typename T>
struct Point3 {
    T x, y, z;
    Point3() = default;
    Point3(T x, T y, T z) : x(x), y(y), z(z) {}
    Point3<T> operator-(const Point3<T>& other) const {
        return Point3<T>(x - other.x, y - other.y, z - other.z);
    }
    Point3<T> operator+(const Point3<T>& other) const {
        return Point3<T>(x + other.x, y + other.y, z + other.z);
    }
    Point3<T> operator*(T scalar) const {
        return Point3<T>(x * scalar, y * scalar, z * scalar);
    }
    Point3<T> operator/(T scalar) const {
        return Point3<T>(x / scalar, y / scalar, z / scalar);
    }
    Point3<T> operator=(const Point3<T>& other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }
    bool operator==(const Point3<T>& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    bool operator!=(const Point3<T>& other) const {
        return !(*this == other);
    }
    T dot(const Point3<T>& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    Point3<T> cross(const Point3<T>& other) const {
        return Point3<T>(y * other.z - z * other.y,
                         z * other.x - x * other.z,
                         x * other.y - y * other.x);
    }
    T length() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    Point3<T> normalized() const {
        T len = length();
        if (len == 0) return Point3<T>(0, 0, 0);
        return Point3<T>(x / len, y / len, z / len);
    }
};

using Point3i = Point3<int>;
using Point3f = Point3<float>;
using Point3d = Point3<double>;

struct Face {
    int v1, v2, v3;
};

struct Color {
    float r, g, b;
    Color() = default;
    Color(float r, float g, float b) : r(r), g(g), b(b) {}
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
    Dimensions3<T> operator-(const Dimensions3<T>& other) const {
        return Dimensions3<T>(width - other.width, height - other.height, depth - other.depth);
    }
    Dimensions3<T> operator+(const Dimensions3<T>& other) const {
        return Dimensions3<T>(width + other.width, height + other.height, depth + other.depth);
    }
    Dimensions3<T> operator*(T scalar) const {
        return Dimensions3<T>(width * scalar, height * scalar, depth * scalar);
    }
    Dimensions3<T> operator/(T scalar) const {
        return Dimensions3<T>(width / scalar, height / scalar, depth / scalar);
    }
    Dimensions3<T> operator=(const Dimensions3<T>& other) {
        width = other.width;
        height = other.height;
        depth = other.depth;
        return *this;
    }
    bool operator==(const Dimensions3<T>& other) const {
        return width == other.width && height == other.height && depth == other.depth;
    }
    bool operator!=(const Dimensions3<T>& other) const {
        return !(*this == other);
    }
    bool operator<(const Dimensions3<T>& other) const {
        return width < other.width && height < other.height && depth < other.depth;
    }
    bool operator>(const Dimensions3<T>& other) const {
        return width > other.width && height > other.height && depth > other.depth;
    }
    bool operator<=(const Dimensions3<T>& other) const {
        return width <= other.width && height <= other.height && depth <= other.depth;
    }
    bool operator>=(const Dimensions3<T>& other) const {
        return width >= other.width && height >= other.height && depth >= other.depth;
    }
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