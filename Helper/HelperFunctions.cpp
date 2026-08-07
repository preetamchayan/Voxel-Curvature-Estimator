#include "HelperFunctions.h"

#include <cmath>

void swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

Color hsv2rgb(const Color& hsv) {
    Color rgb(0.0f, 0.0f, 0.0f);
    Color normalizedHsv = hsv;

    normalizedHsv.r /= 360.0f;
    if (normalizedHsv.r < 0.0f) {
        normalizedHsv.r -= std::floor(normalizedHsv.r);
    }
    if (normalizedHsv.r >= 1.0f) {
        normalizedHsv.r = std::fmod(normalizedHsv.r, 1.0f);
    }

    if (normalizedHsv.g == 0.0f) {
        rgb.r = rgb.g = rgb.b = normalizedHsv.b;
        return rgb;
    }

    normalizedHsv.r *= 6.0f;
    const int i = static_cast<int>(std::floor(normalizedHsv.r));
    const float f = normalizedHsv.r - i;
    const float aa = normalizedHsv.b * (1.0f - normalizedHsv.g);
    const float bb = normalizedHsv.b * (1.0f - (normalizedHsv.g * f));
    const float cc = normalizedHsv.b * (1.0f - (normalizedHsv.g * (1.0f - f)));

    switch(i) {
        case 0: rgb.r = normalizedHsv.b; rgb.g = cc;              rgb.b = aa;              break;
        case 1: rgb.r = bb;              rgb.g = normalizedHsv.b; rgb.b = aa;              break;
        case 2: rgb.r = aa;              rgb.g = normalizedHsv.b; rgb.b = cc;              break;
        case 3: rgb.r = aa;              rgb.g = bb;              rgb.b = normalizedHsv.b; break;
        case 4: rgb.r = cc;              rgb.g = aa;              rgb.b = normalizedHsv.b; break;
        default: rgb.r = normalizedHsv.b; rgb.g = aa;             rgb.b = bb;              break;
    }

    return rgb;
}

void writePointToVoxel(Point3i point, std::ofstream& file) {
    int x = point.x, y = point.y, z = point.z;
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
}