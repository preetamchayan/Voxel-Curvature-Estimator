#include "HelperFunctions.h"

#include <cmath>

void swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

Color hsv2rgb (Color& hsv) {
    Color rgb;
	hsv.r /= 360;
	int i;
	float aa, bb, cc, f;
	if (hsv.g == 0) rgb.r = rgb.g = rgb.b = hsv.b;
	else {
		if (hsv.r == 1.0) hsv.r == 0;
		hsv.r *= 6.0;
		i = std::floor(hsv.r);
		f = hsv.r-i;
		aa = hsv.b * (1 - hsv.g);
		bb = hsv.b * (1 - (hsv.g * f));
		cc = hsv.b * (1 - (hsv.g * (1 - f)));
		switch(i) {
			case 0: rgb.r = hsv.b; rgb.g = cc; rgb.b = aa; break;
			case 1: rgb.r = bb; rgb.g = hsv.b; rgb.b = aa; break;
			case 2: rgb.r = aa; rgb.g = hsv.b; rgb.b = cc; break;
			case 3: rgb.r = aa; rgb.g = bb; rgb.b = hsv.b; break;
			case 4: rgb.r = cc; rgb.g = aa; rgb.b = hsv.b; break;
			case 5: rgb.r = hsv.b; rgb.g = aa; rgb.b = bb; break;
		}
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