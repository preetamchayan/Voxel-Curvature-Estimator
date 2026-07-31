#pragma once

#include "GeometryTypes.h"

#include <fstream>

void swap (int* a, int* b);
Color hsv2rgb (Color& hsv);
void writePointToVoxel(Point3i point, std::ofstream& file);
