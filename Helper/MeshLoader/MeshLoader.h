#pragma once

#include <vector>
#include <array>
#include <string>
#include "../GeometryTypes.h"

class MeshLoader {
private:
    std::vector<Point3d> m_vertices;
    std::vector<Face> m_faces;
    BBox3d m_bounds;

public:
    bool load(const std::string& filename);
    const std::vector<Point3d>& getVertices() const;
    const std::vector<Face>& getFaces() const;
    const BBox3d& getBounds() const;
};