#include "MeshLoader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>
#include <algorithm>

bool MeshLoader::load(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return false;
    }

    m_bounds.xmin = m_bounds.ymin = m_bounds.zmin = std::numeric_limits<double>::max();
    m_bounds.xmax = m_bounds.ymax = m_bounds.zmax = std::numeric_limits<double>::lowest();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (line.size() >= 2 && line[0] == 'v' && line[1] == ' ') {
            std::istringstream stream(line.substr(2));
            Point3d vertex;
            stream >> vertex.x >> vertex.y >> vertex.z;
            m_vertices.push_back(vertex);
            // Update bounds
            m_bounds.xmin = std::min(m_bounds.xmin, vertex.x);
            m_bounds.xmax = std::max(m_bounds.xmax, vertex.x);
            m_bounds.ymin = std::min(m_bounds.ymin, vertex.y);
            m_bounds.ymax = std::max(m_bounds.ymax, vertex.y);
            m_bounds.zmin = std::min(m_bounds.zmin, vertex.z);
            m_bounds.zmax = std::max(m_bounds.zmax, vertex.z);
        } else if (line.size() >= 2 && line[0] == 'f' && line[1] == ' ') {
            std::istringstream stream(line.substr(2));
            Face face;
            stream >> face.v1 >> face.v2 >> face.v3;
            face.v1--; face.v2--; face.v3--; // OBJ indices are 1-based, convert to 0-based
            m_faces.push_back(face);
        }
    }

    file.close();
    return true;
}

const std::vector<Point3d>& MeshLoader::getVertices() const {
    return m_vertices;
}

const std::vector<Face>& MeshLoader::getFaces() const {
    return m_faces;
}

const BBox3d& MeshLoader::getBounds() const {
    return m_bounds;
}