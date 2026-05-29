#include "OcTree.h"
#include <cassert>
#include <climits>
#include <iostream>
#include <cmath>

OcTree ::OcTree(const BBox3i& bounds) : m_bounds(bounds) {
    m_nodes = {-1};
    m_voxelCount = 0;
}

OcTree::OcTree(const std::vector<unsigned char>& voxels, const BBox3i& bounds) : m_bounds(bounds) {
    m_nodes = {-1};
    m_voxelCount = 0;
    build(voxels);
}

bool OcTree::search(Point3i p) {
    return traverse(p, SEARCH);
}

bool OcTree::insert(Point3i p) {
    return traverse(p, INSERT);
}

bool OcTree::remove(Point3i p) {
    m_voxelCount--;
    return traverse(p, DELETE);
}

int OcTree::getVoxelCount() const {
    return m_voxelCount;
}

bool OcTree::inRange(Point3i p, const BBox3i& bounds) {
    return (p.x >= bounds.xmin && p.x <= bounds.xmax &&
            p.y >= bounds.ymin && p.y <= bounds.ymax &&
            p.z >= bounds.zmin && p.z <= bounds.zmax);
}

bool OcTree::isVoxel(const BBox3i& bounds) const {
    return (bounds.xmin == bounds.xmax &&
            bounds.ymin == bounds.ymax &&
            bounds.zmin == bounds.zmax);
}

Point3i OcTree::computeMid(const BBox3i& bounds) const {
    Point3i mid;
    mid.x = std::floor((bounds.xmin + bounds.xmax) / 2.f);
    mid.y = std::floor((bounds.ymin + bounds.ymax) / 2.f);
    mid.z = std::floor((bounds.zmin + bounds.zmax) / 2.f);
    return mid;
}

void OcTree::computeChildBounds(const BBox3i& parentBounds, const Point3i& mid, int childIndex, BBox3i& childBounds) const {
    childBounds.xmin = (childIndex / 4 == 0) ? parentBounds.xmin : mid.x + 1;
    childBounds.xmax = (childIndex / 4 == 0) ? mid.x : parentBounds.xmax;
    childBounds.ymin = (childIndex / 2 % 2 == 0) ? parentBounds.ymin : mid.y + 1;
    childBounds.ymax = (childIndex / 2 % 2 == 0) ? mid.y : parentBounds.ymax;
    childBounds.zmin = (childIndex % 2 == 0) ? parentBounds.zmin : mid.z + 1;
    childBounds.zmax = (childIndex % 2 == 0) ? mid.z : parentBounds.zmax;
}

void OcTree::build(const std::vector<unsigned char>& voxels) {
    int R = m_bounds.xmax - m_bounds.xmin + 3;
    int C = m_bounds.ymax - m_bounds.ymin + 3;
    for (int z = m_bounds.zmin; z <= m_bounds.zmax; z++) {
        for (int y = m_bounds.ymin; y <= m_bounds.ymax; y++) {
            for (int x = m_bounds.xmin; x <= m_bounds.xmax; x++) {
                int index = (z * R * C) + (y * R) + x;
                if (voxels[index] == 1){
                    traverse(Point3i(x, y, z), INSERT);
                    m_voxelCount++;
                }
            }
        }
    }
    std::cout << "Octree built with " << m_voxelCount << " voxels." << std::endl;
    std::cout << "Total nodes in octree: " << m_nodes.size() << std::endl;
    std::cout << "OcTree Memory usage (approx): " << (m_nodes.size() * sizeof(int)) << " Bytes" << std::endl;
    std::cout << "Voxel grid memory usage (approx): " << (voxels.size() * sizeof(unsigned char)) << " Bytes" << std::endl;
}

bool OcTree::traverse(Point3i p, unsigned char flag) {
    if(!inRange(p, m_bounds)) {
        return false; // Out of bounds
    }

    int nodeIndex = 0;
    BBox3i currentBounds = m_bounds;

    while (true) {
        assert(currentBounds.xmin <= currentBounds.xmax &&
               currentBounds.ymin <= currentBounds.ymax &&
               currentBounds.zmin <= currentBounds.zmax);

        assert(nodeIndex < m_nodes.size());

        if (isVoxel(currentBounds)) { // Leaf node
            if(flag == INSERT) {
                if (m_nodes[nodeIndex] != INT_MAX) {
                    m_nodes[nodeIndex] = INT_MAX;
                    m_voxelCount++;
                }
                return true;
            }
            else if(flag == DELETE) {
                m_nodes[nodeIndex] = -1; // Mark as empty
                m_voxelCount--;
                return true;
            }
            else {
                return m_nodes[nodeIndex] == INT_MAX; // Found during search if occupied
            }
        }

        int child0NodeId = m_nodes[nodeIndex];
        
        if (child0NodeId == -1) { // Leaf node
            if (flag == INSERT) {
                child0NodeId = m_nodes[nodeIndex] = m_nodes.size(); // Mark the node Id of the first child (convert to internal node)
                for (int i = 0; i < 8; i++) {
                    m_nodes.push_back(-1); // Initialize all 8 children as empty
                }
            } else {
                return false; // Not found during search
            }
        }

        int i = 0;
        Point3i mid = computeMid(currentBounds);
        for (i = 0; i < 8; i++) {
            int childNodeId = child0NodeId + i;
            assert(childNodeId < m_nodes.size());
            BBox3i childBounds;
            computeChildBounds(currentBounds, mid, i, childBounds);

            if (inRange(p, childBounds)) {
                nodeIndex = childNodeId;
                currentBounds = childBounds;
                break;
            }
        }
        assert(i < 8); // Should always find a child that contains the point
    }
    return true; // Found during search
}

void OcTree::getOccupiedVoxels(std::vector<Point3i>& occupiedVoxels) const {
    struct NodeInfo {
        int index;
        BBox3i bounds;
    };

    occupiedVoxels.clear();

    if(m_nodes[0] == -1) {
        return; // No occupied voxels
    }

    std::vector<NodeInfo> stack = {{0, m_bounds}}; // Start with root node

    while (!stack.empty()) {
        NodeInfo current = stack.back();
        stack.pop_back();

        int nodeValue = m_nodes[current.index];

        if (nodeValue == INT_MAX) {
            // Leaf node that is occupied, add to result
            assert(isVoxel(current.bounds));
            occupiedVoxels.push_back({current.bounds.xmin, current.bounds.ymin, current.bounds.zmin});
            continue;
        }

        Point3i mid = computeMid(current.bounds);
        // Internal node, add non-empty children to stack
        for (int i = 0; i < 8; i++) {
            int childNodeId = nodeValue + i;
            assert(childNodeId < m_nodes.size());
            if (childNodeId >= m_nodes.size() || m_nodes[childNodeId] == -1) {
                continue; // Out of bounds check
            }
            BBox3i childBounds;
            computeChildBounds(current.bounds, mid, i, childBounds);

            stack.push_back({childNodeId, childBounds});
        }
    }
    return;
}