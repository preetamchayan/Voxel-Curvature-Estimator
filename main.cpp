#include "Helper/MeshLoader/MeshLoader.h"
#include "Voxelizer/Voxelizer.h"
#include "Helper/OcTree/OcTree.h"
#include <iostream>
#include <string>
#include <chrono>
#include <cassert>

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_obj> <output_obj>" << std::endl;
        std::cerr << "Example: " << argv[0] << " input.obj output.obj" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = argv[2];

    // Load mesh
    MeshLoader meshLoader;
    if (!meshLoader.load(inputFile)) {
        std::cerr << "Failed to load mesh from: " << inputFile << std::endl;
        return 1;
    }

    std::cout << "Loaded mesh with " << meshLoader.getVertices().size() << " vertices and "
              << meshLoader.getFaces().size() << " faces." << std::endl;

    // Voxelize
    Voxelizer voxelizer(meshLoader.getBounds());

    // Get recommended scale factor range
    int s_low, s_high;
    voxelizer.getRecommendedScaleRange(s_low, s_high);
    std::cout << "Recommended scale factor range: " << s_low << " to " << s_high << std::endl;
    
    float scale;
    std::cout << "Enter scale factor: ";
    std::cin >> scale;
    if (scale <= 0) {
        std::cerr << "Invalid scale factor. Must be positive." << std::endl;
        return 1;
    }
    voxelizer.voxelize(meshLoader, scale);

    std::cout << "Voxelization complete. Total voxels generated: " << voxelizer.getVoxelCount() << std::endl;

    // Export to OBJ
    voxelizer.exportVoxelsOBJ(outputFile);

    // BBox3i bounds = voxelizer.getSceneBounds();
    // OcTree octree(voxelizer.getVoxels(), bounds);

    // int count = 0;
    // double avgSearchTime = 0.0;
    // for(int x = bounds.xmin; x <= bounds.xmax; x++) {
    //     for(int y = bounds.ymin; y <= bounds.ymax; y++) {
    //         for(int z = bounds.zmin; z <= bounds.zmax; z++) {
    //             Point3i p{x, y, z};
    //             std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    //             bool found = voxelizer.search(p);
    //             std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //             std::chrono::duration<double> elapsed_seconds = end - start;
    //             avgSearchTime += elapsed_seconds.count();
    //             if (found) {
    //                 // std::cout << "Voxel at (" << x << ", " << y << ", " << z << ") is occupied." << std::endl;
    //                 count++;
    //                 bool removed = voxelizer.remove(p);
    //                 assert(removed);
    //                 found = voxelizer.search(p);
    //                 assert(!found);
    //             }
    //             // std::cout << "Search for (" << x << ", " << y << ", " << z << ") took " << elapsed_seconds.count() << " seconds." << std::endl;
    //         }
    //     }
    // }
    // std::cout << "Total occupied voxels found in octree search: " << count << std::endl;
    // std::cout << "Average search time: " << avgSearchTime / (bounds.xmax - bounds.xmin + 1) / (bounds.ymax - bounds.ymin + 1) / (bounds.zmax - bounds.zmin + 1) << " seconds" << std::endl;

    return 0;
}