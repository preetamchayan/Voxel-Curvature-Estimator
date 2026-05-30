#include "Helper/MeshLoader/MeshLoader.h"
#include "Voxelizer/Voxelizer.h"
#include "CurvatureEstimator/CurvatureEstimator.h"
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

    //Estimate curvature
    // CurvatureEstimator curvatureEstimator(voxelizer.getVoxels(), voxelizer.getSceneBounds());

    return 0;
}