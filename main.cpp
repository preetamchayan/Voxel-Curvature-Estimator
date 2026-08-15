#include "Helper/MeshLoader/MeshLoader.h"
#include "Voxelizer/MeshVoxelizer.h"
#include "CurvatureEstimator/CurvatureEstimator.h"
#include <iostream>
#include <string>
#include <chrono>
#include <cassert>
#include <filesystem>

int main(int argc, char* argv[]) {
        if (argc != 5 && argc != 6) {
        std::cerr << "Usage: " << argv[0] << " <input_obj> <output_voxel_obj> <output_curvature_obj> <output_curvature_mtl> [output_curvature_log]" << std::endl;
        std::cerr << "Example: " << argv[0] << " input.obj outputVoxel.obj outputCurvature.obj outputCurvature.mtl curvature.log" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFileVoxel = argv[2];
    std::string outputFileCurvature = argv[3];
    std::string outputFileCurvatureMaterial = argv[4];
    std::string outputFileCurvatureLog = argc == 6 ? argv[5] : "";
    if (outputFileCurvatureLog.empty()) {
        outputFileCurvatureLog = "output/curvature.log";
    }
    const std::filesystem::path curvatureLogPath(outputFileCurvatureLog);
    if (!curvatureLogPath.parent_path().empty()) {
        std::filesystem::create_directories(curvatureLogPath.parent_path());
    }

    // Load mesh
    MeshLoader meshLoader;
    if (!meshLoader.load(inputFile)) {
        std::cerr << "Failed to load mesh from: " << inputFile << std::endl;
        return 1;
    }

    std::cout << "Loaded mesh with " << meshLoader.getVertices().size() << " vertices and "
              << meshLoader.getFaces().size() << " faces." << std::endl;

    // Voxelize
    MeshVoxelizer voxelizer(meshLoader.getBounds());

    int s_low, s_high;
    voxelizer.getRecommendedScaleRange(s_low, s_high);
    std::cout << "Recommended scale range: [" << s_low << ", " << s_high << "]" << std::endl;
    
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
    // voxelizer.exportVoxelsOBJ(outputFileVoxel);

    int curveLength;
    std::cout << "Enter curve length for curvature estimation: ";
    std::cin >> curveLength;
    //Estimate curvature
    CurvatureEstimator curvatureEstimator(
        voxelizer.getVoxels(),
        voxelizer.getSceneBounds(),
        voxelizer.getDimensions()
    );
    const auto curvatureStart = std::chrono::steady_clock::now();
    curvatureEstimator.estimateCurvature(curveLength);
    const auto curvatureEnd = std::chrono::steady_clock::now();

    const auto curvatureMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            curvatureEnd - curvatureStart
        ).count();

    std::cout << "CURVATURE_TIME_MS=" << curvatureMilliseconds << std::endl;
    curvatureEstimator.exportCurvatureLog(outputFileCurvatureLog);
    curvatureEstimator.exportCurvatureOBJ(outputFileCurvature, outputFileCurvatureMaterial);

    return 0;
}