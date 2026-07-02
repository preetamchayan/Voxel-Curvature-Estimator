#pragma once

#include <string>
#include <vector>
#include "../MeshVoxelizerBaseEnv.h"

class MeshVoxelizerMetalEnv : public MeshVoxelizerBaseEnv {
public:
    MeshVoxelizerMetalEnv();
    ~MeshVoxelizerMetalEnv();
    void voxelize(
        std::vector<unsigned char> &voxels,
        const std::vector<Point3i> &vertices,
        const std::vector<Face> &faces,
        const BBox3i &scaledBounds,
        const Dimensions3i &dims) override;

private:
    void createMetalObjects();
    void createBuffers(size_t faceCount, size_t vertexCount, size_t voxelCount);
    void createComputePipeline(const std::string& functionName);
    std::string loadShaderSource(const std::string& path);
    void checkMetalError(bool success, const char* message);

private:
    void* m_device = nullptr;
    void* m_commandQueue = nullptr;
    void* m_pipelineState = nullptr;
    void* m_facesBuffer = nullptr;
    void* m_verticesBuffer = nullptr;
    void* m_voxelsBuffer = nullptr;
    void* m_library = nullptr;
    void* m_function = nullptr;
    void* m_paramsBuffer = nullptr;
};