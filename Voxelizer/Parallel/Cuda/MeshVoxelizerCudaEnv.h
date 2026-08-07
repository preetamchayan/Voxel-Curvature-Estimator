#pragma once

#include "../../MeshVoxelizerBaseEnv.h"
#include <cuda_runtime.h>

class MeshVoxelizerCudaEnv : public MeshVoxelizerBaseEnv {
public:
    MeshVoxelizerCudaEnv();
    ~MeshVoxelizerCudaEnv();
    void voxelize(std::vector<unsigned char> &voxels,
                  const std::vector<Point3i> &vertices,
                  const std::vector<Face> &faces,
                  const BBox3i &scaledBounds,
                  const Dimensions3i &dims) override;

private:
    void checkCudaError(cudaError_t error, const char* message);
    void init(const std::vector<int>& flatFaces,
              const std::vector<int>& flatVertices,
              const std::vector<unsigned char>& voxels,
              const BBox3i& scaledBounds,
              const Dimensions3i& dims);

private:
    const unsigned int* m_facesBuffer;
    const int* m_verticesBuffer;
    unsigned char* m_voxelsBuffer;
    unsigned int* m_totalSizeBuffer;
};