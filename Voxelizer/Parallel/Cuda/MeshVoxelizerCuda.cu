#include "MeshVoxelizerCuda.h"
#include <iostream>

#include "MeshVoxelizerKernel.h"

MeshVoxelizerCuda::MeshVoxelizerCuda() {
    // Initialize Cuda context, device, and other resources here
    std::cout << "Initializing Cuda environment..." << std::endl;
}

MeshVoxelizerCuda::~MeshVoxelizerCuda() {
    // Clean up Cuda resources here
    std::cout << "Cleaning up Cuda environment..." << std::endl;
}

void MeshVoxelizerCuda::checkCudaError(cudaError_t error, const char* message) {
    if (error != cudaSuccess) {
        std::cerr << "Cuda error " << error << " at " << message << ": " << cudaGetErrorString(error) << std::endl;
        exit(1);
    }
}

void MeshVoxelizerCuda::init(
    const std::vector<int>& flatFaces,
    const std::vector<int>& flatVertices,
    const std::vector<unsigned char>& voxels,
    const BBox3i& scaledBounds,
    const Dimensions3i& dims)
{
    // Initialize Cuda context and device here
    checkCudaError(cudaSetDevice(0), "Failed to set Cuda device");

    // Allocate device buffers
    checkCudaError(cudaMalloc((void**)&m_facesBuffer, sizeof(unsigned int) * flatFaces.size()), "Failed to allocate faces buffer on device");
    checkCudaError(cudaMalloc((void**)&m_verticesBuffer, sizeof(int) * flatVertices.size()), "Failed to allocate vertices buffer on device");
    checkCudaError(cudaMalloc((void**)&m_voxelsBuffer, sizeof(unsigned char) * voxels.size()), "Failed to allocate voxels buffer on device");
    checkCudaError(cudaMalloc((void**)&m_totalSizeBuffer, sizeof(unsigned int)), "Failed to allocate total size buffer on device");

    checkCudaError(cudaMemcpy((void*)m_facesBuffer, flatFaces.data(), sizeof(unsigned int) * flatFaces.size(), cudaMemcpyHostToDevice), "Failed to copy faces to device");
    checkCudaError(cudaMemcpy((void*)m_verticesBuffer, flatVertices.data(), sizeof(int) * flatVertices.size(), cudaMemcpyHostToDevice), "Failed to copy vertices to device");
    checkCudaError(cudaMemcpy((void*)m_voxelsBuffer, voxels.data(), sizeof(unsigned char) * voxels.size(), cudaMemcpyHostToDevice), "Failed to copy voxels to device");
    checkCudaError(cudaMemset((void*)m_totalSizeBuffer, 0, sizeof(unsigned int)), "Failed to initialize total size buffer");
}

void MeshVoxelizerCuda::voxelize(
    std::vector<unsigned char> &voxels,
    const std::vector<Point3i> &vertices,
    const std::vector<Face> &faces,
    const BBox3i &scaledBounds,
    const Dimensions3i &dims)
{
    // Implement voxelization using Cuda kernels here
    std::cout << "Voxelizing using Cuda..." << std::endl;

    std::vector<int> flatFaces;
    for (const auto &f : faces)
    {
        flatFaces.push_back(f.v1);
        flatFaces.push_back(f.v2);
        flatFaces.push_back(f.v3);
    }
    std::vector<int> flatVertices;
    for (const auto &v : vertices)
    {
        flatVertices.push_back(v.x);
        flatVertices.push_back(v.y);
        flatVertices.push_back(v.z);
    }

    init(flatFaces, flatVertices, voxels, scaledBounds, dims);

    // Launch Cuda kernel to perform voxelization here
    dim3 blockSize(1024);
    dim3 gridSize((faces.size() + blockSize.x - 1) / blockSize.x);
    voxelizeFace<<<gridSize, blockSize>>>(m_facesBuffer, (int)faces.size(), m_verticesBuffer, m_voxelsBuffer, dims.width, dims.height, dims.depth, scaledBounds.xmin, scaledBounds.ymin, scaledBounds.zmin, m_totalSizeBuffer);
    checkCudaError(cudaGetLastError(), "Failed to launch voxelization kernel");
    checkCudaError(cudaDeviceSynchronize(), "Failed to synchronize after kernel launch");

    // Copy results back to host
    checkCudaError(cudaMemcpy(voxels.data(), (void*)m_voxelsBuffer, sizeof(unsigned char) * voxels.size(), cudaMemcpyDeviceToHost), "Failed to copy voxels back to host");
    unsigned int totalSize;
    checkCudaError(cudaMemcpy(&totalSize, (void*)m_totalSizeBuffer, sizeof(unsigned int), cudaMemcpyDeviceToHost), "Failed to copy total size back to host");
    
    std::cout << "Total faces processed: " << totalSize << std::endl;
}