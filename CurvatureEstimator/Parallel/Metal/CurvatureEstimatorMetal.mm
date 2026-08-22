#import <Metal/Metal.h>

#include "CurvatureEstimatorMetal.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
constexpr int kMaxMetalCurveLength = 32;

std::string resolveMetalShaderPath(const std::string& path) {
    std::ifstream file(path);
    if (file.is_open()) {
        return path;
    }

    std::string sourcePath = __FILE__;
    const size_t pos = sourcePath.find_last_of("/\\");
    const std::string directory = (pos != std::string::npos) ? sourcePath.substr(0, pos) : std::string();
    const std::string relativePath = directory + "/" + path;
    file.open(relativePath);
    if (file.is_open()) {
        return relativePath;
    }

    const std::string projectRelative = std::string("CurvatureEstimator/Parallel/Metal/") + path;
    file.open(projectRelative);
    if (file.is_open()) {
        return projectRelative;
    }

    return path;
}

inline int getVoxelID(int x, int y, int z, int R, int C) {
    return x + y * R + z * R * C;
}

using ScalarArg = std::pair<const void*, size_t>;

void dispatchKernel(id<MTLDevice> device,
                    id<MTLCommandQueue> commandQueue,
                    id<MTLLibrary> library,
                    const std::string& functionName,
                    const MTLSize& threadsPerThreadgroup,
                    const MTLSize& gridSize,
                    const std::vector<id<MTLBuffer>>& bufferArgs,
                    const std::vector<ScalarArg>& scalarArgs) {
    NSError* error = nil;
    id<MTLFunction> function = [library newFunctionWithName:[NSString stringWithUTF8String:functionName.c_str()]];
    if (!function) {
        throw std::runtime_error("Metal curvature shader missing function: " + functionName);
    }

    id<MTLComputePipelineState> pipeline = [device newComputePipelineStateWithFunction:function error:&error];
    if (!pipeline) {
        std::cerr << "Metal shader compile error for " << functionName << ": "
                  << [[error localizedDescription] UTF8String] << std::endl;
        throw std::runtime_error("Failed to create Metal curvature pipeline: " + functionName);
    }

    id<MTLCommandBuffer> commandBuffer = [commandQueue commandBuffer];
    id<MTLComputeCommandEncoder> encoder = [commandBuffer computeCommandEncoder];
    [encoder setComputePipelineState:pipeline];

    for (NSUInteger i = 0; i < bufferArgs.size(); ++i) {
        [encoder setBuffer:bufferArgs[i] offset:0 atIndex:i];
    }

    NSUInteger nextIndex = static_cast<NSUInteger>(bufferArgs.size());
    for (const auto& scalarArg : scalarArgs) {
        [encoder setBytes:scalarArg.first length:scalarArg.second atIndex:nextIndex++];
    }

    [encoder dispatchThreadgroups:gridSize threadsPerThreadgroup:threadsPerThreadgroup];
    [encoder endEncoding];
    [commandBuffer commit];
    [commandBuffer waitUntilCompleted];
}
}

CurvatureEstimatorMetal::CurvatureEstimatorMetal() {
    std::cout << "Initializing Metal curvature environment..." << std::endl;
    createMetalObjects();
}

CurvatureEstimatorMetal::~CurvatureEstimatorMetal() {
    std::cout << "Cleaning up Metal curvature environment..." << std::endl;
    if (m_library) {
        id<MTLLibrary> library = (id<MTLLibrary>)m_library;
        [library release];
        m_library = nullptr;
    }
    if (m_commandQueue) {
        id<MTLCommandQueue> commandQueue = (id<MTLCommandQueue>)m_commandQueue;
        [commandQueue release];
        m_commandQueue = nullptr;
    }
    if (m_device) {
        id<MTLDevice> device = (id<MTLDevice>)m_device;
        [device release];
        m_device = nullptr;
    }
}

void CurvatureEstimatorMetal::createMetalObjects() {
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    checkMetalError(device != nil, "Failed to get Metal device");
    m_device = (void*)device;

    id<MTLCommandQueue> queue = [device newCommandQueue];
    checkMetalError(queue != nil, "Failed to create Metal command queue");
    m_commandQueue = (void*)queue;

    std::string shaderSource = loadShaderSource("CurvatureEstimatorKernel.metal");
    NSError* error = nil;
    NSString* source = [NSString stringWithUTF8String:shaderSource.c_str()];
    id<MTLLibrary> library = [device newLibraryWithSource:source options:nil error:&error];
    if (library == nil || error != nil) {
        std::cerr << "Metal Error: Failed to compile curvature shader library" << std::endl;
        if (error != nil) {
            std::cerr << "Metal compiler error: " << [[error localizedDescription] UTF8String] << std::endl;
        }
        throw std::runtime_error("Failed to compile Metal curvature shader library");
    }
    m_library = (void*)library;
}

std::string CurvatureEstimatorMetal::loadShaderSource(const std::string& path) const {
    const std::string resolvedPath = resolveMetalShaderPath(path);
    std::ifstream file(resolvedPath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open Metal curvature kernel source: " + path);
    }

    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}

void CurvatureEstimatorMetal::checkMetalError(bool success, const char* message) const {
    if (!success) {
        throw std::runtime_error(std::string("Metal error at ") + message);
    }
}

void CurvatureEstimatorMetal::preprocessVoxels(std::vector<unsigned char>& voxels,
                                              const Dimensions3i& dims) {
    std::cout << "Preprocessing curvature voxels using Metal..." << std::endl;

    if (voxels.empty()) {
        return;
    }

    id<MTLDevice> device = (id<MTLDevice>)m_device;
    id<MTLCommandQueue> queue = (id<MTLCommandQueue>)m_commandQueue;
    id<MTLLibrary> library = (id<MTLLibrary>)m_library;

    id<MTLBuffer> voxelsBuffer = [device newBufferWithBytes:voxels.data()
                                                    length:voxels.size() * sizeof(unsigned char)
                                                   options:MTLResourceStorageModeShared];
    checkMetalError(voxelsBuffer != nil, "Failed to allocate Metal preprocess voxels buffer");

    const auto dispatchPlane = [&](const std::string& kernelName, int R, int C, int D, int plane) {
        const MTLSize threadsPerThreadgroup = MTLSizeMake(8, 8, 1);
        const MTLSize gridSize = MTLSizeMake((R + 7) / 8, (C + 7) / 8, 1);
        dispatchKernel(device,
                      queue,
                      library,
                      kernelName,
                      threadsPerThreadgroup,
                      gridSize,
                      { voxelsBuffer },
                      { { &R, sizeof(R) }, { &C, sizeof(C) }, { &D, sizeof(D) }, { &plane, sizeof(plane) } });
    };

    dispatchPlane("computeInnerSpaceVoxels", dims.width, dims.height, dims.depth, 2);
    dispatchPlane("computeInnerSpaceVoxels", dims.depth, dims.width, dims.height, 1);
    dispatchPlane("computeInnerSpaceVoxels", dims.height, dims.depth, dims.width, 0);

    const MTLSize block3D = MTLSizeMake(8, 8, 8);
    const MTLSize grid3D = MTLSizeMake((dims.width + 7) / 8, (dims.height + 7) / 8, (dims.depth + 7) / 8);
    dispatchKernel(device,
                  queue,
                  library,
                  "markInteriorVoxels",
                  block3D,
                  grid3D,
                  { voxelsBuffer },
                  { { &dims.width, sizeof(dims.width) },
                    { &dims.height, sizeof(dims.height) },
                    { &dims.depth, sizeof(dims.depth) } });
    dispatchKernel(device,
                  queue,
                  library,
                  "computeFrontierVoxels",
                  block3D,
                  grid3D,
                  { voxelsBuffer },
                  { { &dims.width, sizeof(dims.width) },
                    { &dims.height, sizeof(dims.height) },
                    { &dims.depth, sizeof(dims.depth) } });

    std::memcpy(voxels.data(), [voxelsBuffer contents], voxels.size() * sizeof(unsigned char));
    [voxelsBuffer release];
}

void CurvatureEstimatorMetal::estimateCurvature(int curvLength,
                                               const std::vector<unsigned char>& voxels,
                                               std::vector<int>& curvatures,
                                               const Dimensions3i& dims) {
    std::cout << "Estimating curvature using Metal..." << std::endl;

    if (curvLength < 1) {
        throw std::runtime_error("curve length must be positive");
    }
    if (curvLength > kMaxMetalCurveLength) {
        throw std::runtime_error("Metal curvature curve length exceeds MAX_CURVE_LENGTH");
    }
    if (curvatures.size() != voxels.size()) {
        throw std::runtime_error("curvatures and voxels must have the same grid size");
    }

    const size_t voxelCount = voxels.size();
    std::vector<int> surfaceVoxelIds;
    surfaceVoxelIds.reserve(voxelCount);
    for (size_t id = 0; id < voxelCount; ++id) {
        if (voxels[id] == 1) {
            surfaceVoxelIds.push_back(static_cast<int>(id));
        }
    }

    std::fill(curvatures.begin(), curvatures.end(), std::numeric_limits<int>::max());
    if (surfaceVoxelIds.empty()) {
        return;
    }

    id<MTLDevice> device = (id<MTLDevice>)m_device;
    id<MTLCommandQueue> queue = (id<MTLCommandQueue>)m_commandQueue;
    id<MTLLibrary> library = (id<MTLLibrary>)m_library;

    id<MTLBuffer> voxelsBuffer = [device newBufferWithBytes:voxels.data()
                                                    length:voxels.size() * sizeof(unsigned char)
                                                   options:MTLResourceStorageModeShared];
    id<MTLBuffer> curvaturesBuffer = [device newBufferWithBytes:curvatures.data()
                                                        length:curvatures.size() * sizeof(int)
                                                       options:MTLResourceStorageModeShared];
    id<MTLBuffer> surfaceVoxelIdsBuffer = [device newBufferWithBytes:surfaceVoxelIds.data()
                                                             length:surfaceVoxelIds.size() * sizeof(int)
                                                            options:MTLResourceStorageModeShared];

    const int surfaceVoxelCount = static_cast<int>(surfaceVoxelIds.size());
    const MTLSize threadsPerThreadgroup = MTLSizeMake(256, 1, 1);
    const MTLSize gridSize = MTLSizeMake((surfaceVoxelCount + 255) / 256, 1, 1);
    dispatchKernel(device,
                  queue,
                  library,
                  "estimateCurvature",
                  threadsPerThreadgroup,
                  gridSize,
                  { voxelsBuffer, curvaturesBuffer, surfaceVoxelIdsBuffer },
                  { { &surfaceVoxelCount, sizeof(surfaceVoxelCount) },
                    { &curvLength, sizeof(curvLength) },
                    { &dims.width, sizeof(dims.width) },
                    { &dims.height, sizeof(dims.height) },
                    { &dims.depth, sizeof(dims.depth) } });

    std::memcpy(curvatures.data(), [curvaturesBuffer contents], curvatures.size() * sizeof(int));
    [voxelsBuffer release];
    [curvaturesBuffer release];
    [surfaceVoxelIdsBuffer release];
}
