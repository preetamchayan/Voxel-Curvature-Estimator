#import <Metal/Metal.h>
#include "MeshVoxelizerMetal.h"
#include "../../../Helper/GeometryTypes.h"
#include <iostream>
#include <fstream>
#include <sstream>

MeshVoxelizerMetal::MeshVoxelizerMetal() {
    std::cout << "Initializing Metal environment..." << std::endl;
    createMetalObjects();
}

MeshVoxelizerMetal::~MeshVoxelizerMetal() {
    std::cout << "Cleaning up Metal environment..." << std::endl;
    // Release Metal objects
    if (m_device) {
        id<MTLDevice> device = (id<MTLDevice>)m_device;
        [device release];
        m_device = nullptr;
    }
    if (m_commandQueue) {
        id<MTLCommandQueue> queue = (id<MTLCommandQueue>)m_commandQueue;
        [queue release];
        m_commandQueue = nullptr;
    }
    if (m_library) {
        id<MTLLibrary> library = (id<MTLLibrary>)m_library;
        [library release];
        m_library = nullptr;
    }
    if (m_function) {
        id<MTLFunction> function = (id<MTLFunction>)m_function;
        [function release];
        m_function = nullptr;
    }
    if (m_pipelineState) {
        id<MTLComputePipelineState> pipeline = (id<MTLComputePipelineState>)m_pipelineState;
        [pipeline release];
        m_pipelineState = nullptr;
    }
    if (m_facesBuffer) {
        id<MTLBuffer> buffer = (id<MTLBuffer>)m_facesBuffer;
        [buffer release];
        m_facesBuffer = nullptr;
    }
    if (m_verticesBuffer) {
        id<MTLBuffer> buffer = (id<MTLBuffer>)m_verticesBuffer;
        [buffer release];
        m_verticesBuffer = nullptr;
    }
    if (m_voxelsBuffer) {
        id<MTLBuffer> buffer = (id<MTLBuffer>)m_voxelsBuffer;
        [buffer release];
        m_voxelsBuffer = nullptr;
    }
    if (m_paramsBuffer) {
        id<MTLBuffer> buffer = (id<MTLBuffer>)m_paramsBuffer;
        [buffer release];
        m_paramsBuffer = nullptr;
    }
}

void MeshVoxelizerMetal::createMetalObjects() {
    // Get default Metal device
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    checkMetalError(device != nil, "Failed to get Metal device");
    m_device = (void*)device;
    
    // Create command queue
    id<MTLCommandQueue> commandQueue = [device newCommandQueue];
    checkMetalError(commandQueue != nil, "Failed to create command queue");
    m_commandQueue = (void*)commandQueue;
}

void MeshVoxelizerMetal::createBuffers(size_t faceCount, size_t vertexCount, size_t voxelCount) {
    id<MTLDevice> device = (id<MTLDevice>)m_device;
    
    // Calculate buffer sizes
    // Note: voxels use uint for atomic operations (4 bytes per voxel instead of 1)
    size_t facesSize = faceCount * sizeof(Face);
    size_t verticesSize = vertexCount * sizeof(Point3i);
    size_t voxelsSize = voxelCount * sizeof(uint);  // Use uint for atomic operations
    
    // Create faces buffer
    id<MTLBuffer> facesBuffer = [device newBufferWithLength:facesSize options:MTLResourceStorageModeShared];
    checkMetalError(facesBuffer != nil, "Failed to create faces buffer");
    m_facesBuffer = (void*)facesBuffer;
    
    // Create vertices buffer
    id<MTLBuffer> verticesBuffer = [device newBufferWithLength:verticesSize options:MTLResourceStorageModeShared];
    checkMetalError(verticesBuffer != nil, "Failed to create vertices buffer");
    m_verticesBuffer = (void*)verticesBuffer;
    
    // Create voxels buffer (as uint for atomic operations)
    id<MTLBuffer> voxelsBuffer = [device newBufferWithLength:voxelsSize options:MTLResourceStorageModeShared];
    checkMetalError(voxelsBuffer != nil, "Failed to create voxels buffer");
    m_voxelsBuffer = (void*)voxelsBuffer;
    
    // Create params buffer (for grid dimensions and bounds)
    size_t paramsSize = sizeof(int) * 12; // 3 for bounds min, 3 for bounds max, 3 for dims, 3 for padding
    id<MTLBuffer> paramsBuffer = [device newBufferWithLength:paramsSize options:MTLResourceStorageModeShared];
    checkMetalError(paramsBuffer != nil, "Failed to create params buffer");
    m_paramsBuffer = (void*)paramsBuffer;
}

void MeshVoxelizerMetal::createComputePipeline(const std::string& functionName) {
    id<MTLDevice> device = (id<MTLDevice>)m_device;
    
    // Load shader source
    std::string shaderSource = loadShaderSource("MeshVoxelizerKernel.metal");
    
    // Compile shader to library
    NSError* error = nil;
    NSString* sourceStr = [NSString stringWithUTF8String:shaderSource.c_str()];
    id<MTLLibrary> library = [device newLibraryWithSource:sourceStr options:nil error:&error];
    if (library == nil || error != nil) {
        std::cerr << "Metal Error: Failed to compile Metal shader library" << std::endl;
        if (error != nil) {
            const char* desc = [[error localizedDescription] UTF8String];
            const char* reason = [[error localizedFailureReason] UTF8String];
            const char* suggestion = [[error localizedRecoverySuggestion] UTF8String];
            std::cerr << "Metal compiler error: " << (desc ? desc : "(no description)") << std::endl;
            if (reason) std::cerr << "Failure reason: " << reason << std::endl;
            if (suggestion) std::cerr << "Recovery suggestion: " << suggestion << std::endl;
            NSDictionary* info = [error userInfo];
            if (info) {
                NSLog(@"Metal compiler userInfo: %@", info);
            }
        }
        // Print shader source for diagnostics
        if (!shaderSource.empty()) {
            std::cerr << "--- Shader source begin ---\n" << shaderSource << "\n--- Shader source end ---\n";
        } else {
            std::cerr << "Shader source was empty." << std::endl;
        }
        return;
    }
    m_library = (void*)library;
    
    // Get compute function
    NSString* funcName = [NSString stringWithUTF8String:functionName.c_str()];
    id<MTLFunction> function = [library newFunctionWithName:funcName];
    checkMetalError(function != nil, "Failed to get compute function from library");
    m_function = (void*)function;
    
    // Create compute pipeline
    error = nil;
    id<MTLComputePipelineState> pipelineState = [device newComputePipelineStateWithFunction:function error:&error];
    checkMetalError(pipelineState != nil && error == nil, "Failed to create compute pipeline state");
    m_pipelineState = (void*)pipelineState;
}

std::string MeshVoxelizerMetal::loadShaderSource(const std::string& path) {
    // Read the file (try given path, source dir, and project-relative locations)
    std::ifstream file(path);
    std::string srcFile = __FILE__;
    size_t pos = srcFile.find_last_of("/\\");
    std::string srcDir = (pos != std::string::npos) ? srcFile.substr(0, pos) : std::string();
    std::string altPath = srcDir + "/" + path;
    std::string altPath2 = std::string("Voxelizer/Parallel/Metal/") + path;
    if (!file.is_open()) {
        file.open(altPath);
        if (!file.is_open()) {
            file.open(altPath2);
            if (!file.is_open()) {
                std::cerr << "Failed to open shader file: " << path << " (tried: " << altPath << ", " << altPath2 << ")" << std::endl;
                return std::string();
            }
        }
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    // Expand local #include "file" directives by inlining their contents
    std::stringstream out;
    std::istringstream in(content);
    std::string line;
    const std::string incToken = "#include \"";
    while (std::getline(in, line)) {
        size_t p = line.find(incToken);
        if (p != std::string::npos) {
            size_t start = p + incToken.size();
            size_t end = line.find('"', start);
            if (end != std::string::npos) {
                std::string includeFile = line.substr(start, end - start);
                std::string includedSource = loadShaderSource(includeFile);
                if (includedSource.empty()) {
                    // Try resolving relative to the current file's directory
                    std::string relPath = srcDir + "/" + includeFile;
                    includedSource = loadShaderSource(relPath);
                }
                out << includedSource << "\n";
                continue;
            }
        }
        out << line << "\n";
    }

    return out.str();
}

void MeshVoxelizerMetal::checkMetalError(bool success, const char* message) {
    if (!success) {
        std::cerr << "Metal Error: " << message << std::endl;
    }
}

void MeshVoxelizerMetal::voxelize(
    std::vector<unsigned char> &voxels,
    const std::vector<Point3i> &vertices,
    const std::vector<Face> &faces,
    const BBox3i &scaledBounds,
    const Dimensions3i &dims) {
    
    std::cout << "Voxelizing using Metal..." << std::endl;
    if (faces.empty() || vertices.empty()) {
        std::cerr << "No faces or vertices provided for voxelization" << std::endl;
        return;
    }
    
    // Create buffers
    createBuffers(faces.size(), vertices.size(), voxels.size());
    
    // Create compute pipeline
    createComputePipeline("voxelizeKernel");
    
    id<MTLCommandQueue> commandQueue = (id<MTLCommandQueue>)m_commandQueue;
    id<MTLComputePipelineState> pipelineState = (id<MTLComputePipelineState>)m_pipelineState;
    id<MTLBuffer> facesBuffer = (id<MTLBuffer>)m_facesBuffer;
    id<MTLBuffer> verticesBuffer = (id<MTLBuffer>)m_verticesBuffer;
    id<MTLBuffer> voxelsBuffer = (id<MTLBuffer>)m_voxelsBuffer;
    id<MTLBuffer> paramsBuffer = (id<MTLBuffer>)m_paramsBuffer;
    
    // Copy data to GPU buffers
    memcpy([facesBuffer contents], faces.data(), faces.size() * sizeof(Face));
    memcpy([verticesBuffer contents], vertices.data(), vertices.size() * sizeof(Point3i));
    
    // Initialize voxels buffer to 0 (uint format)
    memset([voxelsBuffer contents], 0, voxels.size() * sizeof(uint));
    
    // Prepare params buffer
    int* params = (int*)[paramsBuffer contents];
    params[0] = scaledBounds.xmin;
    params[1] = scaledBounds.ymin;
    params[2] = scaledBounds.zmin;
    params[3] = scaledBounds.xmax;
    params[4] = scaledBounds.ymax;
    params[5] = scaledBounds.zmax;
    params[6] = dims.width;
    params[7] = dims.height;
    params[8] = dims.depth;
    
    // Create command buffer
    id<MTLCommandBuffer> commandBuffer = [commandQueue commandBuffer];
    checkMetalError(commandBuffer != nil, "Failed to create command buffer");
    
    // Create compute command encoder
    id<MTLComputeCommandEncoder> computeEncoder = [commandBuffer computeCommandEncoder];
    checkMetalError(computeEncoder != nil, "Failed to create compute command encoder");
    
    // Set pipeline and buffers
    [computeEncoder setComputePipelineState:pipelineState];
    [computeEncoder setBuffer:facesBuffer offset:0 atIndex:0];
    [computeEncoder setBuffer:verticesBuffer offset:0 atIndex:1];
    [computeEncoder setBuffer:voxelsBuffer offset:0 atIndex:2];
    [computeEncoder setBuffer:paramsBuffer offset:0 atIndex:3];
    
    // Launch one thread per input face so every triangle is processed.
    // The grid size is the number of threads to launch, not the number of threadgroups.
    MTLSize threadgroupSize = MTLSizeMake(std::min<size_t>(256, std::max<size_t>(1, faces.size())), 1, 1);
    MTLSize gridSize = MTLSizeMake(std::max<size_t>(1, faces.size()), 1, 1);

    [computeEncoder dispatchThreads:gridSize threadsPerThreadgroup:threadgroupSize];
    [computeEncoder endEncoding];
    
    // Commit command buffer
    [commandBuffer commit];
    [commandBuffer waitUntilCompleted];
    
    // Read results back and convert from uint to unsigned char
    uint* uintResults = (uint*)[voxelsBuffer contents];
    for (size_t i = 0; i < voxels.size(); ++i) {
        voxels[i] = uintResults[i] > 0 ? 1 : 0;
    }
}
