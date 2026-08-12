#include "MeshVoxelizerDirectX.h"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>


#ifdef _WIN32
#include <d3dcompiler.h>

// Change this one value to compare DirectX thread-group variants. It controls
// both the HLSL [numthreads] macro and the Dispatch() group count.
#define DX_THREADS_PER_GROUP 64
#define DX_STRINGIFY_IMPL(value) #value
#define DX_STRINGIFY(value) DX_STRINGIFY_IMPL(value)

namespace {

void checkHr(HRESULT hr, const char* message) {
    if (FAILED(hr)) {
        std::cerr << "DirectX error 0x" << std::hex << static_cast<unsigned long>(hr)
                  << std::dec << " at " << message << std::endl;
        std::exit(1);
    }
}

struct VoxelizerParams {
    int numFaces;
    int R;
    int C;
    int D;
    int xmin;
    int ymin;
    int zmin;
    int padding;
};

Microsoft::WRL::ComPtr<ID3D11Buffer> createStructuredBuffer(
    ID3D11Device* device,
    UINT elementSize,
    UINT elementCount,
    const void* initialData,
    UINT bindFlags,
    bool cpuRead = false) {

    D3D11_BUFFER_DESC desc{};
    desc.ByteWidth = elementSize * elementCount;
    desc.Usage = cpuRead ? D3D11_USAGE_STAGING : D3D11_USAGE_DEFAULT;
    desc.BindFlags = cpuRead ? 0 : bindFlags;
    desc.CPUAccessFlags = cpuRead ? D3D11_CPU_ACCESS_READ : 0;
    desc.MiscFlags = cpuRead ? 0 : D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc.StructureByteStride = cpuRead ? 0 : elementSize;

    D3D11_SUBRESOURCE_DATA subresource{};
    subresource.pSysMem = initialData;

    Microsoft::WRL::ComPtr<ID3D11Buffer> buffer;
    checkHr(device->CreateBuffer(&desc, initialData ? &subresource : nullptr, &buffer), "ID3D11Device::CreateBuffer structured buffer");
    return buffer;
}

Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> createSRV(
    ID3D11Device* device,
    ID3D11Buffer* buffer,
    UINT elementCount) {

    D3D11_SHADER_RESOURCE_VIEW_DESC desc{};
    desc.Format = DXGI_FORMAT_UNKNOWN;
    desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    desc.Buffer.FirstElement = 0;
    desc.Buffer.NumElements = elementCount;

    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> srv;
    checkHr(device->CreateShaderResourceView(buffer, &desc, &srv), "ID3D11Device::CreateShaderResourceView");
    return srv;
}

Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> createBufferUAV(
    ID3D11Device* device,
    ID3D11Buffer* buffer,
    UINT elementCount) {

    D3D11_UNORDERED_ACCESS_VIEW_DESC desc{};
    desc.Format = DXGI_FORMAT_UNKNOWN;
    desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    desc.Buffer.FirstElement = 0;
    desc.Buffer.NumElements = elementCount;

    Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> uav;
    checkHr(device->CreateUnorderedAccessView(buffer, &desc, &uav), "ID3D11Device::CreateUnorderedAccessView buffer");
    return uav;
}

}
#endif

MeshVoxelizerDirectX::MeshVoxelizerDirectX() {

    std::cout << "Initializing DirectX environment..." << std::endl;

#ifdef _WIN32
    UINT flags = 0;
#if defined(_DEBUG)
    flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

    D3D_FEATURE_LEVEL requestedLevels[] = {
        D3D_FEATURE_LEVEL_11_1,
        D3D_FEATURE_LEVEL_11_0
    };
    D3D_FEATURE_LEVEL createdLevel{};
    const UINT requestedLevelCount = static_cast<UINT>(sizeof(requestedLevels) / sizeof(requestedLevels[0]));

    HRESULT hr = D3D11CreateDevice(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        flags,
        requestedLevels,
        requestedLevelCount,
        D3D11_SDK_VERSION,
        &m_device,
        &createdLevel,
        &m_context);

    if (FAILED(hr) && (flags & D3D11_CREATE_DEVICE_DEBUG)) {
        flags &= ~D3D11_CREATE_DEVICE_DEBUG;
        hr = D3D11CreateDevice(
            nullptr,
            D3D_DRIVER_TYPE_HARDWARE,
            nullptr,
            flags,
            requestedLevels,
            requestedLevelCount,
            D3D11_SDK_VERSION,
            &m_device,
            &createdLevel,
            &m_context);
    }

    checkHr(hr, "D3D11CreateDevice");
#endif
}

MeshVoxelizerDirectX::~MeshVoxelizerDirectX() = default;

void MeshVoxelizerDirectX::voxelize(
    std::vector<unsigned char> &voxels,
    const std::vector<Point3i> &vertices,
    const std::vector<Face> &faces,
    const BBox3i &scaledBounds,
    const Dimensions3i &dims)
{

#ifndef _WIN32
    std::cerr << "DirectX voxelization is only supported on Windows." << std::endl;
    std::exit(1);
#else
    std::cout << "Voxelizing using DirectX..." << std::endl;

    std::vector<int> flatFaces;
    flatFaces.reserve(faces.size() * 3);
    for (const auto& f : faces) {
        flatFaces.push_back(f.v1);
        flatFaces.push_back(f.v2);
        flatFaces.push_back(f.v3);
    }

    std::vector<int> flatVertices;
    flatVertices.reserve(vertices.size() * 3);
    for (const auto& v : vertices) {
        flatVertices.push_back(v.x);
        flatVertices.push_back(v.y);
        flatVertices.push_back(v.z);
    }

    const size_t numVoxels = static_cast<size_t>(dims.width) * static_cast<size_t>(dims.height) * static_cast<size_t>(dims.depth);
    if (numVoxels > static_cast<size_t>(UINT_MAX)) {
        std::cerr << "DirectX voxel grid is too large for a D3D11 buffer element count." << std::endl;
        std::exit(1);
    }

    const UINT numVoxelWords = static_cast<UINT>((numVoxels + 31) / 32);
    std::vector<uint32_t> zeroVoxelWords(numVoxelWords, 0);
    uint32_t zeroTotal = 0;

    auto facesBuffer = createStructuredBuffer(m_device.Get(), sizeof(int), static_cast<UINT>(flatFaces.size()), flatFaces.data(), D3D11_BIND_SHADER_RESOURCE);
    auto verticesBuffer = createStructuredBuffer(m_device.Get(), sizeof(int), static_cast<UINT>(flatVertices.size()), flatVertices.data(), D3D11_BIND_SHADER_RESOURCE);
    auto voxelsBuffer = createStructuredBuffer(m_device.Get(), sizeof(uint32_t), numVoxelWords, zeroVoxelWords.data(), D3D11_BIND_UNORDERED_ACCESS);
    auto totalBuffer = createStructuredBuffer(m_device.Get(), sizeof(uint32_t), 1, &zeroTotal, D3D11_BIND_UNORDERED_ACCESS);


    auto facesSRV = createSRV(m_device.Get(), facesBuffer.Get(), static_cast<UINT>(flatFaces.size()));
    auto verticesSRV = createSRV(m_device.Get(), verticesBuffer.Get(), static_cast<UINT>(flatVertices.size()));
    auto voxelsUAV = createBufferUAV(m_device.Get(), voxelsBuffer.Get(), numVoxelWords);
    auto totalUAV = createBufferUAV(m_device.Get(), totalBuffer.Get(), 1);

    VoxelizerParams params{};
    params.numFaces = static_cast<int>(faces.size());
    params.R = dims.width;
    params.C = dims.height;
    params.D = dims.depth;
    params.xmin = scaledBounds.xmin;
    params.ymin = scaledBounds.ymin;
    params.zmin = scaledBounds.zmin;

    D3D11_BUFFER_DESC constantDesc{};
    constantDesc.ByteWidth = sizeof(VoxelizerParams);
    constantDesc.Usage = D3D11_USAGE_DEFAULT;
    constantDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;

    D3D11_SUBRESOURCE_DATA constantData{};
    constantData.pSysMem = &params;

    Microsoft::WRL::ComPtr<ID3D11Buffer> constantBuffer;
    checkHr(m_device->CreateBuffer(&constantDesc, &constantData, &constantBuffer), "ID3D11Device::CreateBuffer constant buffer");

        constexpr UINT threadsPerGroup = DX_THREADS_PER_GROUP;
    const D3D_SHADER_MACRO shaderMacros[] = {
        { "THREADS_PER_GROUP", DX_STRINGIFY(DX_THREADS_PER_GROUP) },
        { nullptr, nullptr }
    };

    UINT compileFlags = D3DCOMPILE_ENABLE_STRICTNESS;

#if defined(_DEBUG)
    compileFlags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

    Microsoft::WRL::ComPtr<ID3DBlob> shaderBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;
    HRESULT hr = D3DCompileFromFile(
        L"Voxelizer/Parallel/DirectX/MeshVoxelizerKernel.hlsl",
        shaderMacros,
        D3D_COMPILE_STANDARD_FILE_INCLUDE,

        "main",
        "cs_5_0",
        compileFlags,
        0,
        &shaderBlob,
        &errorBlob);

    if (FAILED(hr)) {
        if (errorBlob) {
            std::cerr << "DirectX HLSL compile error:\n"
                      << static_cast<const char*>(errorBlob->GetBufferPointer()) << std::endl;
        }
        checkHr(hr, "D3DCompileFromFile MeshVoxelizerKernel.hlsl");
    }

    Microsoft::WRL::ComPtr<ID3D11ComputeShader> computeShader;
    checkHr(m_device->CreateComputeShader(shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize(), nullptr, &computeShader),
            "ID3D11Device::CreateComputeShader");

    ID3D11ShaderResourceView* srvs[] = { facesSRV.Get(), verticesSRV.Get() };
    ID3D11UnorderedAccessView* uavs[] = { voxelsUAV.Get(), totalUAV.Get() };

    ID3D11Buffer* cbuffers[] = { constantBuffer.Get() };

    m_context->CSSetShader(computeShader.Get(), nullptr, 0);
    m_context->CSSetShaderResources(0, 2, srvs);
    m_context->CSSetUnorderedAccessViews(0, 2, uavs, nullptr);

    m_context->CSSetConstantBuffers(0, 1, cbuffers);

    const UINT groupCount = (static_cast<UINT>(faces.size()) + threadsPerGroup - 1) / threadsPerGroup;

    m_context->Dispatch(groupCount, 1, 1);

    ID3D11ShaderResourceView* nullSrvs[] = { nullptr, nullptr };
    ID3D11UnorderedAccessView* nullUavs[] = { nullptr, nullptr };

    ID3D11Buffer* nullCBuffers[] = { nullptr };
    m_context->CSSetShader(nullptr, nullptr, 0);
    m_context->CSSetShaderResources(0, 2, nullSrvs);
    m_context->CSSetUnorderedAccessViews(0, 2, nullUavs, nullptr);

    m_context->CSSetConstantBuffers(0, 1, nullCBuffers);

    auto voxelReadback = createStructuredBuffer(m_device.Get(), sizeof(uint32_t), numVoxelWords, nullptr, 0, true);
    auto totalReadback = createStructuredBuffer(m_device.Get(), sizeof(uint32_t), 1, nullptr, 0, true);

    m_context->CopyResource(voxelReadback.Get(), voxelsBuffer.Get());
    m_context->CopyResource(totalReadback.Get(), totalBuffer.Get());
    m_context->Flush();

    D3D11_MAPPED_SUBRESOURCE mapped{};
    checkHr(m_context->Map(voxelReadback.Get(), 0, D3D11_MAP_READ, 0, &mapped), "ID3D11DeviceContext::Map voxel readback");
        const uint32_t* words = static_cast<const uint32_t*>(mapped.pData);
    std::vector<uint32_t> voxelWords(words, words + numVoxelWords);
    for (size_t i = 0; i < numVoxels; ++i) {
        const uint32_t word = voxelWords[i >> 5];
        voxels[i] = ((word >> (i & 31u)) & 1u) ? 1 : 0;
    }
    m_context->Unmap(voxelReadback.Get(), 0);

    checkHr(m_context->Map(totalReadback.Get(), 0, D3D11_MAP_READ, 0, &mapped), "ID3D11DeviceContext::Map total readback");

    uint32_t totalFacesProcessed = *static_cast<const uint32_t*>(mapped.pData);

    m_context->Unmap(totalReadback.Get(), 0);

    std::cout << "Total faces processed: " << totalFacesProcessed << std::endl;
#endif
}