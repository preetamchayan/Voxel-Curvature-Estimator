#include "CurvatureEstimatorDirectX.h"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <d3dcompiler.h>

#define DX_CURVATURE_THREADS_PER_GROUP 64
#define DX_STRINGIFY_IMPL(value) #value
#define DX_STRINGIFY(value) DX_STRINGIFY_IMPL(value)

namespace {
constexpr int kMaxDirectXCurveLength = 32;
constexpr int kOperationInnerSpace = 0;
constexpr int kOperationMarkInterior = 1;
constexpr int kOperationMarkFrontier = 2;
constexpr int kOperationEstimateCurvature = 3;
constexpr size_t kVoxelsPerPackedWord = 10;
constexpr uint32_t kVoxelMask = 0x7u;
constexpr wchar_t kDirectXCurvatureShaderPath[] = L"CurvatureEstimator/Parallel/DirectX/CurvatureEstimatorKernel.hlsl";

struct CurvatureParams {
    int operation;
    int width;
    int height;
    int depth;
    int plane;
    int curveLength;
    int surfaceVoxelCount;
    int padding;
};

void checkHr(HRESULT hr, const char* message) {
    if (FAILED(hr)) {
        throw std::runtime_error(
            std::string("DirectX error 0x") + std::to_string(static_cast<unsigned long>(hr)) +
            " at " + message);
    }
}

Microsoft::WRL::ComPtr<ID3D11Buffer> createStructuredBuffer(
    ID3D11Device* device,
    UINT elementSize,
    UINT elementCount,
    const void* initialData,
    UINT bindFlags,
    bool cpuRead = false) {

    D3D11_BUFFER_DESC desc{};
    desc.ByteWidth = std::max<UINT>(elementSize * std::max<UINT>(elementCount, 1u), 4u);
    desc.Usage = cpuRead ? D3D11_USAGE_STAGING : D3D11_USAGE_DEFAULT;
    desc.BindFlags = cpuRead ? 0 : bindFlags;
    desc.CPUAccessFlags = cpuRead ? D3D11_CPU_ACCESS_READ : 0;
    desc.MiscFlags = cpuRead ? 0 : D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc.StructureByteStride = cpuRead ? 0 : elementSize;

    D3D11_SUBRESOURCE_DATA subresource{};
    subresource.pSysMem = initialData;

    Microsoft::WRL::ComPtr<ID3D11Buffer> buffer;
    checkHr(device->CreateBuffer(&desc, initialData ? &subresource : nullptr, &buffer),
            "ID3D11Device::CreateBuffer structured buffer");
    return buffer;
}

Microsoft::WRL::ComPtr<ID3D11Buffer> createRawBuffer(
    ID3D11Device* device,
    UINT byteCount,
    const void* initialData,
    UINT bindFlags,
    bool cpuRead = false) {

    D3D11_BUFFER_DESC desc{};
    desc.ByteWidth = std::max<UINT>(((byteCount + 3u) / 4u) * 4u, 4u);
    desc.Usage = cpuRead ? D3D11_USAGE_STAGING : D3D11_USAGE_DEFAULT;
    desc.BindFlags = cpuRead ? 0 : bindFlags;
    desc.CPUAccessFlags = cpuRead ? D3D11_CPU_ACCESS_READ : 0;
    desc.MiscFlags = cpuRead ? 0 : D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;

    D3D11_SUBRESOURCE_DATA subresource{};
    subresource.pSysMem = initialData;

    Microsoft::WRL::ComPtr<ID3D11Buffer> buffer;
    checkHr(device->CreateBuffer(&desc, initialData ? &subresource : nullptr, &buffer),
            "ID3D11Device::CreateBuffer raw buffer");
    return buffer;
}

Microsoft::WRL::ComPtr<ID3D11Buffer> createConstantBuffer(
    ID3D11Device* device,
    const CurvatureParams& params) {

    D3D11_BUFFER_DESC desc{};
    desc.ByteWidth = sizeof(CurvatureParams);
    desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;

    D3D11_SUBRESOURCE_DATA data{};
    data.pSysMem = &params;

    Microsoft::WRL::ComPtr<ID3D11Buffer> buffer;
    checkHr(device->CreateBuffer(&desc, &data, &buffer),
            "ID3D11Device::CreateBuffer curvature constant buffer");
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
    desc.Buffer.NumElements = std::max<UINT>(elementCount, 1u);

    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> srv;
    checkHr(device->CreateShaderResourceView(buffer, &desc, &srv),
            "ID3D11Device::CreateShaderResourceView curvature buffer");
    return srv;
}

Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> createUAV(
    ID3D11Device* device,
    ID3D11Buffer* buffer,
    UINT elementCount) {

    D3D11_UNORDERED_ACCESS_VIEW_DESC desc{};
    desc.Format = DXGI_FORMAT_UNKNOWN;
    desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    desc.Buffer.FirstElement = 0;
    desc.Buffer.NumElements = std::max<UINT>(elementCount, 1u);

    Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> uav;
    checkHr(device->CreateUnorderedAccessView(buffer, &desc, &uav),
            "ID3D11Device::CreateUnorderedAccessView curvature buffer");
    return uav;
}

Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> createRawUAV(
    ID3D11Device* device,
    ID3D11Buffer* buffer,
    UINT byteCount) {

    D3D11_UNORDERED_ACCESS_VIEW_DESC desc{};
    desc.Format = DXGI_FORMAT_R32_TYPELESS;
    desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    desc.Buffer.FirstElement = 0;
    desc.Buffer.NumElements = std::max<UINT>(((byteCount + 3u) / 4u), 1u);
    desc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;

    Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> uav;
    checkHr(device->CreateUnorderedAccessView(buffer, &desc, &uav),
            "ID3D11Device::CreateUnorderedAccessView raw curvature buffer");
    return uav;
}

UINT groupCount1D(size_t itemCount) {
    return static_cast<UINT>((itemCount + DX_CURVATURE_THREADS_PER_GROUP - 1) /
                             DX_CURVATURE_THREADS_PER_GROUP);
}

std::vector<uint32_t> packVoxels(const std::vector<unsigned char>& voxels) {
    std::vector<uint32_t> packed((voxels.size() + kVoxelsPerPackedWord - 1) / kVoxelsPerPackedWord, 0u);
    for (size_t index = 0; index < voxels.size(); ++index) {
        const size_t wordIndex = index / kVoxelsPerPackedWord;
        const uint32_t shift = static_cast<uint32_t>((index % kVoxelsPerPackedWord) * 3);
        packed[wordIndex] |= (static_cast<uint32_t>(voxels[index]) & kVoxelMask) << shift;
    }
    if (packed.empty()) {
        packed.push_back(0u);
    }
    return packed;
}

void unpackVoxels(const uint32_t* packed, size_t voxelCount, std::vector<unsigned char>& voxels) {
    for (size_t index = 0; index < voxelCount; ++index) {
        const size_t wordIndex = index / kVoxelsPerPackedWord;
        const uint32_t shift = static_cast<uint32_t>((index % kVoxelsPerPackedWord) * 3);
        voxels[index] = static_cast<unsigned char>((packed[wordIndex] >> shift) & kVoxelMask);
    }
}



Microsoft::WRL::ComPtr<ID3D11ComputeShader> compileComputeShader(
    ID3D11Device* device,
    const D3D_SHADER_MACRO* shaderMacros,
    const char* description) {

    // FXC's optimizer/flow-control flattener can hit "internal error: flattened side effect"
    // or spend excessive time on the full curvature traversal. Keep optimization disabled.
    UINT compileFlags = D3DCOMPILE_ENABLE_STRICTNESS |
                        D3DCOMPILE_SKIP_OPTIMIZATION |
                        D3DCOMPILE_PREFER_FLOW_CONTROL;
#if defined(_DEBUG)
    compileFlags |= D3DCOMPILE_DEBUG;
#endif

        Microsoft::WRL::ComPtr<ID3DBlob> shaderBlob;
    Microsoft::WRL::ComPtr<ID3DBlob> errorBlob;
    std::cout << "Compiling DirectX curvature " << description << " HLSL shader..." << std::endl;
    HRESULT hr = D3DCompileFromFile(
        kDirectXCurvatureShaderPath,
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
            std::cerr << "DirectX curvature " << description << " HLSL compile error:\n"
                      << static_cast<const char*>(errorBlob->GetBufferPointer()) << std::endl;
        }
        checkHr(hr, "D3DCompileFromFile CurvatureEstimatorKernel.hlsl");
    }

    std::cout << "DirectX curvature " << description << " HLSL shader compiled." << std::endl;

    Microsoft::WRL::ComPtr<ID3D11ComputeShader> shader;
    checkHr(device->CreateComputeShader(shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize(),
                                        nullptr, &shader),
            "ID3D11Device::CreateComputeShader curvature");
    std::cout << "DirectX curvature " << description << " compute shader created." << std::endl;
    return shader;
}

void dispatchCurvatureOperation(
    ID3D11DeviceContext* context,
    ID3D11ComputeShader* shader,
    ID3D11Buffer* constantBuffer,
    const CurvatureParams& params,
    ID3D11ShaderResourceView* const* srvs,
    UINT srvCount,
    ID3D11UnorderedAccessView* const* uavs,
    UINT uavCount,
    UINT groupX,
    UINT groupY,
    UINT groupZ) {

    context->UpdateSubresource(constantBuffer, 0, nullptr, &params, 0, 0);
    ID3D11Buffer* cbuffers[] = { constantBuffer };
    context->CSSetShader(shader, nullptr, 0);
    context->CSSetConstantBuffers(0, 1, cbuffers);
    context->CSSetShaderResources(0, srvCount, srvs);
    context->CSSetUnorderedAccessViews(0, uavCount, uavs, nullptr);
    context->Dispatch(groupX, groupY, groupZ);

    std::vector<ID3D11ShaderResourceView*> nullSrvs(srvCount, nullptr);
    std::vector<ID3D11UnorderedAccessView*> nullUavs(uavCount, nullptr);
    ID3D11Buffer* nullCBuffers[] = { nullptr };
    context->CSSetShaderResources(0, srvCount, nullSrvs.data());
    context->CSSetUnorderedAccessViews(0, uavCount, nullUavs.data(), nullptr);
    context->CSSetConstantBuffers(0, 1, nullCBuffers);
    context->CSSetShader(nullptr, nullptr, 0);
}
}
#endif

CurvatureEstimatorDirectX::CurvatureEstimatorDirectX() {
    std::cout << "Initializing DirectX curvature environment..." << std::endl;

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
    std::cout << "Creating DirectX curvature D3D11 device..." << std::endl;
    HRESULT hr = D3D11CreateDevice(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        flags,
        requestedLevels,
        static_cast<UINT>(sizeof(requestedLevels) / sizeof(requestedLevels[0])),
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
            static_cast<UINT>(sizeof(requestedLevels) / sizeof(requestedLevels[0])),
            D3D11_SDK_VERSION,
            &m_device,
            &createdLevel,
            &m_context);
    }
    checkHr(hr, "D3D11CreateDevice curvature");
    std::cout << "DirectX curvature D3D11 device created." << std::endl;

    const D3D_SHADER_MACRO preprocessShaderMacros[] = {
        { "THREADS_PER_GROUP", DX_STRINGIFY(DX_CURVATURE_THREADS_PER_GROUP) },
        { "MAX_CURVE_LENGTH", "32" },
        { "DX_CURVATURE_PREPROCESS_ONLY", "1" },
        { nullptr, nullptr }
    };
    m_preprocessShader = compileComputeShader(m_device.Get(), preprocessShaderMacros, "preprocess");
#else
    throw std::runtime_error("DirectX curvature estimation is only supported on Windows.");
#endif
}

CurvatureEstimatorDirectX::~CurvatureEstimatorDirectX() = default;

void CurvatureEstimatorDirectX::preprocessVoxels(std::vector<unsigned char>& voxels,
                                                 const Dimensions3i& dims) {
#ifndef _WIN32
    throw std::runtime_error("DirectX curvature preprocessing is only supported on Windows.");
#else
    std::cout << "Preprocessing curvature voxels using DirectX GPU..." << std::endl;
    if (voxels.size() > static_cast<size_t>(std::numeric_limits<UINT>::max())) {
        throw std::runtime_error("DirectX curvature voxel grid is too large for D3D11.");
    }

    const std::vector<uint32_t> shaderVoxels = packVoxels(voxels);
    const UINT packedVoxelByteCount = static_cast<UINT>(shaderVoxels.size() * sizeof(uint32_t));
    auto voxelsBuffer = createRawBuffer(m_device.Get(), packedVoxelByteCount, shaderVoxels.data(),
                                        D3D11_BIND_UNORDERED_ACCESS);
    auto voxelsUAV = createRawUAV(m_device.Get(), voxelsBuffer.Get(), packedVoxelByteCount);

    uint32_t dummy = 0;
    auto dummyCurvaturesBuffer = createStructuredBuffer(m_device.Get(), sizeof(int), 1, &dummy,
                                                        D3D11_BIND_UNORDERED_ACCESS);
    auto dummySurfaceIdsBuffer = createStructuredBuffer(m_device.Get(), sizeof(int), 1, &dummy,
                                                        D3D11_BIND_SHADER_RESOURCE);
    auto dummyCurvaturesUAV = createUAV(m_device.Get(), dummyCurvaturesBuffer.Get(), 1);
    auto dummySurfaceIdsSRV = createSRV(m_device.Get(), dummySurfaceIdsBuffer.Get(), 1);

    CurvatureParams params{kOperationInnerSpace, dims.width, dims.height, dims.depth, 0, 0, 0, 0};
    auto constantBuffer = createConstantBuffer(m_device.Get(), params);
    ID3D11ShaderResourceView* srvs[] = { dummySurfaceIdsSRV.Get() };
    ID3D11UnorderedAccessView* uavs[] = { voxelsUAV.Get(), dummyCurvaturesUAV.Get() };

    params = {kOperationInnerSpace, dims.width, dims.height, dims.depth, 2, 0, 0, 0};
    dispatchCurvatureOperation(m_context.Get(), m_preprocessShader.Get(), constantBuffer.Get(), params,
                               srvs, 1, uavs, 2, groupCount1D(dims.width), static_cast<UINT>(dims.height), 1);

    params = {kOperationInnerSpace, dims.width, dims.height, dims.depth, 1, 0, 0, 0};
    dispatchCurvatureOperation(m_context.Get(), m_preprocessShader.Get(), constantBuffer.Get(), params,
                               srvs, 1, uavs, 2, groupCount1D(dims.depth), static_cast<UINT>(dims.width), 1);

    params = {kOperationInnerSpace, dims.width, dims.height, dims.depth, 0, 0, 0, 0};
    dispatchCurvatureOperation(m_context.Get(), m_preprocessShader.Get(), constantBuffer.Get(), params,
                               srvs, 1, uavs, 2, groupCount1D(dims.height), static_cast<UINT>(dims.depth), 1);

    params = {kOperationMarkInterior, dims.width, dims.height, dims.depth, 0, 0, 0, 0};
    dispatchCurvatureOperation(m_context.Get(), m_preprocessShader.Get(), constantBuffer.Get(), params,
                               srvs, 1, uavs, 2, groupCount1D(dims.width), static_cast<UINT>(dims.height), static_cast<UINT>(dims.depth));

    params = {kOperationMarkFrontier, dims.width, dims.height, dims.depth, 0, 0, 0, 0};
    dispatchCurvatureOperation(m_context.Get(), m_preprocessShader.Get(), constantBuffer.Get(), params,
                               srvs, 1, uavs, 2, groupCount1D(dims.width), static_cast<UINT>(dims.height), static_cast<UINT>(dims.depth));

    auto readback = createRawBuffer(m_device.Get(), packedVoxelByteCount, nullptr, 0, true);
    m_context->CopyResource(readback.Get(), voxelsBuffer.Get());
    m_context->Flush();

    D3D11_MAPPED_SUBRESOURCE mapped{};
    checkHr(m_context->Map(readback.Get(), 0, D3D11_MAP_READ, 0, &mapped),
            "ID3D11DeviceContext::Map DirectX curvature preprocess readback");
    const uint32_t* values = static_cast<const uint32_t*>(mapped.pData);
    unpackVoxels(values, voxels.size(), voxels);
    m_context->Unmap(readback.Get(), 0);

    const size_t surfaceVoxelCount = static_cast<size_t>(std::count(voxels.begin(), voxels.end(), static_cast<unsigned char>(1)));
    const size_t interiorVoxelCount = static_cast<size_t>(std::count(voxels.begin(), voxels.end(), static_cast<unsigned char>(2)));
    std::cout << "DirectX curvature preprocessing result: "
              << surfaceVoxelCount << " surface voxels, "
              << interiorVoxelCount << " interior voxels" << std::endl;
#endif
}

void CurvatureEstimatorDirectX::estimateCurvature(
    int curvLength,
    const std::vector<unsigned char>& voxels,
    std::vector<int>& curvatures,
    const Dimensions3i& dims) {
#ifndef _WIN32
    throw std::runtime_error("DirectX curvature estimation is only supported on Windows.");
#else
    std::cout << "Estimating curvature using DirectX..." << std::endl;
    if (curvLength < 1 || curvLength > kMaxDirectXCurveLength) {
        throw std::runtime_error("DirectX curvature curve length must be in [1, 32].");
    }
    if (curvatures.size() != voxels.size()) {
        throw std::runtime_error("curvatures and voxels must have the same grid size");
    }
    if (voxels.size() > static_cast<size_t>(std::numeric_limits<UINT>::max())) {
        throw std::runtime_error("DirectX curvature voxel grid is too large for D3D11.");
    }

    const size_t voxelCount = voxels.size();
    std::vector<int> surfaceVoxelIds;
    surfaceVoxelIds.reserve(voxelCount);
    for (size_t id = 0; id < voxelCount; ++id) {
        if (voxels[id] == 1) {
            surfaceVoxelIds.push_back(static_cast<int>(id));
        }
    }
    std::cout << "DirectX curvature dispatch: " << surfaceVoxelIds.size()
              << " surface voxels out of " << voxelCount << " grid voxels" << std::endl;

    std::fill(curvatures.begin(), curvatures.end(), std::numeric_limits<int>::max());
    if (surfaceVoxelIds.empty()) {
        return;
    }
    if (surfaceVoxelIds.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("DirectX curvature supports at most INT_MAX surface voxels.");
    }

    if (m_estimateShaderCurveLength != curvLength) {
        const std::string curveLengthMacro = std::to_string(curvLength);
        const D3D_SHADER_MACRO estimateShaderMacros[] = {
            { "THREADS_PER_GROUP", DX_STRINGIFY(DX_CURVATURE_THREADS_PER_GROUP) },
            { "MAX_CURVE_LENGTH", curveLengthMacro.c_str() },
            { "DX_CURVATURE_ESTIMATE_ONLY", "1" },
            { "DX_CURVATURE_PARALLEL_PLANE_PASS", "1" },
            { nullptr, nullptr }
        };
        m_estimateShader = compileComputeShader(m_device.Get(), estimateShaderMacros, "estimate_parallel_planes");

        const D3D_SHADER_MACRO combineShaderMacros[] = {
            { "THREADS_PER_GROUP", DX_STRINGIFY(DX_CURVATURE_THREADS_PER_GROUP) },
            { "MAX_CURVE_LENGTH", curveLengthMacro.c_str() },
            { "DX_CURVATURE_ESTIMATE_ONLY", "1" },
            { "DX_CURVATURE_COMBINE_PASS", "1" },
            { nullptr, nullptr }
        };
        m_combineCurvatureShader = compileComputeShader(m_device.Get(), combineShaderMacros, "estimate_combine");
        m_estimateShaderCurveLength = curvLength;
    }


    const std::vector<uint32_t> shaderVoxels = packVoxels(voxels);
    const UINT packedVoxelByteCount = static_cast<UINT>(shaderVoxels.size() * sizeof(uint32_t));
    const UINT surfaceElementCount = static_cast<UINT>(surfaceVoxelIds.size());
    if (surfaceElementCount > std::numeric_limits<UINT>::max() / 9u) {
        throw std::runtime_error("DirectX curvature plane-output buffer would exceed UINT element count.");
    }
    const UINT planeCurvatureElementCount = surfaceElementCount * 9u;

    auto voxelsBuffer = createRawBuffer(m_device.Get(), packedVoxelByteCount,
                                        shaderVoxels.data(), D3D11_BIND_UNORDERED_ACCESS);
    auto surfaceIdsBuffer = createStructuredBuffer(m_device.Get(), sizeof(int), surfaceElementCount,
                                                   surfaceVoxelIds.data(), D3D11_BIND_SHADER_RESOURCE);
    auto curvaturesBuffer = createStructuredBuffer(m_device.Get(), sizeof(int), surfaceElementCount,
                                                   nullptr, D3D11_BIND_UNORDERED_ACCESS);
    auto planeCurvaturesBuffer = createStructuredBuffer(m_device.Get(), sizeof(int), planeCurvatureElementCount,
                                                        nullptr, D3D11_BIND_UNORDERED_ACCESS);

    std::vector<int> surfaceCurvatures(surfaceVoxelIds.size(), std::numeric_limits<int>::max());
    m_context->UpdateSubresource(curvaturesBuffer.Get(), 0, nullptr, surfaceCurvatures.data(), 0, 0);

    auto voxelsUAV = createRawUAV(m_device.Get(), voxelsBuffer.Get(), packedVoxelByteCount);
    auto surfaceIdsSRV = createSRV(m_device.Get(), surfaceIdsBuffer.Get(), surfaceElementCount);
    auto curvaturesUAV = createUAV(m_device.Get(), curvaturesBuffer.Get(), surfaceElementCount);
    auto planeCurvaturesUAV = createUAV(m_device.Get(), planeCurvaturesBuffer.Get(), planeCurvatureElementCount);

    CurvatureParams params{kOperationEstimateCurvature, dims.width, dims.height, dims.depth,
                           0, curvLength, static_cast<int>(surfaceVoxelIds.size()), 0};
    auto constantBuffer = createConstantBuffer(m_device.Get(), params);

    ID3D11ShaderResourceView* srvs[] = { surfaceIdsSRV.Get() };
    ID3D11UnorderedAccessView* uavs[] = {
        voxelsUAV.Get(), curvaturesUAV.Get(), planeCurvaturesUAV.Get()
    };

    const UINT groups = groupCount1D(surfaceVoxelIds.size());
    dispatchCurvatureOperation(
        m_context.Get(),
        m_estimateShader.Get(),
        constantBuffer.Get(),
        params,
        srvs,
        1,
        uavs,
        3,
        groups,
        9,
        1);
        dispatchCurvatureOperation(m_context.Get(), m_combineCurvatureShader.Get(), constantBuffer.Get(), params,
                               srvs, 1, uavs, 3, groups, 1, 1);
    m_context->Flush();

    auto readback = createStructuredBuffer(m_device.Get(), sizeof(int), surfaceElementCount, nullptr, 0, true);
    m_context->CopyResource(readback.Get(), curvaturesBuffer.Get());
    m_context->Flush();

    D3D11_MAPPED_SUBRESOURCE mapped{};
    checkHr(m_context->Map(readback.Get(), 0, D3D11_MAP_READ, 0, &mapped),
            "ID3D11DeviceContext::Map DirectX curvature readback");
    const int* values = static_cast<const int*>(mapped.pData);
        std::copy(values, values + surfaceCurvatures.size(), surfaceCurvatures.begin());
    m_context->Unmap(readback.Get(), 0);

    for (size_t surfaceIndex = 0; surfaceIndex < surfaceVoxelIds.size(); ++surfaceIndex) {
        curvatures[static_cast<size_t>(surfaceVoxelIds[surfaceIndex])] = surfaceCurvatures[surfaceIndex];
    }
#endif
}

