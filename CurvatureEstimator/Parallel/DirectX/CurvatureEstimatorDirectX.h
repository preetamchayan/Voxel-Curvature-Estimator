#pragma once

#include "../../CurvatureEstimatorBase.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <d3d11.h>
#include <wrl/client.h>
#endif

class CurvatureEstimatorDirectX : public CurvatureEstimatorBase {
public:
    CurvatureEstimatorDirectX();
    ~CurvatureEstimatorDirectX() override;

    void preprocessVoxels(std::vector<unsigned char>& voxels,
                          const Dimensions3i& dims) override;

    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;

private:
#ifdef _WIN32
    Microsoft::WRL::ComPtr<ID3D11Device> m_device;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> m_context;
    Microsoft::WRL::ComPtr<ID3D11ComputeShader> m_preprocessShader;
    Microsoft::WRL::ComPtr<ID3D11ComputeShader> m_estimateShader;
    Microsoft::WRL::ComPtr<ID3D11ComputeShader> m_combineCurvatureShader;
    int m_estimateShaderCurveLength = 0;
#endif
};