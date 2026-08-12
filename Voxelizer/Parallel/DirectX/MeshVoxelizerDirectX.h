#pragma once

#include "../../MeshVoxelizerBase.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <d3d11.h>
#include <wrl/client.h>
#endif

class MeshVoxelizerDirectX : public MeshVoxelizerBase {
public:
    MeshVoxelizerDirectX();
    ~MeshVoxelizerDirectX();
    void voxelize(std::vector<unsigned char> &voxels,
                  const std::vector<Point3i> &vertices,
                  const std::vector<Face> &faces,
                  const BBox3i &scaledBounds,
                  const Dimensions3i &dims) override;

private:
#ifdef _WIN32
    Microsoft::WRL::ComPtr<ID3D11Device> m_device;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> m_context;
#endif
};