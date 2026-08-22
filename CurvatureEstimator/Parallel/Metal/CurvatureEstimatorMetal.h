#pragma once

#include "../../CurvatureEstimatorBase.h"

#include <string>
#include <vector>

class CurvatureEstimatorMetal : public CurvatureEstimatorBase {
public:
    CurvatureEstimatorMetal();
    ~CurvatureEstimatorMetal() override;

    void preprocessVoxels(std::vector<unsigned char>& voxels,
                          const Dimensions3i& dims) override;

    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;

private:
    void createMetalObjects();
    std::string loadShaderSource(const std::string& path) const;
    void checkMetalError(bool success, const char* message) const;

private:
    void* m_device = nullptr;
    void* m_commandQueue = nullptr;
    void* m_library = nullptr;
};
