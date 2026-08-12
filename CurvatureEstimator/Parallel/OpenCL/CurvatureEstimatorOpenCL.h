#pragma once

#include "../../CurvatureEstimatorBase.h"

#include <CL/cl.h>
#include <string>
#include <vector>

class CurvatureEstimatorOpenCL : public CurvatureEstimatorBase {
public:
    CurvatureEstimatorOpenCL();
    ~CurvatureEstimatorOpenCL();

    void preprocessVoxels(std::vector<unsigned char>& voxels,
                          const Dimensions3i& dims) override;

    void estimateCurvature(int curvLength,
                           const std::vector<unsigned char>& voxels,
                           std::vector<int>& curvatures,
                           const Dimensions3i& dims) override;

private:
    std::string loadKernelSource(const std::string& path) const;
    void checkOpenCLError(cl_int error, const char* message) const;
    void releaseKernel(cl_kernel& kernel);

private:
    cl_context m_context;
    cl_command_queue m_queue;
    cl_device_id m_device;
    cl_program m_program;
    cl_kernel m_innerSpaceKernel;
    cl_kernel m_markInteriorKernel;
    cl_kernel m_markFrontierKernel;
    cl_kernel m_estimateCurvatureKernel;
};