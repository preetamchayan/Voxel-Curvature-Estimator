#include "CurvatureEstimatorOpenCL.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
constexpr int kMaxOpenCLCurveLength = 64;

std::string getPlatformInfoString(cl_platform_id platform, cl_platform_info paramName) {
    size_t size = 0;
    cl_int err = clGetPlatformInfo(platform, paramName, 0, nullptr, &size);
    if (err != CL_SUCCESS || size == 0) return "<unavailable>";

    std::vector<char> value(size);
    err = clGetPlatformInfo(platform, paramName, size, value.data(), nullptr);
    if (err != CL_SUCCESS) return "<unavailable>";
    return std::string(value.data());
}

std::string getDeviceInfoString(cl_device_id device, cl_device_info paramName) {
    size_t size = 0;
    cl_int err = clGetDeviceInfo(device, paramName, 0, nullptr, &size);
    if (err != CL_SUCCESS || size == 0) return "<unavailable>";

    std::vector<char> value(size);
    err = clGetDeviceInfo(device, paramName, size, value.data(), nullptr);
    if (err != CL_SUCCESS) return "<unavailable>";
    return std::string(value.data());
}

const char* deviceTypeToString(cl_device_type type) {
    if (type & CL_DEVICE_TYPE_GPU) return "GPU";
    if (type & CL_DEVICE_TYPE_CPU) return "CPU";
    if (type & CL_DEVICE_TYPE_ACCELERATOR) return "Accelerator";
    if (type & CL_DEVICE_TYPE_DEFAULT) return "Default";
    return "Unknown";
}
}

CurvatureEstimatorOpenCL::CurvatureEstimatorOpenCL()
    : m_context(nullptr),
      m_queue(nullptr),
      m_device(nullptr),
      m_program(nullptr),
      m_innerSpaceKernel(nullptr),
      m_markInteriorKernel(nullptr),
      m_markFrontierKernel(nullptr),
      m_estimateCurvatureKernel(nullptr) {
    std::cout << "Initializing OpenCL curvature environment..." << std::endl;

    cl_uint numPlatforms = 0;
    checkOpenCLError(clGetPlatformIDs(0, nullptr, &numPlatforms), "clGetPlatformIDs count");
    if (numPlatforms == 0) {
        throw std::runtime_error("No OpenCL platforms found.");
    }

    std::vector<cl_platform_id> platforms(numPlatforms);
    checkOpenCLError(clGetPlatformIDs(numPlatforms, platforms.data(), nullptr), "clGetPlatformIDs");

    cl_platform_id selectedPlatform = nullptr;
    std::vector<cl_device_id> devices;
    for (cl_platform_id platform : platforms) {
        cl_uint numDevices = 0;
        cl_device_type selectedType = CL_DEVICE_TYPE_GPU;
        cl_int status = clGetDeviceIDs(platform, selectedType, 0, nullptr, &numDevices);
        if (status != CL_SUCCESS || numDevices == 0) {
            selectedType = CL_DEVICE_TYPE_CPU;
            status = clGetDeviceIDs(platform, selectedType, 0, nullptr, &numDevices);
            if (status != CL_SUCCESS || numDevices == 0) continue;
        }

        devices.resize(numDevices);
        checkOpenCLError(clGetDeviceIDs(platform, selectedType, numDevices, devices.data(), nullptr), "clGetDeviceIDs");
        selectedPlatform = platform;
        break;
    }

    if (!selectedPlatform || devices.empty()) {
        throw std::runtime_error("No OpenCL GPU or CPU devices found.");
    }

    m_device = devices[0];

    cl_device_type deviceType = 0;
    clGetDeviceInfo(m_device, CL_DEVICE_TYPE, sizeof(deviceType), &deviceType, nullptr);
    std::cout << "Selected OpenCL curvature platform: "
              << getPlatformInfoString(selectedPlatform, CL_PLATFORM_NAME) << std::endl;
    std::cout << "Selected OpenCL curvature device: "
              << getDeviceInfoString(m_device, CL_DEVICE_NAME)
              << " (" << deviceTypeToString(deviceType) << ')' << std::endl;

    cl_int err = CL_SUCCESS;
    m_context = clCreateContext(nullptr, 1, &m_device, nullptr, nullptr, &err);
    checkOpenCLError(err, "clCreateContext");

    m_queue = clCreateCommandQueue(m_context, m_device, 0, &err);
    checkOpenCLError(err, "clCreateCommandQueue");

    const std::string kernelSource = loadKernelSource("CurvatureEstimator/Parallel/OpenCL/CurvatureEstimatorKernel.cl");
    const char* sourceStr = kernelSource.c_str();
    const size_t sourceSize = kernelSource.size();
    m_program = clCreateProgramWithSource(m_context, 1, &sourceStr, &sourceSize, &err);
    checkOpenCLError(err, "clCreateProgramWithSource");

    const std::string buildOptions = "-cl-std=CL1.2 -D MAX_CURVE_LENGTH=" + std::to_string(kMaxOpenCLCurveLength);
    err = clBuildProgram(m_program, 1, &m_device, buildOptions.c_str(), nullptr, nullptr);
    if (err != CL_SUCCESS) {
        size_t logSize = 0;
        clGetProgramBuildInfo(m_program, m_device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &logSize);
        std::vector<char> log(std::max<size_t>(logSize, 1));
        clGetProgramBuildInfo(m_program, m_device, CL_PROGRAM_BUILD_LOG, log.size(), log.data(), nullptr);
        std::cerr << "OpenCL curvature build log:\n" << log.data() << std::endl;
        checkOpenCLError(err, "clBuildProgram CurvatureEstimatorKernel.cl");
    }

    m_innerSpaceKernel = clCreateKernel(m_program, "computeInnerSpaceVoxels", &err);
    checkOpenCLError(err, "clCreateKernel computeInnerSpaceVoxels");
    m_markInteriorKernel = clCreateKernel(m_program, "markInteriorVoxels", &err);
    checkOpenCLError(err, "clCreateKernel markInteriorVoxels");
    m_markFrontierKernel = clCreateKernel(m_program, "computeFrontierVoxels", &err);
    checkOpenCLError(err, "clCreateKernel computeFrontierVoxels");
    m_estimateCurvatureKernel = clCreateKernel(m_program, "estimateCurvature", &err);
    checkOpenCLError(err, "clCreateKernel estimateCurvature");
}

CurvatureEstimatorOpenCL::~CurvatureEstimatorOpenCL() {
    releaseKernel(m_estimateCurvatureKernel);
    releaseKernel(m_markFrontierKernel);
    releaseKernel(m_markInteriorKernel);
    releaseKernel(m_innerSpaceKernel);
    if (m_program) clReleaseProgram(m_program);
    if (m_queue) clReleaseCommandQueue(m_queue);
    if (m_context) clReleaseContext(m_context);
}

std::string CurvatureEstimatorOpenCL::loadKernelSource(const std::string& path) const {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open OpenCL curvature kernel source: " + path);
    }

    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}

void CurvatureEstimatorOpenCL::checkOpenCLError(cl_int error, const char* message) const {
    if (error != CL_SUCCESS) {
        std::ostringstream oss;
        oss << "OpenCL error " << error << " at " << message;
        throw std::runtime_error(oss.str());
    }
}

void CurvatureEstimatorOpenCL::releaseKernel(cl_kernel& kernel) {
    if (kernel) {
        clReleaseKernel(kernel);
        kernel = nullptr;
    }
}

void CurvatureEstimatorOpenCL::preprocessVoxels(std::vector<unsigned char>& voxels,
                                                const Dimensions3i& dims) {
    std::cout << "Preprocessing curvature voxels using OpenCL..." << std::endl;

    cl_int err = CL_SUCCESS;
    const size_t voxelCount = voxels.size();
    cl_mem voxelsBuffer = clCreateBuffer(
        m_context,
        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
        voxelCount * sizeof(unsigned char),
        voxels.data(),
        &err);
    checkOpenCLError(err, "clCreateBuffer preprocess voxels");

    auto runInnerSpacePlane = [&](int R, int C, int D, int plane) {
        checkOpenCLError(clSetKernelArg(m_innerSpaceKernel, 0, sizeof(cl_mem), &voxelsBuffer), "clSetKernelArg inner voxels");
        checkOpenCLError(clSetKernelArg(m_innerSpaceKernel, 1, sizeof(int), &R), "clSetKernelArg inner R");
        checkOpenCLError(clSetKernelArg(m_innerSpaceKernel, 2, sizeof(int), &C), "clSetKernelArg inner C");
        checkOpenCLError(clSetKernelArg(m_innerSpaceKernel, 3, sizeof(int), &D), "clSetKernelArg inner D");
        checkOpenCLError(clSetKernelArg(m_innerSpaceKernel, 4, sizeof(int), &plane), "clSetKernelArg inner plane");

        const size_t globalSize[2] = {
            static_cast<size_t>(R),
            static_cast<size_t>(C)
        };
        checkOpenCLError(clEnqueueNDRangeKernel(m_queue, m_innerSpaceKernel, 2, nullptr, globalSize, nullptr, 0, nullptr, nullptr),
                         "clEnqueueNDRangeKernel computeInnerSpaceVoxels");
        checkOpenCLError(clFinish(m_queue), "clFinish computeInnerSpaceVoxels");
    };

    runInnerSpacePlane(dims.width, dims.height, dims.depth, 2);
    runInnerSpacePlane(dims.depth, dims.width, dims.height, 1);
    runInnerSpacePlane(dims.height, dims.depth, dims.width, 0);

    int width = dims.width;
    int height = dims.height;
    int depth = dims.depth;
    checkOpenCLError(clSetKernelArg(m_markInteriorKernel, 0, sizeof(cl_mem), &voxelsBuffer), "clSetKernelArg mark interior voxels");
    checkOpenCLError(clSetKernelArg(m_markInteriorKernel, 1, sizeof(int), &width), "clSetKernelArg mark interior width");
    checkOpenCLError(clSetKernelArg(m_markInteriorKernel, 2, sizeof(int), &height), "clSetKernelArg mark interior height");
    checkOpenCLError(clSetKernelArg(m_markInteriorKernel, 3, sizeof(int), &depth), "clSetKernelArg mark interior depth");
    const size_t voxelGridSize[3] = {
        static_cast<size_t>(width),
        static_cast<size_t>(height),
        static_cast<size_t>(depth)
    };
    checkOpenCLError(clEnqueueNDRangeKernel(m_queue, m_markInteriorKernel, 3, nullptr, voxelGridSize, nullptr, 0, nullptr, nullptr),
                     "clEnqueueNDRangeKernel markInteriorVoxels");
    checkOpenCLError(clFinish(m_queue), "clFinish markInteriorVoxels");

    checkOpenCLError(clSetKernelArg(m_markFrontierKernel, 0, sizeof(cl_mem), &voxelsBuffer), "clSetKernelArg mark frontier voxels");
    checkOpenCLError(clSetKernelArg(m_markFrontierKernel, 1, sizeof(int), &width), "clSetKernelArg mark frontier width");
    checkOpenCLError(clSetKernelArg(m_markFrontierKernel, 2, sizeof(int), &height), "clSetKernelArg mark frontier height");
    checkOpenCLError(clSetKernelArg(m_markFrontierKernel, 3, sizeof(int), &depth), "clSetKernelArg mark frontier depth");
    checkOpenCLError(clEnqueueNDRangeKernel(m_queue, m_markFrontierKernel, 3, nullptr, voxelGridSize, nullptr, 0, nullptr, nullptr),
                     "clEnqueueNDRangeKernel computeFrontierVoxels");
    checkOpenCLError(clFinish(m_queue), "clFinish computeFrontierVoxels");

    checkOpenCLError(clEnqueueReadBuffer(m_queue, voxelsBuffer, CL_TRUE, 0, voxelCount * sizeof(unsigned char), voxels.data(), 0, nullptr, nullptr),
                     "clEnqueueReadBuffer preprocess voxels");
    clReleaseMemObject(voxelsBuffer);
}

void CurvatureEstimatorOpenCL::estimateCurvature(int curvLength,
                                                 const std::vector<unsigned char>& voxels,
                                                 std::vector<int>& curvatures,
                                                 const Dimensions3i& dims) {
    std::cout << "Estimating curvature using OpenCL..." << std::endl;

    if (curvLength < 1) {
        throw std::runtime_error("curve length must be positive");
    }
    if (curvLength > kMaxOpenCLCurveLength) {
        throw std::runtime_error("OpenCL curvature curve length exceeds MAX_CURVE_LENGTH");
    }
    if (curvatures.size() != voxels.size()) {
        throw std::runtime_error("curvatures and voxels must have the same grid size");
    }

    cl_int err = CL_SUCCESS;
    const size_t voxelCount = voxels.size();
    cl_mem voxelsBuffer = clCreateBuffer(
        m_context,
        CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
        voxelCount * sizeof(unsigned char),
        const_cast<unsigned char*>(voxels.data()),
        &err);
    checkOpenCLError(err, "clCreateBuffer curvature voxels");

    std::fill(curvatures.begin(), curvatures.end(), std::numeric_limits<int>::max());
    cl_mem curvaturesBuffer = clCreateBuffer(
        m_context,
        CL_MEM_WRITE_ONLY | CL_MEM_COPY_HOST_PTR,
        curvatures.size() * sizeof(int),
        curvatures.data(),
        &err);
    checkOpenCLError(err, "clCreateBuffer curvatures");

    int width = dims.width;
    int height = dims.height;
    int depth = dims.depth;
    checkOpenCLError(clSetKernelArg(m_estimateCurvatureKernel, 0, sizeof(cl_mem), &voxelsBuffer), "clSetKernelArg curvature voxels");
    checkOpenCLError(clSetKernelArg(m_estimateCurvatureKernel, 1, sizeof(cl_mem), &curvaturesBuffer), "clSetKernelArg curvature output");
    checkOpenCLError(clSetKernelArg(m_estimateCurvatureKernel, 2, sizeof(int), &curvLength), "clSetKernelArg curvature length");
    checkOpenCLError(clSetKernelArg(m_estimateCurvatureKernel, 3, sizeof(int), &width), "clSetKernelArg curvature width");
    checkOpenCLError(clSetKernelArg(m_estimateCurvatureKernel, 4, sizeof(int), &height), "clSetKernelArg curvature height");
    checkOpenCLError(clSetKernelArg(m_estimateCurvatureKernel, 5, sizeof(int), &depth), "clSetKernelArg curvature depth");

    const size_t globalSize = voxelCount;
    checkOpenCLError(clEnqueueNDRangeKernel(m_queue, m_estimateCurvatureKernel, 1, nullptr, &globalSize, nullptr, 0, nullptr, nullptr),
                     "clEnqueueNDRangeKernel estimateCurvature");
    checkOpenCLError(clFinish(m_queue), "clFinish estimateCurvature");

    checkOpenCLError(clEnqueueReadBuffer(m_queue, curvaturesBuffer, CL_TRUE, 0, curvatures.size() * sizeof(int), curvatures.data(), 0, nullptr, nullptr),
                     "clEnqueueReadBuffer curvatures");

    clReleaseMemObject(curvaturesBuffer);
    clReleaseMemObject(voxelsBuffer);
}