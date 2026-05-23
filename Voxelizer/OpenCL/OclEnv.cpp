#include "OclEnv.h"
#include <iostream>
#include <fstream>
#include <sstream>

OclEnv::OclEnv() : m_context(nullptr), m_queue(nullptr) {
    // Initialize OpenCL context, device, and other resources here
    std::cout << "Initializing OpenCL environment..." << std::endl;

    cl_int err;
    cl_uint numPlatforms = 0;
    checkOpenCLError(clGetPlatformIDs(0, nullptr, &numPlatforms), "clGetPlatformIDs");
    if (numPlatforms == 0)
    {
        std::cerr << "No OpenCL platforms found." << std::endl;
        exit(1);
    }
    std::vector<cl_platform_id> platforms(numPlatforms);
    checkOpenCLError(clGetPlatformIDs(numPlatforms, platforms.data(), nullptr), "clGetPlatformIDs");

    // For simplicity, just use the first platform and device
    cl_platform_id platform = platforms[1];
    cl_uint numDevices = 0;
    checkOpenCLError(clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 0, nullptr, &numDevices), "clGetDeviceIDs");
    if (numDevices == 0)
    {
        std::cerr << "No OpenCL devices found." << std::endl;
        exit(1);
    }
    std::vector<cl_device_id> devices(numDevices);
    checkOpenCLError(clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, numDevices, devices.data(), nullptr), "clGetDeviceIDs");
    m_device = devices[0];
    m_context = clCreateContext(nullptr, 1, &m_device, nullptr, nullptr, &err);
    checkOpenCLError(err, "clCreateContext");
    m_queue = clCreateCommandQueue(m_context, m_device, 0, &err);
    checkOpenCLError(err, "clCreateCommandQueue");

    // Placeholder for running OpenCL kernels
    // In a real implementation, you would set up buffers, compile kernels, and execute them here.
    std::string kernelSource = loadKernelSource("Voxelizer/OpenCL/VoxelizerKernel.cl");
    const char *sourceStr = kernelSource.c_str();
    size_t sourceSize = kernelSource.size();
    m_program = clCreateProgramWithSource(m_context, 1, &sourceStr, &sourceSize, &err);
    checkOpenCLError(err, "clCreateProgramWithSource");
    err = clBuildProgram(m_program, 1, &m_device, nullptr, nullptr, nullptr);
    if (err != CL_SUCCESS)
    {
        size_t logSize;
        clGetProgramBuildInfo(m_program, m_device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &logSize);
        std::vector<char> log(logSize);
        clGetProgramBuildInfo(m_program, m_device, CL_PROGRAM_BUILD_LOG, logSize, log.data(), nullptr);
        std::cerr << "OpenCL build log:\n"
                  << log.data() << std::endl;
        checkOpenCLError(err, "clBuildProgram");
    }
    m_kernel = clCreateKernel(m_program, "voxelize_faces", &err);
    checkOpenCLError(err, "clCreateKernel");
}

OclEnv::~OclEnv()
{
    // Clean up OpenCL resources here
    std::cout << "Cleaning up OpenCL environment..." << std::endl;
    if (m_queue)
    {
        clReleaseCommandQueue(m_queue);
    }
    if (m_context)
    {
        clReleaseContext(m_context);
    }
    if (m_kernel)
    {
        clReleaseKernel(m_kernel);
    }
    if (m_program)
    {
        clReleaseProgram(m_program);
    }
    if (m_facesBuffer)
    {
        clReleaseMemObject(m_facesBuffer);
    }
    if (m_verticesBuffer)
    {
        clReleaseMemObject(m_verticesBuffer);
    }
    if (m_voxelsBuffer)
    {
        clReleaseMemObject(m_voxelsBuffer);
    }
}

std::string OclEnv::loadKernelSource(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Unable to open kernel source: " << path << std::endl;
        exit(1);
    }
    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}

void OclEnv::checkOpenCLError(cl_int error, const char *message)
{
    if (error != CL_SUCCESS)
    {
        std::cerr << "OpenCL error " << error << " at " << message << std::endl;
        exit(1);
    }
}

void OclEnv::voxelize(
    std::vector<unsigned char> &voxels,
    const std::vector<Point3i> &vertices,
    const std::vector<Face> &faces,
    const BBox3i &scaledBounds,
    const Dimensions3i &dims)
{
    // Implement voxelization using OpenCL kernels here
    std::cout << "Voxelizing using OpenCL..." << std::endl;
    cl_int err;
    // Prepare data
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

    m_facesBuffer = clCreateBuffer(m_context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, flatFaces.size() * sizeof(int), flatFaces.data(), &err);
    checkOpenCLError(err, "clCreateBuffer faces");
    m_verticesBuffer = clCreateBuffer(m_context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, flatVertices.size() * sizeof(int), flatVertices.data(), &err);
    checkOpenCLError(err, "clCreateBuffer vertices");
    m_voxelsBuffer = clCreateBuffer(m_context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, voxels.size() * sizeof(unsigned char), voxels.data(), &err);
    checkOpenCLError(err, "clCreateBuffer voxels");

    checkOpenCLError(clSetKernelArg(m_kernel, 0, sizeof(cl_mem), &m_facesBuffer), "clSetKernelArg 0");
    int numFaces = faces.size();
    cl_mem totalSizeBuffer = clCreateBuffer(m_context, CL_MEM_READ_WRITE,
                                            sizeof(int), nullptr, &err);
    int totalSize = 0;
    clEnqueueWriteBuffer(m_queue, totalSizeBuffer, CL_TRUE, 0,
                         sizeof(int), &totalSize, 0, nullptr, nullptr);
    checkOpenCLError(clSetKernelArg(m_kernel, 1, sizeof(int), &numFaces), "clSetKernelArg 1");
    checkOpenCLError(clSetKernelArg(m_kernel, 2, sizeof(cl_mem), &m_verticesBuffer), "clSetKernelArg 2");
    checkOpenCLError(clSetKernelArg(m_kernel, 3, sizeof(cl_mem), &m_voxelsBuffer), "clSetKernelArg 3");
    checkOpenCLError(clSetKernelArg(m_kernel, 4, sizeof(int), &dims.width), "clSetKernelArg 4");
    checkOpenCLError(clSetKernelArg(m_kernel, 5, sizeof(int), &dims.height), "clSetKernelArg 5");
    checkOpenCLError(clSetKernelArg(m_kernel, 6, sizeof(int), &dims.depth), "clSetKernelArg 6");
    checkOpenCLError(clSetKernelArg(m_kernel, 7, sizeof(int), &scaledBounds.xmin), "clSetKernelArg 7");
    checkOpenCLError(clSetKernelArg(m_kernel, 8, sizeof(int), &scaledBounds.ymin), "clSetKernelArg 8");
    checkOpenCLError(clSetKernelArg(m_kernel, 9, sizeof(int), &scaledBounds.zmin), "clSetKernelArg 9");
    checkOpenCLError(clSetKernelArg(m_kernel, 10, sizeof(cl_mem), &totalSizeBuffer), "clSetKernelArg 10");

    size_t globalSize = numFaces;
    checkOpenCLError(clEnqueueNDRangeKernel(m_queue, m_kernel, 1, nullptr, &globalSize, nullptr, 0, nullptr, nullptr), "clEnqueueNDRangeKernel");
    checkOpenCLError(clFinish(m_queue), "clFinish");

    checkOpenCLError(clEnqueueReadBuffer(m_queue, m_voxelsBuffer, CL_TRUE, 0, voxels.size() * sizeof(unsigned char), voxels.data(), 0, nullptr, nullptr), "clEnqueueReadBuffer");
    checkOpenCLError(clEnqueueReadBuffer(m_queue, totalSizeBuffer, CL_TRUE, 0, sizeof(int), &totalSize, 0, nullptr, nullptr), "clEnqueueReadBuffer");

    std::cout << "Total faces processed: " << totalSize << std::endl;
}