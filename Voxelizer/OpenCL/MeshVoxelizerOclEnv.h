#pragma once

#include <CL/cl.h>
#include <vector>
#include <string>

// #include "../../Helper/GeometryTypes.h"
#include "../MeshVoxelizerBaseEnv.h"

class MeshVoxelizerOclEnv : public MeshVoxelizerBaseEnv {
public:
    MeshVoxelizerOclEnv();
    ~MeshVoxelizerOclEnv();
    void voxelize(
        std::vector<unsigned char> &voxels,
        const std::vector<Point3i> &vertices,
        const std::vector<Face> &faces,
        const BBox3i &scaledBounds,
        const Dimensions3i &dims) override;

private:
    std::string loadKernelSource(const std::string& path);
    void checkOpenCLError(cl_int error, const char* message);

private:
    cl_context m_context;
    cl_command_queue m_queue;
    cl_device_id m_device;
    cl_kernel m_kernel;
    cl_mem m_facesBuffer;
    cl_mem m_verticesBuffer;
    cl_mem m_voxelsBuffer;
    cl_program m_program;
};