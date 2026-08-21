add_library(voxelizer_opencl STATIC EXCLUDE_FROM_ALL
    MeshVoxelizerOpenCL.cpp
)

target_link_libraries(voxelizer_opencl PUBLIC
    project_options
    helper
    OpenCL::OpenCL
)

target_include_directories(voxelizer_opencl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
