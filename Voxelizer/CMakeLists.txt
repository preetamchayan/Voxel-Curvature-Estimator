add_subdirectory(Serial)
add_subdirectory(Parallel)

add_library(voxelizer STATIC
    MeshVoxelizer.cpp
)

target_link_libraries(voxelizer PUBLIC
    project_options
    helper
)

target_include_directories(voxelizer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(voxelizer PUBLIC VOXELIZER=${VOXELIZER_BACKEND_VALUE})

if(VOXELIZER_BACKEND_RESOLVED STREQUAL "SERIAL")
    target_link_libraries(voxelizer PUBLIC voxelizer_serial)
elseif(VOXELIZER_BACKEND_RESOLVED STREQUAL "OPENCL")
    target_link_libraries(voxelizer PUBLIC voxelizer_opencl)
elseif(VOXELIZER_BACKEND_RESOLVED STREQUAL "VULKAN")
    target_link_libraries(voxelizer PUBLIC voxelizer_vulkan)
elseif(VOXELIZER_BACKEND_RESOLVED STREQUAL "CUDA")
    target_link_libraries(voxelizer PUBLIC voxelizer_cuda)
elseif(VOXELIZER_BACKEND_RESOLVED STREQUAL "DIRECTX")
    target_link_libraries(voxelizer PUBLIC voxelizer_directx)
elseif(VOXELIZER_BACKEND_RESOLVED STREQUAL "METAL")
    target_link_libraries(voxelizer PUBLIC voxelizer_metal)
endif()
