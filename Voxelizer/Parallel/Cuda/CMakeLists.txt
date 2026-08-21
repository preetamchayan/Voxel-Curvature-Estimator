add_library(voxelizer_cuda STATIC EXCLUDE_FROM_ALL
    MeshVoxelizerCuda.cu
    MeshVoxelizerKernel.cu
)

target_link_libraries(voxelizer_cuda PUBLIC
    project_options
    helper
    CUDA::cudart
)

target_include_directories(voxelizer_cuda PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(voxelizer_cuda PROPERTIES
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    CUDA_SEPARABLE_COMPILATION ON
)
