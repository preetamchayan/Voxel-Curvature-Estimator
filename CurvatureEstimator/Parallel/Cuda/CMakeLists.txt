add_library(curvature_estimator_cuda STATIC EXCLUDE_FROM_ALL
    CurvatureEstimatorCuda.cu
    CurvatureEstimatorKernel.cu
)

target_link_libraries(curvature_estimator_cuda PUBLIC
    project_options
    helper
    CUDA::cudart
)

target_include_directories(curvature_estimator_cuda PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(curvature_estimator_cuda PROPERTIES
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    CUDA_SEPARABLE_COMPILATION ON
)
