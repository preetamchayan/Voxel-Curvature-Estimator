add_subdirectory(Serial)
add_subdirectory(Parallel)

add_library(curvature_estimator STATIC
    CurvatureEstimator.cpp
)

target_link_libraries(curvature_estimator PUBLIC
    project_options
    helper
)

target_include_directories(curvature_estimator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(curvature_estimator PUBLIC CURVATURE_ESTIMATOR=${CURVATURE_BACKEND_VALUE})

if(CURVATURE_BACKEND_RESOLVED STREQUAL "SERIAL")
    target_link_libraries(curvature_estimator PUBLIC curvature_estimator_serial)
elseif(CURVATURE_BACKEND_RESOLVED STREQUAL "OPENCL")
    target_link_libraries(curvature_estimator PUBLIC curvature_estimator_opencl)
elseif(CURVATURE_BACKEND_RESOLVED STREQUAL "VULKAN")
    target_link_libraries(curvature_estimator PUBLIC curvature_estimator_vulkan)
elseif(CURVATURE_BACKEND_RESOLVED STREQUAL "CUDA")
    target_link_libraries(curvature_estimator PUBLIC curvature_estimator_cuda)
elseif(CURVATURE_BACKEND_RESOLVED STREQUAL "DIRECTX")
    target_link_libraries(curvature_estimator PUBLIC curvature_estimator_directx)
elseif(CURVATURE_BACKEND_RESOLVED STREQUAL "METAL")
    target_link_libraries(curvature_estimator PUBLIC curvature_estimator_metal)
endif()
