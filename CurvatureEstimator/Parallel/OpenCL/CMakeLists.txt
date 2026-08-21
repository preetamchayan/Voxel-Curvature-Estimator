add_library(curvature_estimator_opencl STATIC EXCLUDE_FROM_ALL
    CurvatureEstimatorOpenCL.cpp
)

target_link_libraries(curvature_estimator_opencl PUBLIC
    project_options
    helper
    OpenCL::OpenCL
)

target_include_directories(curvature_estimator_opencl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
