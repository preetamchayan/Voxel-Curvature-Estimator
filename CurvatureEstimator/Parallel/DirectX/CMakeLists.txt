add_library(curvature_estimator_directx STATIC EXCLUDE_FROM_ALL
    CurvatureEstimatorDirectX.cpp
)

target_link_libraries(curvature_estimator_directx PUBLIC
    project_options
    helper
    d3d11
    d3dcompiler
    dxgi
)

target_include_directories(curvature_estimator_directx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
