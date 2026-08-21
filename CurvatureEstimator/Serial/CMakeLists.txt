add_library(curvature_estimator_serial STATIC
    CurvatureEstimatorSerial.cpp
)

target_link_libraries(curvature_estimator_serial PUBLIC project_options helper)
target_include_directories(curvature_estimator_serial PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
