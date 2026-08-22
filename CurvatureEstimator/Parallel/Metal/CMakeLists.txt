add_library(curvature_estimator_metal STATIC EXCLUDE_FROM_ALL
    CurvatureEstimatorMetal.mm
)

target_link_libraries(curvature_estimator_metal PUBLIC
    project_options
    helper
    "-framework Metal"
    "-framework Foundation"
)

target_include_directories(curvature_estimator_metal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
