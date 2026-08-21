add_library(curvature_estimator_vulkan STATIC EXCLUDE_FROM_ALL
    CurvatureEstimatorVulkan.cpp
)

target_link_libraries(curvature_estimator_vulkan PUBLIC
    project_options
    helper
    Vulkan::Vulkan
)

target_include_directories(curvature_estimator_vulkan PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_VULKAN_SHADERS)
    find_program(GLSLANG_VALIDATOR glslangValidator HINTS ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE})
    if(GLSLANG_VALIDATOR)
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/CurvatureEstimatorKernel.spv
            COMMAND ${GLSLANG_VALIDATOR} -V ${CMAKE_CURRENT_SOURCE_DIR}/CurvatureEstimatorKernel.comp -o ${CMAKE_CURRENT_SOURCE_DIR}/CurvatureEstimatorKernel.spv
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/CurvatureEstimatorKernel.comp
            VERBATIM
        )
        add_custom_target(curvature_estimator_vulkan_shader DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/CurvatureEstimatorKernel.spv)
        add_dependencies(curvature_estimator_vulkan curvature_estimator_vulkan_shader)
    else()
        message(STATUS "glslangValidator not found; skipping curvature Vulkan shader compilation")
    endif()
endif()
