add_library(voxelizer_vulkan STATIC EXCLUDE_FROM_ALL
    MeshVoxelizerVulkan.cpp
)

target_link_libraries(voxelizer_vulkan PUBLIC
    project_options
    helper
    Vulkan::Vulkan
)

target_include_directories(voxelizer_vulkan PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_VULKAN_SHADERS)
    find_program(GLSLANG_VALIDATOR glslangValidator HINTS ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE})
    if(GLSLANG_VALIDATOR)
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/MeshVoxelizerKernel.spv
            COMMAND ${GLSLANG_VALIDATOR} -V ${CMAKE_CURRENT_SOURCE_DIR}/MeshVoxelizerKernel.comp -o ${CMAKE_CURRENT_SOURCE_DIR}/MeshVoxelizerKernel.spv
            DEPENDS
                ${CMAKE_CURRENT_SOURCE_DIR}/MeshVoxelizerKernel.comp
                ${CMAKE_CURRENT_SOURCE_DIR}/DSSCreator.comp
                ${CMAKE_CURRENT_SOURCE_DIR}/TriangleVoxelizer.comp
            VERBATIM
        )
        add_custom_target(voxelizer_vulkan_shader DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/MeshVoxelizerKernel.spv)
        add_dependencies(voxelizer_vulkan voxelizer_vulkan_shader)
    else()
        message(STATUS "glslangValidator not found; skipping voxelizer Vulkan shader compilation")
    endif()
endif()
