add_subdirectory(MeshLoader)

add_library(helper STATIC
    HelperFunctions.cpp
)

target_link_libraries(helper PUBLIC
    project_options
    mesh_loader
)
target_include_directories(helper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/MeshLoader
)
