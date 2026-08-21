add_library(mesh_loader STATIC
    MeshLoader.cpp
)

target_link_libraries(mesh_loader PUBLIC project_options)
target_include_directories(mesh_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
