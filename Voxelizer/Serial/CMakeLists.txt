add_subdirectory(DSSCreator)
add_subdirectory(TriangleVoxelizer)

add_library(voxelizer_serial STATIC
    MeshVoxelizerSerial.cpp
)

target_link_libraries(voxelizer_serial PUBLIC
    project_options
    helper
    voxelizer_serial_dss_creator
    voxelizer_serial_triangle_voxelizer
)

target_include_directories(voxelizer_serial PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
