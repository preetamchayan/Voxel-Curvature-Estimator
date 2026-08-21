add_library(voxelizer_metal STATIC EXCLUDE_FROM_ALL
    MeshVoxelizerMetal.mm
)

target_link_libraries(voxelizer_metal PUBLIC
    project_options
    helper
    "-framework Metal"
    "-framework Foundation"
)

target_include_directories(voxelizer_metal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
