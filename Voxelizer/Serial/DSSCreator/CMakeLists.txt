add_library(voxelizer_serial_dss_creator STATIC
    DSSCreator.cpp
)

target_link_libraries(voxelizer_serial_dss_creator PUBLIC project_options helper)
target_include_directories(voxelizer_serial_dss_creator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
