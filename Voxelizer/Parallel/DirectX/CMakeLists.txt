add_library(voxelizer_directx STATIC EXCLUDE_FROM_ALL
    MeshVoxelizerDirectX.cpp
)

target_link_libraries(voxelizer_directx PUBLIC
    project_options
    helper
    d3d11
    d3dcompiler
    dxgi
)

target_include_directories(voxelizer_directx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
