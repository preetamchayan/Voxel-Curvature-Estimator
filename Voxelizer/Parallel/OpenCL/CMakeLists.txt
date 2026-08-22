add_library(voxelizer_opencl STATIC EXCLUDE_FROM_ALL
    MeshVoxelizerOpenCL.cpp
)

target_link_libraries(voxelizer_opencl PUBLIC
    project_options
    helper
    "-framework OpenCL"
)

target_include_directories(voxelizer_opencl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk/System/Library/Frameworks/OpenCL.framework/Versions/A/Headers
    /Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk/System/Library/Frameworks/OpenCL.framework/Versions/A/Headers
    /System/Library/Frameworks/OpenCL.framework/Versions/A/Headers
)
