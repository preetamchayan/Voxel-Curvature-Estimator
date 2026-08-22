cmake_minimum_required(VERSION 3.24)

project(VoxelCurvatureEstimator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(VOXEL_CURVATURE_BACKENDS SERIAL OPENCL VULKAN CUDA DIRECTX METAL)
set(VOXELIZER_BACKEND "AUTO" CACHE STRING "Voxelizer backend: AUTO, SERIAL, OPENCL, VULKAN, CUDA, DIRECTX, METAL")
set(CURVATURE_BACKEND "AUTO" CACHE STRING "Curvature estimator backend: AUTO, SERIAL, OPENCL, VULKAN, CUDA, DIRECTX, METAL")
set_property(CACHE VOXELIZER_BACKEND PROPERTY STRINGS AUTO ${VOXEL_CURVATURE_BACKENDS})
set_property(CACHE CURVATURE_BACKEND PROPERTY STRINGS AUTO ${VOXEL_CURVATURE_BACKENDS})

option(ENABLE_OPENCL "Enable OpenCL backends when available" ON)
option(ENABLE_VULKAN "Enable Vulkan backends when available" ON)
option(ENABLE_CUDA "Enable CUDA backends when available" ON)
option(ENABLE_DIRECTX "Enable DirectX backends when available" ON)
option(ENABLE_METAL "Enable Metal backends when available" ON)
option(BUILD_VULKAN_SHADERS "Compile Vulkan GLSL compute shaders to SPIR-V when possible" ON)

# Prefer the same SDK environment variables used by the VS Code tasks when they
# are available. Users can still override any of these with normal CMake cache
# variables on the command line or in a preset.
if(DEFINED ENV{OPENCL_SDK})
    file(TO_CMAKE_PATH "$ENV{OPENCL_SDK}" OPENCL_SDK_ROOT)
    list(APPEND CMAKE_PREFIX_PATH "${OPENCL_SDK_ROOT}")

    if(NOT OpenCL_INCLUDE_DIR AND EXISTS "${OPENCL_SDK_ROOT}/include/CL/cl.h")
        set(OpenCL_INCLUDE_DIR "${OPENCL_SDK_ROOT}/include" CACHE PATH "OpenCL include directory")
    endif()

    if(NOT OpenCL_LIBRARY)
        if(WIN32)
            set(OPENCL_LIBRARY_CANDIDATES
                "${OPENCL_SDK_ROOT}/lib/OpenCL.lib"
                "${OPENCL_SDK_ROOT}/lib/x64/OpenCL.lib"
                "${OPENCL_SDK_ROOT}/Lib/OpenCL.lib"
                "${OPENCL_SDK_ROOT}/Lib/x64/OpenCL.lib"
            )
        elseif(APPLE)
            set(OPENCL_LIBRARY_CANDIDATES
                "${OPENCL_SDK_ROOT}/lib/libOpenCL.dylib"
            )
        else()
            set(OPENCL_LIBRARY_CANDIDATES
                "${OPENCL_SDK_ROOT}/lib/libOpenCL.so"
                "${OPENCL_SDK_ROOT}/lib64/libOpenCL.so"
            )
        endif()

        foreach(OPENCL_LIBRARY_CANDIDATE IN LISTS OPENCL_LIBRARY_CANDIDATES)
            if(EXISTS "${OPENCL_LIBRARY_CANDIDATE}")
                set(OpenCL_LIBRARY "${OPENCL_LIBRARY_CANDIDATE}" CACHE FILEPATH "OpenCL library")
                break()
            endif()
        endforeach()
    endif()
endif()

if(DEFINED ENV{VULKAN_SDK})
    file(TO_CMAKE_PATH "$ENV{VULKAN_SDK}" VULKAN_SDK_ROOT)
    list(APPEND CMAKE_PREFIX_PATH "${VULKAN_SDK_ROOT}")

    if(NOT Vulkan_INCLUDE_DIR AND EXISTS "${VULKAN_SDK_ROOT}/Include/vulkan/vulkan.h")
        set(Vulkan_INCLUDE_DIR "${VULKAN_SDK_ROOT}/Include" CACHE PATH "Vulkan include directory")
    endif()

    if(NOT Vulkan_LIBRARY)
        if(WIN32)
            set(VULKAN_LIBRARY_CANDIDATES
                "${VULKAN_SDK_ROOT}/Lib/vulkan-1.lib"
                "${VULKAN_SDK_ROOT}/lib/vulkan-1.lib"
            )
        elseif(APPLE)
            set(VULKAN_LIBRARY_CANDIDATES
                "${VULKAN_SDK_ROOT}/lib/libvulkan.dylib"
            )
        else()
            set(VULKAN_LIBRARY_CANDIDATES
                "${VULKAN_SDK_ROOT}/lib/libvulkan.so"
                "${VULKAN_SDK_ROOT}/lib64/libvulkan.so"
            )
        endif()

        foreach(VULKAN_LIBRARY_CANDIDATE IN LISTS VULKAN_LIBRARY_CANDIDATES)
            if(EXISTS "${VULKAN_LIBRARY_CANDIDATE}")
                set(Vulkan_LIBRARY "${VULKAN_LIBRARY_CANDIDATE}" CACHE FILEPATH "Vulkan library")
                break()
            endif()
        endforeach()
    endif()

    if(NOT Vulkan_GLSLANG_VALIDATOR_EXECUTABLE)
        if(WIN32)
            set(GLSLANG_VALIDATOR_CANDIDATES
                "${VULKAN_SDK_ROOT}/Bin/glslangValidator.exe"
                "${VULKAN_SDK_ROOT}/bin/glslangValidator.exe"
            )
        else()
            set(GLSLANG_VALIDATOR_CANDIDATES
                "${VULKAN_SDK_ROOT}/bin/glslangValidator"
                "${VULKAN_SDK_ROOT}/Bin/glslangValidator"
            )
        endif()

        foreach(GLSLANG_VALIDATOR_CANDIDATE IN LISTS GLSLANG_VALIDATOR_CANDIDATES)
            if(EXISTS "${GLSLANG_VALIDATOR_CANDIDATE}")
                set(Vulkan_GLSLANG_VALIDATOR_EXECUTABLE "${GLSLANG_VALIDATOR_CANDIDATE}" CACHE FILEPATH "glslangValidator executable")
                break()
            endif()
        endforeach()
    endif()
endif()

if(DEFINED ENV{CUDA_PATH})
    file(TO_CMAKE_PATH "$ENV{CUDA_PATH}" CUDA_SDK_ROOT)
    list(APPEND CMAKE_PREFIX_PATH "${CUDA_SDK_ROOT}")
    if(NOT CUDAToolkit_ROOT)
        set(CUDAToolkit_ROOT "${CUDA_SDK_ROOT}" CACHE PATH "CUDA Toolkit root")
    endif()

    if(NOT CMAKE_CUDA_COMPILER)
        if(WIN32 AND EXISTS "${CUDA_SDK_ROOT}/bin/nvcc.exe")
            set(CMAKE_CUDA_COMPILER "${CUDA_SDK_ROOT}/bin/nvcc.exe" CACHE FILEPATH "CUDA compiler")
        elseif(EXISTS "${CUDA_SDK_ROOT}/bin/nvcc")
            set(CMAKE_CUDA_COMPILER "${CUDA_SDK_ROOT}/bin/nvcc" CACHE FILEPATH "CUDA compiler")
        endif()
    endif()
endif()

include(CheckLanguage)

find_package(OpenCL QUIET)
find_package(Vulkan QUIET)

if(APPLE)
    set(OpenCL_INCLUDE_DIR "")
    foreach(_candidate
        "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX26.5.sdk/System/Library/Frameworks/OpenCL.framework/Versions/A/Headers"
        "/Library/Developer/CommandLineTools/SDKs/MacOSX15.4.sdk/System/Library/Frameworks/OpenCL.framework/Versions/A/Headers"
        "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/System/Library/Frameworks/OpenCL.framework/Versions/A/Headers"
        "/System/Library/Frameworks/OpenCL.framework/Versions/A/Headers"
    )
        if(EXISTS "${_candidate}/CL/cl.h")
            set(OpenCL_INCLUDE_DIR "${_candidate}")
            break()
        endif()
    endforeach()

    if(OpenCL_INCLUDE_DIR)
        set(OpenCL_FOUND TRUE)
        set(OpenCL_LIBRARY "/System/Library/Frameworks/OpenCL.framework")
    endif()
endif()

set(HAVE_OPENCL OFF)
if(ENABLE_OPENCL AND OpenCL_FOUND)
    set(HAVE_OPENCL ON)
endif()

set(HAVE_VULKAN OFF)
if(ENABLE_VULKAN AND Vulkan_FOUND)
    set(HAVE_VULKAN ON)
endif()

set(HAVE_CUDA OFF)
if(ENABLE_CUDA)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND)
            set(HAVE_CUDA ON)
            if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
                set(CMAKE_CUDA_ARCHITECTURES native CACHE STRING "CUDA architectures" FORCE)
            endif()
        endif()
    endif()
endif()

set(HAVE_DIRECTX OFF)
if(ENABLE_DIRECTX AND WIN32)
    set(HAVE_DIRECTX ON)
endif()

set(HAVE_METAL OFF)
if(ENABLE_METAL AND APPLE)
    enable_language(OBJCXX)
    set(HAVE_METAL ON)
endif()

function(resolve_backend INPUT_VAR OUTPUT_VAR)
    string(TOUPPER "${${INPUT_VAR}}" requested)
    if(requested STREQUAL "AUTO")
        if(HAVE_OPENCL)
            set(resolved OPENCL)
        elseif(HAVE_VULKAN)
            set(resolved VULKAN)
        elseif(HAVE_CUDA)
            set(resolved CUDA)
        elseif(HAVE_DIRECTX)
            set(resolved DIRECTX)
        elseif(HAVE_METAL AND NOT "${INPUT_VAR}" STREQUAL "CURVATURE_BACKEND")
            set(resolved METAL)
        else()
            set(resolved SERIAL)
        endif()
    else()
        if(NOT requested IN_LIST VOXEL_CURVATURE_BACKENDS)
            message(FATAL_ERROR "${INPUT_VAR} must be AUTO, SERIAL, OPENCL, VULKAN, CUDA, DIRECTX, or METAL")
        endif()
        set(resolved ${requested})
    endif()

    if(resolved STREQUAL "OPENCL" AND NOT HAVE_OPENCL)
        message(FATAL_ERROR "${INPUT_VAR}=OPENCL was requested, but OpenCL was not found or ENABLE_OPENCL=OFF")
    elseif(resolved STREQUAL "VULKAN" AND NOT HAVE_VULKAN)
        message(FATAL_ERROR "${INPUT_VAR}=VULKAN was requested, but Vulkan was not found or ENABLE_VULKAN=OFF")
    elseif(resolved STREQUAL "CUDA" AND NOT HAVE_CUDA)
        message(FATAL_ERROR "${INPUT_VAR}=CUDA was requested, but CUDA/CUDAToolkit was not found or ENABLE_CUDA=OFF")
    elseif(resolved STREQUAL "DIRECTX" AND NOT HAVE_DIRECTX)
        message(FATAL_ERROR "${INPUT_VAR}=DIRECTX was requested, but DirectX is only available on Windows and ENABLE_DIRECTX must be ON")
    elseif(resolved STREQUAL "METAL" AND NOT HAVE_METAL)
        message(FATAL_ERROR "${INPUT_VAR}=METAL was requested, but Metal is only available on Apple platforms and ENABLE_METAL must be ON")
    endif()

    set(${OUTPUT_VAR} ${resolved} PARENT_SCOPE)
endfunction()

function(backend_numeric_value BACKEND OUTPUT_VAR)
    if(BACKEND STREQUAL "SERIAL")
        set(value 0)
    elseif(BACKEND STREQUAL "OPENCL")
        set(value 1)
    elseif(BACKEND STREQUAL "VULKAN")
        set(value 2)
    elseif(BACKEND STREQUAL "CUDA")
        set(value 3)
    elseif(BACKEND STREQUAL "DIRECTX")
        set(value 4)
    elseif(BACKEND STREQUAL "METAL")
        set(value 5)
    else()
        message(FATAL_ERROR "Unknown backend: ${BACKEND}")
    endif()
    set(${OUTPUT_VAR} ${value} PARENT_SCOPE)
endfunction()

resolve_backend(VOXELIZER_BACKEND VOXELIZER_BACKEND_RESOLVED)
resolve_backend(CURVATURE_BACKEND CURVATURE_BACKEND_RESOLVED)
backend_numeric_value(${VOXELIZER_BACKEND_RESOLVED} VOXELIZER_BACKEND_VALUE)
backend_numeric_value(${CURVATURE_BACKEND_RESOLVED} CURVATURE_BACKEND_VALUE)

message(STATUS "Available backends: OpenCL=${HAVE_OPENCL}, Vulkan=${HAVE_VULKAN}, CUDA=${HAVE_CUDA}, DirectX=${HAVE_DIRECTX}, Metal=${HAVE_METAL}")
message(STATUS "Selected voxelizer backend: ${VOXELIZER_BACKEND_RESOLVED}")
message(STATUS "Selected curvature backend: ${CURVATURE_BACKEND_RESOLVED}")

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)
if(WIN32)
    target_compile_definitions(project_options INTERFACE
        _CRT_SECURE_NO_WARNINGS
        _ALLOW_COMPILER_AND_STL_VERSION_MISMATCH
        NOMINMAX
    )
endif()

target_include_directories(project_options INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(Helper)
add_subdirectory(Voxelizer)
add_subdirectory(CurvatureEstimator)

add_executable(voxelCurvatureApp main.cpp)
target_link_libraries(voxelCurvatureApp PRIVATE
    project_options
    helper
    voxelizer
    curvature_estimator
)

set_target_properties(voxelCurvatureApp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
