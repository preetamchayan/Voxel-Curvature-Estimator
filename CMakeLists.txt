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

include(CheckLanguage)

find_package(OpenCL QUIET)
find_package(Vulkan QUIET)

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
    elseif(resolved STREQUAL "METAL" AND "${INPUT_VAR}" STREQUAL "CURVATURE_BACKEND")
        message(FATAL_ERROR "CURVATURE_BACKEND=METAL was requested, but CurvatureEstimator/Parallel/Metal is currently empty")
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
