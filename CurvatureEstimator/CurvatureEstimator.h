#pragma once

// OS-neutral     -- Windows / Linux (NOT MacOS due to lack of OpenCL / Vulkan / CUDA support)
// Vendor-neutral -- NVIDIA / AMD / Intel / Adreno (Qualcomm) / Mali (ARM) (NOT Apple Silicon)
// Device-neutral -- CPU, GPU, embedded devices
#define SERIAL 0
#define PARALLEL_OPENCL 1  // OS-neutral,   Vendor-neutral, Device-neutral
#define PARALLEL_VULKAN 2  // OS-neutral,   Vendor-neutral, GPU-only
#define PARALLEL_CUDA 3    // OS-neutral,   NVIDIA-only,    GPU-only
#define PARALLEL_DIRECTX 4 // Windows-only, Vendor-neutral, GPU-only
#define PARALLEL_METAL 5   // MacOS-only,   Apple-only,     GPU-only

#ifndef CURVATURE_ESTIMATOR_MODE
#define CURVATURE_ESTIMATOR_MODE PARALLEL_METAL
#endif