#include "CurvatureEstimator.h"

// Include only the selected parallel environment header to avoid optional dependency
#if CURVATURE_ESTIMATOR_MODE == PARALLEL_OPENCL
#include "OpenCL/CurvatureEstimatorOclEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_VULKAN
#include "Vulkan/CurvatureEstimatorVulkanEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_CUDA
#include "Cuda/CurvatureEstimatorCudaEnv.h"
#elif CURVATURE_ESTIMATOR_MODE == PARALLEL_DIRECTX
#include "DirectX/CurvatureEstimatorDirectxEnv.h"
#else
#include "OpenCL/CurvatureEstimatorOclEnv.h"
#endif
#include <algorithm>
#include <climits>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cassert>