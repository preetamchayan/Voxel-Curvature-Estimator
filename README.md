# Voxel Curvature Estimator

Voxel Curvature Estimator is a C++20 project for converting triangle meshes into voxel grids and estimating curvature over the resulting voxelized surface. The project is structured around two major processing stages:

1. **Voxelization** — convert an input `.obj` triangle mesh into a discrete voxel representation.
2. **Curvature estimation** — compute curvature values over the surface voxels and export the result as OBJ/MTL plus an optional curvature log.

The codebase supports multiple compute backends so the same high-level workflow can be tested across CPU and GPU APIs:

- Serial CPU
- OpenCL
- Vulkan compute
- CUDA
- DirectX compute
- Metal

The build system is CMake-based and organized hierarchically so each module owns its own build description.

---

## Project layout

```text
.
├── CMakeLists.txt
├── README.md
├── main.cpp
├── assets/
├── output/
├── benchmark/
├── Helper/
│   ├── CMakeLists.txt
│   ├── GeometryTypes.h
│   ├── HelperFunctions.h
│   ├── HelperFunctions.cpp
│   └── MeshLoader/
│       ├── CMakeLists.txt
│       ├── MeshLoader.h
│       └── MeshLoader.cpp
├── Voxelizer/
│   ├── CMakeLists.txt
│   ├── MeshVoxelizer.h
│   ├── MeshVoxelizer.cpp
│   ├── MeshVoxelizerBase.h
│   ├── Serial/
│   │   ├── CMakeLists.txt
│   │   ├── MeshVoxelizerSerial.h
│   │   ├── MeshVoxelizerSerial.cpp
│   │   ├── DSSCreator/
│   │   │   ├── CMakeLists.txt
│   │   │   ├── DSSCreator.h
│   │   │   └── DSSCreator.cpp
│   │   └── TriangleVoxelizer/
│   │       ├── CMakeLists.txt
│   │       ├── TriangleVoxelizer.h
│   │       └── TriangleVoxelizer.cpp
│   └── Parallel/
│       ├── CMakeLists.txt
│       ├── OpenCL/
│       │   ├── CMakeLists.txt
│       │   ├── MeshVoxelizerOpenCL.h
│       │   ├── MeshVoxelizerOpenCL.cpp
│       │   └── MeshVoxelizerKernel.cl
│       ├── Vulkan/
│       │   ├── CMakeLists.txt
│       │   ├── MeshVoxelizerVulkan.h
│       │   ├── MeshVoxelizerVulkan.cpp
│       │   └── MeshVoxelizerKernel.comp
│       ├── Cuda/
│       │   ├── CMakeLists.txt
│       │   ├── MeshVoxelizerCuda.h
│       │   ├── MeshVoxelizerCuda.cu
│       │   ├── MeshVoxelizerKernel.h
│       │   └── MeshVoxelizerKernel.cu
│       ├── DirectX/
│       │   ├── CMakeLists.txt
│       │   ├── MeshVoxelizerDirectX.h
│       │   ├── MeshVoxelizerDirectX.cpp
│       │   └── MeshVoxelizerKernel.hlsl
│       └── Metal/
│           ├── CMakeLists.txt
│           ├── MeshVoxelizerMetal.h
│           ├── MeshVoxelizerMetal.mm
│           └── MeshVoxelizerKernel.metal
└── CurvatureEstimator/
    ├── CMakeLists.txt
    ├── CurvatureEstimator.h
    ├── CurvatureEstimator.cpp
    ├── CurvatureEstimatorBase.h
    ├── Serial/
    │   ├── CMakeLists.txt
    │   ├── CurvatureEstimatorSerial.h
    │   └── CurvatureEstimatorSerial.cpp
    └── Parallel/
        ├── CMakeLists.txt
        ├── OpenCL/
        │   ├── CMakeLists.txt
        │   ├── CurvatureEstimatorOpenCL.h
        │   ├── CurvatureEstimatorOpenCL.cpp
        │   └── CurvatureEstimatorKernel.cl
        ├── Vulkan/
        │   ├── CMakeLists.txt
        │   ├── CurvatureEstimatorVulkan.h
        │   ├── CurvatureEstimatorVulkan.cpp
        │   └── CurvatureEstimatorKernel.comp
        ├── Cuda/
        │   ├── CMakeLists.txt
        │   ├── CurvatureEstimatorCuda.h
        │   ├── CurvatureEstimatorCuda.cu
        │   ├── CurvatureEstimatorKernel.h
        │   └── CurvatureEstimatorKernel.cu
        ├── DirectX/
        │   ├── CMakeLists.txt
        │   ├── CurvatureEstimatorDirectX.h
        │   ├── CurvatureEstimatorDirectX.cpp
        │   └── CurvatureEstimatorKernel.hlsl
        └── Metal/
            ├── CMakeLists.txt
            ├── CurvatureEstimatorMetal.h
            ├── CurvatureEstimatorMetal.mm
            └── CurvatureEstimatorKernel.metal
```

---

## Build model

The project builds one executable:

```text
voxelCurvatureApp
```

The executable links together:

- `helper`
- `mesh_loader`
- `voxelizer`
- `curvature_estimator`

The voxelizer and curvature estimator each select exactly one backend at configure time.

Backend selection is controlled by these CMake cache variables:

| Variable | Purpose | Values |
|---|---|---|
| `VOXELIZER_BACKEND` | Selects the voxelization backend | `AUTO`, `SERIAL`, `OPENCL`, `VULKAN`, `CUDA`, `DIRECTX`, `METAL` |
| `CURVATURE_BACKEND` | Selects the curvature estimator backend | `AUTO`, `SERIAL`, `OPENCL`, `VULKAN`, `CUDA`, `DIRECTX`, `METAL` |

Both default to `AUTO`.

---

## Backend support matrix

| Backend | Voxelizer | Curvature Estimator | OS / vendor notes |
|---|---:|---:|---|
| Serial | Yes | Yes | Portable CPU implementation |
| OpenCL | Yes | Yes | Vendor-neutral if OpenCL SDK/runtime is available |
| Vulkan | Yes | Yes | Vendor-neutral Vulkan compute path |
| CUDA | Yes | Yes | NVIDIA CUDA only |
| DirectX | Yes | Yes | Windows only |
| Metal | Yes | Yes | Apple only |

---

## What does `AUTO` do?

When either backend variable is set to `AUTO`, CMake checks which dependencies are available and selects the first usable backend in this priority order:

1. OpenCL
2. Vulkan
3. CUDA
4. DirectX
5. Metal, voxelizer only
6. Serial fallback

Examples:

| Available SDKs/platform APIs | `VOXELIZER_BACKEND=AUTO` | `CURVATURE_BACKEND=AUTO` |
|---|---|---|
| OpenCL available | `OPENCL` | `OPENCL` |
| No OpenCL, Vulkan available | `VULKAN` | `VULKAN` |
| CUDA only | `CUDA` | `CUDA` |
| Windows DirectX only | `DIRECTX` | `DIRECTX` |
| Apple Metal only | `METAL` | `SERIAL` |
| No GPU SDK/API found | `SERIAL` | `SERIAL` |

If you want reproducible builds, prefer explicit backend selection instead of `AUTO`.

---

## CMake options

| Option | Default | Description |
|---|---:|---|
| `VOXELIZER_BACKEND` | `AUTO` | Select voxelizer backend |
| `CURVATURE_BACKEND` | `AUTO` | Select curvature estimator backend |
| `ENABLE_OPENCL` | `ON` | Allow OpenCL detection/build |
| `ENABLE_VULKAN` | `ON` | Allow Vulkan detection/build |
| `ENABLE_CUDA` | `ON` | Allow CUDA detection/build |
| `ENABLE_DIRECTX` | `ON` | Allow DirectX detection/build on Windows |
| `ENABLE_METAL` | `ON` | Allow Metal detection/build on Apple platforms |
| `BUILD_VULKAN_SHADERS` | `ON` | Compile Vulkan `.comp` shaders to `.spv` using `glslangValidator` when available |
| `CMAKE_CUDA_ARCHITECTURES` | `native` if CUDA is enabled and unspecified | CUDA GPU architectures to compile for |

CMake also honors common SDK environment variables used by the VS Code task configuration:

| Environment variable | Used for |
|---|---|
| `OPENCL_SDK` | OpenCL include and library discovery |
| `VULKAN_SDK` | Vulkan include/library discovery and `glslangValidator` lookup |
| `CUDA_PATH` | CUDA Toolkit root and `nvcc` lookup |

DirectX is detected on Windows through the Windows SDK libraries, and Metal is detected on Apple platforms through the system frameworks.

---

## Requirements

### Required for all builds

- CMake 3.24 or newer
- A C++20 compiler

Known working compiler families should include:

- MSVC / Visual Studio 2022
- Clang
- GCC

### Optional backend dependencies

#### OpenCL

- OpenCL headers and loader library
- Vendor runtime/driver for the target device

CMake uses:

```cmake
find_package(OpenCL)
```

#### Vulkan

- Vulkan SDK or system Vulkan development package
- `glslangValidator`, optional but recommended, for compiling `.comp` files into `.spv`

CMake uses:

```cmake
find_package(Vulkan)
```

#### CUDA

- NVIDIA CUDA Toolkit
- CUDA-capable NVIDIA GPU and driver

CMake uses:

```cmake
check_language(CUDA)
find_package(CUDAToolkit)
```

#### DirectX

- Windows
- D3D11 / D3DCompiler / DXGI SDK libraries, usually available through the Windows SDK

#### Metal

- macOS
- Apple Clang / Xcode command-line tools
- Metal and Foundation frameworks

---

## Building

### Build command

After configuring a build directory with `cmake -S . -B <build-dir>`, compile the project with:

```bash
cmake --build <build-dir> --config Release
```

Replace `<build-dir>` with the directory used during configuration, such as `build/serial`, `build/auto`, or `build/cuda`.

Examples:

```bash
cmake --build build/serial --config Release
cmake --build build/auto --config Release
cmake --build build/cuda --config Release
```

The `--config Release` argument is required for multi-configuration generators such as Visual Studio and is harmless for most single-configuration generators.

### VS Code tasks and debugging

VS Code users can use the checked-in `.vscode` configuration directly instead of typing the CMake commands manually.

- Use **Terminal > Run Task...** to run build tasks defined in `.vscode/tasks.json`.
- Use **Run and Debug** to start debugging configurations defined in `.vscode/launch.json`.
- Make sure the selected task/configuration matches the backend and build directory you want to use.

These files are intended to provide a quick editor-integrated workflow for configuring, building, running, and debugging the project from inside VS Code.

To choose a particular backend for voxelizer inside VS code, look for something like
```cpp
#ifndef VOXELIZER
#define VOXELIZER SERIAL
#endif
```
in `Voxelizer/MeshVoxelizer.h` and change `SERIAL` to the backend you wish to execute and is available on your system.
Similarly, choose a backend for curvature estimator in `CurvatureEstimator/CurvatureEstimator.h`.

### Serial-only build, most portable

This is the best first build to verify the toolchain.

```powershell
cmake -S . -B build/serial `
  -DVOXELIZER_BACKEND=SERIAL `
  -DCURVATURE_BACKEND=SERIAL `
  -DENABLE_OPENCL=OFF `
  -DENABLE_VULKAN=OFF `
  -DENABLE_CUDA=OFF `
  -DENABLE_DIRECTX=OFF `
  -DENABLE_METAL=OFF

cmake --build build/serial --config Release
```

On Linux/macOS shells:

```bash
cmake -S . -B build/serial \
  -DVOXELIZER_BACKEND=SERIAL \
  -DCURVATURE_BACKEND=SERIAL \
  -DENABLE_OPENCL=OFF \
  -DENABLE_VULKAN=OFF \
  -DENABLE_CUDA=OFF \
  -DENABLE_DIRECTX=OFF \
  -DENABLE_METAL=OFF

cmake --build build/serial --config Release
```

### Auto-select available backends

```bash
cmake -S . -B build/auto
cmake --build build/auto --config Release
```

During configure, CMake prints the detected and selected backends, for example:

```text
-- Available backends: OpenCL=ON, Vulkan=ON, CUDA=OFF, DirectX=OFF, Metal=OFF
-- Selected voxelizer backend: OPENCL
-- Selected curvature backend: OPENCL
```

### OpenCL build

```bash
cmake -S . -B build/opencl \
  -DVOXELIZER_BACKEND=OPENCL \
  -DCURVATURE_BACKEND=OPENCL

cmake --build build/opencl --config Release
```

### Vulkan build

```bash
cmake -S . -B build/vulkan \
  -DVOXELIZER_BACKEND=VULKAN \
  -DCURVATURE_BACKEND=VULKAN \
  -DBUILD_VULKAN_SHADERS=ON

cmake --build build/vulkan --config Release
```

The Vulkan code currently expects SPIR-V files at these project-relative paths:

```text
Voxelizer/Parallel/Vulkan/MeshVoxelizerKernel.spv
CurvatureEstimator/Parallel/Vulkan/CurvatureEstimatorKernel.spv
```

When `BUILD_VULKAN_SHADERS=ON` and `glslangValidator` is found, CMake compiles those files automatically.

### CUDA build

```bash
cmake -S . -B build/cuda \
  -DVOXELIZER_BACKEND=CUDA \
  -DCURVATURE_BACKEND=CUDA \
  -DCMAKE_CUDA_ARCHITECTURES=native

cmake --build build/cuda --config Release
```

You can specify explicit CUDA architectures if needed:

```bash
cmake -S . -B build/cuda \
  -DVOXELIZER_BACKEND=CUDA \
  -DCURVATURE_BACKEND=CUDA \
  -DCMAKE_CUDA_ARCHITECTURES="75;86;89"
```

### DirectX build, Windows only

```powershell
cmake -S . -B build/directx `
  -DVOXELIZER_BACKEND=DIRECTX `
  -DCURVATURE_BACKEND=DIRECTX

cmake --build build/directx --config Release
```

### Mixed backend build

Voxelization and curvature estimation can use different backends.

For example, CUDA voxelization with serial curvature:

```bash
cmake -S . -B build/cuda_serial \
  -DVOXELIZER_BACKEND=CUDA \
  -DCURVATURE_BACKEND=SERIAL

cmake --build build/cuda_serial --config Release
```

---

## Running

The executable expects four or five arguments:

```text
voxelCurvatureApp <input_obj> <output_voxel_obj> <output_curvature_obj> <output_curvature_mtl> [output_curvature_log]
```

Example:

```bash
./build/serial/bin/voxelCurvatureApp \
  assets/armadillo.obj \
  output/armadillo_voxelized.obj \
  output/armadillo_curvature.obj \
  output/armadillo_curvature.mtl \
  output/curvature.log
```

On Windows with a multi-config generator such as Visual Studio, the executable is usually under the configuration directory:

```powershell
.\build\serial\bin\Release\voxelCurvatureApp.exe `
  assets\armadillo.obj `
  output\armadillo_voxelized.obj `
  output\armadillo_curvature.obj `
  output\armadillo_curvature.mtl `
  output\curvature.log
```

At runtime, the program asks interactively for:

1. __Voxelization scale factor__
    * Decides how many surface voxels would be generated.
2. __Curve length for curvature estimation__
    * Decides the neighborhood locality.
    * Smaller neighborhood produces low-curvature values even in high-curvature regions.
    * A very large neighborhood does quite the opposite; so, some balance is needed.
    * Do some hit-and-trial to see which curve-length suits which model at what scale factor.

---

## Output files

Typical outputs are:

| File | Description |
|---|---|
| Voxel OBJ | Voxelized representation of the input mesh |
| Curvature OBJ | Surface voxels colored by curvature material assignment |
| Curvature MTL | Material/color definitions for the curvature OBJ |
| Curvature log | Text log of voxel positions and curvature values |

The current `main.cpp` computes and exports the curvature files and curvature log. The voxel OBJ export call is present but commented out in the source.

---

## Notes and limitations

- CUDA is NVIDIA-specific and requires the CUDA Toolkit.
- DirectX is Windows-only.
- Metal is Apple-only and is implemented for both voxelization and curvature estimation when the system frameworks are available.
- For `AUTO`, the project prefers the first available GPU backend for the voxelizer, while the curvature estimator still falls back to `SERIAL` unless a curvature backend is explicitly selected or a supported GPU backend is available.
- Vulkan shader compilation requires `glslangValidator` if `.spv` files are not already present.
- `AUTO` is convenient, but explicit backend selection is recommended for benchmarking and reproducibility.

---

## Developer notes

This project intentionally keeps the backend hierarchy modular:

- Each backend owns its own `CMakeLists.txt`.
- `Parallel/CMakeLists.txt` files collect optional backend targets.
- `Voxelizer/CMakeLists.txt` and `CurvatureEstimator/CMakeLists.txt` select and link exactly one backend each.
- The root `CMakeLists.txt` owns global options, dependency detection, and final executable construction.

This makes it straightforward to add new backends or replace a backend implementation without changing the rest of the build graph.

---

## Citation

If you use this project in your research or work, please cite it as:

```bibtex
@software{voxel_curvature_estimator_2026,
  author = {Chatterjee, Preetam Chayan},
  title = {Voxel Curvature Estimator},
  year = {2026},
  url = {https://github.com/preetamchayan/Voxel-Curvature-Estimator}
}
```

Or in text format:

> Chatterjee, P. C. (2026). Voxel Curvature Estimator. Retrieved from https://github.com/preetamchayan/Voxel-Curvature-Estimator

---

## License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for the full text.

You are free to use, modify, and distribute this software for any purpose, including commercial applications, provided that:

1. The original copyright notice and license text are included in any copies or substantial portions of the software.
2. The software is provided "as is" without warranty of any kind.

For more details, see the [LICENSE](LICENSE) file.
