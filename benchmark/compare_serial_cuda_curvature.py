"""Build and compare Serial and CUDA curvature results.

This is a focused version of compare_various_path_curvature.py that only builds/runs
Serial and CUDA paths. It intentionally avoids OpenCL/Vulkan dependencies.
"""

from __future__ import annotations

import argparse
import csv
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path


SERIAL_BACKEND = 0
CUDA_BACKEND = 3
DEFAULT_SCALE_FACTOR = "5"
DEFAULT_CURVE_LENGTH = "25"
CURVATURE_TIME_PATTERN = re.compile(r"CURVATURE_TIME_MS=(\d+)")
PROFILE_PATTERN = re.compile(r"(CURVATURE_PROFILE_[A-Z_]+)=([0-9]+(?:\.[0-9]+)?)")

SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPT_DIR.parent
OUTPUT_DIR = REPOSITORY_ROOT / "output"
RESULTS_DIR = SCRIPT_DIR / "results"
INPUT_MESH = REPOSITORY_ROOT / "assets" / "armadillo.obj"

IS_WINDOWS = platform.system() == "Windows"
SERIAL_EXECUTABLE = REPOSITORY_ROOT / ("serialApp.exe" if IS_WINDOWS else "serialApp")
CUDA_EXECUTABLE = REPOSITORY_ROOT / ("cudaApp.exe" if IS_WINDOWS else "cudaApp")

SERIAL_LOG = OUTPUT_DIR / "curvature_serial.log"
CUDA_LOG = OUTPUT_DIR / "curvature_cuda.log"
MISMATCH_CSV = RESULTS_DIR / "serial_cuda_curvature_mismatches.csv"
TIMING_CSV = RESULTS_DIR / "serial_cuda_curvature_times.csv"
PHASE_TIMING_CSV = RESULTS_DIR / "serial_cuda_curvature_phase_times.csv"

COMMON_SOURCES = [
    "main.cpp",
    "Helper/MeshLoader/MeshLoader.cpp",
    "Helper/HelperFunctions.cpp",
    "Voxelizer/MeshVoxelizer.cpp",
    "CurvatureEstimator/CurvatureEstimator.cpp",
]

SERIAL_SOURCES = [
    *COMMON_SOURCES,
    "Voxelizer/Serial/MeshVoxelizerSerial.cpp",
    "Voxelizer/Serial/DSSCreator/DSSCreator.cpp",
    "Voxelizer/Serial/TriangleVoxelizer/TriangleVoxelizer.cpp",
    "CurvatureEstimator/Serial/CurvatureEstimatorSerial.cpp",
]

CUDA_OBJECTS = [
    "Voxelizer/Parallel/Cuda/MeshVoxelizerCuda.obj" if IS_WINDOWS else "Voxelizer/Parallel/Cuda/MeshVoxelizerCuda.o",
    "Voxelizer/Parallel/Cuda/MeshVoxelizerKernel.obj" if IS_WINDOWS else "Voxelizer/Parallel/Cuda/MeshVoxelizerKernel.o",
    "CurvatureEstimator/Parallel/Cuda/CurvatureEstimatorCuda.obj" if IS_WINDOWS else "CurvatureEstimator/Parallel/Cuda/CurvatureEstimatorCuda.o",
    "CurvatureEstimator/Parallel/Cuda/CurvatureEstimatorKernel.obj" if IS_WINDOWS else "CurvatureEstimator/Parallel/Cuda/CurvatureEstimatorKernel.o",
]

CUDA_SOURCE_TO_OBJECT = [
    ("Voxelizer/Parallel/Cuda/MeshVoxelizerCuda.cu", CUDA_OBJECTS[0]),
    ("Voxelizer/Parallel/Cuda/MeshVoxelizerKernel.cu", CUDA_OBJECTS[1]),
    ("CurvatureEstimator/Parallel/Cuda/CurvatureEstimatorCuda.cu", CUDA_OBJECTS[2]),
    ("CurvatureEstimator/Parallel/Cuda/CurvatureEstimatorKernel.cu", CUDA_OBJECTS[3]),
]


class CommandError(RuntimeError):
    pass


def require_environment(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f"{name} must be set before running this comparison.")
    return value


def run_command(
    command: list[str],
    *,
    input_text: str | None = None,
    env: dict[str, str] | None = None,
    description: str,
    verbose: bool = False,
) -> subprocess.CompletedProcess[str]:
    if verbose:
        print(f"\n[{description}]")
        print(" ".join(command))

    result = subprocess.run(
        command,
        cwd=REPOSITORY_ROOT,
        input=input_text,
        text=True,
        capture_output=True,
        env=env,
        check=False,
    )

    if verbose and result.stdout:
        print(result.stdout, end="")
    if verbose and result.stderr:
        print(result.stderr, end="", file=sys.stderr)

    return result


def ensure_success(result: subprocess.CompletedProcess[str], description: str) -> None:
    if result.returncode != 0:
        raise CommandError(
            f"{description} failed with exit code {result.returncode}.\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )


def windows_vc_host_compiler_dir() -> str:
    vc_tools = os.environ.get("VCToolsInstallDir")
    if vc_tools:
        candidate = Path(vc_tools) / "bin" / "Hostx64" / "x64"
        if (candidate / "cl.exe").is_file():
            return str(candidate)

    fallback = Path(
        r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Tools\MSVC\14.29.30133\bin\Hostx64\x64"
    )
    if (fallback / "cl.exe").is_file():
        return str(fallback)

    raise RuntimeError(
        "Could not find x64 MSVC host compiler. Run this from a VS Developer PowerShell "
        "or set VCToolsInstallDir."
    )


def cuda_build_settings(config: str) -> tuple[str, list[str], list[str], Path, Path]:
    cuda_path = Path(require_environment("CUDA_PATH") if IS_WINDOWS else os.environ.get("CUDA_PATH", "/usr/local/cuda"))
    optimization = "-O0" if config == "debug" else "-O2"
    debug_args = ["-g", "-Xcompiler", "/FS"] if IS_WINDOWS and config == "debug" else ["-g"] if config == "debug" else []
    host_compiler_args = ["-ccbin", windows_vc_host_compiler_dir()] if IS_WINDOWS else ["-ccbin", os.environ.get("CUDAHOSTCXX", "g++")]
    cuda_lib_dir = cuda_path / "lib" / "x64" if IS_WINDOWS else cuda_path / "lib64"
    return optimization, debug_args, host_compiler_args, cuda_path, cuda_lib_dir


def build_serial(config: str, verbose: bool) -> None:
    """Build Serial voxelizer + Serial curvature estimator using the NVCC/MSVC path."""
    optimization, debug_args, host_compiler_args, _, _ = cuda_build_settings(config)

    command = [
        "nvcc",
        *host_compiler_args,
        "-std=c++17",
        optimization,
        *debug_args,
        "-D_CRT_SECURE_NO_WARNINGS",
        "-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH",
        "-DNOMINMAX",
        f"-DVOXELIZER={SERIAL_BACKEND}",
        f"-DCURVATURE_ESTIMATOR={SERIAL_BACKEND}",
        *SERIAL_SOURCES,
        "-o",
        str(SERIAL_EXECUTABLE),
    ]
    result = run_command(command, description=f"Build Serial voxelizer + Serial curvature ({config})", verbose=verbose)
    ensure_success(result, f"Serial voxelizer + Serial curvature {config} build")


def build_cuda_objects(config: str, verbose: bool) -> None:
    optimization, debug_args, host_compiler_args, cuda_path, _ = cuda_build_settings(config)

    for source, output in CUDA_SOURCE_TO_OBJECT:
        command = [
            "nvcc",
            *host_compiler_args,
            optimization,
            *debug_args,
            "-c",
            source,
            "-o",
            output,
            f"-I{cuda_path / 'include'}",
        ]
        result = run_command(command, description=f"Compile {source}", verbose=verbose)
        ensure_success(result, f"CUDA object compile for {source}")


def build_cuda(config: str, verbose: bool) -> None:
    optimization, debug_args, host_compiler_args, cuda_path, cuda_lib_dir = cuda_build_settings(config)

    command = [
        "nvcc",
        *host_compiler_args,
        "-std=c++17",
        optimization,
        *debug_args,
        "-D_CRT_SECURE_NO_WARNINGS",
        "-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH",
        "-DNOMINMAX",
        f"-DVOXELIZER={CUDA_BACKEND}",
        f"-DCURVATURE_ESTIMATOR={CUDA_BACKEND}",
        f"-I{cuda_path / 'include'}",
        *COMMON_SOURCES,
        *CUDA_OBJECTS,
        "-o",
        str(CUDA_EXECUTABLE),
        f"-L{cuda_lib_dir}",
        "-lcudart",
    ]
    result = run_command(command, description=f"Build CUDA ({config})", verbose=verbose)
    ensure_success(result, f"CUDA {config} build")


def run_estimator(executable: Path, name: str, log_path: Path, scale_factor: str, curve_length: str, verbose: bool) -> tuple[int, float, dict[str, float]]:
    output_prefix = OUTPUT_DIR / f"curvature_{name.lower()}"
    start = time.perf_counter()
    result = run_command(
        [
            str(executable),
            str(INPUT_MESH),
            str(output_prefix.with_name(f"{output_prefix.name}_voxels.obj")),
            str(output_prefix.with_suffix(".obj")),
            str(output_prefix.with_suffix(".mtl")),
            str(log_path),
        ],
        input_text=f"{scale_factor}\n{curve_length}\n",
        env=os.environ.copy(),
        description=f"Run {name}",
        verbose=verbose,
    )
    wall_time_ms = round((time.perf_counter() - start) * 1000.0, 3)
    ensure_success(result, f"{name} execution")

    if not log_path.is_file():
        raise RuntimeError(f"{name} execution completed but did not create {log_path}.")

    output = result.stdout + result.stderr
    time_match = CURVATURE_TIME_PATTERN.search(output)
    if not time_match:
        raise RuntimeError(f"{name} execution did not report CURVATURE_TIME_MS.")
    profile_times = {phase: float(value) for phase, value in PROFILE_PATTERN.findall(output)}
    return int(time_match.group(1)), wall_time_ms, profile_times


def read_curvature_log(path: Path) -> dict[tuple[int, int, int], int]:
    values: dict[tuple[int, int, int], int] = {}
    with path.open(encoding="utf-8") as log_file:
        for line_number, line in enumerate(log_file, start=1):
            fields = line.split()
            if len(fields) != 4:
                raise ValueError(f"{path}:{line_number}: expected four integers, found {len(fields)} fields.")
            x, y, z, curvature = map(int, fields)
            coordinate = (x, y, z)
            if coordinate in values:
                raise ValueError(f"{path}:{line_number}: duplicate coordinate {coordinate}.")
            values[coordinate] = curvature
    return values


def write_mismatches(
    serial_values: dict[tuple[int, int, int], int],
    cuda_values: dict[tuple[int, int, int], int],
) -> int:
    mismatch_count = 0
    with MISMATCH_CSV.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=("x", "y", "z", "serial_curvature", "cuda_curvature", "difference_type"),
        )
        writer.writeheader()
        for coordinate in sorted(set(serial_values) | set(cuda_values)):
            serial_value = serial_values.get(coordinate)
            cuda_value = cuda_values.get(coordinate)
            if serial_value == cuda_value:
                continue

            difference_type = (
                "missing_from_cuda" if cuda_value is None
                else "missing_from_serial" if serial_value is None
                else "curvature_mismatch"
            )
            writer.writerow({
                "x": coordinate[0],
                "y": coordinate[1],
                "z": coordinate[2],
                "serial_curvature": "" if serial_value is None else serial_value,
                "cuda_curvature": "" if cuda_value is None else cuda_value,
                "difference_type": difference_type,
            })
            mismatch_count += 1
    return mismatch_count


def write_timing_summary(results: dict[str, tuple[int, float, dict[str, float]]]) -> None:
    serial_time_ms = results["serial"][0]
    with TIMING_CSV.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=("estimator", "curvature_time_ms", "process_wall_time_ms", "speedup_vs_serial"),
        )
        writer.writeheader()
        for estimator, (curvature_time_ms, wall_time_ms, _) in results.items():
            speedup = serial_time_ms / curvature_time_ms if estimator != "serial" and curvature_time_ms else None
            writer.writerow({
                "estimator": estimator,
                "curvature_time_ms": curvature_time_ms,
                "process_wall_time_ms": wall_time_ms,
                "speedup_vs_serial": f"{speedup:.6f}" if speedup is not None else "",
            })


def write_phase_timing_summary(profiles: dict[str, dict[str, float]]) -> None:
    phases = sorted(set().union(*(profile.keys() for profile in profiles.values())))
    with PHASE_TIMING_CSV.open("w", newline="", encoding="utf-8") as csv_file:
        fieldnames = ("phase", *(f"{estimator}_ms" for estimator in profiles))
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for phase in phases:
            row = {"phase": phase}
            for estimator, profile in profiles.items():
                row[f"{estimator}_ms"] = profile.get(phase, "")
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare Serial and CUDA curvature output.")
    parser.add_argument("--config", choices=("debug", "release"), default="release", help="Build configuration to use.")
    parser.add_argument("--scale", default=DEFAULT_SCALE_FACTOR, help="Scale factor supplied to the executable prompt.")
    parser.add_argument("--curve-length", default=DEFAULT_CURVE_LENGTH, help="Curve length supplied to the executable prompt.")
    parser.add_argument("--skip-build", action="store_true", help="Run existing executables without rebuilding.")
    parser.add_argument("--verbose", action="store_true", help="Print compiler and executable stdout/stderr.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not INPUT_MESH.is_file():
        print(f"Input mesh not found: {INPUT_MESH}", file=sys.stderr)
        return 1

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    try:
        if not args.skip_build:
            print(f"Building Serial voxelizer + Serial curvature path ({args.config})...")
            build_serial(args.config, args.verbose)
            print(f"Compiling CUDA support objects ({args.config})...")
            build_cuda_objects(args.config, args.verbose)
            print(f"Building CUDA voxelizer + CUDA curvature path ({args.config})...")
            build_cuda(args.config, args.verbose)

        print("Running Serial curvature path...")
        serial_result = run_estimator(SERIAL_EXECUTABLE, "serial", SERIAL_LOG, args.scale, args.curve_length, args.verbose)
        print("Running CUDA curvature path...")
        cuda_result = run_estimator(CUDA_EXECUTABLE, "cuda", CUDA_LOG, args.scale, args.curve_length, args.verbose)

        results = {
            "serial": serial_result,
            "cuda": cuda_result,
        }
        serial_time_ms, _, serial_profile = serial_result
        cuda_time_ms, _, cuda_profile = cuda_result

        serial_values = read_curvature_log(SERIAL_LOG)
        cuda_values = read_curvature_log(CUDA_LOG)
        mismatch_count = write_mismatches(serial_values, cuda_values)
        write_timing_summary(results)
        write_phase_timing_summary({"serial": serial_profile, "cuda": cuda_profile})
    except (OSError, RuntimeError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1

    print(f"Serial curvature log: {SERIAL_LOG}")
    print(f"CUDA curvature log: {CUDA_LOG}")
    print(f"Serial voxels: {len(serial_values)}")
    print(f"CUDA voxels: {len(cuda_values)}")
    print(f"CUDA mismatches vs Serial: {mismatch_count}")
    print(f"Serial curvature time: {serial_time_ms} ms")
    print(f"CUDA curvature time: {cuda_time_ms} ms")
    if cuda_time_ms:
        print(f"CUDA speedup vs Serial: {serial_time_ms / cuda_time_ms:.3f}x")
    print(f"Mismatch CSV: {MISMATCH_CSV}")
    print(f"Timing CSV: {TIMING_CSV}")
    print(f"Phase timing CSV: {PHASE_TIMING_CSV}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
