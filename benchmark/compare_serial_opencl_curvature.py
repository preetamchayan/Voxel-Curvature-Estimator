"""Build and compare Serial and OpenCL curvature results for armadillo.obj."""

from __future__ import annotations

import csv
import os
import re
import subprocess
import sys
import time
from pathlib import Path


SERIAL_ESTIMATOR = 0
OPENCL_ESTIMATOR = 1
SCALE_FACTOR = "5"
CURVE_LENGTH = "25"

SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPT_DIR.parent
OUTPUT_DIR = REPOSITORY_ROOT / "output"
RESULTS_DIR = SCRIPT_DIR / "results"
EXECUTABLE = REPOSITORY_ROOT / "windowsApp.exe"
INPUT_MESH = REPOSITORY_ROOT / "assets" / "armadillo.obj"
SERIAL_LOG = OUTPUT_DIR / "curvature_serial.log"
OPENCL_LOG = OUTPUT_DIR / "curvature_opencl.log"
MISMATCH_CSV = RESULTS_DIR / "serial_opencl_curvature_mismatches.csv"
TIMING_CSV = RESULTS_DIR / "serial_opencl_curvature_times.csv"
PHASE_TIMING_CSV = RESULTS_DIR / "serial_opencl_curvature_phase_times.csv"
CURVATURE_TIME_PATTERN = re.compile(r"CURVATURE_TIME_MS=(\d+)")
PROFILE_PATTERN = re.compile(r"(CURVATURE_PROFILE_[A-Z_]+)=([0-9]+(?:\.[0-9]+)?)")


def require_environment(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f"{name} must be set before running this comparison.")
    return value


def run_command(command: list[str], *, input_text: str | None = None, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=REPOSITORY_ROOT,
        input=input_text,
        text=True,
        capture_output=True,
        env=env,
        check=False,
    )


def build_release(estimator: int, name: str) -> None:
    vulkan_sdk = require_environment("VULKAN_SDK")
    opencl_sdk = require_environment("OPENCL_SDK")
    source_files = [
        "main.cpp",
        "Helper/MeshLoader/MeshLoader.cpp",
        "Helper/HelperFunctions.cpp",
        "Voxelizer/MeshVoxelizer.cpp",
        "Voxelizer/Serial/MeshVoxelizerSerial.cpp",
        "Voxelizer/Serial/DSSCreator/DSSCreator.cpp",
        "Voxelizer/Serial/TriangleVoxelizer/TriangleVoxelizer.cpp",
        "Voxelizer/Parallel/OpenCL/MeshVoxelizerOpenCL.cpp",
        "Voxelizer/Parallel/Vulkan/MeshVoxelizerVulkan.cpp",
        "Voxelizer/Parallel/DirectX/MeshVoxelizerDirectX.cpp",
        "CurvatureEstimator/CurvatureEstimator.cpp",
        "CurvatureEstimator/Serial/CurvatureEstimatorSerial.cpp",
        "CurvatureEstimator/Parallel/OpenCL/CurvatureEstimatorOpenCL.cpp",
    ]
    result = run_command([
        "clang++",
        "-std=c++20",
        "-Wall",
        "-O2",
        "-D_CRT_SECURE_NO_WARNINGS",
        "-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH",
        "-DNOMINMAX",
        f"-DCURVATURE_ESTIMATOR={estimator}",
        f"-I{Path(vulkan_sdk) / 'Include'}",
        f"-I{Path(opencl_sdk) / 'include'}",
        *source_files,
        "-o",
        "windowsApp.exe",
        f"-L{Path(vulkan_sdk) / 'Lib'}",
        f"-L{Path(opencl_sdk) / 'lib'}",
        "-lvulkan-1",
        "-lOpenCL",
        "-ld3d11",
        "-ld3dcompiler",
        "-ldxgi",
    ])
    if result.returncode != 0:
        raise RuntimeError(
            f"{name} Release build failed.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )


def run_estimator(name: str, log_path: Path) -> tuple[int, float, dict[str, float]]:
    output_prefix = OUTPUT_DIR / f"curvature_{name.lower()}"
    start = time.perf_counter()
    result = run_command(
        [
            str(EXECUTABLE),
            str(INPUT_MESH),
            str(output_prefix.with_name(f"{output_prefix.name}_voxels.obj")),
            str(output_prefix.with_suffix(".obj")),
            str(output_prefix.with_suffix(".mtl")),
            str(log_path),
        ],
        input_text=f"{SCALE_FACTOR}\n{CURVE_LENGTH}\n",
    )
    wall_time_ms = round((time.perf_counter() - start) * 1000.0, 3)
    if result.returncode != 0:
        raise RuntimeError(
            f"{name} execution failed.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    if not log_path.is_file():
        raise RuntimeError(f"{name} execution completed but did not create {log_path}.")

    output = result.stdout + result.stderr
    time_match = CURVATURE_TIME_PATTERN.search(output)
    if not time_match:
        raise RuntimeError(f"{name} execution did not report CURVATURE_TIME_MS.")
    profile_times = {name: float(value) for name, value in PROFILE_PATTERN.findall(output)}
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


def write_mismatches(serial_values: dict[tuple[int, int, int], int],
                     opencl_values: dict[tuple[int, int, int], int]) -> int:
    mismatch_count = 0
    with MISMATCH_CSV.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=("x", "y", "z", "serial_curvature", "opencl_curvature", "difference_type"),
        )
        writer.writeheader()
        for coordinate in sorted(set(serial_values) | set(opencl_values)):
            serial_value = serial_values.get(coordinate)
            opencl_value = opencl_values.get(coordinate)
            if serial_value == opencl_value:
                continue

            difference_type = (
                "missing_from_opencl" if opencl_value is None
                else "missing_from_serial" if serial_value is None
                else "curvature_mismatch"
            )
            writer.writerow({
                "x": coordinate[0],
                "y": coordinate[1],
                "z": coordinate[2],
                "serial_curvature": "" if serial_value is None else serial_value,
                "opencl_curvature": "" if opencl_value is None else opencl_value,
                "difference_type": difference_type,
            })
            mismatch_count += 1
    return mismatch_count


def write_timing_summary(serial_time_ms: int, serial_wall_time_ms: float,
                         opencl_time_ms: int, opencl_wall_time_ms: float) -> None:
    speedup = serial_time_ms / opencl_time_ms if opencl_time_ms else None
    with TIMING_CSV.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=("estimator", "curvature_time_ms", "process_wall_time_ms", "opencl_speedup_vs_serial"),
        )
        writer.writeheader()
        writer.writerow({
            "estimator": "serial",
            "curvature_time_ms": serial_time_ms,
            "process_wall_time_ms": serial_wall_time_ms,
            "opencl_speedup_vs_serial": "",
        })
        writer.writerow({
            "estimator": "opencl",
            "curvature_time_ms": opencl_time_ms,
            "process_wall_time_ms": opencl_wall_time_ms,
            "opencl_speedup_vs_serial": f"{speedup:.6f}" if speedup is not None else "",
        })


def write_phase_timing_summary(serial_profile: dict[str, float],
                               opencl_profile: dict[str, float]) -> None:
    phases = sorted(set(serial_profile) | set(opencl_profile))
    with PHASE_TIMING_CSV.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=("phase", "serial_ms", "opencl_ms"))
        writer.writeheader()
        for phase in phases:
            writer.writerow({
                "phase": phase,
                "serial_ms": serial_profile.get(phase, ""),
                "opencl_ms": opencl_profile.get(phase, ""),
            })


def main() -> int:
    if not INPUT_MESH.is_file():
        print(f"Input mesh not found: {INPUT_MESH}", file=sys.stderr)
        return 1

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    try:
        print("Building and running Serial curvature estimator...")
        build_release(SERIAL_ESTIMATOR, "Serial")
        serial_time_ms, serial_wall_time_ms, serial_profile = run_estimator("serial", SERIAL_LOG)

        print("Building and running OpenCL curvature estimator...")
        build_release(OPENCL_ESTIMATOR, "OpenCL")
        opencl_time_ms, opencl_wall_time_ms, opencl_profile = run_estimator("opencl", OPENCL_LOG)

        serial_values = read_curvature_log(SERIAL_LOG)
        opencl_values = read_curvature_log(OPENCL_LOG)
        mismatch_count = write_mismatches(serial_values, opencl_values)
        write_timing_summary(serial_time_ms, serial_wall_time_ms, opencl_time_ms, opencl_wall_time_ms)
        write_phase_timing_summary(serial_profile, opencl_profile)
    except (OSError, RuntimeError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1

    print(f"Serial curvature log: {SERIAL_LOG}")
    print(f"OpenCL curvature log: {OPENCL_LOG}")
    print(f"Serial voxels: {len(serial_values)}")
    print(f"OpenCL voxels: {len(opencl_values)}")
    print(f"Mismatches: {mismatch_count}")
    print(f"Serial curvature time: {serial_time_ms} ms")
    print(f"OpenCL curvature time: {opencl_time_ms} ms")
    print(f"OpenCL speedup vs Serial: {serial_time_ms / opencl_time_ms:.3f}x")
    print(f"Mismatch CSV: {MISMATCH_CSV}")
    print(f"Timing CSV: {TIMING_CSV}")
    print(f"Phase timing CSV: {PHASE_TIMING_CSV}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
