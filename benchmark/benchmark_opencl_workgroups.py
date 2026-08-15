"""Build the Windows Release executable and benchmark OpenCL curvature work-group sizes."""

from __future__ import annotations

import csv
import os
import re
import subprocess
import sys
import time
from pathlib import Path


WORK_GROUP_PERMUTATIONS = (
    (56, 64, 72),
    (56, 72, 64),
    (72, 56, 64),
    (72, 64, 56),
    (64, 56, 72),
    (64, 72, 56),
)
SCALE_FACTOR = "5"
CURVE_LENGTH = "25"
CURVATURE_TIME_PATTERN = re.compile(r"CURVATURE_TIME_MS=(\d+)")
RAW_SURFACE_VOXELS_PATTERN = re.compile(r"CURVATURE_RAW_SURFACE_VOXELS=(\d+)")
RAW_VALID_VOXELS_PATTERN = re.compile(r"CURVATURE_RAW_VALID_VOXELS=(\d+)")
RAW_INVALID_VOXELS_PATTERN = re.compile(r"CURVATURE_RAW_INVALID_VOXELS=(\d+)")
RAW_CHECKSUM_PATTERN = re.compile(r"CURVATURE_RAW_CHECKSUM=(\d+)")

SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPT_DIR.parent
RESULTS_DIR = SCRIPT_DIR / "results"
LOG_PATH = SCRIPT_DIR / "benchmark_opencl_workgroups.log"
EXECUTABLE = REPOSITORY_ROOT / "windowsApp.exe"
INPUT_MESH = REPOSITORY_ROOT / "assets" / "armadillo.obj"


def require_environment(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(f"{name} must be set before running this benchmark.")
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


def build_release() -> None:
    vulkan_sdk = require_environment("VULKAN_SDK")
    opencl_sdk = require_environment("OPENCL_SDK")

    shader_result = run_command([
        "glslangValidator",
        "-V",
        "Voxelizer/Parallel/Vulkan/MeshVoxelizerKernel.comp",
        "-o",
        "Voxelizer/Parallel/Vulkan/MeshVoxelizerKernel.spv",
    ])
    if shader_result.returncode != 0:
        raise RuntimeError(
            "Release build failed while compiling the Vulkan shader.\n"
            f"stdout:\n{shader_result.stdout}\nstderr:\n{shader_result.stderr}"
        )

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
    build_result = run_command([
        "clang++",
        "-std=c++20",
        "-Wall",
        "-O2",
        "-D_CRT_SECURE_NO_WARNINGS",
        "-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH",
        "-DNOMINMAX",
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
    if build_result.returncode != 0:
        raise RuntimeError(
            "Release build failed.\n"
            f"stdout:\n{build_result.stdout}\nstderr:\n{build_result.stderr}"
        )


def run_benchmark(round_number: int, position_in_round: int, work_group_size: int) -> dict[str, object]:
    output_prefix = (
        f"armadillo_scale5_curve25_round{round_number}_position{position_in_round}_wg{work_group_size}"
    )
    output_voxels = RESULTS_DIR / f"{output_prefix}_voxels.obj"
    output_curvature = RESULTS_DIR / f"{output_prefix}_curvature.obj"
    output_material = RESULTS_DIR / f"{output_prefix}_curvature.mtl"
    environment = os.environ.copy()
    environment["CURVATURE_WORK_GROUP_SIZE"] = str(work_group_size)

    start = time.perf_counter()
    result = run_command(
        [
            str(EXECUTABLE),
            str(INPUT_MESH),
            str(output_voxels),
            str(output_curvature),
            str(output_material),
        ],
        input_text=f"{SCALE_FACTOR}\n{CURVE_LENGTH}\n",
        env=environment,
    )
    wall_time_ms = round((time.perf_counter() - start) * 1000.0, 3)
    combined_output = result.stdout + result.stderr
    time_match = CURVATURE_TIME_PATTERN.search(combined_output)
    surface_match = RAW_SURFACE_VOXELS_PATTERN.search(combined_output)
    valid_match = RAW_VALID_VOXELS_PATTERN.search(combined_output)
    invalid_match = RAW_INVALID_VOXELS_PATTERN.search(combined_output)
    checksum_match = RAW_CHECKSUM_PATTERN.search(combined_output)
    has_indicators = all((surface_match, valid_match, invalid_match, checksum_match))

    return {
        "round": round_number,
        "position_in_round": position_in_round,
        "work_group_size": work_group_size,
        "curvature_time_ms": time_match.group(1) if time_match else "",
        "raw_surface_voxels": surface_match.group(1) if surface_match else "",
        "raw_valid_voxels": valid_match.group(1) if valid_match else "",
        "raw_invalid_voxels": invalid_match.group(1) if invalid_match else "",
        "raw_checksum": checksum_match.group(1) if checksum_match else "",
        "process_wall_time_ms": wall_time_ms,
        "return_code": result.returncode,
        "status": "ok" if result.returncode == 0 and time_match and has_indicators else "failed",
        "output": combined_output,
    }


def main() -> int:
    if not INPUT_MESH.is_file():
        print(f"Input mesh not found: {INPUT_MESH}", file=sys.stderr)
        return 1

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = RESULTS_DIR / "benchmark_opencl_workgroups.csv"
    with LOG_PATH.open("w", encoding="utf-8") as log_file:
        log_file.write("OpenCL curvature work-group benchmark\n")
        log_file.write(f"Repository root: {REPOSITORY_ROOT}\n")
        log_file.write(f"Input mesh: {INPUT_MESH}\n")
        log_file.write(f"Scale factor: {SCALE_FACTOR}; curve length: {CURVE_LENGTH}\n")
        log_file.write(f"Work-group permutations: {WORK_GROUP_PERMUTATIONS}\n\n")
        log_file.write("Building Release executable...\n")
        log_file.flush()
        try:
            print("Building Release executable...")
            build_release()
        except RuntimeError as error:
            log_file.write(f"Build failed:\n{error}\n")
            print(error, file=sys.stderr)
            return 1

        log_file.write("Release build completed successfully.\n\n")
        log_file.flush()
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=(
                    "round",
                    "position_in_round",
                    "work_group_size",
                    "curvature_time_ms",
                    "raw_surface_voxels",
                    "raw_valid_voxels",
                    "raw_invalid_voxels",
                    "raw_checksum",
                    "process_wall_time_ms",
                    "return_code",
                    "status",
                ),
            )
            writer.writeheader()

            previous_work_group_size = None
            for round_number, permutation in enumerate(WORK_GROUP_PERMUTATIONS, start=1):
                for position_in_round, work_group_size in enumerate(permutation, start=1):
                    if work_group_size == previous_work_group_size:
                        raise RuntimeError("Benchmark schedule contains consecutive identical work-group sizes.")

                    message = (
                        f"Running round {round_number}/{len(WORK_GROUP_PERMUTATIONS)}, "
                        f"position {position_in_round}/3, local size {work_group_size}..."
                    )
                    print(message)
                    log_file.write(message + "\n")
                    log_file.flush()
                    row = run_benchmark(round_number, position_in_round, work_group_size)
                    summary = (
                        f"status={row['status']}, curvature_ms={row['curvature_time_ms'] or 'unavailable'}, "
                        f"raw_valid={row['raw_valid_voxels'] or 'unavailable'}/"
                        f"{row['raw_surface_voxels'] or 'unavailable'}, "
                        f"raw_invalid={row['raw_invalid_voxels'] or 'unavailable'}, "
                        f"checksum={row['raw_checksum'] or 'unavailable'}, "
                        f"wall_ms={row['process_wall_time_ms']}, return_code={row['return_code']}"
                    )
                    log_file.write(row["output"] + "\n")
                    log_file.write(summary + "\n\n")
                    log_file.flush()
                    writer.writerow({key: row[key] for key in writer.fieldnames})
                    csv_file.flush()

                    print(f"  {summary}")
                    if row["status"] != "ok":
                        print(row["output"], file=sys.stderr)
                    previous_work_group_size = work_group_size

    print(f"Results written to: {csv_path}")
    print(f"Log written to: {LOG_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
