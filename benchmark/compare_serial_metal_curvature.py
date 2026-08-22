"""Build and compare Serial and Metal curvature results for armadillo.obj on macOS.

This script mirrors the project's existing benchmark pattern but follows the
release-Mac build recipe from .vscode/tasks.json, with explicit backend selection
for Serial vs Metal.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
import time
from pathlib import Path

SERIAL_BACKEND = 0
METAL_BACKEND = 5
DEFAULT_SCALE_FACTOR = "5"
DEFAULT_CURVE_LENGTH = "25"
CURVATURE_TIME_PATTERN = re.compile(r"CURVATURE_TIME_MS=(\d+)")
PROFILE_PATTERN = re.compile(r"(CURVATURE_PROFILE_[A-Z_]+)=([0-9]+(?:\.[0-9]+)?)")

SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPT_DIR.parent
OUTPUT_DIR = REPOSITORY_ROOT / "output"
RESULTS_DIR = SCRIPT_DIR / "results"
INPUT_MESH = REPOSITORY_ROOT / "assets" / "armadillo.obj"

SERIAL_EXECUTABLE = REPOSITORY_ROOT / "serialApp"
METAL_EXECUTABLE = REPOSITORY_ROOT / "metalApp"

SERIAL_LOG = OUTPUT_DIR / "curvature_serial.log"
METAL_LOG = OUTPUT_DIR / "curvature_metal.log"
MISMATCH_CSV = RESULTS_DIR / "serial_metal_curvature_mismatches.csv"
TIMING_CSV = RESULTS_DIR / "serial_metal_curvature_times.csv"
PHASE_TIMING_CSV = RESULTS_DIR / "serial_metal_curvature_phase_times.csv"

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

METAL_SOURCES = [
    *COMMON_SOURCES,
    "Voxelizer/Parallel/Metal/MeshVoxelizerMetal.mm",
    "CurvatureEstimator/Parallel/Metal/CurvatureEstimatorMetal.mm",
]

VOX_METAL_CURV_SERIAL = [
    *COMMON_SOURCES,
    "Voxelizer/Parallel/Metal/MeshVoxelizerMetal.mm",
    "CurvatureEstimator/Serial/CurvatureEstimatorSerial.cpp",
]


class CommandError(RuntimeError):
    pass


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


def build_serial(verbose: bool) -> None:
    command = [
        "clang++",
        "-std=c++20",
        "-Wall",
        "-O2",
        "-DVOXELIZER=5",
        "-DCURVATURE_ESTIMATOR=0",
        *VOX_METAL_CURV_SERIAL,
        "-o",
        str(SERIAL_EXECUTABLE),
        "-framework",
        "Metal",
        "-framework",
        "Foundation",
    ]
    result = run_command(command, description="Build Serial release binary", verbose=verbose)
    ensure_success(result, "Serial release build")


def build_metal(verbose: bool) -> None:
    command = [
        "clang++",
        "-std=c++20",
        "-Wall",
        "-O2",
        "-DVOXELIZER=5",
        "-DCURVATURE_ESTIMATOR=5",
        *METAL_SOURCES,
        "-o",
        str(METAL_EXECUTABLE),
        "-framework",
        "Metal",
        "-framework",
        "Foundation",
    ]
    result = run_command(command, description="Build Metal release binary", verbose=verbose)
    ensure_success(result, "Metal release build")


def run_estimator(
    executable: Path,
    name: str,
    log_path: Path,
    scale_factor: str,
    curve_length: str,
    verbose: bool,
) -> tuple[int, float, dict[str, float]]:
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
        raise CommandError(f"{name} execution completed but did not create {log_path}.")

    output = result.stdout + result.stderr
    time_match = CURVATURE_TIME_PATTERN.search(output)
    if not time_match:
        raise CommandError(f"{name} execution did not report CURVATURE_TIME_MS.")
    profile_times = {
        phase: float(value)
        for phase, value in PROFILE_PATTERN.findall(output)
    }
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
    metal_values: dict[tuple[int, int, int], int],
    output_csv: Path,
) -> int:
    mismatch_count = 0
    with output_csv.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=("x", "y", "z", "serial_curvature", "metal_curvature", "difference_type"),
        )
        writer.writeheader()
        for coordinate in sorted(set(serial_values) | set(metal_values)):
            serial_value = serial_values.get(coordinate)
            metal_value = metal_values.get(coordinate)
            if serial_value == metal_value:
                continue

            difference_type = (
                "missing_from_metal" if metal_value is None else "missing_from_serial" if serial_value is None else "curvature_mismatch"
            )
            writer.writerow(
                {
                    "x": coordinate[0],
                    "y": coordinate[1],
                    "z": coordinate[2],
                    "serial_curvature": "" if serial_value is None else serial_value,
                    "metal_curvature": "" if metal_value is None else metal_value,
                    "difference_type": difference_type,
                }
            )
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
            writer.writerow(
                {
                    "estimator": estimator,
                    "curvature_time_ms": curvature_time_ms,
                    "process_wall_time_ms": wall_time_ms,
                    "speedup_vs_serial": f"{speedup:.6f}" if speedup is not None else "",
                }
            )


def write_phase_timing_summary(profiles: dict[str, dict[str, float]]) -> None:
    phases = sorted(set().union(*(profile.keys() for profile in profiles.values())))
    fieldnames = ("phase", *(f"{estimator}_ms" for estimator in profiles))
    with PHASE_TIMING_CSV.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for phase in phases:
            row = {"phase": phase}
            for estimator, profile in profiles.items():
                row[f"{estimator}_ms"] = profile.get(phase, "")
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare Serial and Metal curvature output for armadillo.obj.")
    parser.add_argument("--scale-factor", type=str, default=DEFAULT_SCALE_FACTOR, help="Voxel scale factor to use during voxelization.")
    parser.add_argument("--curve-length", type=str, default=DEFAULT_CURVE_LENGTH, help="Curvature curve length to use.")
    parser.add_argument("--verbose", action="store_true", help="Print each build/test command and output.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not INPUT_MESH.is_file():
        print(f"Input mesh not found: {INPUT_MESH}", file=sys.stderr)
        return 1

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    try:
        print("Building Serial release binary...")
        build_serial(args.verbose)
        serial_result = run_estimator(SERIAL_EXECUTABLE, "serial", SERIAL_LOG, args.scale_factor, args.curve_length, args.verbose)

        print("Building Metal release binary...")
        build_metal(args.verbose)
        metal_result = run_estimator(METAL_EXECUTABLE, "metal", METAL_LOG, args.scale_factor, args.curve_length, args.verbose)

        # remove the executables
        os.system("rm metalApp")
        os.system("rm serialApp")

        results = {
            "serial": serial_result,
            "metal": metal_result,
        }

        serial_values = read_curvature_log(SERIAL_LOG)
        metal_values = read_curvature_log(METAL_LOG)
        mismatch_count = write_mismatches(serial_values, metal_values, MISMATCH_CSV)
        write_timing_summary(results)
        write_phase_timing_summary({
            "serial": serial_result[2],
            "metal": metal_result[2],
        })
    except (CommandError, OSError, RuntimeError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1

    print(f"Serial curvature log: {SERIAL_LOG}")
    print(f"Metal curvature log: {METAL_LOG}")
    print(f"Serial voxels: {len(serial_values)}")
    print(f"Metal voxels: {len(metal_values)}")
    print(f"Metal mismatches vs Serial: {mismatch_count}")
    print(f"Serial curvature time: {serial_result[0]} ms")
    print(f"Metal curvature time: {metal_result[0]} ms")
    print(f"Speedup vs Serial: {serial_result[0] / metal_result[0]:.3f}x")
    print(f"Mismatch CSV: {MISMATCH_CSV}")
    print(f"Timing CSV: {TIMING_CSV}")
    print(f"Phase timing CSV: {PHASE_TIMING_CSV}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
