#!/usr/bin/env python3
"""Run Kalibr Docker vs Ceres benchmarks and summarize key metrics."""

import argparse
import csv
import datetime as _datetime
import json
import pathlib
import re
import shlex
import subprocess
import sys
import time


KALIBR_IMAGE = "wang121ye/kalibr-camera-calibration:20.04"
CERES_IMAGE = "kalibr-camimu-ceres-solver:20.04"
KALIBR_BIN = "/catkin_ws/devel/lib/kalibr/kalibr_calibrate_imu_camera"

BENCHMARK_ROOT = pathlib.Path(
    "/Users/wayne/Documents/work/code/project/ffalcon/production_calibration/data"
)
BENCHMARK_SESSIONS = [
    "2025_03_14_00_10_18",
    "2025_03_14_00_34_14",
    "2025_03_14_00_50_37",
    "2025_03_14_02_13_45",
    "2025_03_14_02_21_41",
    "2025_03_14_10_23_35",
    "2025_04_19_18_43_05",
    "2025_04_19_19_03_03",
    "2025_04_19_19_20_46",
    "2025_04_19_19_35_04",
    "2025_04_19_19_55_25",
    "2025_04_19_20_21_09",
]
TUM_ROOT = pathlib.Path("/Users/wayne/Documents/work/data/TUM")

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
NUMBER_RE = re.compile(
    r"[-+]?(?:(?:\d+\.\d*)|(?:\.\d+)|(?:\d+))(?:[eE][-+]?\d+)?"
)


def repo_dir() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def timestamp() -> str:
    return _datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def quote_command(command) -> str:
    return " ".join(shlex.quote(str(part)) for part in command)


def clean_log(text: str) -> str:
    return ANSI_RE.sub("", text.replace("\r", "\n"))


def write_text(path: pathlib.Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def require_file(path: pathlib.Path, label: str) -> pathlib.Path:
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def docker_image_exists(image: str) -> bool:
    result = subprocess.run(
        ["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode != 0:
        return False
    return image in {line.strip() for line in result.stdout.splitlines()}


def pull_image(image: str, print_only: bool = False) -> None:
    command = ["docker", "pull", image]
    print(quote_command(command), flush=True)
    if not print_only:
        subprocess.check_call(command)


def ensure_images(args) -> None:
    if args.pull_kalibr:
        pull_image(args.kalibr_image, args.print_only)
    if (
        args.ceres_mode == "docker"
        and not args.print_only
        and not docker_image_exists(args.ceres_image)
    ):
        raise RuntimeError(
            f"Ceres Docker image not found: {args.ceres_image}. "
            "Build it with: docker build -f docker/Dockerfile "
            "-t kalibr-camimu-ceres-solver:20.04 ."
        )
    if args.ceres_mode == "native" and not args.print_only:
        for path, label in [
            (args.ceres_bin, "native Ceres binary"),
            (args.compare_bin, "native compare binary"),
        ]:
            require_file(path, label)


def run_logged(command, run_dir: pathlib.Path, label: str, print_only: bool):
    run_dir.mkdir(parents=True, exist_ok=True)
    write_text(run_dir / f"{label}.command.txt", quote_command(command) + "\n")
    if print_only:
        print(quote_command(command), flush=True)
        return 0, 0.0, ""

    raw_path = run_dir / f"{label}.raw.log"
    clean_path = run_dir / f"{label}.clean.log"
    start = time.monotonic()
    proc = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    with raw_path.open("w", encoding="utf-8") as raw_log:
        for line in proc.stdout:
            raw_log.write(line)
            if "Progress" not in line:
                sys.stdout.write(line)
                sys.stdout.flush()
    return_code = proc.wait()
    elapsed_s = time.monotonic() - start
    raw_text = raw_path.read_text(encoding="utf-8", errors="replace")
    cleaned = clean_log(raw_text)
    write_text(clean_path, cleaned)
    return return_code, elapsed_s, cleaned


def container_copy_script(names):
    commands = ["mkdir -p /out/input"]
    for name in sorted(set(names)):
        commands.append(
            "cp "
            + shlex.quote("/data/" + name)
            + " "
            + shlex.quote("/out/input/" + name)
        )
    return commands


def kalibr_corner_command(args, dataset: pathlib.Path, run_dir: pathlib.Path,
                          imu_data_names):
    required = [
        "cam0_640x400_corners.pkl",
        "0_save_timestamp.txt",
        "aprilgrid.yaml",
        "cam0-camchain-640x400.yaml",
        "imu.yaml",
        *imu_data_names,
    ]
    for name in required:
        require_file(dataset / name, f"benchmark input {name}")
    kalibr_args = [
        KALIBR_BIN,
        "--corner_file",
        "/out/input/cam0_640x400_corners.pkl",
        "--image_timestamp_file",
        "/out/input/0_save_timestamp.txt",
        "--imu_data_file",
        *[f"/out/input/{name}" for name in imu_data_names],
        "--target",
        "/out/input/aprilgrid.yaml",
        "--cams",
        "/out/input/cam0-camchain-640x400.yaml",
        "--imu",
        *["/out/input/imu.yaml" for _ in imu_data_names],
        "--dont-show-report",
        "--max-iter",
        str(args.kalibr_max_iter),
        "--timeoffset-padding",
        str(args.timeoffset_padding),
        "--pose-knots-per-second",
        str(args.pose_knots_per_second),
        "--bias-knots-per-second",
        str(args.bias_knots_per_second),
        "--trim-imu-edge-count",
        str(args.trim_imu_edge_count),
    ]
    if args.kalibr_extra_arg:
        kalibr_args.extend(args.kalibr_extra_arg)
    script = (
        "set -euo pipefail; "
        + "; ".join(container_copy_script(required))
        + "; "
        + quote_command(kalibr_args)
    )
    return [
        "docker",
        "run",
        "--rm",
        "--platform",
        args.platform,
        "-v",
        f"{dataset}:/data:ro",
        "-v",
        f"{run_dir}:/out",
        "-w",
        "/out",
        args.kalibr_image,
        "/bin/bash",
        "-lc",
        script,
    ]


def ceres_corner_command(args, dataset: pathlib.Path, run_dir: pathlib.Path,
                         imu_data_names, kalibr_result: pathlib.Path = None):
    required = [
        "cam0-camchain-640x400.yaml",
        "imu.yaml",
        "aprilgrid.yaml",
        "cam0_640x400_corners.csv",
        "cam0_640x400_corner_poses.csv",
        *imu_data_names,
    ]
    for name in required:
        require_file(dataset / name, f"Ceres input {name}")
    multi_imu = len(imu_data_names) > 1
    use_kalibr_init = multi_imu and kalibr_result and kalibr_result.is_file()

    if args.ceres_mode == "docker":
        command = [
            "docker",
            "run",
            "--rm",
            "--platform",
            args.platform,
            "-v",
            f"{dataset}:/data:ro",
            "-v",
            f"{run_dir}:/out",
            "-w",
            "/out",
        ]
        kalibr_result_path = None
        if use_kalibr_init:
            command.extend([
                "-v",
                f"{kalibr_result.parent}:/kalibr_result:ro",
            ])
            kalibr_result_path = f"/kalibr_result/{kalibr_result.name}"
        command.extend([args.ceres_image, "calibrate_cam_imu"])
        data_prefix = "/data"
        result_path = "/out/result.yaml"
    else:
        command = [str(args.ceres_bin)]
        data_prefix = str(dataset)
        result_path = str(run_dir / "result.yaml")
        kalibr_result_path = str(kalibr_result) if use_kalibr_init else None

    command.extend([
        "--corner-defaults",
        "--cam",
        f"{data_prefix}/cam0-camchain-640x400.yaml",
        "--target",
        f"{data_prefix}/aprilgrid.yaml",
    ])
    for _ in imu_data_names:
        command.extend(["--imu", f"{data_prefix}/imu.yaml"])
    for name in imu_data_names:
        command.extend(["--imu-data", f"{data_prefix}/{name}"])
    command.extend(
        [
            "--corners",
            f"{data_prefix}/cam0_640x400_corners.csv",
            "--corner-poses",
            f"{data_prefix}/cam0_640x400_corner_poses.csv",
            "--pose-fit-motion-lambda",
            "0.0001",
            "--pose-fit-boundary-anchors",
        ]
    )
    if multi_imu:
        if kalibr_result_path:
            command.extend([
                "--kalibr-result",
                kalibr_result_path,
                "--init-from-kalibr",
            ])
        command.extend([
            "--staged",
            "--stage-free",
            "pbg,pbegt",
            "--stage-iterations",
            f"{args.ceres_multi_imu_stage_iterations},{args.ceres_multi_imu_stage_iterations}",
            "--stage-solver-initial-trust-region-radii",
            "1,1",
            "--stage-solver-max-trust-region-radii",
            "10000000,10000000",
        ])
    else:
        command.extend([
            "--estimate-time-shift-prior",
            "--estimate-orientation-gravity-prior",
            "--time-shift-prior-sigma",
            "0.0001",
            "--pose-motion-prior",
            "--pose-motion-translation-variance",
            "10",
            "--pose-motion-rotation-variance",
            "1",
        ])
    command.extend([
        "--max-iterations",
        str(args.ceres_max_iterations),
        "--solver-max-trust-region-radius",
        "10000000",
        "--solver-absolute-parameter-tolerance",
        "-1",
        "--output-result",
        result_path,
    ])
    command.extend(args.ceres_extra_arg)
    return command


def kalibr_bag_command(args, bag: pathlib.Path, cams: pathlib.Path,
                       imu: pathlib.Path, target: pathlib.Path,
                       run_dir: pathlib.Path, imu_model="scale-misalignment"):
    if not args.print_only:
        for path, label in [(bag, "bag"), (cams, "camchain"), (imu, "imu yaml"),
                            (target, "target")]:
            require_file(path, label)
    kalibr_args = [
        KALIBR_BIN,
        "--bag",
        f"/out/input/{bag.name}",
        "--target",
        f"/out/input/{target.name}",
        "--cams",
        f"/out/input/{cams.name}",
        "--imu",
        f"/out/input/{imu.name}",
        "--imu-models",
        imu_model,
        "--dont-show-report",
        "--max-iter",
        str(args.kalibr_tum_max_iter),
        "--timeoffset-padding",
        str(args.tum_timeoffset_padding),
        "--pose-knots-per-second",
        str(args.pose_knots_per_second),
        "--bias-knots-per-second",
        str(args.bias_knots_per_second),
    ]
    script = (
        "set -euo pipefail; "
        "mkdir -p /out/input; "
        + "ln -sf "
        + shlex.quote(f"/bag/{bag.name}")
        + " "
        + shlex.quote(f"/out/input/{bag.name}")
        + "; "
        + "cp "
        + shlex.quote(f"/cams/{cams.name}")
        + " "
        + shlex.quote(f"/out/input/{cams.name}")
        + "; "
        + "cp "
        + shlex.quote(f"/imu/{imu.name}")
        + " "
        + shlex.quote(f"/out/input/{imu.name}")
        + "; "
        + "cp "
        + shlex.quote(f"/target/{target.name}")
        + " "
        + shlex.quote(f"/out/input/{target.name}")
        + "; "
        + quote_command(kalibr_args)
    )
    return [
        "docker",
        "run",
        "--rm",
        "--platform",
        args.platform,
        "-v",
        f"{bag.parent}:/bag:ro",
        "-v",
        f"{cams.parent}:/cams:ro",
        "-v",
        f"{imu.parent}:/imu:ro",
        "-v",
        f"{target.parent}:/target:ro",
        "-v",
        f"{run_dir}:/out",
        "-w",
        "/out",
        args.kalibr_image,
        "/bin/bash",
        "-lc",
        script,
    ]


def prepare_tum_command(args, source_type: str, out_dir: pathlib.Path,
                        tum_root: pathlib.Path):
    script = [
        sys.executable,
        str(repo_dir() / "tools" / "prepare_ceres_inputs.py"),
        "--source-type",
        source_type,
        "--cams",
        str(tum_root / "dataset-calib-imu2_512_16/dso/camchain.yaml"),
        "--imu",
        str(tum_root / "dataset-calib-imu2_512_16/dso/imu_config.yaml"),
        "--target",
        str(tum_root / "april_6x6_80x80cm.yaml"),
        "--out-dir",
        str(out_dir),
    ]
    if source_type == "bag":
        script.extend(["--bag", str(tum_root / "dataset-calib-imu1_512_16.bag")])
    else:
        script.extend(
            [
                "--euroc-backend",
                "kalibr-docker",
                "--euroc-dir",
                str(tum_root / "dataset-calib-imu2_512_16"),
            ]
        )
    return script


def ceres_tum_command(args, input_dir: pathlib.Path, run_dir: pathlib.Path,
                      tum_root: pathlib.Path):
    if not args.print_only:
        for name in ["imu.csv", "cam0_corners.csv", "cam1_corners.csv",
                     "cam0_corner_poses.csv"]:
            require_file(input_dir / name, f"TUM prepared {name}")
    if args.ceres_mode == "docker":
        command = [
            "docker",
            "run",
            "--rm",
            "--platform",
            args.platform,
            "-v",
            f"{input_dir}:/input:ro",
            "-v",
            f"{tum_root}:/tum:ro",
            "-v",
            f"{run_dir}:/out",
            "-w",
            "/out",
            args.ceres_image,
            "calibrate_cam_imu",
        ]
        input_prefix = "/input"
        tum_prefix = "/tum"
        result_path = "/out/result.yaml"
    else:
        command = [str(args.ceres_bin)]
        input_prefix = str(input_dir)
        tum_prefix = str(tum_root)
        result_path = str(run_dir / "result.yaml")

    command.extend([
        "--corner-defaults",
        "--cam",
        f"{tum_prefix}/dataset-calib-imu2_512_16/dso/camchain.yaml",
        "--imu",
        f"{tum_prefix}/dataset-calib-imu2_512_16/dso/imu_config.yaml",
        "--target",
        f"{tum_prefix}/april_6x6_80x80cm.yaml",
        "--imu-data",
        f"{input_prefix}/imu.csv",
        "--corners",
        f"{input_prefix}/cam0_corners.csv",
        "--corners",
        f"{input_prefix}/cam1_corners.csv",
        "--corner-poses",
        f"{input_prefix}/cam0_corner_poses.csv",
        "--init-from-camchain",
        "--estimate-time-shift-prior",
        "--estimate-orientation-gravity-prior",
        "--pose-fit-motion-lambda",
        "0.0001",
        "--pose-fit-boundary-anchors",
        "--time-shift-prior-sigma",
        "0.0001",
        "--pose-motion-prior",
        "--pose-motion-translation-variance",
        "10",
        "--pose-motion-rotation-variance",
        "1",
        "--imu-model",
        "scale-misalignment",
        "--max-iterations",
        str(args.ceres_tum_max_iterations),
        "--solver-max-trust-region-radius",
        "10000000",
        "--solver-absolute-parameter-tolerance",
        "-1",
        "--output-result",
        result_path,
    ])
    command.extend(args.ceres_extra_arg)
    return command


def first_result_txt(run_dir: pathlib.Path):
    results = sorted(run_dir.rglob("*-results-imucam.txt"))
    return results[-1] if results else None


def extract_first_number(text: str):
    match = NUMBER_RE.search(text)
    return match.group(0) if match else ""


def parse_kalibr_result(path: pathlib.Path):
    row = {}
    if not path or not path.is_file():
        return row
    text = path.read_text(encoding="utf-8", errors="replace")
    patterns = {
        "kalibr_reproj_px": r"Reprojection error \(cam0\) \[px\]:\s+mean\s+([^,\n]+)",
        "kalibr_gyro_rad_s": r"Gyroscope error \(imu0\) \[rad/s\]:\s+mean\s+([^,\n]+)",
        "kalibr_accel_m_s2": r"Accelerometer error \(imu0\) \[m/s\^2\]:\s+mean\s+([^,\n]+)",
        "kalibr_reproj_norm": r"Reprojection error \(cam0\):\s+mean\s+([^,\n]+)",
        "kalibr_gyro_norm": r"Gyroscope error \(imu0\):\s+mean\s+([^,\n]+)",
        "kalibr_accel_norm": r"Accelerometer error \(imu0\):\s+mean\s+([^,\n]+)",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, text)
        if match:
            row[key] = match.group(1).strip()
    ts_match = re.search(
        r"timeshift cam0 to imu0:[^\n]*\n\s*(" + NUMBER_RE.pattern + r")",
        text,
    )
    if not ts_match:
        ts_match = re.search(
            r"cam0 to imu0 time:[^\n]*\n\s*(" + NUMBER_RE.pattern + r")",
            text,
        )
    if ts_match:
        row["kalibr_time_shift_s"] = ts_match.group(1)
    for kind, unit, key_prefix in [
        ("Gyroscope", r"\[rad/s\]", "kalibr_gyro_imu"),
        ("Accelerometer", r"\[m/s\^2\]", "kalibr_accel_imu"),
    ]:
        for match in re.finditer(
            rf"{kind} error \(imu(\d+)\) {unit}:\s+mean\s+([^,\n]+)",
            text,
        ):
            row[f"{key_prefix}{match.group(1)}"] = match.group(2).strip()
    return row


def parse_kalibr_log(text: str):
    row = {}
    match = re.search(
        r"Optimizing elapsed\s+(" + NUMBER_RE.pattern + r")s", text
    )
    if match:
        row["kalibr_optimize_s"] = match.group(1)
    return row


def parse_key_values_line(prefix: str, text: str):
    row = {}
    for line in text.splitlines():
        if not line.startswith(prefix):
            continue
        for key, value in re.findall(
            r"([A-Za-z_][A-Za-z0-9_]*)=([-+0-9.eE]+)", line
        ):
            row[key] = value
    return row


def parse_ceres_log(text: str):
    row = {}
    last_iteration_total_time = None
    for line in text.splitlines():
        if line.startswith("residual_stats "):
            match = re.match(r"residual_stats ([A-Za-z0-9_]+): (.*)", line)
            if match:
                name = match.group(1)
                values = parse_key_values_line("", match.group(2))
                if "mean" in values:
                    row[f"ceres_{name}_mean"] = values["mean"]
        elif line.startswith("Solver Summary"):
            row["ceres_solver_summary_seen"] = "1"
        iter_match = re.match(
            r"\s*(\d+)\s+[-+0-9.eE]+\s+[-+0-9.eE]+\s+[-+0-9.eE]+\s+"
            r"[-+0-9.eE]+\s+[-+0-9.eE]+\s+[-+0-9.eE]+\s+\d+\s+"
            r"[-+0-9.eE]+\s+([-+0-9.eE]+)\s*$",
            line,
        )
        if iter_match:
            last_iteration_total_time = iter_match.group(2)
    for key, pattern in {
        "ceres_time_shift_s": r"time_shift_s:\s*(" + NUMBER_RE.pattern + r")",
        "ceres_final_cost": r"Final\s+cost:\s*(" + NUMBER_RE.pattern + r")",
        "ceres_iterations": r"Iterations:\s*(\d+)",
    }.items():
        match = re.search(pattern, text)
        if match:
            row[key] = match.group(1)
    if last_iteration_total_time is not None:
        row["ceres_optimize_s"] = last_iteration_total_time
    return row


def parse_compare_log(text: str):
    row = {}
    for line in text.splitlines():
        if line.startswith("ceres_vs_kalibr:"):
            for key, value in re.findall(
                r"([A-Za-z_][A-Za-z0-9_]*)=([-+0-9.eE]+)", line
            ):
                row[f"compare_{key}"] = value
        elif line.startswith("residual_delta_ceres_minus_kalibr:"):
            for key, value in re.findall(
                r"([A-Za-z_][A-Za-z0-9_]*)=([-+0-9.eE]+)", line
            ):
                row[f"delta_{key}"] = value
        elif line.startswith("camera_chain_delta_ceres_minus_kalibr:"):
            values = dict(
                re.findall(r"([A-Za-z_][A-Za-z0-9_]*)=([-+0-9.eE]+)", line)
            )
            camera = values.pop("camera", "")
            for key, value in values.items():
                row[f"camera{camera}_{key}"] = value
    return row


def compare_command(args, kalibr_result: pathlib.Path,
                    ceres_result: pathlib.Path, run_dir: pathlib.Path):
    if args.ceres_mode == "native":
        return [
            str(args.compare_bin),
            "--kalibr-result",
            str(kalibr_result),
            "--ceres-result",
            str(ceres_result),
        ]
    return [
        "docker",
        "run",
        "--rm",
        "--platform",
        args.platform,
        "-v",
        f"{kalibr_result.parent}:/kalibr:ro",
        "-v",
        f"{ceres_result.parent}:/ceres:ro",
        "-v",
        f"{run_dir}:/out",
        "-w",
        "/out",
        args.ceres_image,
        "compare_kalibr_result",
        "--kalibr-result",
        f"/kalibr/{kalibr_result.name}",
        "--ceres-result",
        f"/ceres/{ceres_result.name}",
    ]


def summarize_yaml(path: pathlib.Path):
    # Keep this dependency-free; only read stable single-line fields.
    row = {}
    if not path.is_file():
        return row
    text = path.read_text(encoding="utf-8", errors="replace")
    for key, pattern in {
        "result_time_shift_s": r"^time_shift_s:\s*(" + NUMBER_RE.pattern + r")",
        "result_gravity": r"^gravity:\s*\[([^\]]+)\]",
    }.items():
        match = re.search(pattern, text, re.MULTILINE)
        if match:
            row[key] = match.group(1).strip()
    row["result_imu_chain_count"] = str(len(re.findall(r"imu_index:", text)))
    row["result_camera_chain_count"] = str(len(re.findall(r"camera_index:", text)))
    return row


def run_compare(args, row, run_dir, kalibr_result, ceres_result):
    if not kalibr_result or not ceres_result.is_file():
        return row
    compare_dir = run_dir / "compare"
    command = compare_command(args, kalibr_result, ceres_result, compare_dir)
    code, elapsed, text = run_logged(command, compare_dir, "compare", args.print_only)
    row["compare_return_code"] = str(code)
    row["compare_elapsed_s"] = f"{elapsed:.6f}"
    row["compare_log"] = str(compare_dir / "compare.clean.log")
    row.update(parse_compare_log(text))
    return row


def run_benchmark_case(args, name, dataset: pathlib.Path, out_root: pathlib.Path,
                       imu_data_names, label: str):
    case_dir = out_root / name / label
    kalibr_dir = case_dir / "kalibr"
    ceres_dir = case_dir / "ceres"
    row = {
        "case": name,
        "label": label,
        "dataset": str(dataset),
        "imu_count": str(len(imu_data_names)),
        "imu_data": ",".join(imu_data_names),
    }
    print(f"[{name}/{label}] Kalibr", flush=True)
    k_cmd = kalibr_corner_command(args, dataset, kalibr_dir, imu_data_names)
    k_code, k_elapsed, k_log = run_logged(k_cmd, kalibr_dir, "kalibr", args.print_only)
    kalibr_result = first_result_txt(kalibr_dir)
    row.update(
        {
            "kalibr_return_code": str(k_code),
            "kalibr_elapsed_s": f"{k_elapsed:.6f}",
            "kalibr_log": str(kalibr_dir / "kalibr.clean.log"),
            "kalibr_result": str(kalibr_result or ""),
        }
    )
    row.update(parse_kalibr_log(k_log))
    if kalibr_result:
        row.update(parse_kalibr_result(kalibr_result))

    print(f"[{name}/{label}] Ceres", flush=True)
    c_cmd = ceres_corner_command(args, dataset, ceres_dir, imu_data_names,
                                 kalibr_result)
    c_code, c_elapsed, c_log = run_logged(c_cmd, ceres_dir, "ceres", args.print_only)
    ceres_result = ceres_dir / "result.yaml"
    row.update(
        {
            "ceres_return_code": str(c_code),
            "ceres_elapsed_s": f"{c_elapsed:.6f}",
            "ceres_log": str(ceres_dir / "ceres.clean.log"),
            "ceres_result": str(ceres_result if ceres_result.is_file() else ""),
        }
    )
    row.update(parse_ceres_log(c_log))
    row.update(summarize_yaml(ceres_result))
    return run_compare(args, row, case_dir, kalibr_result, ceres_result)


def run_benchmark_single(args, out_root: pathlib.Path):
    rows = []
    sessions = args.session or BENCHMARK_SESSIONS
    for index, session in enumerate(sessions, start=1):
        dataset = (args.benchmark_root / session / "cam_imu").resolve()
        name = f"benchmark_{index:02d}_{session}"
        rows.append(run_benchmark_case(args, name, dataset, out_root,
                                       ["data1.csv"], "single_imu1"))
        write_summary(out_root / "benchmark_single_summary.csv", rows)
    return rows


def run_benchmark_multi(args, out_root: pathlib.Path):
    session = args.multi_session or (args.session[0] if args.session else BENCHMARK_SESSIONS[0])
    dataset = (args.benchmark_root / session / "cam_imu").resolve()
    name = f"benchmark_multi_{session}"
    rows = [
        run_benchmark_case(
            args, name, dataset, out_root,
            ["data1.csv", "data2.csv", "data3.csv", "data4.csv"],
            "joint_4imu",
        )
    ]
    for imu_index in range(1, 5):
        rows.append(
            run_benchmark_case(
                args, name, dataset, out_root,
                [f"data{imu_index}.csv"],
                f"single_imu{imu_index}",
            )
        )
        write_summary(out_root / "benchmark_multi_imu_summary.csv", rows)
    return rows


def run_tum_case(args, name, source_type: str, out_root: pathlib.Path):
    tum_root = args.tum_root.resolve()
    case_dir = out_root / name
    prepared_dir = case_dir / "prepared"
    prep_cmd = prepare_tum_command(args, source_type, prepared_dir, tum_root)
    print(f"[{name}] prepare Ceres inputs", flush=True)
    prep_code, prep_elapsed, _ = run_logged(prep_cmd, case_dir / "prepare",
                                            "prepare", args.print_only)
    row = {
        "case": name,
        "label": source_type,
        "dataset": str(tum_root),
        "prepare_return_code": str(prep_code),
        "prepare_elapsed_s": f"{prep_elapsed:.6f}",
        "prepared_dir": str(prepared_dir),
    }
    if source_type == "bag":
        bag = tum_root / "dataset-calib-imu1_512_16.bag"
    else:
        bag = prepared_dir / "euroc_input.bag"
    kalibr_dir = case_dir / "kalibr"
    k_cmd = kalibr_bag_command(
        args,
        bag,
        tum_root / "dataset-calib-imu2_512_16/dso/camchain.yaml",
        tum_root / "dataset-calib-imu2_512_16/dso/imu_config.yaml",
        tum_root / "april_6x6_80x80cm.yaml",
        kalibr_dir,
    )
    print(f"[{name}] Kalibr", flush=True)
    k_code, k_elapsed, k_log = run_logged(
        k_cmd, kalibr_dir, "kalibr", args.print_only
    )
    kalibr_result = first_result_txt(kalibr_dir)
    row.update(
        {
            "kalibr_return_code": str(k_code),
            "kalibr_elapsed_s": f"{k_elapsed:.6f}",
            "kalibr_result": str(kalibr_result or ""),
            "kalibr_log": str(kalibr_dir / "kalibr.clean.log"),
        }
    )
    row.update(parse_kalibr_log(k_log))
    if kalibr_result:
        row.update(parse_kalibr_result(kalibr_result))
    ceres_dir = case_dir / "ceres"
    c_cmd = ceres_tum_command(args, prepared_dir, ceres_dir, tum_root)
    print(f"[{name}] Ceres", flush=True)
    c_code, c_elapsed, c_log = run_logged(c_cmd, ceres_dir, "ceres", args.print_only)
    ceres_result = ceres_dir / "result.yaml"
    row.update(
        {
            "ceres_return_code": str(c_code),
            "ceres_elapsed_s": f"{c_elapsed:.6f}",
            "ceres_result": str(ceres_result if ceres_result.is_file() else ""),
            "ceres_log": str(ceres_dir / "ceres.clean.log"),
        }
    )
    row.update(parse_ceres_log(c_log))
    row.update(summarize_yaml(ceres_result))
    return run_compare(args, row, case_dir, kalibr_result, ceres_result)


def run_tum(args, out_root: pathlib.Path):
    rows = [
        run_tum_case(args, "tum_imu1_bag", "bag", out_root),
        run_tum_case(args, "tum_imu2_euroc_kalibr_export", "euroc", out_root),
    ]
    write_summary(out_root / "tum_summary.csv", rows)
    return rows


def write_summary(path: pathlib.Path, rows):
    if not rows:
        return
    keys = []
    preferred = [
        "case",
        "label",
        "imu_count",
        "imu_data",
        "kalibr_return_code",
        "ceres_return_code",
        "compare_return_code",
        "kalibr_elapsed_s",
        "ceres_elapsed_s",
        "kalibr_optimize_s",
        "ceres_optimize_s",
        "compare_elapsed_s",
        "kalibr_reproj_px",
        "ceres_reprojection_px_mean",
        "compare_rotation_deg",
        "compare_translation_m",
        "compare_time_shift_s",
        "compare_gravity_norm",
        "kalibr_gyro_rad_s",
        "ceres_gyro_rad_s_mean",
        "kalibr_accel_m_s2",
        "ceres_accel_m_s2_mean",
        "result_imu_chain_count",
        "result_camera_chain_count",
        "kalibr_result",
        "ceres_result",
        "dataset",
    ]
    all_keys = set()
    for row in rows:
        all_keys.update(row.keys())
    keys.extend([key for key in preferred if key in all_keys])
    keys.extend(sorted(all_keys.difference(keys)))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    write_text(path.with_suffix(".json"), json.dumps(rows, ensure_ascii=False, indent=2))
    print(f"summary: {path}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run benchmark/TUM Kalibr Docker and Ceres comparisons."
    )
    parser.add_argument("--suite", action="append",
                        choices=["benchmark-single", "benchmark-multi-imu", "tum", "all"],
                        default=[])
    parser.add_argument("--out-root", type=pathlib.Path,
                        default=repo_dir() / "out" / "docker_benchmarks" / timestamp())
    parser.add_argument("--benchmark-root", type=pathlib.Path,
                        default=BENCHMARK_ROOT)
    parser.add_argument("--tum-root", type=pathlib.Path, default=TUM_ROOT)
    parser.add_argument("--session", action="append", default=[],
                        help="Benchmark session name. Repeat to run a subset.")
    parser.add_argument("--multi-session",
                        help="Benchmark session used by --suite benchmark-multi-imu.")
    parser.add_argument("--kalibr-image", default=KALIBR_IMAGE)
    parser.add_argument("--ceres-image", default=CERES_IMAGE)
    parser.add_argument("--ceres-mode", choices=["native", "docker"],
                        default="native")
    parser.add_argument("--ceres-bin", type=pathlib.Path,
                        default=repo_dir() / "build" / "calibrate_cam_imu")
    parser.add_argument("--compare-bin", type=pathlib.Path,
                        default=repo_dir() / "build" / "compare_kalibr_result")
    parser.add_argument("--platform", default="linux/amd64")
    parser.add_argument("--pull-kalibr", action="store_true")
    parser.add_argument("--print-only", action="store_true")
    parser.add_argument("--kalibr-max-iter", type=int, default=30)
    parser.add_argument("--kalibr-tum-max-iter", type=int, default=30)
    parser.add_argument("--ceres-max-iterations", type=int, default=150)
    parser.add_argument("--ceres-multi-imu-stage-iterations", type=int,
                        default=30)
    parser.add_argument("--ceres-tum-max-iterations", type=int, default=100)
    parser.add_argument("--timeoffset-padding", type=float, default=0.04)
    parser.add_argument("--tum-timeoffset-padding", type=float, default=0.03)
    parser.add_argument("--pose-knots-per-second", type=int, default=100)
    parser.add_argument("--bias-knots-per-second", type=int, default=50)
    parser.add_argument("--trim-imu-edge-count", type=int, default=1000)
    parser.add_argument("--kalibr-extra-arg", action="append", default=[])
    parser.add_argument("--ceres-extra-arg", action="append", default=[])
    return parser.parse_args()


def main():
    args = parse_args()
    args.out_root = args.out_root.expanduser().resolve()
    args.benchmark_root = args.benchmark_root.expanduser().resolve()
    args.tum_root = args.tum_root.expanduser().resolve()
    args.ceres_bin = args.ceres_bin.expanduser().resolve()
    args.compare_bin = args.compare_bin.expanduser().resolve()
    suites = set(args.suite or ["all"])
    if "all" in suites:
        suites.update(["benchmark-single", "benchmark-multi-imu", "tum"])
    ensure_images(args)

    all_rows = []
    if "benchmark-single" in suites:
        rows = run_benchmark_single(args, args.out_root / "benchmark_single")
        write_summary(args.out_root / "benchmark_single" / "summary.csv", rows)
        all_rows.extend(rows)
    if "benchmark-multi-imu" in suites:
        rows = run_benchmark_multi(args, args.out_root / "benchmark_multi_imu")
        write_summary(args.out_root / "benchmark_multi_imu" / "summary.csv", rows)
        all_rows.extend(rows)
    if "tum" in suites:
        rows = run_tum(args, args.out_root / "tum")
        write_summary(args.out_root / "tum" / "summary.csv", rows)
        all_rows.extend(rows)

    write_summary(args.out_root / "summary.csv", all_rows)
    return 0 if all(
        row.get("kalibr_return_code", "0") == "0"
        and row.get("ceres_return_code", "0") == "0"
        for row in all_rows
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
