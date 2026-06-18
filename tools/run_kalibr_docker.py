#!/usr/bin/env python3
"""Run Kalibr cam-IMU calibration in Docker on extracted-corner datasets."""

import argparse
import datetime as _datetime
import pathlib
import re
import shlex
import subprocess
import sys


KALIBR_IMAGE = "kalibr-camera-calibration:20.04"
KALIBR_BIN = "/catkin_ws/devel/lib/kalibr/kalibr_calibrate_imu_camera"
DEFAULT_DATASET = pathlib.Path("/Users/wayne/Documents/work/data/cam_imu_2")

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def _repo_dir():
    return pathlib.Path(__file__).resolve().parents[1]


def _timestamp():
    return _datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def _container_path(base_dir, name):
    return base_dir + "/" + pathlib.PurePosixPath(name).name


def _require_file(dataset, filename, label):
    path = dataset / filename
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def _strip_log(text):
    # Kalibr progress lines use carriage returns and ANSI colors.
    return ANSI_RE.sub("", text.replace("\r", "\n"))


def _last_capture(pattern, text):
    matches = list(re.finditer(pattern, text))
    if not matches:
        return None
    return matches[-1].groups()


def _extract_summary(clean_text):
    summary = []
    patterns = [
        ("imu_readings", r"Read\s+(\d+)\s+imu readings"),
        ("accel_error_terms", r"Added\s+(\d+)\s+of\s+\d+\s+accelerometer error terms"),
        ("gyro_error_terms", r"Added\s+(\d+)\s+of\s+\d+\s+gyroscope error terms"),
        ("design_variables", r"(\d+)\s+design variables"),
        ("error_terms", r"(\d+)\s+error terms"),
        ("jacobian_rows", r"Jacobian[^0-9]*(\d+)\s*x\s*(\d+)"),
        ("time_shift_s", r"(?:timeshift|cam0 to imu0 time)[^\n]*\n\s*([+-]?[0-9.eE+-]+)"),
        ("reprojection_mean_px", r"Reprojection error \(cam0\) \[px\]:\s+mean\s+([0-9.eE+-]+)"),
        ("gyro_mean_rad_s", r"Gyroscope error \(imu0\) \[rad/s\]:\s+mean\s+([0-9.eE+-]+)"),
        ("accel_mean_m_s2", r"Accelerometer error \(imu0\) \[m/s\^2\]:\s+mean\s+([0-9.eE+-]+)"),
    ]
    for key, pattern in patterns:
        groups = _last_capture(pattern, clean_text)
        if not groups:
            continue
        if key == "jacobian_rows":
            summary.append(f"jacobian={groups[0]}x{groups[1]}")
            continue
        summary.append(f"{key}={groups[0]}")
    return summary


def _write_text(path, text):
    path.write_text(text, encoding="utf-8")


def build_kalibr_args(args, input_dir="/out/input"):
    kalibr_args = [
        KALIBR_BIN,
        "--corner_file",
        _container_path(input_dir, args.corner_file),
        "--image_timestamp_file",
        _container_path(input_dir, args.image_timestamp_file),
        "--imu_data_file",
        _container_path(input_dir, args.imu_data_file),
        "--target",
        _container_path(input_dir, args.target),
        "--cams",
        _container_path(input_dir, args.cams),
        "--imu",
        _container_path(input_dir, args.imu),
        "--dont-show-report",
        "--max-iter",
        str(args.max_iter),
        "--timeoffset-padding",
        str(args.timeoffset_padding),
        "--pose-knots-per-second",
        str(args.pose_knots_per_second),
        "--bias-knots-per-second",
        str(args.bias_knots_per_second),
    ]
    if not args.omit_trim_arg:
        kalibr_args.extend(["--trim-imu-edge-count", str(args.trim_imu_edge_count)])
    if args.export_poses:
        kalibr_args.append("--export-poses")
    for extra in args.extra_arg:
        kalibr_args.append(extra)
    return kalibr_args


def build_docker_command(args, run_dir):
    kalibr_args = build_kalibr_args(args)
    staged_names = [
        args.corner_file,
        args.image_timestamp_file,
        args.imu_data_file,
        args.target,
        args.cams,
        args.imu,
    ]
    copy_inputs = [
        "mkdir -p /out/input",
        *(
            "cp "
            + shlex.quote(_container_path("/data", name))
            + " "
            + shlex.quote(_container_path("/out/input", name))
            for name in staged_names
        ),
    ]
    container_script = (
        "set -euo pipefail; "
        + "; ".join(copy_inputs)
        + "; "
        + " ".join(shlex.quote(x) for x in kalibr_args)
    )
    command = [
        "docker",
        "run",
        "--rm",
        "--platform",
        args.platform,
        "-v",
        f"{args.dataset}:/data:ro",
        "-v",
        f"{run_dir}:/out",
        "-w",
        "/out",
        args.image,
        "/bin/bash",
        "-lc",
        container_script,
    ]
    return command, kalibr_args


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run Kalibr's official cam-IMU CLI in Docker for Ceres comparison."
    )
    parser.add_argument("--dataset", type=pathlib.Path, default=DEFAULT_DATASET)
    parser.add_argument("--image", default=KALIBR_IMAGE)
    parser.add_argument("--platform", default="linux/amd64")
    parser.add_argument("--corner-file", default="cam0_640x400_corners.pkl")
    parser.add_argument("--image-timestamp-file", default="0_save_timestamp.txt")
    parser.add_argument("--imu-data-file", default="data1.csv")
    parser.add_argument("--target", default="aprilgrid.yaml")
    parser.add_argument("--cams", default="cam0-camchain-640x400.yaml")
    parser.add_argument("--imu", default="imu.yaml")
    parser.add_argument("--out-root", type=pathlib.Path,
                        default=_repo_dir() / "out" / "kalibr_runs")
    parser.add_argument("--run-name")
    parser.add_argument("--max-iter", type=int, default=0)
    parser.add_argument("--timeoffset-padding", type=float, default=0.04)
    parser.add_argument("--pose-knots-per-second", type=int, default=100)
    parser.add_argument("--bias-knots-per-second", type=int, default=50)
    parser.add_argument("--trim-imu-edge-count", type=int, default=1000)
    parser.add_argument("--omit-trim-arg", action="store_true",
                        help="Omit --trim-imu-edge-count and use Kalibr's source default.")
    parser.add_argument("--export-poses", action="store_true")
    parser.add_argument("--extra-arg", action="append", default=[],
                        help="Append one raw argument to kalibr_calibrate_imu_camera.")
    parser.add_argument("--stream-progress", action="store_true",
                        help="Print Kalibr progress lines to stdout. Raw logs always keep them.")
    parser.add_argument("--print-only", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    args.dataset = args.dataset.expanduser().resolve()
    args.out_root = args.out_root.expanduser().resolve()
    if args.max_iter < 0:
        raise ValueError("--max-iter must be non-negative")
    if args.trim_imu_edge_count < 0:
        raise ValueError("--trim-imu-edge-count must be non-negative")
    for label, filename in [
        ("corner pickle", args.corner_file),
        ("image timestamp file", args.image_timestamp_file),
        ("IMU data file", args.imu_data_file),
        ("target yaml", args.target),
        ("camera chain yaml", args.cams),
        ("IMU yaml", args.imu),
    ]:
        _require_file(args.dataset, filename, label)

    run_name = args.run_name or f"{args.dataset.name}_iter{args.max_iter}_{_timestamp()}"
    run_dir = args.out_root / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    docker_command, kalibr_args = build_docker_command(args, run_dir)
    command_text = (
        "# Host command\n"
        + " ".join(shlex.quote(x) for x in docker_command)
        + "\n\n# Kalibr command inside container\n"
        + " ".join(shlex.quote(x) for x in kalibr_args)
        + "\n"
    )
    _write_text(run_dir / "command.txt", command_text)
    print(f"run_dir: {run_dir}", flush=True)
    print(command_text, end="", flush=True)
    if args.print_only:
        return 0

    raw_log_path = run_dir / "kalibr_raw.log"
    clean_log_path = run_dir / "kalibr_clean.log"
    with raw_log_path.open("w", encoding="utf-8") as raw_log:
        proc = subprocess.Popen(
            docker_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            if args.stream_progress or "Progress" not in line:
                sys.stdout.write(line)
                sys.stdout.flush()
            raw_log.write(line)
        ret = proc.wait()

    raw_text = raw_log_path.read_text(encoding="utf-8", errors="replace")
    clean_text = _strip_log(raw_text)
    _write_text(clean_log_path, clean_text)
    summary = _extract_summary(clean_text)
    summary_text = "\n".join(summary + [f"return_code={ret}"]) + "\n"
    _write_text(run_dir / "summary.txt", summary_text)
    print(summary_text, end="")
    return ret


if __name__ == "__main__":
    raise SystemExit(main())
