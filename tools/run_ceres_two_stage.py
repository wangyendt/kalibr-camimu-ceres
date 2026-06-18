#!/usr/bin/env python3
"""Run the two-stage Ceres cam-IMU workflow used for TUM parity.

Stage 1 estimates global calibration blocks and IMU intrinsics with a lower
pose/bias knot rate. Stage 2 reloads that Ceres result, fixes global blocks,
and refines only pose/bias at a higher knot rate.
"""

import argparse
import pathlib
import shlex
import subprocess
import sys
import time


def repo_dir():
    return pathlib.Path(__file__).resolve().parents[1]


def quote_command(command):
    return " ".join(shlex.quote(str(part)) for part in command)


def require_file(path, label, print_only=False):
    path = pathlib.Path(path).expanduser().resolve()
    if not print_only and not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def append_common_args(command, args):
    command.extend(
        [
            "--cam",
            str(args.cam),
            "--imu",
            str(args.imu),
            "--target",
            str(args.target),
            "--imu-data",
            str(args.imu_data),
        ]
    )
    for corner_csv in args.corners:
        command.extend(["--corners", str(corner_csv)])
    command.extend(["--corner-poses", str(args.corner_poses)])
    if args.kalibr_result:
        command.extend(["--kalibr-result", str(args.kalibr_result)])


def write_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def run_logged(name, command, out_dir, print_only=False):
    stage_dir = out_dir / name
    stage_dir.mkdir(parents=True, exist_ok=True)
    command_path = stage_dir / "command.txt"
    stdout_path = stage_dir / "stdout.log"
    write_text(command_path, quote_command(command) + "\n")
    print(f"[{name}] {quote_command(command)}", flush=True)
    if print_only:
        return 0, 0.0
    start = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=str(repo_dir()),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    elapsed_s = time.monotonic() - start
    write_text(stdout_path, completed.stdout)
    print(f"[{name}] return_code={completed.returncode} elapsed_s={elapsed_s:.3f}", flush=True)
    return completed.returncode, elapsed_s


def build_stage1_command(args, stage1_result, stage1_diag):
    command = [str(args.calibrate_bin)]
    append_common_args(command, args)
    command.extend(
        [
            "--init-from-camchain",
            "--estimate-time-shift-prior",
            "--estimate-orientation-gravity-prior",
            "--pose-fit-motion-lambda",
            str(args.pose_fit_motion_lambda),
            "--pose-fit-boundary-anchors",
            "--time-shift-prior-sigma",
            str(args.time_shift_prior_sigma),
            "--pose-motion-prior",
            "--pose-motion-translation-variance",
            str(args.pose_motion_translation_variance),
            "--pose-motion-rotation-variance",
            str(args.pose_motion_rotation_variance),
            "--imu-model",
            args.imu_model,
            "--pose-kps",
            str(args.stage1_pose_kps),
            "--bias-kps",
            str(args.stage1_bias_kps),
            "--max-iterations",
            str(args.stage1_max_iterations),
            "--solver-max-trust-region-radius",
            str(args.solver_max_trust_region_radius),
            "--solver-min-relative-decrease",
            str(args.solver_min_relative_decrease),
            "--output-result",
            str(stage1_result),
            "--export-imu-diagnostics",
            str(stage1_diag),
        ]
    )
    command.extend(args.stage1_extra_arg)
    return command


def build_stage2_command(args, stage1_result, stage2_result, stage2_diag):
    command = [str(args.calibrate_bin)]
    append_common_args(command, args)
    command.extend(
        [
            "--init-from-result",
            str(stage1_result),
            "--imu-model",
            args.imu_model,
            "--pose-kps",
            str(args.stage2_pose_kps),
            "--bias-kps",
            str(args.stage2_bias_kps),
            "--max-iterations",
            str(args.stage2_max_iterations),
            "--solver-max-trust-region-radius",
            str(args.solver_max_trust_region_radius),
            "--solver-min-relative-decrease",
            str(args.solver_min_relative_decrease),
            "--fix-camera-extrinsic",
            "--fix-time-shift",
            "--fix-gravity",
            "--fix-imu-intrinsics",
            "--output-result",
            str(stage2_result),
            "--export-imu-diagnostics",
            str(stage2_diag),
        ]
    )
    command.extend(args.stage2_extra_arg)
    return command


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run Ceres cam-IMU two-stage calibration/refinement."
    )
    root = repo_dir()
    parser.add_argument("--cam", required=True)
    parser.add_argument("--imu", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--imu-data", required=True)
    parser.add_argument("--corners", action="append", required=True)
    parser.add_argument("--corner-poses", required=True)
    parser.add_argument("--kalibr-result")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument(
        "--calibrate-bin",
        default=str(root / "build" / "calibrate_cam_imu"),
    )
    parser.add_argument("--imu-model", default="scale-misalignment")
    parser.add_argument("--stage1-pose-kps", type=float, default=20.0)
    parser.add_argument("--stage1-bias-kps", type=float, default=10.0)
    parser.add_argument("--stage1-max-iterations", type=int, default=80)
    parser.add_argument("--stage2-pose-kps", type=float, default=100.0)
    parser.add_argument("--stage2-bias-kps", type=float, default=50.0)
    parser.add_argument("--stage2-max-iterations", type=int, default=80)
    parser.add_argument("--pose-fit-motion-lambda", type=float, default=1e-4)
    parser.add_argument("--time-shift-prior-sigma", type=float, default=1e-4)
    parser.add_argument("--pose-motion-translation-variance", type=float, default=10.0)
    parser.add_argument("--pose-motion-rotation-variance", type=float, default=1.0)
    parser.add_argument("--solver-max-trust-region-radius", type=float, default=1e7)
    parser.add_argument("--solver-min-relative-decrease", type=float, default=1e-9)
    parser.add_argument("--stage1-extra-arg", action="append", default=[])
    parser.add_argument("--stage2-extra-arg", action="append", default=[])
    parser.add_argument("--print-only", action="store_true")
    args = parser.parse_args(argv)

    args.cam = require_file(args.cam, "camera config", args.print_only)
    args.imu = require_file(args.imu, "IMU config", args.print_only)
    args.target = require_file(args.target, "target config", args.print_only)
    args.imu_data = require_file(args.imu_data, "IMU CSV", args.print_only)
    args.corners = [
        require_file(corner_csv, f"corner CSV {index}", args.print_only)
        for index, corner_csv in enumerate(args.corners)
    ]
    args.corner_poses = require_file(
        args.corner_poses, "corner poses CSV", args.print_only
    )
    if args.kalibr_result:
        args.kalibr_result = require_file(
            args.kalibr_result, "Kalibr result", args.print_only
        )
    args.calibrate_bin = require_file(
        args.calibrate_bin, "calibrate_cam_imu binary", args.print_only
    )
    args.out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    if not args.print_only:
        args.out_dir.mkdir(parents=True, exist_ok=True)
    return args


def main(argv=None):
    args = parse_args(argv)
    stage1_result = args.out_dir / "stage1_global" / "result.yaml"
    stage1_diag = args.out_dir / "stage1_global" / "imu_diagnostics.csv"
    stage2_result = args.out_dir / "stage2_pose_bias" / "result.yaml"
    stage2_diag = args.out_dir / "stage2_pose_bias" / "imu_diagnostics.csv"

    stage1_command = build_stage1_command(args, stage1_result, stage1_diag)
    rc, _ = run_logged("stage1_global", stage1_command, args.out_dir, args.print_only)
    if rc != 0:
        return rc
    if not args.print_only and not stage1_result.is_file():
        print(f"stage1 result missing: {stage1_result}", file=sys.stderr)
        return 2

    stage2_command = build_stage2_command(
        args, stage1_result, stage2_result, stage2_diag
    )
    rc, _ = run_logged("stage2_pose_bias", stage2_command, args.out_dir, args.print_only)
    if rc == 0:
        print(f"final result: {stage2_result}", flush=True)
        print(f"final IMU diagnostics: {stage2_diag}", flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
