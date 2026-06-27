#!/usr/bin/env python3
"""Run Kalibr Docker vs Ceres benchmarks and summarize key metrics."""

import argparse
import ast
import csv
import datetime as _datetime
import json
import math
import pathlib
import re
import shlex
import subprocess
import sys
import time


KALIBR_AMD64_IMAGE = "kalibr-camera-calibration:20.04-amd64"
KALIBR_ARM64_IMAGE = "kalibr-camera-calibration:20.04-arm64"
CERES_IMAGE = "kalibr-camimu-ceres-solver:20.04"
KALIBR_BIN = "/catkin_ws/devel/lib/kalibr/kalibr_calibrate_imu_camera"
KALIBR_DOCKER_REPO = pathlib.Path(
    "/Users/wayne/Documents/work/code/project/ffalcon/kalibr-docker"
)
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
CERES_ITERATION_FIELDS = [
    "iteration",
    "cost",
    "cost_change",
    "gradient_norm",
    "step_norm",
    "tr_ratio",
    "tr_radius",
    "linear_solver_iterations",
    "iteration_time_s",
    "total_time_s",
]
POLICY_STEP_THRESHOLDS = [0.1, 0.07, 0.05, 0.03, 0.02]
POLICY_COST_PLATEAU_THRESHOLDS = [0.1, 0.05, 0.02, 0.01, 0.005, 0.001]
POLICY_COST_PLATEAU_MIN_ITERATIONS = [40, 60]
POLICY_COST_PLATEAU_CONSECUTIVE = 3


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


def _as_float_matrix(value):
    return [[float(item) for item in row] for row in value]


def _as_float_vector(value):
    return [float(item) for item in value]


def _parse_yaml_inline_value(text: str, key: str):
    pattern = re.compile(rf"^\s*{re.escape(key)}:\s*(.+)$")
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            return ast.literal_eval(match.group(1))
    raise ValueError(f"missing {key}")


def read_ceres_seed_result(path: pathlib.Path):
    text = path.read_text(encoding="utf-8")
    return {
        "path": path,
        "T_c_b": _as_float_matrix(_parse_yaml_inline_value(text, "T_c_b")),
        "time_shift_s": float(_parse_yaml_inline_value(text, "time_shift_s")),
        "gravity": _as_float_vector(_parse_yaml_inline_value(text, "gravity")),
    }


def mat4_identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat4_mul(lhs, rhs):
    return [
        [
            sum(lhs[row][k] * rhs[k][col] for k in range(4))
            for col in range(4)
        ]
        for row in range(4)
    ]


def rigid_inverse(T):
    out = mat4_identity()
    for row in range(3):
        for col in range(3):
            out[row][col] = T[col][row]
    for row in range(3):
        out[row][3] = -sum(out[row][col] * T[col][3] for col in range(3))
    return out


def ceres_rotation_matrix_to_vector(R):
    """Return a rotation vector compatible with native Ceres result YAML.

    The native Ceres code stores rotation vectors with the Kalibr convention
    R = Exp(-r), so this is the negative of the standard SO(3) logarithm.
    """
    cos_angle = (sum(R[i][i] for i in range(3)) - 1.0) * 0.5
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.acos(cos_angle)
    if angle < 1e-12:
        return [0.0, 0.0, 0.0]
    if math.pi - angle < 1e-7:
        axis = [
            math.sqrt(max(0.0, (R[0][0] + 1.0) * 0.5)),
            math.sqrt(max(0.0, (R[1][1] + 1.0) * 0.5)),
            math.sqrt(max(0.0, (R[2][2] + 1.0) * 0.5)),
        ]
        if R[0][1] < 0.0:
            axis[1] = -axis[1]
        if R[0][2] < 0.0:
            axis[2] = -axis[2]
        norm = math.sqrt(sum(item * item for item in axis))
        if norm < 1e-12:
            return [0.0, 0.0, 0.0]
        return [-item / norm * angle for item in axis]
    scale = 1.0 / (2.0 * math.sin(angle))
    axis = [
        (R[2][1] - R[1][2]) * scale,
        (R[0][2] - R[2][0]) * scale,
        (R[1][0] - R[0][1]) * scale,
    ]
    return [-item * angle for item in axis]


def imu_extrinsic_from_T_i_b(T_i_b):
    R = [[T_i_b[row][col] for col in range(3)] for row in range(3)]
    t = [T_i_b[row][3] for row in range(3)]
    r_b = [-sum(R[col][row] * t[col] for col in range(3)) for row in range(3)]
    r_i_b = ceres_rotation_matrix_to_vector(R)
    return r_b, r_i_b


def yaml_matrix(matrix):
    return "[" + ", ".join(
        "[" + ", ".join(f"{float(value):.17g}" for value in row) + "]"
        for row in matrix
    ) + "]"


def yaml_vector(values):
    return "[" + ", ".join(f"{float(value):.17g}" for value in values) + "]"


def _ceres_log_residual_mean(log_path: pathlib.Path, name: str):
    if not log_path.is_file():
        return None
    text = log_path.read_text(encoding="utf-8", errors="replace")
    match = re.search(
        rf"residual_stats {re.escape(name)}:.*?\bmean=({NUMBER_RE.pattern})",
        text,
    )
    if not match:
        return None
    return float(match.group(1))


def _single_seed_result_weight(result, metric: str):
    log_path = result["path"].parent / "ceres.clean.log"
    if metric == "accel":
        value = _ceres_log_residual_mean(log_path, "accel_m_s2")
    elif metric == "gyro":
        value = _ceres_log_residual_mean(log_path, "gyro_rad_s")
    elif metric == "reprojection":
        value = _ceres_log_residual_mean(log_path, "reprojection_px")
    elif metric == "combined":
        accel = _ceres_log_residual_mean(log_path, "accel_m_s2")
        gyro = _ceres_log_residual_mean(log_path, "gyro_rad_s")
        reproj = _ceres_log_residual_mean(log_path, "reprojection_px")
        parts = [item for item in [accel, gyro, reproj] if item is not None]
        value = sum(parts) / len(parts) if parts else None
    else:
        value = None
    if value is None or not math.isfinite(value) or value <= 0.0:
        return 1.0
    return 1.0 / (value * value)


def _weighted_translation(single_results, metric: str):
    # Diagnostic-only policy: each single result is camera-to-that-IMU, so these
    # translations are not in a common body frame. Averaging can expose basin
    # conflicts, but it is not a physically valid production seed.
    weights = [_single_seed_result_weight(result, metric)
               for result in single_results]
    total_weight = sum(weights)
    if not math.isfinite(total_weight) or total_weight <= 0.0:
        return [single_results[0]["T_c_b"][row][3] for row in range(3)], weights
    translation = []
    for row in range(3):
        translation.append(
            sum(weights[index] * result["T_c_b"][row][3]
                for index, result in enumerate(single_results)) / total_weight
        )
    return translation, weights


def write_ceres_single_seed_result(single_results, output_path: pathlib.Path,
                                   body_policy: str = "reference",
                                   weight_metric: str = "accel"):
    if len(single_results) < 2:
        raise ValueError("single-seed initialization requires at least two IMUs")
    reference = single_results[0]
    T_c_body = [[float(value) for value in row] for row in reference["T_c_b"]]
    weight_debug = []
    if body_policy == "residual-weighted-translation":
        translation, weights = _weighted_translation(single_results, weight_metric)
        for row in range(3):
            T_c_body[row][3] = translation[row]
        weight_debug = weights
    elif body_policy != "reference":
        raise ValueError(f"unsupported Ceres single-seed body policy: {body_policy}")

    T_body_c = rigid_inverse(T_c_body)
    imu_offsets = [0.0]
    imu_extrinsics = [([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])]
    for result in single_results[1:]:
        T_i_b = mat4_mul(rigid_inverse(result["T_c_b"]), T_c_body)
        imu_extrinsics.append(imu_extrinsic_from_T_i_b(T_i_b))
        imu_offsets.append(reference["time_shift_s"] - result["time_shift_s"])

    lines = [
        "format_version: 1",
        "camera_to_body:",
        f"  T_c_b: {yaml_matrix(T_c_body)}",
        f"  T_b_c: {yaml_matrix(T_body_c)}",
        f"time_shift_s: {reference['time_shift_s']:.17g}",
        f"imu_time_offsets_s: {yaml_vector(imu_offsets)}",
        f"gravity: {yaml_vector(reference['gravity'])}",
        "imu_extrinsic:",
        "  r_b: [0, 0, 0]",
        "  r_i_b: [0, 0, 0]",
        "camera_chain:",
        "  - camera_index: 0",
        f"    T_c_b: {yaml_matrix(T_c_body)}",
        f"    time_shift_s: {reference['time_shift_s']:.17g}",
        "imu_chain:",
    ]
    for imu_index, (r_b, r_i_b) in enumerate(imu_extrinsics):
        time_offset = imu_offsets[imu_index]
        lines.extend([
            f"  - imu_index: {imu_index}",
            f"    r_b: {yaml_vector(r_b)}",
            f"    r_i_b: {yaml_vector(r_i_b)}",
            f"    time_offset_s: {time_offset:.17g}",
            "    camera0_effective_time_shift_s: "
            f"{reference['time_shift_s'] - time_offset:.17g}",
        ])
    if weight_debug:
        lines.append("single_seed_body_policy:")
        lines.append(f"  policy: {body_policy}")
        lines.append(f"  weight_metric: {weight_metric}")
        lines.append(f"  weights: {yaml_vector(weight_debug)}")
    lines.extend([
        "imu_intrinsics:",
        "  accel_M: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]",
        "  gyro_M: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]",
        "  gyro_accel_sensitivity: [[0, 0, 0], [0, 0, 0], [0, 0, 0]]",
        "  gyro_sensing_rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]",
        "  gyro_sensing_rotation_vector: [0, 0, 0]",
        "  accel_axis_rx_i: [0, 0, 0]",
        "  accel_axis_ry_i: [0, 0, 0]",
        "  accel_axis_rz_i: [0, 0, 0]",
        "",
    ])
    write_text(output_path, "\n".join(lines))
    return output_path


def write_rows_csv(path: pathlib.Path, rows, preferred=None,
                   write_json: bool = True) -> None:
    if not rows:
        return
    keys = []
    preferred = preferred or []
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
    if write_json:
        write_text(
            path.with_suffix(".json"),
            json.dumps(rows, ensure_ascii=False, indent=2),
        )


def require_file(path: pathlib.Path, label: str) -> pathlib.Path:
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def short_git_revision(path: pathlib.Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "--short=12", "HEAD"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode != 0:
        return ""
    return result.stdout.strip()


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


def docker_image_id(image: str) -> str:
    result = subprocess.run(
        ["docker", "image", "inspect", image, "--format", "{{.Id}}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode != 0:
        return ""
    return result.stdout.strip()


def pull_image(image: str, print_only: bool = False) -> None:
    command = ["docker", "pull", image]
    print(quote_command(command), flush=True)
    if not print_only:
        subprocess.check_call(command)


def kalibr_variant_configs(args):
    platforms = args.kalibr_platform or ["linux/amd64", "linux/arm64"]
    variants = []
    seen = set()
    for platform in platforms:
        if platform in seen:
            continue
        seen.add(platform)
        if platform == "linux/amd64":
            variants.append({
                "label": "amd64",
                "platform": platform,
                "image": args.kalibr_amd64_image,
            })
        elif platform == "linux/arm64":
            variants.append({
                "label": "arm64",
                "platform": platform,
                "image": args.kalibr_arm64_image,
            })
        else:
            raise ValueError(f"unsupported Kalibr platform: {platform}")
    return variants


def select_kalibr_run(kalibr_runs, preferred_label: str):
    for run in kalibr_runs:
        if run["variant"]["label"] == preferred_label and run["result"]:
            return run
    for run in kalibr_runs:
        if run["result"]:
            return run
    return None


def run_metadata(args):
    metadata = {
        "ceres_mode": args.ceres_mode,
        "ceres_platform": args.ceres_platform if args.ceres_mode == "docker" else "native",
        "ceres_default_policy": (
            "corner-defaults topology; 1cam+Nimu runner uses no-Kalibr "
            "trimmed selector unless disabled"
        ),
        "ceres_extra_args": " ".join(args.ceres_extra_arg),
        "ceres_multi_imu_init": args.ceres_multi_imu_init,
        "ceres_multi_imu_candidate_preset":
            args.ceres_multi_imu_candidate_preset,
        "ceres_multi_imu_single_seed_body": args.ceres_multi_imu_single_seed_body,
        "ceres_multi_imu_single_seed_weight_metric": (
            args.ceres_multi_imu_single_seed_weight_metric
        ),
        "ceres_repo_commit": short_git_revision(repo_dir()),
        "kalibr_docker_repo": str(args.kalibr_docker_repo),
        "kalibr_docker_repo_commit": short_git_revision(args.kalibr_docker_repo),
        "kalibr_extra_args": " ".join(args.kalibr_extra_arg),
    }
    if args.reuse_kalibr_from:
        metadata["kalibr_reuse_root"] = str(args.reuse_kalibr_from)
    if args.ceres_mode == "docker":
        metadata["ceres_image"] = args.ceres_image
        metadata["ceres_image_id"] = docker_image_id(args.ceres_image)
    else:
        metadata["ceres_binary"] = str(args.ceres_bin)
    return metadata


def ensure_images(args) -> None:
    if args.pull_kalibr and not args.reuse_kalibr_from:
        for variant in args.kalibr_variants:
            pull_image(variant["image"], args.print_only)
    if not args.print_only and not args.reuse_kalibr_from:
        for variant in args.kalibr_variants:
            if not docker_image_exists(variant["image"]):
                raise RuntimeError(
                    f"Kalibr Docker image not found for {variant['label']}: "
                    f"{variant['image']}. Build it from kalibr-docker first."
                )
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
                          imu_data_names, variant):
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
    if len(imu_data_names) > 1:
        kalibr_args.append("--imu-delay-by-correlation")
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
        variant["platform"],
        "-v",
        f"{dataset}:/data:ro",
        "-v",
        f"{run_dir}:/out",
        "-w",
        "/out",
        variant["image"],
        "/bin/bash",
        "-lc",
        script,
    ]


def ceres_corner_command(args, dataset: pathlib.Path, run_dir: pathlib.Path,
                         imu_data_names, kalibr_result: pathlib.Path = None,
                         init_result: pathlib.Path = None, extra_args=None):
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
    use_kalibr_init = (
        multi_imu
        and args.ceres_multi_imu_init == "kalibr"
        and kalibr_result
        and kalibr_result.is_file()
    )
    use_result_init = (
        multi_imu
        and args.ceres_multi_imu_init == "ceres-single"
        and init_result
        and init_result.is_file()
    )

    if args.ceres_mode == "docker":
        command = [
            "docker",
            "run",
            "--rm",
            "--platform",
            args.ceres_platform,
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
        init_result_path = None
        if use_result_init:
            command.extend([
                "-v",
                f"{init_result.parent}:/init_result:ro",
            ])
            init_result_path = f"/init_result/{init_result.name}"
        command.extend([args.ceres_image, "calibrate_cam_imu"])
        data_prefix = "/data"
        result_path = "/out/result.yaml"
    else:
        command = [str(args.ceres_bin)]
        data_prefix = str(dataset)
        result_path = str(run_dir / "result.yaml")
        kalibr_result_path = str(kalibr_result) if use_kalibr_init else None
        init_result_path = str(init_result) if use_result_init else None

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
        elif init_result_path:
            command.extend([
                "--init-from-result",
                init_result_path,
                "--pose-motion-prior",
                "--pose-motion-translation-variance",
                "10",
                "--pose-motion-rotation-variance",
                "1",
            ])
        else:
            command.extend([
                "--estimate-time-shift-prior",
                "--estimate-orientation-gravity-prior",
                "--estimate-camera-translation-prior",
                "--pose-motion-prior",
                "--pose-motion-translation-variance",
                "10",
                "--pose-motion-rotation-variance",
                "1",
            ])
            if args.ceres_multi_imu_time_shift_prior_sigma > 0.0:
                command.extend([
                    "--time-shift-prior-sigma",
                    str(args.ceres_multi_imu_time_shift_prior_sigma),
                ])
        if args.ceres_multi_imu_estimate_lever_prior:
            command.append("--estimate-imu-chain-lever-prior")
        if args.ceres_multi_imu_lever_prior_max_accel_rms >= 0.0:
            command.extend([
                "--imu-chain-prior-max-lever-accel-rms",
                str(args.ceres_multi_imu_lever_prior_max_accel_rms),
            ])
        if args.ceres_multi_imu_translation_prior_camera_sigma_m > 0.0:
            command.extend([
                "--estimate-multi-imu-translation-prior",
                "--multi-imu-translation-prior-camera-sigma-m",
                str(args.ceres_multi_imu_translation_prior_camera_sigma_m),
            ])
        if args.ceres_multi_imu_translation_prior_lever_sigma_m > 0.0:
            command.extend([
                "--estimate-multi-imu-translation-prior",
                "--multi-imu-translation-prior-lever-sigma-m",
                str(args.ceres_multi_imu_translation_prior_lever_sigma_m),
            ])
        if args.ceres_multi_imu_translation_prior_accel_bias_sigma > 0.0:
            command.extend([
                "--estimate-multi-imu-translation-prior",
                "--multi-imu-translation-prior-accel-bias-sigma",
                str(args.ceres_multi_imu_translation_prior_accel_bias_sigma),
            ])
        stage_masks = [
            mask.strip()
            for mask in args.ceres_multi_imu_stage_free.split(",")
            if mask.strip()
        ]
        stage_count = max(1, len(stage_masks))
        stage_iterations = ",".join(
            [str(args.ceres_multi_imu_stage_iterations)] * stage_count
        )
        stage_initial_radii = ",".join(["1"] * stage_count)
        stage_max_radii = ",".join(["10000000"] * stage_count)
        command.extend([
            "--staged",
            "--stage-free",
            args.ceres_multi_imu_stage_free,
            "--stage-iterations",
            stage_iterations,
            "--stage-solver-initial-trust-region-radii",
            stage_initial_radii,
            "--stage-solver-max-trust-region-radii",
            stage_max_radii,
            "--imu-extrinsic-translation-bound-m",
            str(args.ceres_multi_imu_extrinsic_translation_bound_m),
            "--imu-extrinsic-rotation-bound-rad",
            str(args.ceres_multi_imu_extrinsic_rotation_bound_rad),
        ])
        if args.ceres_multi_imu_stage_step_tolerances:
            command.extend([
                "--stage-solver-absolute-step-tolerances",
                args.ceres_multi_imu_stage_step_tolerances,
            ])
        if args.ceres_multi_imu_stage_cost_tolerances:
            command.extend([
                "--stage-solver-absolute-cost-change-tolerances",
                args.ceres_multi_imu_stage_cost_tolerances,
            ])
        if args.ceres_multi_imu_stage_extrinsic_translation_bounds_m:
            command.extend([
                "--stage-imu-extrinsic-translation-bounds-m",
                args.ceres_multi_imu_stage_extrinsic_translation_bounds_m,
            ])
        if args.ceres_multi_imu_stage_extrinsic_rotation_bounds_rad:
            command.extend([
                "--stage-imu-extrinsic-rotation-bounds-rad",
                args.ceres_multi_imu_stage_extrinsic_rotation_bounds_rad,
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
    if args.ceres_max_iterations is not None:
        command.extend(["--max-iterations", str(args.ceres_max_iterations)])
    command.extend(["--output-result", result_path])
    command.extend(args.ceres_extra_arg)
    if extra_args:
        command.extend(extra_args)
    return command


def kalibr_bag_command(args, bag: pathlib.Path, cams: pathlib.Path,
                       imu: pathlib.Path, target: pathlib.Path,
                       run_dir: pathlib.Path, variant,
                       imu_model="scale-misalignment"):
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
    if args.kalibr_extra_arg:
        kalibr_args.extend(args.kalibr_extra_arg)
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
        variant["platform"],
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
        variant["image"],
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
        "--image",
        args.kalibr_prepare_image,
        "--platform",
        args.kalibr_prepare_platform,
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
            args.ceres_platform,
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
        "--output-result",
        result_path,
    ])
    if args.ceres_tum_max_iterations is not None:
        command.extend([
            "--max-iterations",
            str(args.ceres_tum_max_iterations),
        ])
    command.extend(args.ceres_extra_arg)
    return command


def first_result_txt(run_dir: pathlib.Path):
    results = sorted(run_dir.rglob("*-results-imucam.txt"))
    return results[-1] if results else None


def read_summary_csv_if_exists(path: pathlib.Path):
    if not path.is_file():
        return []
    return read_summary_csv(path)


def build_reuse_summary_index(root: pathlib.Path):
    index = {}
    if not root:
        return index
    summary_paths = []
    for pattern in ["summary.csv", "*_summary.csv"]:
        summary_paths.extend(root.rglob(pattern))
    for path in sorted(set(summary_paths)):
        for row in read_summary_csv_if_exists(path):
            case = row.get("case", "")
            label = row.get("label", "")
            variant = row.get("kalibr_variant", "")
            if not case or not label or not variant:
                continue
            key = (case, label, variant)
            index.setdefault(key, row)
    return index


def reused_kalibr_dirs(args, out_root: pathlib.Path, name: str, label: str,
                       variant_label: str, label_subdir: bool):
    if not args.reuse_kalibr_from:
        return []
    root = args.reuse_kalibr_from
    tail = (
        pathlib.Path(name) / label / f"kalibr_{variant_label}"
        if label_subdir
        else pathlib.Path(name) / f"kalibr_{variant_label}"
    )
    candidates = [
        root / out_root.name / tail,
        root / tail,
    ]
    parts = name.split("_", 3)
    if len(parts) == 4 and parts[0] == "benchmark" and parts[1] == "multi":
        session_suffix = parts[3]
        for base in (root / out_root.name, root):
            if not base.is_dir():
                continue
            for matched_case in sorted(base.glob(f"benchmark_multi_*_{session_suffix}")):
                matched_tail = (
                    matched_case / label / f"kalibr_{variant_label}"
                    if label_subdir
                    else matched_case / f"kalibr_{variant_label}"
                )
                candidates.append(matched_tail)
    deduped = []
    seen = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(candidate)
    return deduped


def load_reused_kalibr_run(args, out_root: pathlib.Path, name: str,
                           label: str, variant, label_subdir: bool):
    candidates = reused_kalibr_dirs(
        args, out_root, name, label, variant["label"], label_subdir
    )
    for kalibr_dir in candidates:
        kalibr_result = first_result_txt(kalibr_dir)
        if kalibr_result:
            clean_log = kalibr_dir / "kalibr.clean.log"
            log_text = (
                clean_log.read_text(encoding="utf-8", errors="replace")
                if clean_log.is_file()
                else ""
            )
            summary_row = args.reuse_kalibr_summary_index.get(
                (name, label, variant["label"]), {}
            )
            kalibr_row = {
                key: value
                for key, value in summary_row.items()
                if key.startswith("kalibr_")
            }
            kalibr_row.update({
                "kalibr_variant": variant["label"],
                "kalibr_platform": variant["platform"],
                "kalibr_image": variant["image"],
                "kalibr_image_id": variant.get("image_id", ""),
                "kalibr_return_code": kalibr_row.get("kalibr_return_code", "0"),
                "kalibr_log": str(clean_log),
                "kalibr_result": str(kalibr_result),
                "kalibr_reused_from": str(kalibr_dir),
            })
            kalibr_row.update(parse_kalibr_log(log_text))
            kalibr_row.update(parse_kalibr_result(kalibr_result))
            return {
                "variant": variant,
                "result": kalibr_result,
                "row": kalibr_row,
            }
    searched = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        f"reused Kalibr result not found for {name}/{label} "
        f"{variant['label']} under:\n  {searched}"
    )


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


def parse_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def policy_threshold_label(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def parse_ceres_iteration_trace(text: str):
    rows = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) != len(CERES_ITERATION_FIELDS):
            continue
        if not parts[0].isdigit():
            continue
        if not all(NUMBER_RE.fullmatch(part) for part in parts[1:]):
            continue
        rows.append(dict(zip(CERES_ITERATION_FIELDS, parts)))
    return rows


def first_step_trigger(trace, threshold: float):
    for row in trace:
        iteration = parse_int(row.get("iteration"))
        if iteration is None or iteration <= 0:
            continue
        value = parse_float(row.get("step_norm"))
        if value is not None and value >= 0.0 and value < threshold:
            return row
    return None


def first_cost_plateau_trigger(trace, threshold: float, min_iteration: int,
                               consecutive: int):
    window = []
    for row in trace:
        iteration = parse_int(row.get("iteration"))
        cost_change = parse_float(row.get("cost_change"))
        if iteration is None or cost_change is None:
            window = []
            continue
        if abs(cost_change) < threshold:
            window.append(row)
        else:
            window = []
        if len(window) > consecutive:
            window = window[-consecutive:]
        if len(window) == consecutive and iteration >= min_iteration:
            return row
    return None


def earlier_iteration(left, right):
    if left is None:
        return right
    if right is None:
        return left
    left_iter = parse_int(left.get("iteration"))
    right_iter = parse_int(right.get("iteration"))
    if left_iter is None:
        return right
    if right_iter is None:
        return left
    return left if left_iter <= right_iter else right


def build_stop_policy_replay_rows(trace):
    if not trace:
        return []
    final = trace[-1]
    final_iteration = parse_int(final.get("iteration"))
    final_total_time = parse_float(final.get("total_time_s"))
    rows = []

    def add_row(policy, metric, threshold=None, min_iteration="",
                consecutive="", trigger=None):
        trigger_iteration = (
            parse_int(trigger.get("iteration")) if trigger is not None else None
        )
        trigger_total_time = (
            parse_float(trigger.get("total_time_s")) if trigger is not None else None
        )
        row = {
            "policy": policy,
            "metric": metric,
            "threshold": "" if threshold is None else f"{threshold:g}",
            "min_iteration": str(min_iteration) if min_iteration != "" else "",
            "consecutive": str(consecutive) if consecutive != "" else "",
            "trigger_iteration": (
                "" if trigger_iteration is None else str(trigger_iteration)
            ),
            "trigger_cost": "" if trigger is None else trigger.get("cost", ""),
            "trigger_cost_change": (
                "" if trigger is None else trigger.get("cost_change", "")
            ),
            "trigger_gradient_norm": (
                "" if trigger is None else trigger.get("gradient_norm", "")
            ),
            "trigger_step_norm": (
                "" if trigger is None else trigger.get("step_norm", "")
            ),
            "trigger_total_time_s": (
                "" if trigger_total_time is None else f"{trigger_total_time:.9g}"
            ),
            "final_iteration": "" if final_iteration is None else str(final_iteration),
            "final_total_time_s": (
                "" if final_total_time is None else f"{final_total_time:.9g}"
            ),
            "saved_iterations": "",
            "saved_time_s": "",
        }
        if trigger_iteration is not None and final_iteration is not None:
            row["saved_iterations"] = str(final_iteration - trigger_iteration)
        if trigger_total_time is not None and final_total_time is not None:
            row["saved_time_s"] = f"{final_total_time - trigger_total_time:.9g}"
        rows.append(row)

    step_0p02 = None
    for threshold in POLICY_STEP_THRESHOLDS:
        trigger = first_step_trigger(trace, threshold)
        label = policy_threshold_label(threshold)
        if threshold == 0.02:
            step_0p02 = trigger
        add_row(
            f"step_lt_{label}",
            "step_norm",
            threshold=threshold,
            trigger=trigger,
        )

    for min_iteration in POLICY_COST_PLATEAU_MIN_ITERATIONS:
        for threshold in POLICY_COST_PLATEAU_THRESHOLDS:
            trigger = first_cost_plateau_trigger(
                trace, threshold, min_iteration,
                POLICY_COST_PLATEAU_CONSECUTIVE,
            )
            label = policy_threshold_label(threshold)
            add_row(
                (
                    f"abs_cost_change_lt_{label}_x"
                    f"{POLICY_COST_PLATEAU_CONSECUTIVE}_min{min_iteration}"
                ),
                "abs_cost_change",
                threshold=threshold,
                min_iteration=min_iteration,
                consecutive=POLICY_COST_PLATEAU_CONSECUTIVE,
                trigger=trigger,
            )

        for threshold in [0.01, 0.005]:
            cost_trigger = first_cost_plateau_trigger(
                trace, threshold, min_iteration,
                POLICY_COST_PLATEAU_CONSECUTIVE,
            )
            trigger = earlier_iteration(step_0p02, cost_trigger)
            label = policy_threshold_label(threshold)
            add_row(
                (
                    "step_lt_0p02_or_abs_cost_change_lt_"
                    f"{label}_x{POLICY_COST_PLATEAU_CONSECUTIVE}"
                    f"_min{min_iteration}"
                ),
                "step_norm_or_abs_cost_change",
                threshold=threshold,
                min_iteration=min_iteration,
                consecutive=POLICY_COST_PLATEAU_CONSECUTIVE,
                trigger=trigger,
            )
    return rows


def summarize_ceres_iteration_trace(trace):
    row = {}
    if not trace:
        return row
    last = trace[-1]
    for source, target in [
        ("iteration", "ceres_last_iteration"),
        ("cost", "ceres_last_cost"),
        ("cost_change", "ceres_last_cost_change"),
        ("gradient_norm", "ceres_last_gradient_norm"),
        ("step_norm", "ceres_last_step_norm"),
        ("tr_ratio", "ceres_last_tr_ratio"),
        ("tr_radius", "ceres_last_tr_radius"),
        ("iteration_time_s", "ceres_last_iteration_time_s"),
        ("total_time_s", "ceres_last_total_time_s"),
    ]:
        row[target] = last.get(source, "")
    replay_rows = build_stop_policy_replay_rows(trace)
    selected = {
        "step_lt_0p02": "ceres_policy_step_lt_0p02_iter",
        "abs_cost_change_lt_0p01_x3_min60":
            "ceres_policy_cost_lt_0p01_x3_min60_iter",
        "abs_cost_change_lt_0p005_x3_min60":
            "ceres_policy_cost_lt_0p005_x3_min60_iter",
        "step_lt_0p02_or_abs_cost_change_lt_0p01_x3_min60":
            "ceres_policy_step0p02_or_cost0p01_x3_min60_iter",
        "step_lt_0p02_or_abs_cost_change_lt_0p005_x3_min60":
            "ceres_policy_step0p02_or_cost0p005_x3_min60_iter",
    }
    for replay in replay_rows:
        key = selected.get(replay["policy"])
        if key:
            row[key] = replay.get("trigger_iteration", "")
            row[key.replace("_iter", "_saved_iterations")] = replay.get(
                "saved_iterations", ""
            )
    return row


def write_ceres_run_policy_artifacts(run_dir: pathlib.Path, text: str):
    trace = parse_ceres_iteration_trace(text)
    if not trace:
        return {}
    replay_rows = build_stop_policy_replay_rows(trace)
    trace_path = run_dir / "ceres_iteration_trace.csv"
    replay_path = run_dir / "ceres_stop_policy_replay.csv"
    write_rows_csv(
        trace_path,
        trace,
        preferred=CERES_ITERATION_FIELDS,
    )
    write_rows_csv(
        replay_path,
        replay_rows,
        preferred=[
            "policy",
            "metric",
            "threshold",
            "min_iteration",
            "consecutive",
            "trigger_iteration",
            "saved_iterations",
            "saved_time_s",
            "trigger_cost_change",
            "trigger_step_norm",
            "trigger_total_time_s",
            "final_iteration",
            "final_total_time_s",
        ],
    )
    row = {
        "ceres_iteration_trace": str(trace_path),
        "ceres_stop_policy_replay": str(replay_path),
    }
    row.update(summarize_ceres_iteration_trace(trace))
    return row


def write_ceres_policy_artifacts(out_dir: pathlib.Path, rows) -> bool:
    metadata_keys = [
        "case",
        "label",
        "dataset",
        "imu_count",
        "imu_data",
        "ceres_mode",
        "ceres_platform",
        "ceres_extra_args",
        "ceres_return_code",
        "ceres_elapsed_s",
        "ceres_optimize_s",
        "ceres_iterations",
        "ceres_termination",
        "ceres_result",
        "ceres_log",
        "compare_rotation_deg",
        "compare_translation_m",
        "compare_time_shift_s",
        "compare_gravity_norm",
    ]
    combined_trace = []
    combined_replay = []
    seen_logs = set()
    for row in rows:
        log_value = row.get("ceres_log", "")
        if not log_value:
            continue
        log_path = pathlib.Path(log_value)
        if log_path in seen_logs or not log_path.is_file():
            continue
        seen_logs.add(log_path)
        text = log_path.read_text(encoding="utf-8", errors="replace")
        trace = parse_ceres_iteration_trace(text)
        if not trace:
            continue
        log_summary = parse_ceres_log(text)
        run_metadata = {
            key: row.get(key) or log_summary.get(key, "")
            for key in metadata_keys
            if key in row or key in log_summary
        }
        write_ceres_run_policy_artifacts(log_path.parent, text)
        for item in trace:
            combined = {}
            combined.update(run_metadata)
            combined.update(item)
            combined_trace.append(combined)
        for item in build_stop_policy_replay_rows(trace):
            combined = {}
            combined.update(run_metadata)
            combined.update(item)
            combined_replay.append(combined)

    if not combined_trace and not combined_replay:
        return False

    trace_path = out_dir / "ceres_iteration_trace.csv"
    replay_path = out_dir / "ceres_stop_policy_replay.csv"
    write_rows_csv(
        trace_path,
        combined_trace,
        preferred=[
            "case",
            "label",
            "imu_count",
            "imu_data",
            "ceres_termination",
            "ceres_iterations",
            *CERES_ITERATION_FIELDS,
            "ceres_elapsed_s",
            "ceres_optimize_s",
            "ceres_result",
            "ceres_log",
        ],
    )
    write_rows_csv(
        replay_path,
        combined_replay,
        preferred=[
            "case",
            "label",
            "imu_count",
            "imu_data",
            "ceres_termination",
            "ceres_iterations",
            "policy",
            "metric",
            "threshold",
            "min_iteration",
            "consecutive",
            "trigger_iteration",
            "saved_iterations",
            "saved_time_s",
            "trigger_cost_change",
            "trigger_step_norm",
            "trigger_total_time_s",
            "final_iteration",
            "final_total_time_s",
            "compare_rotation_deg",
            "compare_translation_m",
            "compare_time_shift_s",
            "compare_gravity_norm",
            "ceres_result",
            "ceres_log",
        ],
    )
    write_text(
        out_dir / "ceres_policy_artifacts.json",
        json.dumps(
            {
                "version": 1,
                "iteration_trace_csv": str(trace_path),
                "stop_policy_replay_csv": str(replay_path),
                "step_thresholds": POLICY_STEP_THRESHOLDS,
                "cost_plateau_thresholds": POLICY_COST_PLATEAU_THRESHOLDS,
                "cost_plateau_min_iterations":
                    POLICY_COST_PLATEAU_MIN_ITERATIONS,
                "cost_plateau_consecutive":
                    POLICY_COST_PLATEAU_CONSECUTIVE,
                "note": (
                    "These artifacts replay candidate stop triggers from "
                    "Ceres logs; they do not change solver execution."
                ),
            },
            ensure_ascii=False,
            indent=2,
        ),
    )
    print(f"policy artifacts: {trace_path}", flush=True)
    print(f"policy artifacts: {replay_path}", flush=True)
    return True


def parse_ceres_log(text: str):
    row = {}
    last_iteration_total_time = None
    trace = parse_ceres_iteration_trace(text)
    row.update(summarize_ceres_iteration_trace(trace))
    for line in text.splitlines():
        if line.startswith("solver options:"):
            values = parse_key_values_line("", line)
            for key in [
                "function_tolerance",
                "gradient_tolerance",
                "parameter_tolerance",
                "initial_trust_region_radius",
                "max_trust_region_radius",
                "min_trust_region_radius",
                "min_relative_decrease",
                "absolute_cost_change_tolerance",
                "absolute_step_tolerance",
                "absolute_parameter_tolerance",
                "use_nonmonotonic_steps",
                "max_consecutive_nonmonotonic_steps",
            ]:
                if key in values:
                    row[f"ceres_solver_{key}"] = values[key]
        elif line.startswith("corner defaults active:"):
            values = parse_key_values_line("", line)
            for key in [
                "timeoffset_padding_s",
                "camera_time_offset_buffer_s",
            ]:
                if key in values:
                    row[f"ceres_{key}"] = values[key]
        elif line.startswith("absolute_stop"):
            values = parse_key_values_line("", line)
            for key, value in values.items():
                row[f"ceres_absolute_stop_{key}"] = value
        elif line.startswith("estimated time shift prior:"):
            values = parse_key_values_line("", line)
            for key, value in values.items():
                row[f"ceres_time_shift_prior_{key}"] = value
        elif line.startswith("estimated IMU chain prior:"):
            values = parse_key_values_line("", line)
            imu = values.get("imu", "")
            row["ceres_imu_chain_prior_count"] = str(
                int(row.get("ceres_imu_chain_prior_count", "0") or "0") + 1
            )
            row.setdefault("ceres_imu_chain_prior_boundary_peak_rejected", "0")
            if imu:
                for key, value in values.items():
                    if key == "imu":
                        continue
                    row[f"ceres_imu_chain_imu{imu}_{key}"] = value
            if bool_row(values, "time_offset_boundary_peak_rejected"):
                row["ceres_imu_chain_prior_boundary_peak_rejected"] = "1"
                row["ceres_imu_chain_prior_boundary_reject_count"] = str(
                    int(row.get(
                        "ceres_imu_chain_prior_boundary_reject_count", "0"
                    ) or "0") + 1
                )
            time_offset = parse_float(values.get("time_offset_s"))
            if time_offset is not None:
                previous = float_row(
                    row, "ceres_imu_chain_prior_max_abs_time_offset_s", 0.0
                )
                row["ceres_imu_chain_prior_max_abs_time_offset_s"] = (
                    f"{max(previous, abs(time_offset)):.12g}"
                )
            peak = parse_float(values.get("peak_correlation"))
            if peak is not None:
                previous = float_row(
                    row, "ceres_imu_chain_prior_min_peak_correlation", math.inf
                )
                row["ceres_imu_chain_prior_min_peak_correlation"] = (
                    f"{min(previous, peak):.12g}"
                )
        elif line.startswith("residual_stats "):
            match = re.match(r"residual_stats ([A-Za-z0-9_]+): (.*)", line)
            if match:
                name = match.group(1)
                values = parse_key_values_line("", match.group(2))
                for key in ["mean", "median", "std", "rms", "max"]:
                    if key in values:
                        row[f"ceres_{name}_{key}"] = values[key]
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
        "ceres_initial_cost": r"Initial\s+cost:\s*(" + NUMBER_RE.pattern + r")",
        "ceres_final_cost": r"Final\s+cost:\s*(" + NUMBER_RE.pattern + r")",
        "ceres_iterations": r"Iterations:\s*(\d+)",
        "ceres_termination": r"Termination:\s*([A-Z_]+)",
    }.items():
        match = re.search(pattern, text)
        if match:
            row[key] = match.group(1)
    if last_iteration_total_time is not None:
        row["ceres_optimize_s"] = last_iteration_total_time
    return row


def float_row(row, key, default=math.inf):
    try:
        value = row.get(key, "")
        return float(value)
    except (TypeError, ValueError):
        return default


def bool_row(row, key, default=False):
    value = str(row.get(key, "")).strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    return default


def adaptive_residual_score(row):
    return (
        float_row(row, "ceres_reprojection_px_mean", 0.0) +
        float_row(row, "ceres_gyro_rad_s_mean", 0.0) +
        float_row(row, "ceres_accel_m_s2_mean", 0.0)
    )


def choose_adaptive_ceres_candidate(args, short_candidate, long_candidate):
    short_row = short_candidate["row"]
    long_row = long_candidate["row"]
    if long_row.get("ceres_return_code") != "0":
        return short_candidate, "short_long_failed"

    short_accel_mean = float_row(short_row, "ceres_accel_m_s2_mean")
    long_accel_mean = float_row(long_row, "ceres_accel_m_s2_mean")
    short_accel_max = float_row(short_row, "ceres_accel_m_s2_max")
    long_accel_max = float_row(long_row, "ceres_accel_m_s2_max")

    health_values = [
        short_accel_mean, long_accel_mean, short_accel_max, long_accel_max
    ]
    if not all(math.isfinite(value) for value in health_values):
        return short_candidate, "short_missing_residual_health"
    if long_accel_mean > short_accel_mean * args.ceres_multi_imu_adaptive_accel_mean_ratio:
        return short_candidate, "short_accel_mean_health"
    if long_accel_max > short_accel_max * args.ceres_multi_imu_adaptive_accel_max_ratio:
        return short_candidate, "short_accel_max_health"

    short_score = adaptive_residual_score(short_row)
    long_score = adaptive_residual_score(long_row)
    score_epsilon = 1e-12 * max(1.0, abs(short_score))
    if long_score < short_score - score_epsilon:
        return long_candidate, "long_residual_score"
    return short_candidate, "short_residual_score"


def clone_args(args, **overrides):
    values = vars(args).copy()
    values.update(overrides)
    return argparse.Namespace(**values)


def safe_label(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", text).strip("_")


def is_no_kalibr_accuracy_preset(value: str) -> bool:
    return value in {
        "no-kalibr-accuracy",
        "no-kalibr-accuracy-trimmed",
    }


def no_kalibr_accuracy_candidate_configs(args):
    """Candidate family for no-Kalibr 1cam+Nimu accuracy experiments.

    Kalibr output is not used for initialization or candidate selection. Compare
    results may still be produced afterwards by the benchmark harness.
    """
    long_extra_args = [
        "--solver-absolute-step-tolerance=-1",
        "--stage-solver-absolute-step-tolerances=-1,-1",
        "--stage-solver-absolute-cost-change-tolerances=-1,5",
    ]
    long_extra_args.extend(args.ceres_multi_imu_adaptive_long_extra_arg)
    chain_args = clone_args(
        args,
        ceres_multi_imu_init="chain-prior",
        ceres_multi_imu_translation_prior_lever_sigma_m=0.01,
        ceres_multi_imu_translation_prior_accel_bias_sigma=0.05,
        ceres_multi_imu_stage_free="pbg,pbegti,pbegti",
        ceres_multi_imu_stage_extrinsic_translation_bounds_m=
            "0.05,0.05,0.003",
        ceres_multi_imu_stage_extrinsic_rotation_bounds_rad=
            "0.05,0.05,0.005",
    )
    chain_long_extra_args = [
        "--solver-absolute-step-tolerance=-1",
        "--stage-solver-absolute-step-tolerances=-1,-1,-1",
        "--stage-solver-absolute-cost-change-tolerances=-1,5,5",
    ]
    chain_accel_refine_extra_args = [
        "--imu-chain-prior-refine-with-accel",
        "--imu-chain-prior-refine-gyro-weight=10",
        "--imu-chain-prior-refine-accel-weight=1",
        "--imu-chain-prior-refine-rotation-bound-rad=0.1",
        "--imu-chain-prior-refine-lever-sigma-m=0.01",
        "--imu-chain-prior-refine-accel-bias-sigma=0.05",
    ]
    configs = [
        {
            "label": "single_short",
            "needs_single_seed": True,
            "uses_single_init": True,
            "uses_single_time_seed": False,
            "args": clone_args(args, ceres_multi_imu_init="ceres-single"),
            "extra_args": [],
        },
        {
            "label": "single_long",
            "needs_single_seed": True,
            "uses_single_init": True,
            "uses_single_time_seed": False,
            "args": clone_args(args, ceres_multi_imu_init="ceres-single"),
            "extra_args": long_extra_args,
        },
        {
            "label": "chain_multi_t_wide_tight",
            "needs_single_seed": False,
            "uses_single_init": False,
            "uses_single_time_seed": False,
            "args": chain_args,
            "extra_args": [],
        },
        {
            "label": "chain_multi_ograv_t_wide_tight",
            "needs_single_seed": False,
            "uses_single_init": False,
            "uses_single_time_seed": False,
            "args": chain_args,
            "extra_args": ["--estimate-multi-imu-orientation-gravity-prior"],
        },
        {
            "label": "chain_accel_refine_t_wide_tight",
            "needs_single_seed": False,
            "uses_single_init": False,
            "uses_single_time_seed": False,
            "args": chain_args,
            "extra_args": chain_accel_refine_extra_args,
        },
        {
            "label": "chain_multi_single_time_t_wide_tight",
            "needs_single_seed": True,
            "uses_single_init": False,
            "uses_single_time_seed": True,
            "args": chain_args,
            "extra_args": [],
        },
        {
            "label": "chain_multi_ograv_single_time_t_wide_tight",
            "needs_single_seed": True,
            "uses_single_init": False,
            "uses_single_time_seed": True,
            "args": chain_args,
            "extra_args": ["--estimate-multi-imu-orientation-gravity-prior"],
        },
        {
            "label": "chain_long_t_wide_tight",
            "needs_single_seed": False,
            "uses_single_init": False,
            "uses_single_time_seed": False,
            "args": chain_args,
            "extra_args": chain_long_extra_args,
        },
        {
            "label": "chain_long_single_time_t_wide_tight",
            "needs_single_seed": True,
            "uses_single_init": False,
            "uses_single_time_seed": True,
            "args": chain_args,
            "extra_args": chain_long_extra_args,
        },
    ]
    if args.ceres_multi_imu_candidate_preset == "no-kalibr-accuracy-trimmed":
        keep = {
            "single_short",
            "single_long",
            "chain_multi_t_wide_tight",
            "chain_multi_ograv_t_wide_tight",
            "chain_accel_refine_t_wide_tight",
            "chain_long_single_time_t_wide_tight",
        }
        configs = [config for config in configs if config["label"] in keep]
    return configs


def finite_candidate(candidate):
    row = candidate["row"]
    if row.get("ceres_return_code") != "0":
        return False
    if not candidate["result"].is_file():
        return False
    return math.isfinite(adaptive_residual_score(row))


def best_residual_candidate(candidates):
    viable = [candidate for candidate in candidates if finite_candidate(candidate)]
    if not viable:
        return candidates[0], "first_no_viable_candidate"
    return min(viable, key=lambda candidate: adaptive_residual_score(candidate["row"])), (
        "best_residual_score"
    )


def candidate_time_shift_s(candidate):
    if not candidate:
        return math.nan
    row = candidate.get("row", {})
    for key in ["result_time_shift_s", "ceres_time_shift_s"]:
        value = float_row(row, key, math.nan)
        if math.isfinite(value):
            return value
    result_path = candidate.get("result")
    if result_path and pathlib.Path(result_path).is_file():
        try:
            return float(read_ceres_seed_result(pathlib.Path(result_path))[
                "time_shift_s"
            ])
        except (OSError, ValueError, SyntaxError, KeyError):
            return math.nan
    return math.nan


def best_single_time_seed_candidate(args, candidates):
    by_label = {candidate["label"]: candidate for candidate in candidates}
    single_short = by_label.get("single_short")
    single_long = by_label.get("single_long")
    if single_short and single_long:
        single_best, _ = choose_adaptive_ceres_candidate(
            args, single_short, single_long
        )
        return single_best
    single_candidates = [
        candidate for candidate in candidates
        if candidate["label"].startswith("single_")
    ]
    if single_candidates:
        return best_residual_candidate(single_candidates)[0]
    return None


def no_kalibr_chain_rejection_reasons(args, single_row, chain_row):
    reasons = []
    if bool_row(chain_row, "ceres_time_shift_prior_boundary_peak_rejected"):
        reasons.append("time_boundary")
    if bool_row(chain_row, "ceres_imu_chain_prior_boundary_peak_rejected"):
        reasons.append("imu_time_boundary")

    chain_accel_mean = float_row(chain_row, "ceres_accel_m_s2_mean")
    chain_accel_max = float_row(chain_row, "ceres_accel_m_s2_max")
    single_accel_mean = float_row(single_row, "ceres_accel_m_s2_mean")
    single_accel_max = float_row(single_row, "ceres_accel_m_s2_max")
    health_values = [
        chain_accel_mean, single_accel_mean, chain_accel_max, single_accel_max
    ]
    if not all(math.isfinite(value) for value in health_values):
        reasons.append("missing_health")
        return reasons

    if chain_accel_mean > single_accel_mean * args.ceres_multi_imu_accuracy_chain_accel_mean_ratio:
        reasons.append("accel_mean")
    if chain_accel_max > single_accel_max * args.ceres_multi_imu_accuracy_chain_accel_max_ratio:
        reasons.append("accel_max")
    if chain_accel_max > args.ceres_multi_imu_accuracy_chain_accel_max_abs:
        reasons.append("accel_max_abs")

    chain_score = adaptive_residual_score(chain_row)
    single_score = adaptive_residual_score(single_row)
    if chain_score > single_score * args.ceres_multi_imu_accuracy_chain_score_ratio:
        reasons.append("score")
    return reasons


def no_kalibr_accuracy_candidate_status(args, candidates, selected_candidate):
    by_label = {candidate["label"]: candidate for candidate in candidates}
    single_short = by_label.get("single_short")
    single_long = by_label.get("single_long")
    if single_short and single_long:
        single_best, _ = choose_adaptive_ceres_candidate(
            args, single_short, single_long
        )
    else:
        single_best, _ = best_residual_candidate(candidates)

    statuses = {}
    selected_label = selected_candidate["label"] if selected_candidate else ""
    for candidate in candidates:
        label = candidate["label"]
        if label == selected_label:
            statuses[label] = "selected"
            continue
        if not finite_candidate(candidate):
            statuses[label] = "invalid"
            continue
        if label.startswith("chain_") and finite_candidate(single_best):
            reasons = no_kalibr_chain_rejection_reasons(
                args, single_best["row"], candidate["row"]
            )
            statuses[label] = (
                "rejected:" + ",".join(reasons)
                if reasons else "viable_not_best"
            )
        elif label.startswith("single_"):
            statuses[label] = "single_not_selected"
        else:
            statuses[label] = "not_selected"
    return statuses


def choose_no_kalibr_accuracy_candidate(args, candidates):
    by_label = {candidate["label"]: candidate for candidate in candidates}
    single_short = by_label.get("single_short")
    single_long = by_label.get("single_long")
    chain_candidates = [
        candidate for candidate in candidates
        if candidate["label"].startswith("chain_")
    ]

    if single_short and single_long:
        single_best, single_decision = choose_adaptive_ceres_candidate(
            args, single_short, single_long
        )
    else:
        single_best, single_decision = best_residual_candidate(candidates)

    finite_chains = [
        candidate for candidate in chain_candidates if finite_candidate(candidate)
    ]
    if not finite_chains:
        return single_best, f"single_{single_decision}_chain_unavailable"
    if not finite_candidate(single_best):
        return min(
            finite_chains,
            key=lambda candidate: adaptive_residual_score(candidate["row"]),
        ), "chain_single_unavailable"

    single_row = single_best["row"]
    rejected_reasons = []
    viable_chains = []
    for chain in finite_chains:
        chain_row = chain["row"]
        chain_label = chain["label"]
        reasons = no_kalibr_chain_rejection_reasons(args, single_row, chain_row)
        if reasons:
            rejected_reasons.extend(
                f"{chain_label}:{reason}" for reason in reasons
            )
            continue
        viable_chains.append(chain)

    if viable_chains:
        best_chain = min(
            viable_chains,
            key=lambda candidate: adaptive_residual_score(candidate["row"]),
        )
        return best_chain, (
            f"{best_chain['label']}_score_within_{single_best['label']}"
        )
    if rejected_reasons:
        return single_best, (
            f"single_{single_decision}_chain_rejected_"
            + ",".join(rejected_reasons)
        )
    return single_best, f"single_{single_decision}_chain_score_health"


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
        args.ceres_platform,
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


def run_compare(args, row, run_dir, kalibr_result, ceres_result,
                compare_label="compare", prefix=""):
    if not kalibr_result or not ceres_result.is_file():
        return row
    compare_dir = run_dir / compare_label
    command = compare_command(args, kalibr_result, ceres_result, compare_dir)
    code, elapsed, text = run_logged(command, compare_dir, "compare", args.print_only)
    row[f"{prefix}compare_return_code"] = str(code)
    row[f"{prefix}compare_elapsed_s"] = f"{elapsed:.6f}"
    row[f"{prefix}compare_log"] = str(compare_dir / "compare.clean.log")
    parsed = parse_compare_log(text)
    if prefix:
        for key, value in parsed.items():
            row[f"{prefix}{key}"] = value
    else:
        row.update(parsed)
    return row


def run_benchmark_case(args, name, dataset: pathlib.Path, out_root: pathlib.Path,
                       imu_data_names, label: str,
                       ceres_init_result: pathlib.Path = None):
    case_dir = out_root / name / label
    ceres_dir = case_dir / "ceres"
    base_row = {
        "case": name,
        "label": label,
        "dataset": str(dataset),
        "imu_count": str(len(imu_data_names)),
        "imu_data": ",".join(imu_data_names),
    }
    base_row.update(args.metadata)

    kalibr_runs = []
    for variant in args.kalibr_variants:
        if args.reuse_kalibr_from:
            print(
                f"[{name}/{label}] reuse Kalibr {variant['label']}",
                flush=True,
            )
            kalibr_runs.append(
                load_reused_kalibr_run(
                    args, out_root, name, label, variant, label_subdir=True
                )
            )
        else:
            kalibr_dir = case_dir / f"kalibr_{variant['label']}"
            print(f"[{name}/{label}] Kalibr {variant['label']}", flush=True)
            k_cmd = kalibr_corner_command(
                args, dataset, kalibr_dir, imu_data_names, variant
            )
            k_code, k_elapsed, k_log = run_logged(
                k_cmd, kalibr_dir, "kalibr", args.print_only
            )
            kalibr_result = first_result_txt(kalibr_dir)
            kalibr_row = {
                "kalibr_variant": variant["label"],
                "kalibr_platform": variant["platform"],
                "kalibr_image": variant["image"],
                "kalibr_image_id": variant.get("image_id", ""),
                "kalibr_return_code": str(k_code),
                "kalibr_elapsed_s": f"{k_elapsed:.6f}",
                "kalibr_log": str(kalibr_dir / "kalibr.clean.log"),
                "kalibr_result": str(kalibr_result or ""),
            }
            kalibr_row.update(parse_kalibr_log(k_log))
            if kalibr_result:
                kalibr_row.update(parse_kalibr_result(kalibr_result))
            kalibr_runs.append({
                "variant": variant,
                "result": kalibr_result,
                "row": kalibr_row,
            })

    multi_imu = len(imu_data_names) > 1
    init_run = (
        select_kalibr_run(kalibr_runs, args.ceres_init_kalibr_variant)
        if multi_imu
        and args.ceres_multi_imu_init == "kalibr"
        and not is_no_kalibr_accuracy_preset(
            args.ceres_multi_imu_candidate_preset)
        else None
    )
    init_result = init_run["result"] if init_run else None

    def run_ceres_candidate(candidate_label, candidate_dir, extra_args=None,
                            command_args=None, candidate_init_result=None):
        command_args = command_args or args
        candidate_init_result = candidate_init_result or ceres_init_result
        print(f"[{name}/{label}] Ceres {candidate_label}", flush=True)
        c_cmd = ceres_corner_command(
            command_args, dataset, candidate_dir, imu_data_names,
            init_result, candidate_init_result, extra_args=extra_args
        )
        c_code, c_elapsed, c_log = run_logged(
            c_cmd, candidate_dir, "ceres", args.print_only
        )
        result_path = candidate_dir / "result.yaml"
        row = {
            "ceres_candidate": candidate_label,
            "ceres_return_code": str(c_code),
            "ceres_elapsed_s": f"{c_elapsed:.6f}",
            "ceres_log": str(candidate_dir / "ceres.clean.log"),
            "ceres_result": str(result_path if result_path.is_file() else ""),
            "ceres_init_kalibr_variant": (
                init_run["variant"]["label"] if init_run else ""
            ),
            "ceres_init_kalibr_result": str(init_result or ""),
            "ceres_init_result": str(ceres_init_result or ""),
        }
        row.update(parse_ceres_log(c_log))
        row.update(write_ceres_run_policy_artifacts(candidate_dir, c_log))
        row.update(summarize_yaml(result_path))
        return {
            "label": candidate_label,
            "dir": candidate_dir,
            "result": result_path,
            "row": row,
        }

    all_ceres_candidates = []
    if multi_imu and args.ceres_multi_imu_adaptive_long_solve:
        short_candidate = run_ceres_candidate(
            "short", case_dir / "ceres_short"
        )
        long_extra_args = [
            "--solver-absolute-step-tolerance=-1",
            "--stage-solver-absolute-step-tolerances=-1,-1",
        ]
        long_extra_args.extend(args.ceres_multi_imu_adaptive_long_extra_arg)
        long_candidate = run_ceres_candidate(
            "long", case_dir / "ceres_long", long_extra_args
        )
        all_ceres_candidates = [short_candidate, long_candidate]
        selected_candidate, decision = choose_adaptive_ceres_candidate(
            args, short_candidate, long_candidate
        )
        ceres_result = selected_candidate["result"]
        ceres_row = dict(selected_candidate["row"])
        ceres_row["ceres_adaptive_decision"] = decision
        ceres_row["ceres_adaptive_short_result"] = str(short_candidate["result"])
        ceres_row["ceres_adaptive_long_result"] = str(long_candidate["result"])
        ceres_row["ceres_adaptive_short_log"] = str(short_candidate["dir"] / "ceres.clean.log")
        ceres_row["ceres_adaptive_long_log"] = str(long_candidate["dir"] / "ceres.clean.log")
        ceres_row["ceres_adaptive_short_score"] = f"{adaptive_residual_score(short_candidate['row']):.12g}"
        ceres_row["ceres_adaptive_long_score"] = f"{adaptive_residual_score(long_candidate['row']):.12g}"
        short_elapsed = float_row(short_candidate["row"], "ceres_elapsed_s", 0.0)
        long_elapsed = float_row(long_candidate["row"], "ceres_elapsed_s", 0.0)
        ceres_row["ceres_adaptive_short_elapsed_s"] = (
            short_candidate["row"].get("ceres_elapsed_s", "")
        )
        ceres_row["ceres_adaptive_long_elapsed_s"] = (
            long_candidate["row"].get("ceres_elapsed_s", "")
        )
        if math.isfinite(short_elapsed) and math.isfinite(long_elapsed):
            ceres_row["ceres_adaptive_total_elapsed_s"] = (
                f"{short_elapsed + long_elapsed:.6f}"
            )
        for source_label, candidate in [
            ("short", short_candidate),
            ("long", long_candidate),
        ]:
            for metric_key in [
                "ceres_accel_m_s2_mean",
                "ceres_accel_m_s2_max",
            ]:
                ceres_row[f"ceres_adaptive_{source_label}_{metric_key}"] = (
                    candidate["row"].get(metric_key, "")
                )
        print(
            f"[{name}/{label}] adaptive Ceres selected "
            f"{selected_candidate['label']} ({decision})",
            flush=True,
        )
    elif multi_imu and is_no_kalibr_accuracy_preset(
            args.ceres_multi_imu_candidate_preset):
        candidates = []
        configs = no_kalibr_accuracy_candidate_configs(args)
        pending_single_time_configs = []
        for config in configs:
            if config.get("uses_single_time_seed"):
                pending_single_time_configs.append(config)
                continue
            candidate_init_result = (
                ceres_init_result if config.get("uses_single_init") else None
            )
            candidate_extra_args = list(config["extra_args"])
            candidates.append(
                run_ceres_candidate(
                    config["label"],
                    case_dir / f"ceres_{safe_label(config['label'])}",
                    candidate_extra_args,
                    command_args=config["args"],
                    candidate_init_result=candidate_init_result,
                )
            )
        single_time_seed_s = math.nan
        single_time_seed_candidate = best_single_time_seed_candidate(
            args, candidates
        )
        if single_time_seed_candidate:
            single_time_seed_s = candidate_time_shift_s(single_time_seed_candidate)
        if not math.isfinite(single_time_seed_s) and ceres_init_result:
            single_time_seed_s = read_ceres_seed_result(
                ceres_init_result
            )["time_shift_s"]
        for config in pending_single_time_configs:
            candidate_init_result = (
                ceres_init_result if config.get("uses_single_init") else None
            )
            candidate_extra_args = list(config["extra_args"])
            if math.isfinite(single_time_seed_s):
                candidate_extra_args.extend([
                    "--initial-time-shift-s",
                    f"{single_time_seed_s:.17g}",
                ])
            candidates.append(
                run_ceres_candidate(
                    config["label"],
                    case_dir / f"ceres_{safe_label(config['label'])}",
                    candidate_extra_args,
                    command_args=config["args"],
                    candidate_init_result=candidate_init_result,
                )
            )
        all_ceres_candidates = candidates
        selected_candidate, decision = choose_no_kalibr_accuracy_candidate(
            args, candidates
        )
        candidate_status = no_kalibr_accuracy_candidate_status(
            args, candidates, selected_candidate
        )
        ceres_result = selected_candidate["result"]
        ceres_row = dict(selected_candidate["row"])
        ceres_row["ceres_accuracy_preset"] = args.ceres_multi_imu_candidate_preset
        ceres_row["ceres_accuracy_decision"] = decision
        ceres_row["ceres_accuracy_selected_candidate"] = selected_candidate["label"]
        total_elapsed = 0.0
        total_elapsed_finite = True
        for candidate in candidates:
            prefix = f"ceres_accuracy_{safe_label(candidate['label'])}"
            row = candidate["row"]
            for key in [
                "ceres_return_code",
                "ceres_elapsed_s",
                "ceres_optimize_s",
                "ceres_iterations",
                "ceres_final_cost",
                "ceres_reprojection_px_mean",
                "ceres_gyro_rad_s_mean",
                "ceres_accel_m_s2_mean",
                "ceres_accel_m_s2_max",
                "ceres_time_shift_prior_boundary_peak_rejected",
                "ceres_time_shift_prior_peak_correlation",
                "ceres_time_shift_prior_second_best_discrete_shift_samples",
                "ceres_time_shift_prior_second_best_correlation",
                "ceres_time_shift_prior_zero_lag_correlation",
                "ceres_imu_chain_prior_count",
                "ceres_imu_chain_prior_boundary_peak_rejected",
                "ceres_imu_chain_prior_boundary_reject_count",
                "ceres_imu_chain_prior_max_abs_time_offset_s",
                "ceres_imu_chain_prior_min_peak_correlation",
            ]:
                ceres_row[f"{prefix}_{key}"] = row.get(key, "")
            ceres_row[f"{prefix}_selection_status"] = candidate_status.get(
                candidate["label"], ""
            )
            ceres_row[f"{prefix}_result"] = str(candidate["result"])
            ceres_row[f"{prefix}_log"] = str(
                candidate["dir"] / "ceres.clean.log"
            )
            ceres_row[f"{prefix}_score"] = (
                f"{adaptive_residual_score(row):.12g}"
            )
            elapsed = float_row(row, "ceres_elapsed_s", math.nan)
            if math.isfinite(elapsed):
                total_elapsed += elapsed
            else:
                total_elapsed_finite = False
        if total_elapsed_finite:
            ceres_row["ceres_accuracy_total_elapsed_s"] = f"{total_elapsed:.6f}"
        print(
            f"[{name}/{label}] no-Kalibr accuracy selected "
            f"{selected_candidate['label']} ({decision})",
            flush=True,
        )
    else:
        selected_candidate = run_ceres_candidate("default", ceres_dir)
        all_ceres_candidates = [selected_candidate]
        ceres_result = selected_candidate["result"]
        ceres_row = selected_candidate["row"]

    rows = []
    for kalibr_run in kalibr_runs:
        row = {}
        row.update(base_row)
        row.update(kalibr_run["row"])
        row.update(ceres_row)
        row = run_compare(
            args,
            row,
            case_dir,
            kalibr_run["result"],
            ceres_result,
            f"compare_{kalibr_run['variant']['label']}",
        )
        if is_no_kalibr_accuracy_preset(args.ceres_multi_imu_candidate_preset):
            for candidate in all_ceres_candidates:
                prefix = f"candidate_{safe_label(candidate['label'])}_"
                row = run_compare(
                    args,
                    row,
                    case_dir,
                    kalibr_run["result"],
                    candidate["result"],
                    (
                        f"compare_{kalibr_run['variant']['label']}_"
                        f"{safe_label(candidate['label'])}"
                    ),
                    prefix=prefix,
                )
        rows.append(row)
    return rows


def run_benchmark_single(args, out_root: pathlib.Path):
    rows = []
    sessions = args.session or BENCHMARK_SESSIONS
    for index, session in enumerate(sessions, start=1):
        dataset = (args.benchmark_root / session / "cam_imu").resolve()
        name = f"benchmark_{index:02d}_{session}"
        rows.extend(run_benchmark_case(args, name, dataset, out_root,
                                       ["data1.csv"], "single_imu1"))
        write_summary(out_root / "benchmark_single_summary.csv", rows)
    return rows


def ceres_single_seed_result_candidates(args, out_root: pathlib.Path,
                                        name: str, imu_index: int):
    label = f"single_imu{imu_index}"
    bases = [out_root]
    if args.ceres_multi_imu_single_seed_root:
        root = args.ceres_multi_imu_single_seed_root
        bases.extend([root, root / out_root.name])
    candidates = []
    case_names = [name]
    parts = name.split("_", 3)
    session_suffix = parts[3] if len(parts) == 4 else ""
    for base in bases:
        if not base:
            continue
        for case_name in case_names:
            candidates.append(base / case_name / label / "ceres" / "result.yaml")
        if session_suffix and base.is_dir():
            for matched_case in sorted(
                base.glob(f"benchmark_multi_*_{session_suffix}")
            ):
                candidates.append(matched_case / label / "ceres" / "result.yaml")
    deduped = []
    seen = set()
    for candidate in candidates:
        key = str(candidate)
        if key not in seen:
            seen.add(key)
            deduped.append(candidate)
    return deduped


def find_existing_ceres_single_seed_results(args, out_root: pathlib.Path,
                                            name: str):
    paths = []
    for imu_index in range(1, 5):
        found = None
        for candidate in ceres_single_seed_result_candidates(
            args, out_root, name, imu_index
        ):
            if candidate.is_file():
                found = candidate
                break
        if not found:
            return None
        paths.append(found)
    return paths


def run_ceres_single_seed_cases(args, name: str, dataset: pathlib.Path,
                                out_root: pathlib.Path):
    paths = []
    for imu_index in range(1, 5):
        label = f"single_seed_imu{imu_index}"
        ceres_dir = out_root / name / label / "ceres"
        print(f"[{name}/{label}] Ceres seed", flush=True)
        command = ceres_corner_command(
            args, dataset, ceres_dir, [f"data{imu_index}.csv"], None, None
        )
        code, _, log = run_logged(command, ceres_dir, "ceres", args.print_only)
        if not args.print_only and code != 0:
            raise RuntimeError(
                f"[{name}/{label}] Ceres seed failed (exit {code}); "
                f"see {ceres_dir}"
            )
        paths.append(ceres_dir / "result.yaml")
        write_ceres_run_policy_artifacts(ceres_dir, log)
    return paths


def build_ceres_single_seed_result(args, name: str, dataset: pathlib.Path,
                                   out_root: pathlib.Path):
    single_paths = find_existing_ceres_single_seed_results(args, out_root, name)
    if single_paths:
        print(
            f"[{name}/joint_4imu] build Ceres single-seed from existing singles",
            flush=True,
        )
    else:
        if args.print_only:
            raise RuntimeError(
                "--ceres-multi-imu-init=ceres-single needs existing single "
                "results in print-only mode; pass "
                "--ceres-multi-imu-single-seed-root"
            )
        single_paths = run_ceres_single_seed_cases(args, name, dataset, out_root)
    single_results = [read_ceres_seed_result(path) for path in single_paths]
    seed_path = out_root / name / "joint_4imu" / "ceres_single_seed" / "result.yaml"
    write_ceres_single_seed_result(
        single_results,
        seed_path,
        body_policy=args.ceres_multi_imu_single_seed_body,
        weight_metric=args.ceres_multi_imu_single_seed_weight_metric,
    )
    print(
        f"[{name}/joint_4imu] Ceres single-seed: {seed_path} "
        f"body={args.ceres_multi_imu_single_seed_body} "
        f"weight_metric={args.ceres_multi_imu_single_seed_weight_metric}",
        flush=True,
    )
    return seed_path


def run_benchmark_multi(args, out_root: pathlib.Path):
    sessions = [args.multi_session] if args.multi_session else (
        args.session or BENCHMARK_SESSIONS
    )
    rows = []
    for index, session in enumerate(sessions, start=1):
        dataset = (args.benchmark_root / session / "cam_imu").resolve()
        name = f"benchmark_multi_{index:02d}_{session}"
        ran_single_rows = False
        if (
            args.benchmark_multi_subset == "all"
            and args.ceres_multi_imu_init == "ceres-single"
            and not args.ceres_multi_imu_single_seed_root
        ):
            for imu_index in range(1, 5):
                rows.extend(
                    run_benchmark_case(
                        args, name, dataset, out_root,
                        [f"data{imu_index}.csv"],
                        f"single_imu{imu_index}",
                    )
                )
                write_summary(out_root / "benchmark_multi_imu_summary.csv", rows)
            ran_single_rows = True
        ceres_joint_init_result = None
        if (
            args.benchmark_multi_subset in ("all", "joint")
            and (
                args.ceres_multi_imu_init == "ceres-single"
                or is_no_kalibr_accuracy_preset(
                    args.ceres_multi_imu_candidate_preset)
            )
        ):
            ceres_joint_init_result = build_ceres_single_seed_result(
                args, name, dataset, out_root
            )
        if args.benchmark_multi_subset in ("all", "joint"):
            rows.extend(
                run_benchmark_case(
                    args, name, dataset, out_root,
                    ["data1.csv", "data2.csv", "data3.csv", "data4.csv"],
                    "joint_4imu",
                    ceres_joint_init_result,
                )
            )
            write_summary(out_root / "benchmark_multi_imu_summary.csv", rows)
        if args.benchmark_multi_subset in ("all", "single") and not ran_single_rows:
            for imu_index in range(1, 5):
                rows.extend(
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
    if not args.print_only and prep_code != 0:
        raise RuntimeError(
            f"[{name}] prepare Ceres inputs failed (exit {prep_code}); "
            f"see {case_dir / 'prepare'}"
        )
    row = {
        "case": name,
        "label": source_type,
        "dataset": str(tum_root),
        "prepare_return_code": str(prep_code),
        "prepare_elapsed_s": f"{prep_elapsed:.6f}",
        "prepared_dir": str(prepared_dir),
    }
    row.update(args.metadata)
    if source_type == "bag":
        bag = tum_root / "dataset-calib-imu1_512_16.bag"
    else:
        bag = prepared_dir / "euroc_input.bag"

    kalibr_runs = []
    for variant in args.kalibr_variants:
        if args.reuse_kalibr_from:
            print(f"[{name}] reuse Kalibr {variant['label']}", flush=True)
            kalibr_runs.append(
                load_reused_kalibr_run(
                    args, out_root, name, source_type, variant,
                    label_subdir=False,
                )
            )
        else:
            kalibr_dir = case_dir / f"kalibr_{variant['label']}"
            k_cmd = kalibr_bag_command(
                args,
                bag,
                tum_root / "dataset-calib-imu2_512_16/dso/camchain.yaml",
                tum_root / "dataset-calib-imu2_512_16/dso/imu_config.yaml",
                tum_root / "april_6x6_80x80cm.yaml",
                kalibr_dir,
                variant,
            )
            print(f"[{name}] Kalibr {variant['label']}", flush=True)
            k_code, k_elapsed, k_log = run_logged(
                k_cmd, kalibr_dir, "kalibr", args.print_only
            )
            kalibr_result = first_result_txt(kalibr_dir)
            kalibr_row = {
                "kalibr_variant": variant["label"],
                "kalibr_platform": variant["platform"],
                "kalibr_image": variant["image"],
                "kalibr_image_id": variant.get("image_id", ""),
                "kalibr_return_code": str(k_code),
                "kalibr_elapsed_s": f"{k_elapsed:.6f}",
                "kalibr_result": str(kalibr_result or ""),
                "kalibr_log": str(kalibr_dir / "kalibr.clean.log"),
            }
            kalibr_row.update(parse_kalibr_log(k_log))
            if kalibr_result:
                kalibr_row.update(parse_kalibr_result(kalibr_result))
            kalibr_runs.append({
                "variant": variant,
                "result": kalibr_result,
                "row": kalibr_row,
            })

    ceres_dir = case_dir / "ceres"
    c_cmd = ceres_tum_command(args, prepared_dir, ceres_dir, tum_root)
    print(f"[{name}] Ceres", flush=True)
    c_code, c_elapsed, c_log = run_logged(c_cmd, ceres_dir, "ceres", args.print_only)
    ceres_result = ceres_dir / "result.yaml"
    ceres_row = {
        "ceres_return_code": str(c_code),
        "ceres_elapsed_s": f"{c_elapsed:.6f}",
        "ceres_result": str(ceres_result if ceres_result.is_file() else ""),
        "ceres_log": str(ceres_dir / "ceres.clean.log"),
    }
    ceres_row.update(parse_ceres_log(c_log))
    ceres_row.update(write_ceres_run_policy_artifacts(ceres_dir, c_log))
    ceres_row.update(summarize_yaml(ceres_result))

    rows = []
    for kalibr_run in kalibr_runs:
        case_row = {}
        case_row.update(row)
        case_row.update(kalibr_run["row"])
        case_row.update(ceres_row)
        case_row = run_compare(
            args,
            case_row,
            case_dir,
            kalibr_run["result"],
            ceres_result,
            f"compare_{kalibr_run['variant']['label']}",
        )
        rows.append(case_row)
    return rows


def run_tum(args, out_root: pathlib.Path):
    rows = []
    rows.extend(run_tum_case(args, "tum_imu1_bag", "bag", out_root))
    rows.extend(run_tum_case(args, "tum_imu2_euroc_kalibr_export", "euroc", out_root))
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
        "kalibr_variant",
        "kalibr_platform",
        "kalibr_image",
        "kalibr_image_id",
        "ceres_mode",
        "ceres_platform",
        "ceres_extra_args",
        "kalibr_extra_args",
        "ceres_repo_commit",
        "kalibr_docker_repo_commit",
        "kalibr_return_code",
        "ceres_return_code",
        "compare_return_code",
        "kalibr_elapsed_s",
        "ceres_elapsed_s",
        "kalibr_optimize_s",
        "ceres_optimize_s",
        "compare_elapsed_s",
        "ceres_termination",
        "ceres_iterations",
        "ceres_initial_cost",
        "ceres_final_cost",
        "ceres_last_iteration",
        "ceres_last_cost_change",
        "ceres_last_gradient_norm",
        "ceres_last_step_norm",
        "ceres_policy_step_lt_0p02_iter",
        "ceres_policy_step0p02_or_cost0p01_x3_min60_iter",
        "ceres_policy_step0p02_or_cost0p005_x3_min60_iter",
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
        "ceres_iteration_trace",
        "ceres_stop_policy_replay",
        "ceres_init_kalibr_variant",
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
    write_ceres_policy_artifacts(path.parent, rows)


def read_summary_csv(path: pathlib.Path):
    with path.open("r", encoding="utf-8", newline="") as input_file:
        return list(csv.DictReader(input_file))


def refresh_existing_policy_artifacts(out_root: pathlib.Path) -> int:
    patterns = ["summary.csv", "*_summary.csv"]
    summary_paths = []
    for pattern in patterns:
        summary_paths.extend(out_root.rglob(pattern))
    refreshed = 0
    for path in sorted(set(summary_paths)):
        rows = read_summary_csv(path)
        if write_ceres_policy_artifacts(path.parent, rows):
            refreshed += 1
    print(f"policy_artifacts_refreshed: {refreshed}", flush=True)
    return 0 if refreshed > 0 else 1


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run the standard Kalibr Docker and Ceres benchmark suites. "
            "Advanced ablation flags are still accepted but hidden from help."
        )
    )
    hidden = argparse.SUPPRESS
    parser.add_argument("--suite", action="append",
                        choices=["benchmark-single", "benchmark-multi-imu", "tum", "all"],
                        default=[])
    parser.add_argument("--out-root", type=pathlib.Path,
                        default=repo_dir() / "out" / "docker_benchmarks" / timestamp())
    parser.add_argument("--policy-artifacts-only", action="store_true",
                        help=hidden)
    parser.add_argument("--benchmark-root", type=pathlib.Path,
                        default=BENCHMARK_ROOT)
    parser.add_argument("--tum-root", type=pathlib.Path, default=TUM_ROOT)
    parser.add_argument("--session", action="append", default=[],
                        help="Benchmark session name. Repeat to run a subset.")
    parser.add_argument("--multi-session",
                        help="Benchmark session used by --suite benchmark-multi-imu.")
    parser.add_argument("--benchmark-multi-subset",
                        choices=["all", "joint", "single"], default="all",
                        help="Subset for --suite benchmark-multi-imu; default runs joint_4imu plus single_imu1..4.")
    parser.add_argument("--kalibr-amd64-image", default=KALIBR_AMD64_IMAGE,
                        help=hidden)
    parser.add_argument("--kalibr-arm64-image", default=KALIBR_ARM64_IMAGE,
                        help=hidden)
    parser.add_argument("--kalibr-platform", action="append",
                        choices=["linux/amd64", "linux/arm64"],
                        default=[],
                        help="Kalibr Docker platform to run. Repeat to select a subset; default runs both.")
    parser.add_argument("--kalibr-prepare-image",
                        help=hidden)
    parser.add_argument("--kalibr-prepare-platform",
                        choices=["linux/amd64", "linux/arm64"],
                        help=hidden)
    parser.add_argument("--kalibr-docker-repo", type=pathlib.Path,
                        default=KALIBR_DOCKER_REPO, help=hidden)
    parser.add_argument("--reuse-kalibr-from", type=pathlib.Path,
                        help="Skip Kalibr Docker and reuse Kalibr outputs from a previous out-root or suite directory.")
    parser.add_argument("--ceres-image", default=CERES_IMAGE, help=hidden)
    parser.add_argument("--ceres-mode", choices=["native", "docker"],
                        default="native", help=hidden)
    parser.add_argument("--ceres-platform", default="linux/amd64",
                        help=hidden)
    parser.add_argument("--ceres-init-kalibr-variant",
                        choices=["arm64", "amd64"], default="arm64",
                        help=hidden)
    parser.add_argument("--ceres-bin", type=pathlib.Path,
                        default=repo_dir() / "build" / "calibrate_cam_imu",
                        help=hidden)
    parser.add_argument("--compare-bin", type=pathlib.Path,
                        default=repo_dir() / "build" / "compare_kalibr_result",
                        help=hidden)
    parser.add_argument("--pull-kalibr", action="store_true", help=hidden)
    parser.add_argument("--print-only", action="store_true")
    parser.add_argument("--kalibr-max-iter", type=int, default=30,
                        help=hidden)
    parser.add_argument("--kalibr-tum-max-iter", type=int, default=30,
                        help=hidden)
    parser.add_argument("--ceres-max-iterations", type=int,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-stage-iterations", type=int,
                        default=30, help=hidden)
    parser.add_argument("--ceres-multi-imu-init",
                        choices=["kalibr", "chain-prior", "ceres-single"],
                        default="chain-prior",
                        help="Initialization source for single-candidate multi-IMU benchmark Ceres runs. ceres-single seeds joint from Ceres-only single-IMU results; kalibr is opt-in.")
    parser.add_argument("--ceres-multi-imu-candidate-preset",
                        choices=[
                            "none", "no-kalibr-accuracy",
                            "no-kalibr-accuracy-trimmed"
                        ],
                        default="no-kalibr-accuracy-trimmed",
                        help="Run the topology-default multi-candidate no-Kalibr 1cam+Nimu Ceres preset. no-kalibr-accuracy keeps all diagnostic candidates; no-kalibr-accuracy-trimmed keeps short/long single, three short chain multi-prior candidates, and the health-gated long chain single-time candidate. Kalibr results are not used for initialization or candidate selection.")
    parser.add_argument("--ceres-multi-imu-single-seed-root",
                        type=pathlib.Path,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-single-seed-body",
                        choices=["reference", "residual-weighted-translation"],
                        default="reference",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-single-seed-weight-metric",
                        choices=["accel", "gyro", "reprojection", "combined"],
                        default="accel",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-time-shift-prior-sigma",
                        type=float, default=0.0,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-estimate-lever-prior",
                        action="store_true",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-lever-prior-max-accel-rms",
                        type=float, default=-1.0,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-translation-prior-camera-sigma-m",
                        type=float, default=-1.0,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-translation-prior-lever-sigma-m",
                        type=float, default=-1.0,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-translation-prior-accel-bias-sigma",
                        type=float, default=-1.0,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-stage-free", default="pbg,pbegti",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-stage-step-tolerances",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-stage-cost-tolerances",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-stage-extrinsic-translation-bounds-m",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-stage-extrinsic-rotation-bounds-rad",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-adaptive-long-solve",
                        action="store_true",
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-adaptive-accel-mean-ratio",
                        type=float, default=1.05,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-adaptive-accel-max-ratio",
                        type=float, default=1.5,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-adaptive-long-extra-arg",
                        action="append", default=[],
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-accuracy-chain-score-ratio",
                        type=float, default=1.03,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-accuracy-chain-accel-mean-ratio",
                        type=float, default=1.5,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-accuracy-chain-accel-max-ratio",
                        type=float, default=2.0,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-accuracy-chain-accel-max-abs",
                        type=float, default=3.0,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-extrinsic-translation-bound-m",
                        type=float, default=0.003,
                        help=hidden)
    parser.add_argument("--ceres-multi-imu-extrinsic-rotation-bound-rad",
                        type=float, default=0.005,
                        help=hidden)
    parser.add_argument("--ceres-tum-max-iterations", type=int,
                        help=hidden)
    parser.add_argument("--timeoffset-padding", type=float, default=0.04,
                        help=hidden)
    parser.add_argument("--tum-timeoffset-padding", type=float, default=0.03,
                        help=hidden)
    parser.add_argument("--pose-knots-per-second", type=int, default=100,
                        help=hidden)
    parser.add_argument("--bias-knots-per-second", type=int, default=50,
                        help=hidden)
    parser.add_argument("--trim-imu-edge-count", type=int, default=1000,
                        help=hidden)
    parser.add_argument("--kalibr-extra-arg", action="append", default=[],
                        help="Extra argument forwarded to Kalibr; repeatable escape hatch.")
    parser.add_argument("--ceres-extra-arg", action="append", default=[],
                        help="Extra argument forwarded to Ceres; repeatable escape hatch.")
    return parser.parse_args()


def main():
    args = parse_args()
    args.out_root = args.out_root.expanduser().resolve()
    args.benchmark_root = args.benchmark_root.expanduser().resolve()
    args.tum_root = args.tum_root.expanduser().resolve()
    args.ceres_bin = args.ceres_bin.expanduser().resolve()
    args.compare_bin = args.compare_bin.expanduser().resolve()
    args.kalibr_docker_repo = args.kalibr_docker_repo.expanduser().resolve()
    if args.reuse_kalibr_from:
        args.reuse_kalibr_from = args.reuse_kalibr_from.expanduser().resolve()
        if not args.reuse_kalibr_from.is_dir():
            raise FileNotFoundError(
                f"--reuse-kalibr-from not found: {args.reuse_kalibr_from}"
            )
    if args.policy_artifacts_only:
        return refresh_existing_policy_artifacts(args.out_root)
    if args.ceres_max_iterations is not None and args.ceres_max_iterations < 0:
        raise ValueError("--ceres-max-iterations must be non-negative")
    if (args.ceres_tum_max_iterations is not None and
            args.ceres_tum_max_iterations < 0):
        raise ValueError("--ceres-tum-max-iterations must be non-negative")
    if args.ceres_multi_imu_stage_iterations < 0:
        raise ValueError("--ceres-multi-imu-stage-iterations must be non-negative")
    if args.ceres_multi_imu_adaptive_accel_mean_ratio <= 0.0:
        raise ValueError("--ceres-multi-imu-adaptive-accel-mean-ratio must be positive")
    if args.ceres_multi_imu_adaptive_accel_max_ratio <= 0.0:
        raise ValueError("--ceres-multi-imu-adaptive-accel-max-ratio must be positive")
    if args.ceres_multi_imu_accuracy_chain_score_ratio <= 0.0:
        raise ValueError("--ceres-multi-imu-accuracy-chain-score-ratio must be positive")
    if args.ceres_multi_imu_accuracy_chain_accel_mean_ratio <= 0.0:
        raise ValueError("--ceres-multi-imu-accuracy-chain-accel-mean-ratio must be positive")
    if args.ceres_multi_imu_accuracy_chain_accel_max_ratio <= 0.0:
        raise ValueError("--ceres-multi-imu-accuracy-chain-accel-max-ratio must be positive")
    if args.ceres_multi_imu_accuracy_chain_accel_max_abs <= 0.0:
        raise ValueError("--ceres-multi-imu-accuracy-chain-accel-max-abs must be positive")
    if (is_no_kalibr_accuracy_preset(args.ceres_multi_imu_candidate_preset)
            and args.ceres_multi_imu_init == "kalibr"):
        raise ValueError(
            "--ceres-multi-imu-candidate-preset no-kalibr-* cannot be "
            "combined with --ceres-multi-imu-init=kalibr; pass "
            "--ceres-multi-imu-candidate-preset none for Kalibr-init runs"
        )
    if args.kalibr_prepare_image is None:
        args.kalibr_prepare_image = args.kalibr_arm64_image
    if args.kalibr_prepare_platform is None:
        args.kalibr_prepare_platform = "linux/arm64"
    args.kalibr_variants = kalibr_variant_configs(args)
    suites = set(args.suite or ["all"])
    if "all" in suites:
        suites.update(["benchmark-single", "benchmark-multi-imu", "tum"])
    args.reuse_kalibr_summary_index = build_reuse_summary_index(
        args.reuse_kalibr_from
    )
    ensure_images(args)
    for variant in args.kalibr_variants:
        variant["image_id"] = docker_image_id(variant["image"])
    args.metadata = run_metadata(args)

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
