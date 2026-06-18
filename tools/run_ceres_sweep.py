#!/usr/bin/env python3
"""Run reproducible Ceres cam-IMU calibration sweeps and summarize results."""

import argparse
import csv
import datetime as _datetime
import pathlib
import re
import shlex
import subprocess
import time


DEFAULT_DATASET = pathlib.Path("/Users/wayne/Documents/work/data/cam_imu_2")

DEFAULT_FILES = {
    "cam": "cam0-camchain-640x400.yaml",
    "imu": "imu.yaml",
    "target": "aprilgrid.yaml",
    "imu_data": "data1.csv",
    "corners": "cam0_640x400_corners.csv",
    "corner_poses": "cam0_640x400_corner_poses.csv",
    "kalibr_result": "cam0_640x400_corners-1-results-imucam.txt",
}

PRESET_DEFINITIONS = {
    "smoke-fixed": {
        "args": [
        "--init-from-kalibr",
        "--fix-poses",
        "--fix-biases",
        "--fix-time-shift",
        "--fix-gravity",
        "--max-frames",
        "20",
        "--imu-stride",
        "200",
        "--max-imu-residuals",
        "50",
        "--max-iterations",
        "1",
        "--top-residuals",
        "2",
        ],
    },
    "corner-dry-run": {
        "args": [
        "--corner-defaults",
        "--init-from-kalibr",
        "--dry-run",
        ],
    },
    "current-full": {
        "args": [
        "--corner-defaults",
        "--init-from-kalibr",
        "--time-shift-prior-sigma",
        "0.0001",
        "--fix-gravity",
        "--pose-motion-prior",
        "--pose-motion-translation-variance",
        "10",
        "--pose-motion-rotation-variance",
        "1",
        "--staged",
        "--stage-iterations",
        "0,1,4,5",
        "--top-residuals",
        "8",
        "--inspect-times",
        "368.848,376.071",
        "--inspect-window",
        "0.01",
        ],
    },
    "independent-full": {
        "args": [
            "--corner-defaults",
            "--estimate-time-shift-prior",
            "--estimate-orientation-gravity-prior",
            "--pose-fit-motion-lambda",
            "0.0001",
            "--pose-fit-boundary-anchors",
            "--time-shift-prior-sigma",
            "0.0001",
            "--fix-gravity",
            "--pose-motion-prior",
            "--pose-motion-translation-variance",
            "10",
            "--pose-motion-rotation-variance",
            "1",
            "--staged",
            "--stage-iterations",
            "0,1,4,5",
            "--top-residuals",
            "8",
        ],
    },
    "independent-legacy-posefit-full": {
        "args": [
            "--corner-defaults",
            "--estimate-time-shift-prior",
            "--estimate-orientation-gravity-prior",
            "--time-shift-prior-sigma",
            "0.0001",
            "--fix-gravity",
            "--pose-motion-prior",
            "--pose-motion-translation-variance",
            "10",
            "--pose-motion-rotation-variance",
            "1",
            "--staged",
            "--stage-iterations",
            "0,1,4,5",
            "--top-residuals",
            "8",
        ],
    },
    "independent-final-pe-full": {
        "args": [
            "--corner-defaults",
            "--estimate-time-shift-prior",
            "--estimate-orientation-gravity-prior",
            "--pose-fit-motion-lambda",
            "0.0001",
            "--pose-fit-boundary-anchors",
            "--time-shift-prior-sigma",
            "0.0001",
            "--fix-gravity",
            "--pose-motion-prior",
            "--pose-motion-translation-variance",
            "10",
            "--pose-motion-rotation-variance",
            "1",
            "--staged",
            "--stage-free",
            "e,bt,pbt,pe",
            "--stage-iterations",
            "0,1,4,8",
            "--top-residuals",
            "8",
        ],
    },
    "independent-capped-pe-full": {
        "args": [
            "--corner-defaults",
            "--estimate-time-shift-prior",
            "--estimate-orientation-gravity-prior",
            "--pose-fit-motion-lambda",
            "0.0001",
            "--pose-fit-boundary-anchors",
            "--time-shift-prior-sigma",
            "0.0001",
            "--fix-gravity",
            "--pose-motion-prior",
            "--pose-motion-translation-variance",
            "10",
            "--pose-motion-rotation-variance",
            "1",
            "--solver-initial-trust-region-radius",
            "10000",
            "--staged",
            "--stage-free",
            "e,bt,pbt,pe",
            "--stage-iterations",
            "0,1,4,10",
            "--stage-solver-max-trust-region-radii",
            "1e16,1e16,1e16,10000000",
            "--top-residuals",
            "8",
        ],
    },
    # Independent init + single joint optimization (no staging), closest to
    # Kalibr's own main optimization. On production cam_imu data this drives the
    # extrinsic translation delta from ~14 cm (conservative staged) down to
    # ~2-4 mm. Gravity direction is left free (no --fix-gravity); all variables
    # are optimized jointly with a capped max trust-region radius. Remaining
    # ~2-4 mm needs matching Kalibr's LM/stop internals; see
    # docs/experiment/20260616_Ceres与KalibrDocker多数据集速度精度对比.md.
    "independent-joint-full": {
        "args": [
            "--corner-defaults",
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
            "--max-iterations",
            "150",
            "--solver-max-trust-region-radius",
            "10000000",
            "--top-residuals",
            "8",
        ],
    },
    "camchain-dry-run": {
        "files": {"cam": "cam0_640x400_corners-1-camchain-imucam.yaml"},
        "args": [
            "--corner-defaults",
            "--init-from-camchain",
            "--dry-run",
            "--max-frames",
            "20",
            "--imu-stride",
            "1000",
            "--max-imu-residuals",
            "5",
        ],
    },
    "camchain-full": {
        "files": {"cam": "cam0_640x400_corners-1-camchain-imucam.yaml"},
        "args": [
            "--corner-defaults",
            "--init-from-camchain",
            "--estimate-orientation-gravity-prior",
            "--time-shift-prior-sigma",
            "0.0001",
            "--fix-gravity",
            "--pose-motion-prior",
            "--pose-motion-translation-variance",
            "10",
            "--pose-motion-rotation-variance",
            "1",
            "--staged",
            "--stage-iterations",
            "0,1,4,5",
            "--top-residuals",
            "8",
        ],
    },
}

PRESET_DEFINITIONS["kalibr-dry-run"] = PRESET_DEFINITIONS["corner-dry-run"]

PRESETS = {name: definition["args"] for name, definition in PRESET_DEFINITIONS.items()}


def repo_dir():
    return pathlib.Path(__file__).resolve().parents[1]


def timestamp():
    return _datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def sanitize_name(name):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._") or "variant"


def require_file(path, label):
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def option_present(tokens, name):
    return name in tokens


def option_value(tokens, name):
    for index, token in enumerate(tokens[:-1]):
        if token == name:
            return tokens[index + 1]
    return None


def parse_variant(spec):
    if "=" not in spec:
        raise ValueError(
            "variant must have form name='--arg value ...': " + spec
        )
    name, arg_text = spec.split("=", 1)
    name = sanitize_name(name)
    if not name:
        raise ValueError("variant name must be non-empty")
    return name, shlex.split(arg_text)


def key_values_from_line(line):
    values = {}
    for key, value in re.findall(
        r"([A-Za-z_][A-Za-z0-9_]*)=([-+0-9.eE]+)", line
    ):
        values[key] = value
    return values


def parse_solver_options_line(line):
    values = key_values_from_line(line)
    match = re.search(r"\blinear_solver=([A-Za-z0-9_]+)", line)
    if match:
        values["linear_solver"] = match.group(1)
    return values


def parse_camchain_init_line(line):
    values = {}
    for key, value in key_values_from_line(line).items():
        if key != "translation_m":
            values["camchain_init_" + key] = value

    translation_match = re.search(
        r"\btranslation_m=([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
        line,
    )
    if translation_match:
        values["camchain_init_translation_x_m"] = translation_match.group(1)
        values["camchain_init_translation_y_m"] = translation_match.group(2)
        values["camchain_init_translation_z_m"] = translation_match.group(3)
    return values


def parse_vector3(line, key):
    match = re.search(
        rf"\b{re.escape(key)}=([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
        line,
    )
    if not match:
        return {}
    return {
        f"{key}_x": match.group(1),
        f"{key}_y": match.group(2),
        f"{key}_z": match.group(3),
    }


def parse_orientation_init_line(line):
    values = {}
    vector_keys = {"gyro_bias_rad_s", "gravity_m_s2", "singular_values"}
    for key, value in key_values_from_line(line).items():
        if key not in vector_keys:
            values["orientation_init_" + key] = value
    for key in vector_keys:
        for vector_key, vector_value in parse_vector3(line, key).items():
            values["orientation_init_" + vector_key] = vector_value
    return values


def parse_pose_init_line(line):
    values = {}
    for key, value in key_values_from_line(line).items():
        values["pose_init_" + key] = value
    return values


def parse_iteration_state_line(line):
    label_match = re.search(r"\blabel=([A-Za-z0-9_.-]+)", line)
    label = sanitize_name(label_match.group(1)) if label_match else "solver"
    return label, key_values_from_line(line)


def update_min_value(row, prefix, key, value, iteration, use_abs=False):
    try:
        numeric_value = float(value)
    except ValueError:
        return
    score = abs(numeric_value) if use_abs else numeric_value
    score_key = f"{prefix}_min_abs_{key}" if use_abs else f"{prefix}_min_{key}"
    raw_key = f"{prefix}_min_abs_{key}_raw" if use_abs else score_key
    iter_key = f"{score_key}_iter"
    previous = row.get(score_key)
    if previous is None or score < float(previous):
        row[score_key] = f"{score:.17g}"
        row[raw_key] = value
        if iteration is not None:
            row[iter_key] = iteration


def update_iteration_trace(row, label, values):
    prefix = f"trace_{label}"
    iteration = values.get("iter")
    for key, value in values.items():
        row[f"{prefix}_last_{key}"] = value
    for key in (
        "reference_translation_m",
        "reference_rotation_deg",
        "reference_gravity_norm",
        "parameter_delta",
    ):
        if key in values:
            if key == "parameter_delta" and iteration == "0":
                continue
            update_min_value(row, prefix, key, values[key], iteration)
    if "reference_time_shift_s" in values:
        update_min_value(
            row, prefix, "reference_time_shift_s",
            values["reference_time_shift_s"], iteration, use_abs=True
        )


def parse_summary_text(text):
    row = {}
    build_prefixes = ("problem built:", "stage built")
    for line in text.splitlines():
        if line.startswith("corner defaults active:") or line.startswith(
            "kalibr corner defaults active:"
        ):
            for key, value in key_values_from_line(line).items():
                row["preset_" + key] = value
        elif line.startswith("solver options:"):
            for key, value in parse_solver_options_line(line).items():
                row["solver_" + key] = value
        if line.startswith(build_prefixes):
            values = key_values_from_line(line)
            row.update(values)
            if line.startswith("stage built"):
                stage_match = re.match(r"stage built \[([^\]]+)\]:", line)
                if stage_match:
                    stage_name = sanitize_name(stage_match.group(1))
                    for key, value in values.items():
                        row[f"stage_{stage_name}_{key}"] = value
            for key in ("active_parameter_blocks", "tangent_params"):
                if key in values:
                    max_key = f"max_{key}"
                    previous = row.get(max_key)
                    current_value = int(values[key])
                    if previous is None or current_value > int(previous):
                        row[max_key] = str(current_value)
        elif line.startswith("stage state"):
            stage_match = re.match(r"stage state \[([^\]]+)\]:", line)
            if stage_match:
                stage_name = sanitize_name(stage_match.group(1))
                decision_match = re.search(r"\bdecision=([A-Za-z0-9_]+)", line)
                if decision_match:
                    row[f"stage_{stage_name}_state_decision"] = (
                        decision_match.group(1)
                    )
                for key, value in key_values_from_line(line).items():
                    row[f"stage_{stage_name}_state_{key}"] = value
        elif line.startswith("kalibr_delta:"):
            for key, value in key_values_from_line(line).items():
                row["kalibr_delta_" + key] = value
        elif line.startswith("imu model:"):
            match = re.search(r"\bmodel=([A-Za-z0-9_-]+)", line)
            if match:
                row["imu_model"] = match.group(1)
            for key, value in key_values_from_line(line).items():
                row["imu_" + key] = value
        elif line.startswith("initialized from camchain:"):
            row.update(parse_camchain_init_line(line))
        elif line.startswith("estimated time shift prior:"):
            for key, value in key_values_from_line(line).items():
                row["time_shift_init_" + key] = value
        elif line.startswith("estimated orientation/gravity prior:"):
            row.update(parse_orientation_init_line(line))
        elif line.startswith("initialized pose controls:"):
            row.update(parse_pose_init_line(line))
        elif line.startswith("iteration_state"):
            label, values = parse_iteration_state_line(line)
            update_iteration_trace(row, label, values)
        elif line.startswith("absolute_stop"):
            label_match = re.search(r"\blabel=([A-Za-z0-9_.-]+)", line)
            label = sanitize_name(label_match.group(1)) if label_match else ""
            prefix = "absolute_stop" + (f"_{label}" if label else "")
            for key, value in key_values_from_line(line).items():
                row[f"{prefix}_{key}"] = value
        elif line.startswith("residual_stats "):
            match = re.match(r"residual_stats ([A-Za-z0-9_]+): (.*)", line)
            if match:
                residual_name = match.group(1)
                for key, value in key_values_from_line(match.group(2)).items():
                    row[f"{residual_name}_{key}"] = value
        elif line.startswith("ceres_vs_kalibr:"):
            for key, value in key_values_from_line(line).items():
                row["compare_" + key] = value
        elif line.startswith("residual_delta_ceres_minus_kalibr:"):
            for key, value in key_values_from_line(line).items():
                row["delta_" + key] = value
        elif line.startswith("imu_intrinsics_delta_ceres_minus_kalibr:"):
            for key, value in key_values_from_line(line).items():
                row["imu_delta_" + key] = value
    return row


def file_arg(args, files, key):
    return files.get(key) or getattr(args, key)


def kalibr_result_path(args, files=None):
    files = files or {}
    return args.dataset / file_arg(args, files, "kalibr_result")


def base_command(args, files=None):
    dataset = args.dataset
    files = files or {}
    return [
        str(args.calibrate_bin),
        "--cam",
        str(require_file(dataset / file_arg(args, files, "cam"), "camera config")),
        "--imu",
        str(require_file(dataset / file_arg(args, files, "imu"), "IMU config")),
        "--target",
        str(require_file(dataset / file_arg(args, files, "target"), "target config")),
        "--imu-data",
        str(require_file(dataset / file_arg(args, files, "imu_data"), "IMU CSV")),
        "--corners",
        str(require_file(dataset / file_arg(args, files, "corners"), "corner CSV")),
        "--corner-poses",
        str(require_file(dataset / file_arg(args, files, "corner_poses"),
                         "corner pose CSV")),
    ]


def write_text(path, text):
    path.write_text(text, encoding="utf-8")


def run_command(command, cwd, output_path):
    start = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    elapsed_s = time.monotonic() - start
    write_text(output_path, completed.stdout)
    return completed.returncode, elapsed_s, completed.stdout


def build_variants(args):
    variants = []
    presets = args.preset
    if not presets and not args.variant:
        presets = ["smoke-fixed"]
    for preset in presets:
        if preset == "none":
            continue
        if preset not in PRESET_DEFINITIONS:
            raise ValueError(f"unknown preset: {preset}")
        definition = PRESET_DEFINITIONS[preset]
        variants.append((preset, list(definition["args"]),
                         dict(definition.get("files", {}))))
    for spec in args.variant:
        name, variant_args = parse_variant(spec)
        variants.append((name, variant_args, {}))
    if not variants:
        raise ValueError("no variants selected")
    return variants


def run_variant(args, sweep_dir, name, variant_args, files=None):
    variant_dir = sweep_dir / sanitize_name(name)
    variant_dir.mkdir(parents=True, exist_ok=True)
    tokens = (
        base_command(args, files)
        + list(args.base_arg)
        + variant_args
        + list(args.extra_arg)
    )
    if option_present(tokens, "--init-from-kalibr") and not option_present(
        tokens, "--kalibr-result"
    ):
        tokens.extend(
            [
                "--kalibr-result",
                str(require_file(kalibr_result_path(args, files), "Kalibr result")),
            ]
        )

    result_yaml = option_value(tokens, "--output-result")
    is_dry_run = option_present(tokens, "--dry-run")
    if not is_dry_run and result_yaml is None:
        result_yaml = str(variant_dir / "result.yaml")
        tokens.extend(["--output-result", result_yaml])

    command_path = variant_dir / "command.txt"
    stdout_path = variant_dir / "stdout.log"
    compare_path = variant_dir / "compare.txt"
    write_text(command_path, " ".join(shlex.quote(x) for x in tokens) + "\n")

    print(f"[{name}] running: {command_path}", flush=True)
    return_code, elapsed_s, stdout = run_command(tokens, args.cwd, stdout_path)

    row = {
        "variant": name,
        "return_code": str(return_code),
        "elapsed_s": f"{elapsed_s:.6f}",
        "command": str(command_path),
        "stdout": str(stdout_path),
        "result_yaml": result_yaml or "",
        "compare": "",
    }
    row.update(parse_summary_text(stdout))

    if return_code == 0 and result_yaml and pathlib.Path(result_yaml).is_file():
        compare_cmd = [
            str(args.compare_bin),
            "--kalibr-result",
            str(kalibr_result_path(args, files)),
            "--ceres-result",
            result_yaml,
        ]
        compare_code, compare_elapsed_s, compare_stdout = run_command(
            compare_cmd, args.cwd, compare_path
        )
        row["compare_return_code"] = str(compare_code)
        row["compare_elapsed_s"] = f"{compare_elapsed_s:.6f}"
        row["compare"] = str(compare_path)
        row.update(parse_summary_text(compare_stdout))
    return row


def write_summary_csv(path, rows):
    preferred = [
        "variant",
        "return_code",
        "elapsed_s",
        "camera",
        "gyro",
        "accel",
        "gyro_priors",
        "accel_priors",
        "pose_priors",
        "time_priors",
        "parameter_blocks",
        "active_parameter_blocks",
        "tangent_params",
        "kalibr_style_error_terms",
        "imu_model",
        "solver_linear_solver",
        "solver_num_threads",
        "solver_initial_trust_region_radius",
        "solver_max_trust_region_radius",
        "solver_min_trust_region_radius",
        "solver_min_relative_decrease",
        "solver_absolute_cost_change_tolerance",
        "solver_absolute_step_tolerance",
        "solver_absolute_parameter_tolerance",
        "solver_use_nonmonotonic_steps",
        "solver_max_consecutive_nonmonotonic_steps",
        "kalibr_delta_rotation_deg",
        "kalibr_delta_translation_m",
        "kalibr_delta_time_shift_s",
        "kalibr_delta_gravity_norm",
        "compare_rotation_deg",
        "compare_translation_m",
        "compare_time_shift_s",
        "compare_gravity_norm",
        "camchain_init_time_shift_s",
        "camchain_init_translation_x_m",
        "camchain_init_translation_y_m",
        "camchain_init_translation_z_m",
        "camchain_init_kalibr_translation_delta_m",
        "camchain_init_kalibr_time_delta_s",
        "time_shift_init_shift_s",
        "time_shift_init_kalibr_delta_s",
        "orientation_init_boundary_anchors",
        "orientation_init_ceres_refine",
        "orientation_init_refine_iterations",
        "orientation_init_refine_final_cost",
        "orientation_init_kalibr_rotation_delta_deg",
        "orientation_init_kalibr_gravity_delta_norm",
        "orientation_init_gyro_bias_rad_s_x",
        "orientation_init_gyro_bias_rad_s_y",
        "orientation_init_gyro_bias_rad_s_z",
        "orientation_init_gravity_m_s2_x",
        "orientation_init_gravity_m_s2_y",
        "orientation_init_gravity_m_s2_z",
        "pose_init_used",
        "pose_init_skipped",
        "pose_init_boundary_anchors",
        "pose_init_coeffs",
        "pose_init_fit_motion_lambda",
        "pose_init_rms_translation_m",
        "pose_init_rms_rotation_rad",
        "trace_custom_3_free_pe_min_reference_translation_m",
        "trace_custom_3_free_pe_min_reference_translation_m_iter",
        "trace_custom_3_free_pe_min_parameter_delta",
        "trace_custom_3_free_pe_min_parameter_delta_iter",
        "trace_custom_3_free_pe_last_reference_translation_m",
        "trace_custom_3_free_pe_last_reference_rotation_deg",
        "trace_custom_3_free_pe_last_reference_time_shift_s",
        "trace_custom_3_free_pe_last_parameter_delta",
        "trace_custom_3_free_pe_last_cost",
        "absolute_stop_custom_3_free_pe_iter",
        "absolute_stop_custom_3_free_pe_cost_change",
        "absolute_stop_custom_3_free_pe_step_norm",
        "absolute_stop_custom_3_free_pe_parameter_delta",
        "absolute_stop_custom_3_free_pe_cost_trigger",
        "absolute_stop_custom_3_free_pe_step_trigger",
        "absolute_stop_custom_3_free_pe_parameter_trigger",
        "reprojection_px_mean",
        "gyro_rad_s_mean",
        "accel_m_s2_mean",
        "delta_reproj_px",
        "delta_gyro_rad_s",
        "delta_accel_m_s2",
        "imu_delta_accel_M_fro",
        "imu_delta_accel_M_rel",
        "imu_delta_gyro_M_fro",
        "imu_delta_gyro_M_rel",
        "imu_delta_gyro_A_fro",
        "imu_delta_gyro_A_rel",
        "imu_delta_gyro_C_fro",
        "imu_delta_gyro_C_rel",
        "result_yaml",
        "compare",
        "stdout",
        "command",
    ]
    keys = set()
    for row in rows:
        keys.update(row.keys())
    fieldnames = [key for key in preferred if key in keys]
    fieldnames.extend(sorted(keys.difference(fieldnames)))
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run Ceres cam-IMU calibration variants and summarize outputs."
    )
    root = repo_dir()
    parser.add_argument("--dataset", type=pathlib.Path, default=DEFAULT_DATASET)
    parser.add_argument("--calibrate-bin", type=pathlib.Path,
                        default=root / "build" / "calibrate_cam_imu")
    parser.add_argument("--compare-bin", type=pathlib.Path,
                        default=root / "build" / "compare_kalibr_result")
    parser.add_argument("--out-root", type=pathlib.Path,
                        default=root / "out" / "ceres_sweeps")
    parser.add_argument("--run-name")
    parser.add_argument("--preset", action="append",
                        choices=sorted([*PRESET_DEFINITIONS.keys(), "none"]),
                        default=[])
    parser.add_argument("--variant", action="append", default=[],
                        help="Custom variant: name='--arg value ...'")
    parser.add_argument("--base-arg", action="append", default=[],
                        help="Token appended before every variant.")
    parser.add_argument("--extra-arg", action="append", default=[],
                        help="Token appended after every variant.")
    parser.add_argument("--cam", default=DEFAULT_FILES["cam"])
    parser.add_argument("--imu", default=DEFAULT_FILES["imu"])
    parser.add_argument("--target", default=DEFAULT_FILES["target"])
    parser.add_argument("--imu-data", default=DEFAULT_FILES["imu_data"])
    parser.add_argument("--corners", default=DEFAULT_FILES["corners"])
    parser.add_argument("--corner-poses", default=DEFAULT_FILES["corner_poses"])
    parser.add_argument("--kalibr-result", default=DEFAULT_FILES["kalibr_result"])
    parser.add_argument("--stop-on-failure", action="store_true")
    args = parser.parse_args()
    args.dataset = args.dataset.expanduser().resolve()
    args.calibrate_bin = args.calibrate_bin.expanduser().resolve()
    args.compare_bin = args.compare_bin.expanduser().resolve()
    args.out_root = args.out_root.expanduser().resolve()
    args.cwd = root
    require_file(args.calibrate_bin, "calibrate binary")
    require_file(args.compare_bin, "compare binary")
    return args


def main():
    args = parse_args()
    run_name = args.run_name or f"ceres_sweep_{timestamp()}"
    sweep_dir = args.out_root / run_name
    sweep_dir.mkdir(parents=True, exist_ok=True)

    variants = build_variants(args)
    rows = []
    for name, variant_args, files in variants:
        row = run_variant(args, sweep_dir, name, variant_args, files)
        rows.append(row)
        if args.stop_on_failure and row.get("return_code") != "0":
            break

    summary_csv = sweep_dir / "summary.csv"
    write_summary_csv(summary_csv, rows)
    print(f"summary_csv: {summary_csv}")
    return 0 if all(row.get("return_code") == "0" for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
