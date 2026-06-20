#!/usr/bin/env python3
"""Evaluate stereo-via-IMU loop closure error for TUM cam-IMU runs."""

import argparse
import csv
import math
import pathlib
import re
import sys

import numpy as np
import yaml


NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


def load_yaml(path):
    with pathlib.Path(path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def as_matrix(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"expected 4x4 matrix, got {matrix.shape}")
    return matrix


def rotation_angle_deg(rotation):
    cos_angle = (float(np.trace(rotation)) - 1.0) * 0.5
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def transform_error(direct, composed):
    error = np.linalg.inv(direct) @ composed
    return {
        "loop_rotation_deg": rotation_angle_deg(error[:3, :3]),
        "loop_translation_m": float(np.linalg.norm(error[:3, 3])),
        "baseline_direct_m": float(np.linalg.norm(direct[:3, 3])),
        "baseline_composed_m": float(np.linalg.norm(composed[:3, 3])),
    }


def load_direct_baseline(camchain_path):
    data = load_yaml(camchain_path)
    cam1 = data.get("cam1", {})
    if "T_cn_cnm1" in cam1:
        return as_matrix(cam1["T_cn_cnm1"])

    # Fallback for camchains without an explicit adjacent baseline.
    t_cam0_imu = as_matrix(data["cam0"]["T_cam_imu"])
    t_cam1_imu = as_matrix(data["cam1"]["T_cam_imu"])
    return t_cam1_imu @ np.linalg.inv(t_cam0_imu)


def parse_first_matrix_after(lines, start_index):
    rows = []
    collecting = False
    for line in lines[start_index:]:
        if "[" in line:
            collecting = True
        if not collecting:
            continue
        numbers = [float(value) for value in NUMBER_RE.findall(line)]
        if numbers:
            rows.append(numbers)
        if len(rows) == 4:
            return as_matrix(rows)
    raise ValueError("could not parse 4x4 matrix")


def load_kalibr_camera_transforms(path):
    text = pathlib.Path(path).read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()
    transforms = {}
    for camera_index in [0, 1]:
        markers = [
            f"Transformation T_cam{camera_index}_imu",
            f"Transformation (cam{camera_index})",
        ]
        for index, line in enumerate(lines):
            if any(marker in line for marker in markers):
                transforms[camera_index] = parse_first_matrix_after(lines, index + 1)
                break
        if camera_index not in transforms:
            raise ValueError(f"missing Kalibr T_ci for cam{camera_index}: {path}")
    return transforms


def load_ceres_camera_transforms(path):
    data = load_yaml(path)
    transforms = {}
    for camera in data.get("camera_chain", []):
        if "camera_index" not in camera or "T_c_b" not in camera:
            continue
        transforms[int(camera["camera_index"])] = as_matrix(camera["T_c_b"])
    for camera_index in [0, 1]:
        if camera_index not in transforms:
            raise ValueError(f"missing Ceres T_c_b for camera {camera_index}: {path}")
    return transforms


def evaluate_solver(direct_baseline, transforms):
    # T_cn_cnm1 in the TUM camchain is T_cam1_cam0.  The loop compares this
    # direct stereo transform with T_cam1_imu * T_imu_cam0.
    composed = transforms[1] @ np.linalg.inv(transforms[0])
    return transform_error(direct_baseline, composed)


def read_summary(path):
    with pathlib.Path(path).open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def format_float(value):
    return f"{value:.12g}"


def main():
    parser = argparse.ArgumentParser(
        description="Compute T_cam1_cam0 vs T_cam1_imu*T_imu_cam0 loop error."
    )
    parser.add_argument("--summary", type=pathlib.Path, required=True,
                        help="run_docker_benchmark TUM summary.csv")
    parser.add_argument("--camchain", type=pathlib.Path, required=True,
                        help="TUM calibrated camchain.yaml with T_cn_cnm1")
    parser.add_argument("--out", type=pathlib.Path,
                        help="optional CSV output path")
    args = parser.parse_args()

    direct_baseline = load_direct_baseline(args.camchain)
    rows = []
    seen = set()
    for source in read_summary(args.summary):
        common = {
            "case": source.get("case", ""),
            "label": source.get("label", ""),
            "metric_direction": "T_cam1_cam0",
            "direct_source": str(args.camchain),
        }
        kalibr_result = source.get("kalibr_result", "")
        if kalibr_result:
            key = ("kalibr", source.get("case", ""), source.get("kalibr_variant", ""),
                   kalibr_result)
            if key not in seen:
                seen.add(key)
                metrics = evaluate_solver(
                    direct_baseline,
                    load_kalibr_camera_transforms(kalibr_result),
                )
                row = dict(common)
                row.update({
                    "solver": "kalibr",
                    "variant": source.get("kalibr_variant", ""),
                    "platform": source.get("kalibr_platform", ""),
                    "result": kalibr_result,
                    "reprojection_px_mean": source.get("kalibr_reproj_px", ""),
                    "gyro_rad_s_mean": source.get("kalibr_gyro_rad_s", ""),
                    "accel_m_s2_mean": source.get("kalibr_accel_m_s2", ""),
                })
                row.update({key: format_float(value)
                            for key, value in metrics.items()})
                rows.append(row)

        ceres_result = source.get("ceres_result", "")
        if ceres_result:
            key = ("ceres", source.get("case", ""), ceres_result)
            if key not in seen:
                seen.add(key)
                metrics = evaluate_solver(
                    direct_baseline,
                    load_ceres_camera_transforms(ceres_result),
                )
                row = dict(common)
                row.update({
                    "solver": "ceres",
                    "variant": "native",
                    "platform": source.get("ceres_platform", ""),
                    "result": ceres_result,
                    "reprojection_px_mean": source.get("ceres_reprojection_px_mean", ""),
                    "gyro_rad_s_mean": source.get("ceres_gyro_rad_s_mean", ""),
                    "accel_m_s2_mean": source.get("ceres_accel_m_s2_mean", ""),
                })
                row.update({key: format_float(value)
                            for key, value in metrics.items()})
                rows.append(row)

    fieldnames = [
        "case",
        "label",
        "solver",
        "variant",
        "platform",
        "metric_direction",
        "loop_rotation_deg",
        "loop_translation_m",
        "baseline_direct_m",
        "baseline_composed_m",
        "reprojection_px_mean",
        "gyro_rad_s_mean",
        "accel_m_s2_mean",
        "result",
        "direct_source",
    ]
    output = sys.stdout
    should_close = False
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        output = args.out.open("w", encoding="utf-8", newline="")
        should_close = True
    try:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    finally:
        if should_close:
            output.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
