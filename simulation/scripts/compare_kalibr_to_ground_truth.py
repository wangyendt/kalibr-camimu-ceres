#!/usr/bin/env python3
"""Compare Kalibr output YAMLs against simulator ground truth."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def _matrix4(value):
    arr = np.asarray(value, dtype=float)
    if arr.shape != (4, 4):
        raise ValueError(f"expected 4x4 matrix, got {arr.shape}")
    return arr


def _rot_deg(T_est, T_ref):
    dR = T_est[:3, :3] @ T_ref[:3, :3].T
    return float(Rotation.from_matrix(dR).magnitude() * 180.0 / math.pi)


def _trans_m(T_est, T_ref):
    return float(np.linalg.norm(T_est[:3, 3] - T_ref[:3, 3]))


def _summary_values(path):
    if not path or not path.is_file():
        return {}
    values = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        try:
            values[key] = float(value)
        except ValueError:
            values[key] = value
    return values


def compare(ground_truth_path, camchain_path, imu_path, summary_path, label):
    gt = yaml.safe_load(ground_truth_path.read_text())
    camchain = yaml.safe_load(camchain_path.read_text())
    imu_chain = yaml.safe_load(imu_path.read_text()) if imu_path else {}
    summary = _summary_values(summary_path)

    rows = []
    for idx, gt_camera in enumerate(gt.get("cameras", [])):
        key = f"cam{idx}"
        if key not in camchain:
            continue
        T_gt = _matrix4(gt_camera["T_cam_body"])
        T_est = _matrix4(camchain[key]["T_cam_imu"])
        gt_ts = float(gt_camera.get("time_shift_s", gt_camera.get("time_offset_s", 0.0)))
        est_ts = float(camchain[key].get("timeshift_cam_imu", 0.0))
        rows.append(
            {
                "label": label,
                "type": "camera",
                "index": idx,
                "rotation_deg": _rot_deg(T_est, T_gt),
                "translation_m": _trans_m(T_est, T_gt),
                "time_shift_error_s": est_ts - gt_ts,
                "estimated_time_shift_s": est_ts,
                "truth_time_shift_s": gt_ts,
            }
        )

    for idx, gt_imu in enumerate(gt.get("imus", [])):
        key = f"imu{idx}"
        if key not in imu_chain:
            continue
        T_gt = _matrix4(gt_imu["T_imu_body"])
        T_est = _matrix4(imu_chain[key]["T_i_b"])
        gt_ts = float(gt_imu.get("time_offset_s", 0.0))
        est_ts = float(imu_chain[key].get("time_offset", 0.0))
        rows.append(
            {
                "label": label,
                "type": "imu",
                "index": idx,
                "rotation_deg": _rot_deg(T_est, T_gt),
                "translation_m": _trans_m(T_est, T_gt),
                "time_offset_error_s": est_ts - gt_ts,
                "estimated_time_offset_s": est_ts,
                "truth_time_offset_s": gt_ts,
            }
        )

    for row in rows:
        row["elapsed_wall_s"] = summary.get("elapsed_wall_s")
        row["reprojection_mean_px"] = summary.get("reprojection_mean_px")
        row["gyro_mean_rad_s"] = summary.get("gyro_mean_rad_s")
        row["accel_mean_m_s2"] = summary.get("accel_mean_m_s2")
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--camchain", type=Path, required=True)
    parser.add_argument("--imu", type=Path)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--label", default="")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    rows = compare(
        args.ground_truth,
        args.camchain,
        args.imu,
        args.summary,
        args.label or args.camchain.parent.name,
    )
    if args.json:
        print(json.dumps(rows, indent=2))
        return
    keys = sorted({key for row in rows for key in row.keys()})
    print(",".join(keys))
    for row in rows:
        print(",".join(str(row.get(key, "")) for key in keys))


if __name__ == "__main__":
    main()
