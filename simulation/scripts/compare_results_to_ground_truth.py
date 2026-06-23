#!/usr/bin/env python3
"""Compare calibration result YAML against simulator ground truth."""

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


def _ceres_rotvec_to_matrix(rotvec):
    # Ceres core/so3.h stores rotation vectors with Exp(-r) convention.
    return Rotation.from_rotvec(-np.asarray(rotvec, dtype=float)).as_matrix()


def _items_by_index(items, key):
    return {int(item[key]): item for item in items or []}


def compare(ground_truth_path, result_path, label):
    with open(ground_truth_path) as f:
        gt = yaml.safe_load(f)
    with open(result_path) as f:
        result = yaml.safe_load(f)

    rows = []
    gt_cameras = gt.get("cameras", [])
    result_cameras = result.get("camera_chain")
    if not result_cameras:
        result_cameras = [{"camera_index": 0, "T_c_b": result["camera_to_body"]["T_c_b"], "time_shift_s": result.get("time_shift_s", 0.0)}]
    result_cameras = _items_by_index(result_cameras, "camera_index")
    for idx, gt_camera in enumerate(gt_cameras):
        if idx not in result_cameras:
            continue
        T_gt = _matrix4(gt_camera["T_cam_body"])
        T_est = _matrix4(result_cameras[idx]["T_c_b"])
        gt_ts = float(gt_camera.get("time_shift_s", gt_camera.get("time_offset_s", 0.0)))
        est_ts = float(result_cameras[idx].get("time_shift_s", result.get("time_shift_s", 0.0)))
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

    gt_imus = gt.get("imus", [])
    result_imus = result.get("imu_chain")
    if not result_imus:
        result_imus = [{"imu_index": 0, "r_b": result.get("imu_extrinsic", {}).get("r_b", [0, 0, 0]), "r_i_b": result.get("imu_extrinsic", {}).get("r_i_b", [0, 0, 0]), "time_offset_s": 0.0}]
    result_imus = _items_by_index(result_imus, "imu_index")
    for idx, gt_imu in enumerate(gt_imus):
        if idx not in result_imus:
            continue
        T_gt = _matrix4(gt_imu["T_imu_body"])
        r_b_gt = np.asarray(gt_imu.get("r_b"), dtype=float)
        if r_b_gt.shape != (3,):
            r_b_gt = -T_gt[:3, :3].T @ T_gt[:3, 3]
        R_gt = T_gt[:3, :3]
        r_b_est = np.asarray(result_imus[idx].get("r_b", [0, 0, 0]), dtype=float)
        r_i_b_est = np.asarray(result_imus[idx].get("r_i_b", [0, 0, 0]), dtype=float)
        R_est = _ceres_rotvec_to_matrix(r_i_b_est)
        gt_ts = float(gt_imu.get("time_offset_s", 0.0))
        est_ts = float(result_imus[idx].get("time_offset_s", 0.0))
        rows.append(
            {
                "label": label,
                "type": "imu",
                "index": idx,
                "rotation_deg": float((Rotation.from_matrix(R_est @ R_gt.T).magnitude() * 180.0 / math.pi)),
                "lever_m": float(np.linalg.norm(r_b_est - r_b_gt)),
                "time_offset_error_s": est_ts - gt_ts,
                "estimated_time_offset_s": est_ts,
                "truth_time_offset_s": gt_ts,
            }
        )

    residuals = result.get("residual_statistics", {})
    for row in rows:
        row["reprojection_rms_px"] = residuals.get("reprojection_px", {}).get("rms")
        row["gyro_rms_rad_s"] = residuals.get("gyro_rad_s", {}).get("rms")
        row["accel_rms_m_s2"] = residuals.get("accel_m_s2", {}).get("rms")
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--label", default="")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    rows = compare(args.ground_truth, args.result, args.label or args.result.parent.name)
    if args.json:
        print(json.dumps(rows, indent=2))
        return
    keys = sorted({key for row in rows for key in row.keys()})
    print(",".join(keys))
    for row in rows:
        print(",".join(str(row.get(key, "")) for key in keys))


if __name__ == "__main__":
    main()
