#!/usr/bin/env python3
import stat
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATED_ROOT = REPO_ROOT / "simulation" / "generated"
sys.path.insert(0, str(REPO_ROOT))

from simulation.camimu_sim import run_simulation


def matrix(rotation=None, translation=None):
    T = np.eye(4)
    if rotation is not None:
        T[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    if translation is not None:
        T[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return T.tolist()


def T_sensor_body_from_lever(lever_b, rotvec=None):
    R_i_b = Rotation.from_rotvec(rotvec or [0.0, 0.0, 0.0]).as_matrix()
    t_i_b = -R_i_b @ np.asarray(lever_b, dtype=float)
    return matrix(R_i_b, t_i_b)


def T_camera_body_from_lever(lever_b, rotvec=None):
    R_c_b = Rotation.from_rotvec(rotvec or [0.0, 0.0, 0.0]).as_matrix()
    t_c_b = -R_c_b @ np.asarray(lever_b, dtype=float)
    return matrix(R_c_b, t_c_b)


def base_config(name, camera_count, imu_count):
    out = GENERATED_ROOT / name
    cameras = []
    for i in range(camera_count):
        lever = [0.12 * i, 0.015 * i, 0.0]
        cameras.append(
            {
                "name": f"cam{i}",
                "fps": 20.0 if i == 0 else 18.0,
                "T_cam_body": {"matrix": T_camera_body_from_lever(lever, [0.0, 0.0, 0.01 * i])},
                "camera_model": "pinhole",
                "distortion_model": "radtan",
                "intrinsics": [430.0, 428.0, 320.0, 240.0],
                "distortion_coeffs": [-0.035, 0.012, 0.0003, -0.0002],
                "resolution": [640, 480],
                "clock": {"offset_s": 0.0, "drift_ppm": 0.0, "jitter_std_s": 0.0},
                "observation": {
                    "pixel_noise_std_px": 0.15,
                    "rolling_shutter": {
                        "readout_time_s": 0.0015,
                        "direction": "top_to_bottom",
                        "reference_row_fraction": 0.5,
                    },
                    "exposure": {
                        "exposure_time_s": 0.001,
                        "samples": 3,
                        "blur_noise_scale": 0.15,
                        "blur_dropout_threshold_px": 4.0,
                    },
                    "position_dependent_covariance": {
                        "enabled": True,
                        "edge_multiplier": 1.4,
                        "radial_power": 2.0,
                        "u_scale": 1.0,
                        "v_scale": 1.1,
                        "correlation": 0.05,
                    },
                    "dropout_probability": 0.015,
                    "outlier_probability": 0.0,
                    "border_px": 4.0,
                    "min_visible_corners": 24,
                },
                "output_corners_csv": str(out / f"cam{i}_corners.csv"),
                "output_corner_poses_csv": str(out / f"cam{i}_corner_poses.csv"),
            }
        )

    imu_levers = [
        [0.0, 0.0, 0.0],
        [0.055, -0.025, 0.018],
        [-0.048, 0.032, -0.012],
        [0.025, 0.052, 0.024],
    ]
    imu_rotvecs = [
        [0.0, 0.0, 0.0],
        [0.012, -0.006, 0.018],
        [-0.01, 0.014, -0.009],
        [0.008, 0.011, 0.012],
    ]
    imus = []
    for i in range(imu_count):
        imus.append(
            {
                "name": f"imu{i}",
                "fps": 200.0 + 25.0 * (i % 2),
                "T_imu_body": {"matrix": T_sensor_body_from_lever(imu_levers[i], imu_rotvecs[i])},
                "clock": {"offset_s": 0.0, "drift_ppm": 0.0, "jitter_std_s": 0.0},
                "noise": {
                    "gyroscope_noise_density": 0.00008 + i * 0.00001,
                    "accelerometer_noise_density": 0.0009 + i * 0.00008,
                    "gyroscope_random_walk": 0.000002,
                    "accelerometer_random_walk": 0.00002,
                    "gyro_quantization_rad_s": 0.0,
                    "accel_quantization_m_s2": 0.0,
                },
                "deterministic": {
                    "gyro_bias_rad_s": [0.0005 * (i + 1), -0.0003 * i, 0.0002 * (i + 1)],
                    "accel_bias_m_s2": [0.006 * i, -0.004 * (i + 1), 0.003 * i],
                    "gyro_scale_misalignment": [
                        [1.0 + 0.0008 * i, 0.0002 * i, 0.0],
                        [-0.0001 * i, 1.0 - 0.0006 * i, 0.0001 * i],
                        [0.0, -0.00015 * i, 1.0 + 0.0005 * i],
                    ],
                    "accel_scale_misalignment": [
                        [1.0 - 0.001 * i, 0.00015 * i, 0.0],
                        [0.0001 * i, 1.0 + 0.0007 * i, -0.00012 * i],
                        [0.0, 0.0002 * i, 1.0 - 0.0004 * i],
                    ],
                    "gyro_accel_sensitivity": [
                        [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0],
                    ],
                },
                "output_imu_csv": str(out / f"imu{i}.csv"),
                "output_imu_yaml": str(out / f"imu{i}.yaml"),
            }
        )

    return {
        "metadata": {
            "name": name,
            "random_seed": 20260621 + camera_count * 10 + imu_count,
            "description": f"{camera_count} camera(s), {imu_count} IMU(s) synthetic cam-IMU calibration dataset",
        },
        "paths": {
            "output_dir": str(out),
            "target_points_csv": str(out / "target_points.csv"),
            "camchain_yaml": str(out / "camchain.yaml"),
            "target_yaml": str(out / "aprilgrid.yaml"),
            "trajectory_keyframes_csv": str(out / "trajectory_keyframes.csv"),
            "sensor_poses_csv": str(out / "sensor_poses.csv"),
            "ground_truth_yaml": str(out / "ground_truth.yaml"),
            "manifest_json": str(out / "manifest.json"),
        },
        "simulation": {
            "start_s": 0.0,
            "end_s": 10.0,
            "gravity_m_s2": [0.0, 0.0, -9.80665],
        },
        "trajectory": {
            "type": "analytic",
            "rotation_interpolation": "rotation_spline",
            "duration_s": 10.0,
            "keyframe_hz": 100.0,
            "radius_m": 0.95,
            "lateral_m": 0.26,
            "vertical_m": 0.58,
            "vertical_amp_m": 0.14,
            "roll_amp_deg": 20.0,
            "yaw_cycles": 1.35,
            "pitch_cycles": 1.8,
        },
        "targets": [
            {
                "name": "target0",
                "aprilgrid": {
                    "tag_cols": 6,
                    "tag_rows": 6,
                    "tag_size_m": 0.036,
                    "tag_spacing_ratio": 0.3,
                },
                "T_target0_target": {"matrix": matrix()},
                "corner_noise": {"frame": "target"},
            },
            {
                "name": "target1",
                "aprilgrid": {
                    "tag_cols": 4,
                    "tag_rows": 4,
                    "tag_size_m": 0.036,
                    "tag_spacing_ratio": 0.3,
                },
                "T_target0_target": {
                    "matrix": matrix(
                        Rotation.from_euler("xyz", [0.0, -6.0, 8.0], degrees=True).as_matrix(),
                        [0.34, 0.03, 0.035],
                    )
                },
                "corner_id_offset": 100000,
                "corner_noise": {"frame": "target"},
            },
        ],
        "cameras": cameras,
        "imus": imus,
        "outputs": {"sensor_pose_hz": 20.0},
    }


def calibration_command(manifest):
    build = REPO_ROOT / "build" / "calibrate_cam_imu"
    parts = [
        str(build),
        "--corner-defaults",
        "--cam",
        manifest["camchain_yaml"],
        "--target",
        manifest["target_yaml"],
    ]
    for imu in manifest["imus"]:
        parts.extend(["--imu", imu["imu_yaml"], "--imu-data", imu["imu_csv"]])
    for camera in manifest["cameras"]:
        parts.extend(["--corners", camera["corners_csv"]])
    if manifest["cameras"]:
        parts.extend(["--corner-poses", manifest["cameras"][0]["corner_poses_csv"]])
    output_result = Path(manifest["output_dir"]) / "calibration_result.yaml"
    parts.extend(
        [
            "--init-from-camchain",
            "--estimate-time-shift-prior",
            "--estimate-orientation-gravity-prior",
            "--pose-fit-motion-lambda",
            "0.0001",
            "--pose-fit-boundary-anchors",
            "--imu-trim-edge-count",
            "0",
            "--time-shift-prior-sigma",
            "0.0001",
            "--pose-motion-prior",
            "--pose-motion-translation-variance",
            "10",
            "--pose-motion-rotation-variance",
            "1",
            "--max-iterations",
            "30",
            "--output-result",
            str(output_result),
        ]
    )
    if len(manifest["cameras"]) > 1:
        parts.append("--fix-camera-chain-extrinsics")
    return parts


def write_shell_command(path, command):
    quoted = []
    for part in command:
        if all(ch.isalnum() or ch in "._/:-" for ch in part):
            quoted.append(part)
        else:
            quoted.append("'" + part.replace("'", "'\\''") + "'")
    text = "#!/usr/bin/env bash\nset -euo pipefail\n\n" + " \\\n  ".join(quoted) + ' \\\n  "$@"\n'
    path.write_text(text)
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR)


def main():
    GENERATED_ROOT.mkdir(parents=True, exist_ok=True)
    scenarios = [
        ("one_cam_one_imu", 1, 1),
        ("one_cam_four_imus", 1, 4),
        ("two_cams_two_imus", 2, 2),
    ]
    for name, cams, imus in scenarios:
        cfg = base_config(name, cams, imus)
        out = Path(cfg["paths"]["output_dir"])
        out.mkdir(parents=True, exist_ok=True)
        cfg_path = out / "sim_config.yaml"
        cfg_path.write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
        manifest = run_simulation(cfg_path)
        cmd = calibration_command(manifest)
        write_shell_command(out / "run_calibration.sh", cmd)
        print(f"{name}: {manifest['output_dir']}")


if __name__ == "__main__":
    main()
