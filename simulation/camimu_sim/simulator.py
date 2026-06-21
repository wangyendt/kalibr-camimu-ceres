import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping, Optional, Union

import numpy as np
import yaml

from .se3 import (
    invert_transform,
    mat3,
    matrix_rows,
    parse_transform,
    perturb_transform,
    transform_matrix,
    transform_points,
    vec3,
)
from .trajectory import CubicPoseTrajectory, load_trajectory


def _abs_path(path: Union[str, Path], base_dir: Path) -> Path:
    p = Path(path).expanduser()
    if not p.is_absolute():
        p = (base_dir / p).resolve()
    return p


def _write_yaml(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def _time_grid(start_s: float, end_s: float, fps: float) -> np.ndarray:
    if fps <= 0.0:
        raise ValueError("fps must be positive")
    count = int(math.floor((end_s - start_s) * fps)) + 1
    return start_s + np.arange(count, dtype=float) / fps


def _sensor_timestamp(true_t: float, start_s: float, clock: Mapping[str, Any], rng: np.random.Generator) -> float:
    offset = float(clock.get("offset_s", 0.0))
    drift = float(clock.get("drift_ppm", 0.0)) * 1e-6
    jitter = float(clock.get("jitter_std_s", 0.0))
    return true_t + offset + drift * (true_t - start_s) + rng.normal(0.0, jitter)


def _timestamp_ns(timestamp_s: float) -> int:
    return int(round(timestamp_s * 1e9))


def aprilgrid_points(grid: Mapping[str, Any]) -> np.ndarray:
    tag_cols = int(grid.get("tag_cols", grid.get("tagCols", 6)))
    tag_rows = int(grid.get("tag_rows", grid.get("tagRows", 6)))
    tag_size = float(grid.get("tag_size_m", grid.get("tagSize", 0.036)))
    spacing = float(grid.get("tag_spacing_ratio", grid.get("tagSpacing", 0.3)))
    step = tag_size * (1.0 + spacing)
    points = []
    for row in range(tag_rows):
        for col in range(tag_cols):
            x0 = col * step
            y0 = row * step
            points.extend(
                [
                    [x0, y0, 0.0],
                    [x0 + tag_size, y0, 0.0],
                    [x0 + tag_size, y0 + tag_size, 0.0],
                    [x0, y0 + tag_size, 0.0],
                ]
            )
    return np.asarray(points, dtype=float)


def _load_noise_matrix(node: Mapping[str, Any], n: int, base_dir: Path) -> np.ndarray:
    if not node:
        return np.zeros((n, 3))
    if "matrix" in node:
        arr = np.asarray(node["matrix"], dtype=float)
    elif "path" in node:
        path = _abs_path(node["path"], base_dir)
        if path.suffix == ".npy":
            arr = np.load(path)
        elif path.suffix in {".yaml", ".yml"}:
            with path.open() as f:
                arr = np.asarray(yaml.safe_load(f), dtype=float)
        else:
            arr = np.loadtxt(path, delimiter=",")
    else:
        arr = np.zeros((n, 3))
    arr = np.asarray(arr, dtype=float)
    if arr.size == 0:
        return np.zeros((n, 3))
    if arr.shape != (n, 3):
        raise ValueError(f"corner noise matrix shape must be {(n, 3)}, got {arr.shape}")
    return arr


def build_targets(config: Mapping[str, Any], base_dir: Path, rng: np.random.Generator) -> list[dict[str, Any]]:
    targets = []
    for target_index, node in enumerate(config.get("targets", [])):
        grid = node.get("aprilgrid", {})
        local_ideal = aprilgrid_points(grid)
        noise_node = node.get("corner_noise", {})
        noise = _load_noise_matrix(noise_node, len(local_ideal), base_dir)
        frame = str(noise_node.get("frame", "target")).lower()
        T_target0_target = parse_transform(node.get("T_target0_target", node.get("T_target_target0")))
        T_target0_target = perturb_transform(T_target0_target, node.get("pose_perturbation"), rng)
        if frame == "target0":
            world_points = transform_points(T_target0_target, local_ideal) + noise
        else:
            world_points = transform_points(T_target0_target, local_ideal + noise)
        id_offset = int(node.get("corner_id_offset", target_index * 100000))
        targets.append(
            {
                "name": node.get("name", f"target{target_index}"),
                "index": target_index,
                "grid": grid,
                "T_target0_target": T_target0_target,
                "local_ideal": local_ideal,
                "noise": noise,
                "world_points": world_points,
                "corner_ids": np.arange(len(local_ideal), dtype=int) + id_offset,
            }
        )
    if not targets:
        raise ValueError("simulation config must contain at least one target")
    return targets


def target_center(targets: list[dict[str, Any]]) -> np.ndarray:
    all_points = np.vstack([t["world_points"] for t in targets])
    return np.mean(all_points, axis=0)


def _camera_model_values(camera: Mapping[str, Any]) -> tuple[str, str, list[float], list[float], int, int]:
    model = str(camera.get("camera_model", "pinhole")).lower()
    distortion_model = str(camera.get("distortion_model", "radtan")).lower()
    intrinsics = [float(v) for v in camera.get("intrinsics", [420.0, 420.0, 320.0, 240.0])]
    distortion = [float(v) for v in camera.get("distortion_coeffs", [0.0, 0.0, 0.0, 0.0])]
    resolution = [int(v) for v in camera.get("resolution", [640, 480])]
    if model == "pinhole":
        fx, fy, cx, cy = intrinsics[:4]
    elif model == "omni":
        _, fx, fy, cx, cy = intrinsics[:5]
    elif model in {"eucm", "ds", "double-sphere", "double_sphere"}:
        fx, fy, cx, cy = intrinsics[2:6]
    else:
        raise ValueError(f"unsupported camera model: {model}")
    return model, distortion_model, intrinsics, distortion, resolution[0], resolution[1]


def _distort_normalized(y: np.ndarray, distortion_model: str, coeffs: list[float]) -> np.ndarray:
    x, v = float(y[0]), float(y[1])
    if distortion_model in {"", "none"}:
        return y
    if distortion_model == "radtan":
        k1, k2, p1, p2 = (coeffs + [0.0, 0.0, 0.0, 0.0])[:4]
        r2 = x * x + v * v
        r4 = r2 * r2
        radial = k1 * r2 + k2 * r4
        return np.array(
            [
                x + x * radial + 2.0 * p1 * x * v + p2 * (r2 + 2.0 * x * x),
                v + v * radial + 2.0 * p2 * x * v + p1 * (r2 + 2.0 * v * v),
            ]
        )
    if distortion_model in {"equidistant", "equi"}:
        k1, k2, k3, k4 = (coeffs + [0.0, 0.0, 0.0, 0.0])[:4]
        r = math.sqrt(x * x + v * v)
        if r <= 1e-8:
            return y
        theta = math.atan(r)
        theta2 = theta * theta
        theta_d = theta * (1.0 + k1 * theta2 + k2 * theta2**2 + k3 * theta2**3 + k4 * theta2**4)
        return (theta_d / r) * y
    if distortion_model == "fov":
        w = coeffs[0] if coeffs else 0.0
        r = float(np.linalg.norm(y))
        if w * w < 1e-5 or r * r < 1e-5:
            return y
        scale = math.atan(2.0 * math.tan(0.5 * w) * r) / (r * w)
        return scale * y
    raise ValueError(f"unsupported distortion model: {distortion_model}")


def project_point(camera: Mapping[str, Any], p_c: np.ndarray) -> Optional[np.ndarray]:
    model, distortion_model, intrinsics, distortion, _, _ = _camera_model_values(camera)
    x, y, z = [float(v) for v in p_c]
    eps = 1e-12
    if model == "pinhole":
        if abs(z) <= eps or z <= 0.0:
            return None
        yn = np.array([x / z, y / z])
        fx, fy, cx, cy = intrinsics[:4]
    elif model == "omni":
        xi, fx, fy, cx, cy = intrinsics[:5]
        d = float(np.linalg.norm(p_c))
        denom = z + xi * d
        if d <= eps or abs(denom) <= eps:
            return None
        yn = np.array([x / denom, y / denom])
    elif model == "eucm":
        alpha, beta, fx, fy, cx, cy = intrinsics[:6]
        d2 = beta * (x * x + y * y) + z * z
        if d2 <= eps:
            return None
        d = math.sqrt(d2)
        denom = alpha * d + (1.0 - alpha) * z
        if abs(denom) <= eps:
            return None
        yn = np.array([x / denom, y / denom])
    elif model in {"ds", "double-sphere", "double_sphere"}:
        xi, alpha, fx, fy, cx, cy = intrinsics[:6]
        r2 = x * x + y * y
        d1 = math.sqrt(r2 + z * z)
        k = xi * d1 + z
        d2 = math.sqrt(r2 + k * k)
        denom = alpha * d2 + (1.0 - alpha) * k
        if abs(denom) <= eps:
            return None
        yn = np.array([x / denom, y / denom])
    else:
        raise ValueError(f"unsupported camera model: {model}")
    yd = _distort_normalized(yn, distortion_model, distortion)
    return np.array([fx * yd[0] + cx, fy * yd[1] + cy])


def _pixel_noise_sigma(
    pixel: np.ndarray,
    observation: Mapping[str, Any],
    width: int,
    height: int,
    camera: Mapping[str, Any],
) -> tuple[float, float, float]:
    base = float(observation.get("pixel_noise_std_px", 0.0))
    cov = observation.get("position_dependent_covariance", {})
    if not cov.get("enabled", False):
        return base, base, 0.0

    model, _, intrinsics, _, _, _ = _camera_model_values(camera)
    if model == "pinhole":
        default_center = [intrinsics[2], intrinsics[3]]
    elif model == "omni":
        default_center = [intrinsics[3], intrinsics[4]]
    else:
        default_center = [intrinsics[-2], intrinsics[-1]]
    center = np.asarray(cov.get("center_px", default_center), dtype=float).reshape(2)
    half_diag = max(1e-12, 0.5 * math.sqrt(width * width + height * height))
    radial = min(1.0, float(np.linalg.norm(np.asarray(pixel) - center) / half_diag))
    edge_multiplier = float(cov.get("edge_multiplier", 1.0))
    radial_power = float(cov.get("radial_power", 2.0))
    scale = 1.0 + (edge_multiplier - 1.0) * (radial ** radial_power)
    sigma_u = float(cov.get("base_std_px", base)) * scale * float(cov.get("u_scale", 1.0))
    sigma_v = float(cov.get("base_std_px", base)) * scale * float(cov.get("v_scale", 1.0))
    rho = float(cov.get("correlation", 0.0))
    rho = max(-0.999, min(0.999, rho))
    return sigma_u, sigma_v, rho


def _sample_corner_noise(
    rng: np.random.Generator,
    sigma_u: float,
    sigma_v: float,
    rho: float,
) -> np.ndarray:
    if sigma_u <= 0.0 and sigma_v <= 0.0:
        return np.zeros(2)
    cov = np.array(
        [
            [sigma_u * sigma_u, rho * sigma_u * sigma_v],
            [rho * sigma_u * sigma_v, sigma_v * sigma_v],
        ]
    )
    return rng.multivariate_normal(np.zeros(2), cov)


def _rolling_shutter_offset_s(
    pixel_y: float,
    observation: Mapping[str, Any],
    height: int,
) -> float:
    rs = observation.get("rolling_shutter", {})
    readout = float(rs.get("readout_time_s", 0.0))
    if readout == 0.0:
        return 0.0
    direction = str(rs.get("direction", "top_to_bottom")).lower()
    row_fraction = float(pixel_y) / max(1.0, float(height - 1))
    if direction in {"bottom_to_top", "bottom-up"}:
        row_fraction = 1.0 - row_fraction
    reference = float(rs.get("reference_row_fraction", 0.5))
    return (row_fraction - reference) * readout


def _exposure_offsets_s(observation: Mapping[str, Any]) -> np.ndarray:
    exposure = observation.get("exposure", {})
    exposure_time = float(exposure.get("exposure_time_s", 0.0))
    samples = int(exposure.get("samples", 1))
    if exposure_time <= 0.0 or samples <= 1:
        return np.array([0.0])
    return np.linspace(-0.5 * exposure_time, 0.5 * exposure_time, samples)


def _project_world_point_at(
    camera: Mapping[str, Any],
    trajectory: CubicPoseTrajectory,
    timestamp_s: float,
    point_w: np.ndarray,
) -> Optional[np.ndarray]:
    t_w_b, R_w_b, _ = trajectory.pose(timestamp_s)
    p_b = R_w_b.T @ (point_w - t_w_b)
    p_c = camera["_T_cam_body"][:3, :3] @ p_b + camera["_T_cam_body"][:3, 3]
    return project_point(camera, p_c)


def _write_target_points(path: Path, targets: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "target_index",
                "target_name",
                "corner_id",
                "local_corner_id",
                "x_target0_m",
                "y_target0_m",
                "z_target0_m",
                "noise_x_m",
                "noise_y_m",
                "noise_z_m",
            ]
        )
        for target in targets:
            for local_id, corner_id in enumerate(target["corner_ids"]):
                p = target["world_points"][local_id]
                n = target["noise"][local_id]
                writer.writerow([target["index"], target["name"], int(corner_id), local_id, *p, *n])


def _write_camchain(path: Path, cameras: list[Mapping[str, Any]]) -> None:
    lines = []
    for idx, camera in enumerate(cameras):
        model, distortion_model, intrinsics, distortion, width, height = _camera_model_values(camera)
        lines.append(f"cam{idx}:")
        lines.append(f"  rostopic: {camera.get('rostopic', f'/cam{idx}/image_raw')}")
        lines.append(f"  camera_model: {model}")
        lines.append(f"  distortion_model: {distortion_model}")
        lines.append(f"  intrinsics: {intrinsics}")
        lines.append(f"  distortion_coeffs: {distortion}")
        lines.append(f"  resolution: [{width}, {height}]")
        lines.append("  T_cam_imu:")
        for row in matrix_rows(camera["_T_cam_body"]):
            lines.append("    - [" + ", ".join(f"{v:.12g}" for v in row) + "]")
        lines.append(f"  timeshift_cam_imu: {float(camera.get('time_offset_s', 0.0)):.12g}")
        if idx > 0:
            T_ci_cprev = camera["_T_cam_body"] @ invert_transform(cameras[idx - 1]["_T_cam_body"])
            lines.append("  T_cn_cnm1:")
            for row in matrix_rows(T_ci_cprev):
                lines.append("    - [" + ", ".join(f"{v:.12g}" for v in row) + "]")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def _write_target_yaml(path: Path, target0: dict[str, Any]) -> None:
    grid = target0["grid"]
    data = {
        "target_type": "aprilgrid",
        "tagCols": int(grid.get("tag_cols", grid.get("tagCols", 6))),
        "tagRows": int(grid.get("tag_rows", grid.get("tagRows", 6))),
        "tagSize": float(grid.get("tag_size_m", grid.get("tagSize", 0.036))),
        "tagSpacing": float(grid.get("tag_spacing_ratio", grid.get("tagSpacing", 0.3))),
    }
    _write_yaml(path, data)


def _write_imu_yaml(path: Path, imu: Mapping[str, Any]) -> None:
    noise = imu.get("noise", {})
    data = {
        "rostopic": f"/{imu.get('name', 'imu')}",
        "update_rate": float(imu.get("fps", imu.get("update_rate_hz", 200.0))),
        "accelerometer_noise_density": float(noise.get("accelerometer_noise_density", noise.get("accel_noise_density", 0.0))),
        "accelerometer_random_walk": float(noise.get("accelerometer_random_walk", noise.get("accel_random_walk", 0.0))),
        "gyroscope_noise_density": float(noise.get("gyroscope_noise_density", noise.get("gyro_noise_density", 0.0))),
        "gyroscope_random_walk": float(noise.get("gyroscope_random_walk", noise.get("gyro_random_walk", 0.0))),
    }
    _write_yaml(path, data)


def _simulate_camera(
    camera: Mapping[str, Any],
    trajectory: CubicPoseTrajectory,
    targets: list[dict[str, Any]],
    start_s: float,
    end_s: float,
    rng: np.random.Generator,
    corner_csv: Path,
    pose_csv: Path,
) -> dict[str, Any]:
    model, _, _, _, width, height = _camera_model_values(camera)
    observation = camera.get("observation", {})
    fps = float(camera.get("fps", 20.0))
    border = float(observation.get("border_px", 2.0))
    min_corners = int(observation.get("min_visible_corners", 12))
    pixel_std = float(observation.get("pixel_noise_std_px", 0.0))
    dropout = float(observation.get("dropout_probability", 0.0))
    outlier = float(observation.get("outlier_probability", 0.0))
    clock = camera.get("clock", {})
    T_cam_body = camera["_T_cam_body"]
    times = _time_grid(start_s, end_s, fps)
    corner_csv.parent.mkdir(parents=True, exist_ok=True)
    pose_csv.parent.mkdir(parents=True, exist_ok=True)
    frames = 0
    corners = 0
    with corner_csv.open("w", newline="") as corner_file, pose_csv.open("w", newline="") as pose_file:
        corner_writer = csv.writer(corner_file)
        pose_writer = csv.writer(pose_file)
        corner_writer.writerow(
            [
                "timestamp_ns",
                "corner_id",
                "u_px",
                "v_px",
                "target_x_m",
                "target_y_m",
                "target_z_m",
                "target_index",
                "local_corner_id",
                "corner_time_offset_s",
                "sigma_u_px",
                "sigma_v_px",
                "cov_uv_px2",
                "blur_length_px",
            ]
        )
        pose_writer.writerow(["timestamp_ns", *[f"T{r}{c}" for r in range(4) for c in range(4)]])
        for true_t in times:
            state = trajectory.evaluate(true_t)
            rows = []
            for target in targets:
                for local_id, point_w in enumerate(target["world_points"]):
                    pixel0 = _project_world_point_at(camera, trajectory, true_t, point_w)
                    if pixel0 is None:
                        continue
                    corner_offset_s = _rolling_shutter_offset_s(pixel0[1], observation, height)
                    corner_time_s = true_t + corner_offset_s
                    exposure_offsets = _exposure_offsets_s(observation)
                    exposure_pixels = []
                    for exposure_offset_s in exposure_offsets:
                        exposure_pixel = _project_world_point_at(
                            camera, trajectory, corner_time_s + float(exposure_offset_s), point_w
                        )
                        if exposure_pixel is None:
                            exposure_pixels = []
                            break
                        exposure_pixels.append(exposure_pixel)
                    if not exposure_pixels:
                        continue
                    pixel = np.mean(np.asarray(exposure_pixels), axis=0)
                    if pixel is None:
                        continue
                    if not (border <= pixel[0] < width - border and border <= pixel[1] < height - border):
                        continue
                    if rng.random() < dropout:
                        continue
                    blur_length = 0.0
                    if len(exposure_pixels) > 1:
                        exposure_array = np.asarray(exposure_pixels)
                        blur_length = float(np.max(np.linalg.norm(exposure_array - pixel, axis=1)) * 2.0)
                    blur = observation.get("exposure", {})
                    blur_noise_scale = float(blur.get("blur_noise_scale", 0.0))
                    blur_dropout_threshold = float(blur.get("blur_dropout_threshold_px", 0.0))
                    if blur_dropout_threshold > 0.0 and blur_length > blur_dropout_threshold:
                        continue
                    sigma_u, sigma_v, rho = _pixel_noise_sigma(pixel, observation, width, height, camera)
                    blur_sigma = blur_noise_scale * blur_length
                    sigma_u = math.sqrt(sigma_u * sigma_u + blur_sigma * blur_sigma)
                    sigma_v = math.sqrt(sigma_v * sigma_v + blur_sigma * blur_sigma)
                    pixel = pixel + _sample_corner_noise(rng, sigma_u, sigma_v, rho)
                    if rng.random() < outlier:
                        pixel = np.array([rng.uniform(border, width - border), rng.uniform(border, height - border)])
                    cov_uv = rho * sigma_u * sigma_v
                    rows.append(
                        [
                            int(target["corner_ids"][local_id]),
                            float(pixel[0]),
                            float(pixel[1]),
                            float(point_w[0]),
                            float(point_w[1]),
                            float(point_w[2]),
                            target["index"],
                            local_id,
                            float(corner_offset_s),
                            float(sigma_u),
                            float(sigma_v),
                            float(cov_uv),
                            float(blur_length),
                        ]
                    )
            if len(rows) < min_corners:
                continue
            stamp_s = _sensor_timestamp(true_t, start_s, clock, rng)
            stamp_ns = _timestamp_ns(stamp_s)
            for row in rows:
                corner_writer.writerow([stamp_ns, *row])
            T_w_body = transform_matrix(state.R_w_b, state.t_w_b)
            T_w_cam = T_w_body @ invert_transform(T_cam_body)
            pose_writer.writerow([stamp_ns, *[v for row in matrix_rows(T_w_cam) for v in row]])
            frames += 1
            corners += len(rows)
    return {"camera": camera.get("name", "camera"), "model": model, "frames": frames, "corners": corners}


def _common_lever_accel(omega_b: np.ndarray, alpha_b: np.ndarray, r_b: np.ndarray) -> np.ndarray:
    return np.cross(alpha_b, r_b) + np.cross(omega_b, np.cross(omega_b, r_b))


def _simulate_imu(
    imu: Mapping[str, Any],
    trajectory: CubicPoseTrajectory,
    gravity_w: np.ndarray,
    start_s: float,
    end_s: float,
    rng: np.random.Generator,
    imu_csv: Path,
) -> dict[str, Any]:
    fps = float(imu.get("fps", imu.get("update_rate_hz", 200.0)))
    dt = 1.0 / fps
    noise = imu.get("noise", {})
    allan = noise.get("allan", {})
    deterministic = imu.get("deterministic", {})
    clock = imu.get("clock", {})
    T_i_b = imu["_T_imu_body"]
    R_i_b = T_i_b[:3, :3]
    t_i_b = T_i_b[:3, 3]
    r_b = -R_i_b.T @ t_i_b
    R_b_i = R_i_b.T
    M_gyro = mat3(deterministic.get("gyro_scale_misalignment"), default_identity=True)
    M_accel = mat3(deterministic.get("accel_scale_misalignment"), default_identity=True)
    A_g = mat3(deterministic.get("gyro_accel_sensitivity"), default_identity=False)
    R_gyro_i = mat3(deterministic.get("gyro_sensing_rotation"), default_identity=True)
    gyro_bias = vec3(deterministic.get("gyro_bias_rad_s", [0.0, 0.0, 0.0]))
    accel_bias = vec3(deterministic.get("accel_bias_m_s2", [0.0, 0.0, 0.0]))
    rx_i = vec3(deterministic.get("accel_axis_rx_i", [0.0, 0.0, 0.0]))
    ry_i = vec3(deterministic.get("accel_axis_ry_i", [0.0, 0.0, 0.0]))
    rz_i = vec3(deterministic.get("accel_axis_rz_i", [0.0, 0.0, 0.0]))
    use_size_effect = bool(deterministic.get("use_accel_size_effect", False))
    gyro_density = float(noise.get("gyroscope_noise_density", noise.get("gyro_noise_density", allan.get("gyroscope_noise_density", allan.get("gyro_noise_density", 0.0)))))
    accel_density = float(noise.get("accelerometer_noise_density", noise.get("accel_noise_density", allan.get("accelerometer_noise_density", allan.get("accel_noise_density", 0.0)))))
    gyro_rw = float(noise.get("gyroscope_random_walk", noise.get("gyro_random_walk", allan.get("gyroscope_random_walk", allan.get("gyro_random_walk", 0.0)))))
    accel_rw = float(noise.get("accelerometer_random_walk", noise.get("accel_random_walk", allan.get("accelerometer_random_walk", allan.get("accel_random_walk", 0.0)))))
    gyro_white_std = gyro_density * math.sqrt(fps) + float(noise.get("gyro_white_noise_std_rad_s", 0.0))
    accel_white_std = accel_density * math.sqrt(fps) + float(noise.get("accel_white_noise_std_m_s2", 0.0))
    gyro_quant = float(noise.get("gyro_quantization_rad_s", 0.0))
    accel_quant = float(noise.get("accel_quantization_m_s2", 0.0))
    gyro_sat = float(noise.get("gyro_saturation_rad_s", 0.0))
    accel_sat = float(noise.get("accel_saturation_m_s2", 0.0))
    times = _time_grid(start_s, end_s, fps)
    imu_csv.parent.mkdir(parents=True, exist_ok=True)
    with imu_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["#timestamp_ns", "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s", "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2"])
        for true_t in times:
            state = trajectory.evaluate(true_t)
            h_b = state.R_w_b.T @ (state.a_w - gravity_w)
            lever = _common_lever_accel(state.omega_b, state.alpha_b, r_b)
            a_b = h_b + lever
            if gyro_rw > 0.0:
                gyro_bias += rng.normal(0.0, gyro_rw * math.sqrt(dt), 3)
            if accel_rw > 0.0:
                accel_bias += rng.normal(0.0, accel_rw * math.sqrt(dt), 3)
            if use_size_effect:
                rx_b = r_b + R_b_i @ rx_i
                ry_b = r_b + R_b_i @ ry_i
                rz_b = r_b + R_b_i @ rz_i
                axis_specific = R_i_b @ h_b
                axis_specific[0] += (R_i_b @ _common_lever_accel(state.omega_b, state.alpha_b, rx_b))[0]
                axis_specific[1] += (R_i_b @ _common_lever_accel(state.omega_b, state.alpha_b, ry_b))[1]
                axis_specific[2] += (R_i_b @ _common_lever_accel(state.omega_b, state.alpha_b, rz_b))[2]
                accel = M_accel @ axis_specific + accel_bias
            else:
                accel = M_accel @ (R_i_b @ a_b) + accel_bias
            R_gyro_b = R_gyro_i @ R_i_b
            gyro = M_gyro @ (R_gyro_b @ state.omega_b) + A_g @ (R_gyro_b @ a_b) + gyro_bias
            if gyro_white_std > 0.0:
                gyro += rng.normal(0.0, gyro_white_std, 3)
            if accel_white_std > 0.0:
                accel += rng.normal(0.0, accel_white_std, 3)
            if gyro_quant > 0.0:
                gyro = np.round(gyro / gyro_quant) * gyro_quant
            if accel_quant > 0.0:
                accel = np.round(accel / accel_quant) * accel_quant
            if gyro_sat > 0.0:
                gyro = np.clip(gyro, -gyro_sat, gyro_sat)
            if accel_sat > 0.0:
                accel = np.clip(accel, -accel_sat, accel_sat)
            stamp_s = _sensor_timestamp(true_t, start_s, clock, rng)
            writer.writerow([_timestamp_ns(stamp_s), *gyro.tolist(), *accel.tolist()])
    return {"imu": imu.get("name", "imu"), "samples": len(times)}


def _write_sensor_poses(
    path: Path,
    trajectory: CubicPoseTrajectory,
    cameras: list[Mapping[str, Any]],
    imus: list[Mapping[str, Any]],
    start_s: float,
    end_s: float,
    fps: float,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    times = _time_grid(start_s, end_s, fps)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns", "sensor_type", "sensor_name", *[f"T{r}{c}" for r in range(4) for c in range(4)]])
        for t in times:
            state = trajectory.evaluate(t)
            T_w_b = transform_matrix(state.R_w_b, state.t_w_b)
            for camera in cameras:
                T_w_cam = T_w_b @ invert_transform(camera["_T_cam_body"])
                writer.writerow([_timestamp_ns(t), "camera", camera.get("name", "camera"), *[v for row in matrix_rows(T_w_cam) for v in row]])
            for imu in imus:
                T_w_imu = T_w_b @ invert_transform(imu["_T_imu_body"])
                writer.writerow([_timestamp_ns(t), "imu", imu.get("name", "imu"), *[v for row in matrix_rows(T_w_imu) for v in row]])


def run_simulation(
    config_path: Union[str, Path],
    output_dir_override: Optional[Union[str, Path]] = None,
) -> dict[str, Any]:
    config_path = Path(config_path).expanduser().resolve()
    with config_path.open() as f:
        config = yaml.safe_load(f)
    base_dir = config_path.parent
    seed = int(config.get("metadata", {}).get("random_seed", config.get("random_seed", 7)))
    rng = np.random.default_rng(seed)
    paths = config.setdefault("paths", {})
    output_dir = _abs_path(output_dir_override or paths.get("output_dir", base_dir / "output"), base_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    targets = build_targets(config, base_dir, rng)
    center = target_center(targets)
    trajectory = load_trajectory(config.get("trajectory", {}), center)
    start_s = float(config.get("simulation", {}).get("start_s", trajectory.start_s))
    end_s = float(config.get("simulation", {}).get("end_s", trajectory.end_s))
    start_s = max(start_s, trajectory.start_s)
    end_s = min(end_s, trajectory.end_s)
    gravity_w = vec3(config.get("simulation", {}).get("gravity_m_s2", [0.0, 0.0, -9.80665]))

    cameras = [dict(camera) for camera in config.get("cameras", [])]
    imus = [dict(imu) for imu in config.get("imus", [])]
    if not cameras:
        raise ValueError("simulation config must contain at least one camera")
    if not imus:
        raise ValueError("simulation config must contain at least one IMU")
    for camera in cameras:
        T = parse_transform(camera.get("T_cam_body", camera.get("T_sensor_body")))
        camera["_T_cam_body"] = perturb_transform(T, camera.get("extrinsic_perturbation"), rng)
    for imu in imus:
        T = parse_transform(imu.get("T_imu_body", imu.get("T_sensor_body")))
        imu["_T_imu_body"] = perturb_transform(T, imu.get("extrinsic_perturbation"), rng)

    target_points_path = _abs_path(paths.get("target_points_csv", output_dir / "target_points.csv"), base_dir)
    camchain_path = _abs_path(paths.get("camchain_yaml", output_dir / "camchain.yaml"), base_dir)
    target_yaml_path = _abs_path(paths.get("target_yaml", output_dir / "aprilgrid.yaml"), base_dir)
    trajectory_csv_path = _abs_path(paths.get("trajectory_keyframes_csv", output_dir / "trajectory_keyframes.csv"), base_dir)
    sensor_poses_path = _abs_path(paths.get("sensor_poses_csv", output_dir / "sensor_poses.csv"), base_dir)
    manifest_path = _abs_path(paths.get("manifest_json", output_dir / "manifest.json"), base_dir)
    ground_truth_path = _abs_path(paths.get("ground_truth_yaml", output_dir / "ground_truth.yaml"), base_dir)

    _write_target_points(target_points_path, targets)
    _write_camchain(camchain_path, cameras)
    _write_target_yaml(target_yaml_path, targets[0])
    trajectory.write_keyframes(trajectory_csv_path)
    _write_sensor_poses(sensor_poses_path, trajectory, cameras, imus, start_s, end_s, float(config.get("outputs", {}).get("sensor_pose_hz", 20.0)))

    camera_summaries = []
    camera_outputs = []
    for index, camera in enumerate(cameras):
        name = camera.get("name", f"cam{index}")
        corner_csv = _abs_path(camera.get("output_corners_csv", output_dir / f"{name}_corners.csv"), base_dir)
        pose_csv = _abs_path(camera.get("output_corner_poses_csv", output_dir / f"{name}_corner_poses.csv"), base_dir)
        summary = _simulate_camera(camera, trajectory, targets, start_s, end_s, rng, corner_csv, pose_csv)
        camera_summaries.append(summary)
        camera_outputs.append({"name": name, "corners_csv": str(corner_csv), "corner_poses_csv": str(pose_csv)})

    imu_summaries = []
    imu_outputs = []
    for index, imu in enumerate(imus):
        name = imu.get("name", f"imu{index}")
        imu_csv = _abs_path(imu.get("output_imu_csv", output_dir / f"{name}.csv"), base_dir)
        imu_yaml = _abs_path(imu.get("output_imu_yaml", output_dir / f"{name}.yaml"), base_dir)
        _write_imu_yaml(imu_yaml, imu)
        summary = _simulate_imu(imu, trajectory, gravity_w, start_s, end_s, rng, imu_csv)
        imu_summaries.append(summary)
        imu_outputs.append({"name": name, "imu_csv": str(imu_csv), "imu_yaml": str(imu_yaml)})

    ground_truth = {
        "frames": {
            "world": "target0",
            "body": "reference_imu_body",
            "transform_convention": "T_ab maps point coordinates from frame b to frame a",
        },
        "gravity_m_s2": gravity_w.tolist(),
        "targets": [
            {
                "name": t["name"],
                "T_target0_target": matrix_rows(t["T_target0_target"]),
                "corner_id_offset": int(t["corner_ids"][0]),
                "corner_count": int(len(t["corner_ids"])),
            }
            for t in targets
        ],
        "cameras": [
            {
                "name": camera.get("name", f"cam{i}"),
                "T_cam_body": matrix_rows(camera["_T_cam_body"]),
                "fps": float(camera.get("fps", 20.0)),
                "time_offset_s": float(camera.get("time_offset_s", 0.0)),
            }
            for i, camera in enumerate(cameras)
        ],
        "imus": [
            {
                "name": imu.get("name", f"imu{i}"),
                "T_imu_body": matrix_rows(imu["_T_imu_body"]),
                "r_b": (-imu["_T_imu_body"][:3, :3].T @ imu["_T_imu_body"][:3, 3]).tolist(),
                "fps": float(imu.get("fps", imu.get("update_rate_hz", 200.0))),
            }
            for i, imu in enumerate(imus)
        ],
    }
    _write_yaml(ground_truth_path, ground_truth)

    manifest = {
        "config": str(config_path),
        "output_dir": str(output_dir),
        "target_points_csv": str(target_points_path),
        "camchain_yaml": str(camchain_path),
        "target_yaml": str(target_yaml_path),
        "trajectory_keyframes_csv": str(trajectory_csv_path),
        "sensor_poses_csv": str(sensor_poses_path),
        "ground_truth_yaml": str(ground_truth_path),
        "cameras": camera_outputs,
        "imus": imu_outputs,
        "summary": {"cameras": camera_summaries, "imus": imu_summaries},
    }
    _write_json(manifest_path, manifest)
    return manifest
