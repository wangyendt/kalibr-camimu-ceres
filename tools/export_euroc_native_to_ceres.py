#!/usr/bin/env python3
"""Export EuRoC image/IMU folders to ceres_cam_imu CSV inputs without Kalibr.

The detector is cpp_tools/cv/apriltag_detection.  The output CSV schema is the
same neutral schema consumed by the C++ calibration binary and previously
produced through Kalibr Docker.
"""

import argparse
import csv
import pathlib
import sys
from dataclasses import dataclass

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class AprilGrid:
    tag_cols: int
    tag_rows: int
    tag_size_m: float
    tag_spacing_ratio: float

    @property
    def tag_count(self):
        return self.tag_cols * self.tag_rows

    @property
    def grid_cols(self):
        return 2 * self.tag_cols

    @property
    def default_min_tags_for_valid_obs(self):
        return max(self.tag_rows, self.tag_cols) + 1

    def tag_corner_point(self, tag_id, corner_slot):
        row = tag_id // self.tag_cols
        col = tag_id % self.tag_cols
        step = self.tag_size_m * (1.0 + self.tag_spacing_ratio)
        x0 = float(col) * step
        y0 = float(row) * step
        s = self.tag_size_m
        # Kalibr exposes Aprilgrid target coordinates through float-backed
        # observation arrays; matching that keeps native CSVs byte-stable.
        def k(value):
            return float(np.float32(value))
        if corner_slot == 0:
            return (k(x0), k(y0), 0.0)
        if corner_slot == 1:
            return (k(x0 + s), k(y0), 0.0)
        if corner_slot == 2:
            return (k(x0 + s), k(y0 + s), 0.0)
        if corner_slot == 3:
            return (k(x0), k(y0 + s), 0.0)
        raise ValueError("invalid corner slot")

    def kalibr_corner_id(self, tag_id, corner_slot):
        base_id = (tag_id // self.tag_cols) * self.grid_cols * 2 + (tag_id % self.tag_cols) * 2
        return [base_id, base_id + 1, base_id + self.grid_cols + 1, base_id + self.grid_cols][corner_slot]

    def output_corner_id(self, tag_id, corner_slot, mode):
        if mode == "kalibr":
            return self.kalibr_corner_id(tag_id, corner_slot)
        if mode == "tag":
            return tag_id * 4 + corner_slot
        raise ValueError(f"unsupported corner id mode: {mode}")


def repo_dir():
    return pathlib.Path(__file__).resolve().parents[1]


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def load_target(path):
    data = load_yaml(path)
    if data.get("target_type") != "aprilgrid":
        raise ValueError("native EuRoC export currently supports target_type=aprilgrid only")
    return AprilGrid(
        tag_cols=int(data["tagCols"]),
        tag_rows=int(data["tagRows"]),
        tag_size_m=float(data["tagSize"]),
        tag_spacing_ratio=float(data["tagSpacing"]),
    )


def load_cpp_tools(root):
    root = pathlib.Path(root).expanduser().resolve()
    sys.path.insert(0, str(root / "cv" / "apriltag_detection" / "lib"))
    sys.path.insert(0, str(root / "cv" / "camera_models" / "lib"))
    try:
        import apriltag_detection
    except ImportError as exc:
        raise ImportError(
            "failed to import cpp_tools apriltag_detection; run "
            "`tools/build_cpp_tools_pybinds.sh` or pass --cpp-tools-root"
        ) from exc
    return apriltag_detection


def make_detector(apriltag_detection, tag_family, black_border):
    normalized = tag_family.lower().replace("-", "").replace("_", "")
    if normalized.startswith("tag"):
        normalized = normalized[3:]
    family_name = "tag_codes_" + normalized
    family_fn = getattr(apriltag_detection, family_name, None)
    if family_fn is None:
        supported = sorted(name.replace("tag_codes_", "") for name in dir(apriltag_detection)
                           if name.startswith("tag_codes_"))
        raise ValueError(f"unsupported tag family {tag_family}; supported: {supported}")
    return apriltag_detection.TagDetector(family_fn(), int(black_border))


def euroc_mav0(euroc_dir):
    root = pathlib.Path(euroc_dir).expanduser().resolve()
    mav0 = root / "mav0"
    return mav0 if mav0.is_dir() else root


def numeric_csv_rows(path):
    with open(path, "r", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row or not row[0] or row[0].startswith("#"):
                continue
            try:
                int(row[0])
            except ValueError:
                continue
            yield row


def read_image_index(cam_dir):
    data_csv = cam_dir / "data.csv"
    if not data_csv.is_file():
        raise FileNotFoundError(f"camera data.csv not found: {data_csv}")
    frames = []
    for row in numeric_csv_rows(data_csv):
        if len(row) < 2:
            continue
        frames.append((int(row[0]), row[1]))
    return frames


def copy_imu(mav0, out_dir, imu_index=0):
    imu_csv = mav0 / f"imu{imu_index}" / "data.csv"
    if not imu_csv.is_file():
        raise FileNotFoundError(f"EuRoC imu{imu_index}/data.csv not found under {mav0}")
    out_csv = out_dir / "imu.csv"
    count = 0
    with open(out_csv, "w", newline="") as output:
        writer = csv.writer(output)
        for row in numeric_csv_rows(imu_csv):
            if len(row) < 7:
                continue
            writer.writerow(row[:7])
            count += 1
    print(f"exported {count} IMU samples to {out_csv}")


def camera_keys(camchain):
    keys = sorted(
        key for key in camchain.keys()
        if isinstance(key, str) and key.startswith("cam") and key[3:].isdigit()
    )
    if not keys:
        raise ValueError("camchain does not contain camN entries")
    return keys


def camera_matrix(camera):
    intrinsics = [float(x) for x in camera["intrinsics"]]
    fx, fy, cx, cy = intrinsics
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def camera_resolution(camera):
    resolution = camera.get("resolution", [0, 0])
    if len(resolution) < 2:
        return 0.0, 0.0
    return float(resolution[0]), float(resolution[1])


def equidistant_distort(points, coeffs):
    points = np.asarray(points, dtype=np.float64)
    coeffs = np.asarray(coeffs, dtype=np.float64).reshape(-1)
    k1, k2, k3, k4 = coeffs[:4]
    x = points[:, 0]
    y = points[:, 1]
    radius = np.sqrt(x * x + y * y)
    theta = np.arctan(radius)
    theta2 = theta * theta
    theta4 = theta2 * theta2
    theta6 = theta4 * theta2
    theta8 = theta4 * theta4
    theta_distorted = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8)
    scale = np.ones_like(radius)
    mask = radius > 1e-8
    scale[mask] = theta_distorted[mask] / radius[mask]
    return np.column_stack((x * scale, y * scale))


def equidistant_undistort(points, coeffs):
    points = np.asarray(points, dtype=np.float64)
    coeffs = np.asarray(coeffs, dtype=np.float64).reshape(-1)
    k1, k2, k3, k4 = coeffs[:4]
    x = points[:, 0]
    y = points[:, 1]
    theta_distorted = np.sqrt(x * x + y * y)
    theta = theta_distorted.copy()
    for _ in range(20):
        theta2 = theta * theta
        theta4 = theta2 * theta2
        theta6 = theta4 * theta2
        theta8 = theta4 * theta4
        denominator = 1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8
        theta = theta_distorted / denominator
    scale = np.ones_like(theta_distorted)
    mask = theta_distorted > 1e-12
    scale[mask] = np.tan(theta[mask]) / theta_distorted[mask]
    return np.column_stack((x * scale, y * scale))


def keypoints_to_euclidean(camera, image_points):
    image_points = np.asarray(image_points, dtype=np.float64)
    intrinsics = [float(x) for x in camera["intrinsics"]]
    fx, fy, cx, cy = intrinsics
    normalized = np.column_stack(((image_points[:, 0] - cx) / fx, (image_points[:, 1] - cy) / fy))
    distortion_model = str(camera.get("distortion_model", "")).lower()
    coeffs = np.asarray(camera.get("distortion_coeffs", []), dtype=np.float64)
    if distortion_model in ("equidistant", "fisheye"):
        if coeffs.size < 4:
            raise ValueError("equidistant camera requires 4 distortion coefficients")
        normalized = equidistant_undistort(normalized, coeffs[:4])
    elif distortion_model in ("radtan", "plumb_bob"):
        normalized = cv2.undistortPoints(
            image_points.reshape(-1, 1, 2),
            camera_matrix(camera),
            coeffs,
        ).reshape(-1, 2)
    elif distortion_model not in ("", "none"):
        raise ValueError(f"unsupported distortion model for native PnP: {distortion_model}")
    rays = np.column_stack((normalized[:, 0], normalized[:, 1], np.ones(len(normalized))))
    width, height = camera_resolution(camera)
    valid = np.ones(len(image_points), dtype=bool)
    if width > 0 and height > 0:
        valid &= image_points[:, 0] >= 0.0
        valid &= image_points[:, 1] >= 0.0
        valid &= image_points[:, 0] < width
        valid &= image_points[:, 1] < height
    return rays, valid


def pnp_inputs(camera, object_points, image_points):
    distortion_model = str(camera.get("distortion_model", "")).lower()
    coeffs = np.asarray(camera.get("distortion_coeffs", []), dtype=np.float64).reshape(-1, 1)
    object_points = np.asarray(object_points, dtype=np.float64)
    image_points = np.asarray(image_points, dtype=np.float64)

    if distortion_model in ("equidistant", "fisheye", "radtan", "plumb_bob"):
        rays, valid = keypoints_to_euclidean(camera, image_points)
        z_axis_cosine = rays[:, 2] / np.linalg.norm(rays, axis=1)
        valid &= z_axis_cosine > np.cos(np.deg2rad(80.0))
        normalized = rays[valid, :2] / rays[valid, 2:3]
        return object_points[valid], normalized, np.eye(3, dtype=np.float64), np.zeros((4, 1), dtype=np.float64)

    k_matrix = camera_matrix(camera)

    if distortion_model in ("", "none"):
        return object_points, image_points, k_matrix, np.zeros((4, 1), dtype=np.float64)

    raise ValueError(f"unsupported distortion model for native PnP: {distortion_model}")


def solve_pose_t_c(camera, object_points, image_points):
    obj, img, k_matrix, distortion = pnp_inputs(camera, object_points, image_points)
    if len(obj) < 4:
        return None
    ok, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        k_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    t_c_t = np.eye(4, dtype=np.float64)
    t_c_t[:3, :3] = rotation
    t_c_t[:3, 3] = tvec.reshape(3)
    return np.linalg.inv(t_c_t)


def project_points(camera, object_points, t_t_c):
    object_points = np.asarray(object_points, dtype=np.float64)
    t_c_t = np.linalg.inv(t_t_c)
    object_points_h = np.column_stack((object_points, np.ones(len(object_points))))
    points_camera = (t_c_t @ object_points_h.T).T[:, :3]
    k_matrix = camera_matrix(camera)
    distortion_model = str(camera.get("distortion_model", "")).lower()
    coeffs = np.asarray(camera.get("distortion_coeffs", []), dtype=np.float64).reshape(-1, 1)
    if distortion_model in ("equidistant", "fisheye"):
        normalized = points_camera[:, :2] / points_camera[:, 2:3]
        distorted = equidistant_distort(normalized, coeffs[:4])
        projected = distorted @ np.array([[k_matrix[0, 0], 0.0], [0.0, k_matrix[1, 1]]])
        projected += np.array([k_matrix[0, 2], k_matrix[1, 2]])
        return projected.reshape(-1, 2)
    elif distortion_model in ("radtan", "plumb_bob"):
        rvec, _ = cv2.Rodrigues(t_c_t[:3, :3])
        rvec = np.ascontiguousarray(rvec, dtype=np.float64)
        tvec = np.ascontiguousarray(t_c_t[:3, 3].reshape(3, 1), dtype=np.float64)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, k_matrix, coeffs)
    elif distortion_model in ("", "none"):
        rvec, _ = cv2.Rodrigues(t_c_t[:3, :3])
        rvec = np.ascontiguousarray(rvec, dtype=np.float64)
        tvec = np.ascontiguousarray(t_c_t[:3, 3].reshape(3, 1), dtype=np.float64)
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            k_matrix,
            np.zeros((4, 1), dtype=np.float64),
        )
    else:
        raise ValueError(f"unsupported distortion model for native projection: {distortion_model}")
    return projected.reshape(-1, 2)


def filter_reprojection_outliers(camera, rows, t_t_c, args):
    if not args.kalibr_postprocess or not args.filter_corner_outliers or not rows:
        return rows
    object_points = [row[3] for row in rows]
    image_points = np.asarray([(row[1], row[2]) for row in rows], dtype=np.float64)
    projected = project_points(camera, object_points, t_t_c)
    errors = np.linalg.norm(image_points - projected, axis=1)
    mean = float(np.mean(errors))
    std = float(np.std(errors))
    threshold = mean + args.filter_corner_sigma_threshold * std
    filtered = []
    for row, error in zip(rows, errors):
        remove = error > threshold and error > args.filter_corner_min_reproj_error
        if not remove:
            filtered.append(row)
    return filtered


def image_observations(image_path, detector, grid, args):
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"failed to read image: {image_path}")
    detections = sorted(detector.extract_tags(np.ascontiguousarray(image)), key=lambda det: det.id)
    valid_detections = []
    image_height, image_width = image.shape[:2]
    min_tags_for_valid_obs = args.min_tags_for_valid_obs
    if min_tags_for_valid_obs <= 0:
        min_tags_for_valid_obs = grid.default_min_tags_for_valid_obs
    source_slots = list(args.corner_permutation)
    if sorted(source_slots) != [0, 1, 2, 3]:
        raise ValueError("--corner-permutation must contain each slot 0,1,2,3 exactly once")

    raw_corner_blocks = []
    for detection in detections:
        tag_id = int(detection.id)
        if tag_id < 0 or tag_id >= grid.tag_count:
            continue
        if args.max_hamming >= 0 and int(detection.hamming_distance) > args.max_hamming:
            continue
        corners = np.asarray(list(detection.corners), dtype=np.float32)[source_slots]
        if args.kalibr_postprocess:
            too_close_to_border = np.any(corners[:, 0] < args.min_border_distance)
            too_close_to_border |= np.any(corners[:, 0] > float(image_width) - args.min_border_distance)
            too_close_to_border |= np.any(corners[:, 1] < args.min_border_distance)
            too_close_to_border |= np.any(corners[:, 1] > float(image_height) - args.min_border_distance)
            if too_close_to_border:
                continue
        valid_detections.append((tag_id, corners))
        raw_corner_blocks.append(corners)

    if args.kalibr_postprocess and len(valid_detections) < min_tags_for_valid_obs:
        return [], image

    if raw_corner_blocks:
        raw_corners = np.vstack(raw_corner_blocks).astype(np.float32)
        refined_corners = raw_corners.copy()
        if args.kalibr_postprocess and args.subpix_refinement:
            refined_view = refined_corners.reshape(-1, 1, 2)
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.1,
            )
            cv2.cornerSubPix(image, refined_view, (2, 2), (-1, -1), criteria)
            refined_corners = refined_view.reshape(-1, 2)
    else:
        raw_corners = np.empty((0, 2), dtype=np.float32)
        refined_corners = np.empty((0, 2), dtype=np.float32)

    rows = []
    flat_index = 0
    complete_valid_tags = 0
    for tag_id, _corners in valid_detections:
        tag_rows = []
        for corner_slot in range(4):
            raw_pixel = raw_corners[flat_index]
            refined_pixel = refined_corners[flat_index]
            flat_index += 1
            corner_valid = True
            if args.kalibr_postprocess:
                displacement = refined_pixel - raw_pixel
                displacement_squared = float(np.dot(displacement, displacement))
                if displacement_squared > args.max_subpix_displacement_squared:
                    corner_valid = False
            corner_id = grid.output_corner_id(tag_id, corner_slot, args.corner_id_mode)
            target = grid.tag_corner_point(tag_id, corner_slot)
            pixel = refined_pixel
            if corner_valid:
                tag_rows.append((corner_id, float(pixel[0]), float(pixel[1]), target))
        if len(tag_rows) == 4:
            complete_valid_tags += 1
        rows.extend(tag_rows)
    if (
        args.kalibr_postprocess
        and args.strict_min_tags_after_subpix
        and complete_valid_tags < min_tags_for_valid_obs
    ):
        return [], image
    return rows, image


def export_camera(camera_index, camera_key, camera, mav0, out_dir, detector, grid, args):
    cam_dir = mav0 / camera_key
    image_dir = cam_dir / "data"
    if not image_dir.is_dir():
        raise FileNotFoundError(f"camera image directory not found: {image_dir}")
    frames = read_image_index(cam_dir)
    if args.max_frames and args.max_frames > 0:
        frames = frames[:args.max_frames]

    corners_csv = out_dir / f"cam{camera_index}_corners.csv"
    poses_csv = out_dir / "cam0_corner_poses.csv"
    frame_count = 0
    corner_count = 0
    pose_count = 0
    with open(corners_csv, "w", newline="") as corners_handle:
        corners_writer = csv.writer(corners_handle)
        corners_writer.writerow([
            "timestamp_ns",
            "corner_id",
            "pixel_x",
            "pixel_y",
            "target_x",
            "target_y",
            "target_z",
        ])
        pose_handle = None
        pose_writer = None
        if camera_index == 0:
            pose_handle = open(poses_csv, "w", newline="")
            pose_writer = csv.writer(pose_handle)
            pose_writer.writerow(["timestamp_ns"] + [
                f"T_t_c_{row}{col}" for row in range(4) for col in range(4)
            ])
        try:
            for timestamp_ns, filename in frames:
                rows, _image = image_observations(
                    image_dir / filename,
                    detector,
                    grid,
                    args,
                )
                if not rows:
                    continue
                t_t_c = None
                if args.kalibr_postprocess or camera_index == 0:
                    if len(rows) >= args.min_pnp_corners:
                        object_points = [row[3] for row in rows]
                        image_points = [(row[1], row[2]) for row in rows]
                        t_t_c = solve_pose_t_c(camera, object_points, image_points)
                    if args.kalibr_postprocess and t_t_c is None:
                        continue
                    if t_t_c is not None:
                        rows = filter_reprojection_outliers(camera, rows, t_t_c, args)
                        if not rows:
                            continue
                rows = sorted(rows, key=lambda row: row[0])
                frame_count += 1
                for corner_id, pixel_x, pixel_y, target in rows:
                    corners_writer.writerow([timestamp_ns, corner_id, pixel_x, pixel_y] + list(target))
                    corner_count += 1
                if camera_index == 0 and t_t_c is not None:
                    pose_writer.writerow([timestamp_ns] + t_t_c.reshape(-1).tolist())
                    pose_count += 1
        finally:
            if pose_handle is not None:
                pose_handle.close()

    print(
        f"exported cam{camera_index}: {frame_count} frames, "
        f"{corner_count} corners to {corners_csv}"
    )
    if camera_index == 0:
        print(f"exported cam0 poses: {pose_count} frames to {poses_csv}")
        if pose_count == 0 and not args.allow_missing_poses:
            raise RuntimeError("no cam0 poses were solved; check corner ordering and camera model")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--euroc-dir", required=True)
    parser.add_argument("--cams", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--cpp-tools-root", default=str(repo_dir() / "third_party" / "cpp_tools"))
    parser.add_argument("--tag-family", default="tag36h11")
    parser.add_argument("--tag-black-border", type=int, default=2)
    parser.add_argument("--max-hamming", type=int, default=-1)
    parser.add_argument("--imu-index", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--min-pnp-corners", type=int, default=12)
    parser.add_argument("--corner-permutation", type=int, nargs=4, default=[0, 1, 2, 3])
    parser.add_argument("--corner-id-mode", choices=["kalibr", "tag"], default="kalibr")
    parser.add_argument("--min-tags-for-valid-obs", type=int, default=0)
    parser.add_argument("--min-border-distance", type=float, default=4.0)
    parser.add_argument("--max-subpix-displacement-squared", type=float, default=1.5)
    parser.add_argument("--filter-corner-sigma-threshold", type=float, default=2.0)
    parser.add_argument("--filter-corner-min-reproj-error", type=float, default=0.2)
    parser.add_argument("--disable-kalibr-postprocess", dest="kalibr_postprocess", action="store_false")
    parser.add_argument("--disable-subpix-refinement", dest="subpix_refinement", action="store_false")
    parser.add_argument("--disable-corner-outlier-filter", dest="filter_corner_outliers", action="store_false")
    parser.add_argument("--strict-min-tags-after-subpix", action="store_true")
    parser.set_defaults(
        kalibr_postprocess=True,
        subpix_refinement=True,
        filter_corner_outliers=True,
    )
    parser.add_argument("--allow-missing-poses", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = pathlib.Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    mav0 = euroc_mav0(args.euroc_dir)
    grid = load_target(args.target)
    camchain = load_yaml(args.cams)
    apriltag_detection = load_cpp_tools(args.cpp_tools_root)
    detector = make_detector(apriltag_detection, args.tag_family, args.tag_black_border)

    copy_imu(mav0, out_dir, args.imu_index)
    for camera_index, camera_key in enumerate(camera_keys(camchain)):
        export_camera(
            camera_index,
            camera_key,
            camchain[camera_key],
            mav0,
            out_dir,
            detector,
            grid,
            args,
        )


if __name__ == "__main__":
    main()
