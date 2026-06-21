import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, RotationSpline

from .se3 import ensure_rotvec_continuity, left_jacobian_so3, look_at_rotation, matrix_rows


@dataclass
class KinematicState:
    timestamp_s: float
    t_w_b: np.ndarray
    r_w_b: np.ndarray
    r_dot: np.ndarray
    r_ddot: np.ndarray
    v_w: np.ndarray
    a_w: np.ndarray
    R_w_b: np.ndarray
    omega_b: np.ndarray
    alpha_b: np.ndarray


class CubicPoseTrajectory:
    def __init__(
        self,
        times_s: np.ndarray,
        translations: np.ndarray,
        rotvecs: np.ndarray,
        rotation_interpolation: str = "rotvec_cubic",
    ):
        if len(times_s) < 4:
            raise ValueError("trajectory needs at least 4 pose samples")
        order = np.argsort(times_s)
        self.times_s = np.asarray(times_s, dtype=float)[order]
        self.translations = np.asarray(translations, dtype=float)[order]
        self.rotvecs = ensure_rotvec_continuity(np.asarray(rotvecs, dtype=float)[order])
        self.rotation_interpolation = str(rotation_interpolation or "rotvec_cubic").lower()
        if np.any(np.diff(self.times_s) <= 0.0):
            raise ValueError("trajectory timestamps must be strictly increasing")
        self.translation_spline = CubicSpline(self.times_s, self.translations, axis=0, bc_type="natural")
        self.rotvec_spline = None
        self.rotation_spline = None
        if self.rotation_interpolation in {"rotvec_cubic", "rotation_vector_cubic", "6d_cubic"}:
            self.rotvec_spline = CubicSpline(self.times_s, self.rotvecs, axis=0, bc_type="natural")
        elif self.rotation_interpolation in {"rotation_spline", "so3_cubic", "quaternion_spline"}:
            self.rotation_spline = RotationSpline(self.times_s, Rotation.from_rotvec(self.rotvecs))
        else:
            raise ValueError(f"unknown rotation interpolation: {rotation_interpolation}")

    @property
    def start_s(self) -> float:
        return float(self.times_s[0])

    @property
    def end_s(self) -> float:
        return float(self.times_s[-1])

    def _clamp_time(self, timestamp_s: float) -> float:
        eps = 1e-10
        return min(max(float(timestamp_s), self.start_s + eps), self.end_s - eps)

    def pose(self, timestamp_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = self._clamp_time(timestamp_s)
        t_w_b = np.asarray(self.translation_spline(t, 0), dtype=float).reshape(3)
        if self.rotation_spline is not None:
            rotation = self.rotation_spline(t)
            return t_w_b, rotation.as_matrix(), rotation.as_rotvec()
        r_w_b = np.asarray(self.rotvec_spline(t, 0), dtype=float).reshape(3)
        return t_w_b, Rotation.from_rotvec(r_w_b).as_matrix(), r_w_b

    def evaluate(self, timestamp_s: float) -> KinematicState:
        t = self._clamp_time(timestamp_s)
        t_w_b = np.asarray(self.translation_spline(t, 0), dtype=float).reshape(3)
        v_w = np.asarray(self.translation_spline(t, 1), dtype=float).reshape(3)
        a_w = np.asarray(self.translation_spline(t, 2), dtype=float).reshape(3)
        if self.rotation_spline is not None:
            rotation = self.rotation_spline(t)
            R_w_b = rotation.as_matrix()
            r_w_b = rotation.as_rotvec()
            omega_b = np.asarray(self.rotation_spline(t, 1), dtype=float).reshape(3)
            alpha_b = np.asarray(self.rotation_spline(t, 2), dtype=float).reshape(3)
            r_dot = np.zeros(3)
            r_ddot = np.zeros(3)
        else:
            r_w_b = np.asarray(self.rotvec_spline(t, 0), dtype=float).reshape(3)
            r_dot = np.asarray(self.rotvec_spline(t, 1), dtype=float).reshape(3)
            r_ddot = np.asarray(self.rotvec_spline(t, 2), dtype=float).reshape(3)
            R_w_b = Rotation.from_rotvec(r_w_b).as_matrix()
            # SciPy uses the standard Rodrigues convention R = Exp(r).  The body
            # angular velocity is therefore J_right(r) * r_dot = J_left(-r) * r_dot.
            # C++ uses the opposite rotvec sign internally, so this is the Python
            # side counterpart of its -J_left(r_cpp) * r_dot_cpp expression.
            j_right = left_jacobian_so3(-r_w_b)
            omega_b = j_right @ r_dot
            alpha_b = j_right @ r_ddot
        return KinematicState(t, t_w_b, r_w_b, r_dot, r_ddot, v_w, a_w, R_w_b, omega_b, alpha_b)

    def write_keyframes(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_s", "tx", "ty", "tz", "qw", "qx", "qy", "qz"])
            for t, p, r in zip(self.times_s, self.translations, self.rotvecs):
                q = Rotation.from_rotvec(r).as_quat()
                writer.writerow([f"{t:.9f}", *[f"{x:.12g}" for x in p], f"{q[3]:.12g}", f"{q[0]:.12g}", f"{q[1]:.12g}", f"{q[2]:.12g}"])


def _timestamp(row: Mapping[str, str]) -> float:
    if "timestamp_s" in row:
        return float(row["timestamp_s"])
    if "timestamp_ns" in row:
        return float(row["timestamp_ns"]) * 1e-9
    if "timestamp" in row:
        return float(row["timestamp"])
    first = next(iter(row.values()))
    value = float(first)
    return value * 1e-9 if value > 1e12 else value


def load_pose_csv(path: Path, rotation_interpolation: str = "rotvec_cubic") -> CubicPoseTrajectory:
    rows: list[dict[str, str]]
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty pose CSV: {path}")

    times = []
    translations = []
    rotations = []
    for row in rows:
        times.append(_timestamp(row))
        if {"tx", "ty", "tz", "qw", "qx", "qy", "qz"}.issubset(row):
            translations.append([float(row["tx"]), float(row["ty"]), float(row["tz"])])
            rotations.append(Rotation.from_quat([float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])]).as_rotvec())
            continue
        matrix_keys = [f"T{r}{c}" for r in range(4) for c in range(4)]
        if set(matrix_keys).issubset(row):
            T = np.array([float(row[k]) for k in matrix_keys], dtype=float).reshape(4, 4)
            translations.append(T[:3, 3])
            rotations.append(Rotation.from_matrix(T[:3, :3]).as_rotvec())
            continue
        values = list(row.values())
        if len(values) >= 17:
            T = np.array([float(v) for v in values[1:17]], dtype=float).reshape(4, 4)
            translations.append(T[:3, 3])
            rotations.append(Rotation.from_matrix(T[:3, :3]).as_rotvec())
            continue
        raise ValueError("pose CSV must contain tx/ty/tz+quaternion or timestamp plus 16 matrix values")
    return CubicPoseTrajectory(
        np.asarray(times),
        np.asarray(translations),
        np.asarray(rotations),
        rotation_interpolation,
    )


def analytic_calibration_sweep(
    node: Mapping[str, Any],
    target_center_w: np.ndarray,
) -> CubicPoseTrajectory:
    duration_s = float(node.get("duration_s", 10.0))
    sample_hz = float(node.get("keyframe_hz", 80.0))
    radius_m = float(node.get("radius_m", 0.95))
    lateral_m = float(node.get("lateral_m", 0.28))
    vertical_m = float(node.get("vertical_m", 0.55))
    vertical_amp_m = float(node.get("vertical_amp_m", 0.16))
    roll_amp_deg = float(node.get("roll_amp_deg", 18.0))
    yaw_cycles = float(node.get("yaw_cycles", 1.2))
    pitch_cycles = float(node.get("pitch_cycles", 1.7))
    start_s = float(node.get("start_s", 0.0))
    count = int(math.floor(duration_s * sample_hz)) + 1
    times = start_s + np.arange(count, dtype=float) / sample_hz
    translations = []
    rotvecs = []
    for t in times:
        u = (t - start_s) / max(duration_s, 1e-9)
        x = target_center_w[0] + lateral_m * math.sin(2.0 * math.pi * yaw_cycles * u)
        y = target_center_w[1] - radius_m + 0.18 * math.sin(2.0 * math.pi * (yaw_cycles * 0.7) * u + 0.4)
        z = target_center_w[2] + vertical_m + vertical_amp_m * math.sin(2.0 * math.pi * pitch_cycles * u + 0.7)
        p = np.array([x, y, z], dtype=float)
        aim = target_center_w + np.array([
            0.08 * math.sin(2.0 * math.pi * 1.6 * u),
            0.05 * math.cos(2.0 * math.pi * 1.1 * u),
            0.02 * math.sin(2.0 * math.pi * 2.1 * u),
        ])
        R_w_b = look_at_rotation(p, aim)
        roll = math.radians(roll_amp_deg) * math.sin(2.0 * math.pi * 2.3 * u + 0.2)
        R_w_b = R_w_b @ Rotation.from_rotvec([0.0, 0.0, roll]).as_matrix()
        translations.append(p)
        rotvecs.append(Rotation.from_matrix(R_w_b).as_rotvec())
    interpolation = str(node.get("rotation_interpolation", node.get("rotation_backend", "rotation_spline")))
    return CubicPoseTrajectory(times, np.asarray(translations), np.asarray(rotvecs), interpolation)


def load_trajectory(node: Mapping[str, Any], target_center_w: np.ndarray) -> CubicPoseTrajectory:
    kind = str(node.get("type", "analytic")).lower()
    if kind in {"pose_csv", "csv", "file"}:
        path = Path(node["path"]).expanduser()
        interpolation = str(node.get("rotation_interpolation", node.get("rotation_backend", "rotvec_cubic")))
        return load_pose_csv(path, interpolation)
    if kind in {"analytic", "calibration_sweep"}:
        return analytic_calibration_sweep(node, target_center_w)
    raise ValueError(f"unknown trajectory type: {kind}")


def pose_matrix_rows(state: KinematicState) -> list[float]:
    T = np.eye(4)
    T[:3, :3] = state.R_w_b
    T[:3, 3] = state.t_w_b
    return [float(v) for row in matrix_rows(T) for v in row]
