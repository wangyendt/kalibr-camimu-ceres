import math
from typing import Any, Mapping, Optional

import numpy as np
from scipy.spatial.transform import Rotation


def vec3(value: Any, default: Optional[list[float]] = None) -> np.ndarray:
    if value is None:
        value = default if default is not None else [0.0, 0.0, 0.0]
    arr = np.asarray(value, dtype=float).reshape(3)
    return arr


def mat3(value: Any, default_identity: bool = False) -> np.ndarray:
    if value is None:
        return np.eye(3) if default_identity else np.zeros((3, 3))
    arr = np.asarray(value, dtype=float)
    if arr.size != 9:
        raise ValueError("3x3 matrix must contain 9 values")
    return arr.reshape(3, 3)


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def left_jacobian_so3(r: np.ndarray) -> np.ndarray:
    r = np.asarray(r, dtype=float).reshape(3)
    theta2 = float(np.dot(r, r))
    k = skew(r)
    if theta2 < 1e-12:
        return np.eye(3) + 0.5 * k + (1.0 / 6.0) * (k @ k)
    theta = math.sqrt(theta2)
    a = (1.0 - math.cos(theta)) / theta2
    b = (theta - math.sin(theta)) / (theta2 * theta)
    return np.eye(3) + a * k + b * (k @ k)


def transform_matrix(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    t[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return t


def invert_transform(t_ab: np.ndarray) -> np.ndarray:
    r_ab = t_ab[:3, :3]
    p_ab = t_ab[:3, 3]
    inv = np.eye(4)
    inv[:3, :3] = r_ab.T
    inv[:3, 3] = -r_ab.T @ p_ab
    return inv


def transform_points(t_ab: np.ndarray, points_b: np.ndarray) -> np.ndarray:
    points_b = np.asarray(points_b, dtype=float)
    return (t_ab[:3, :3] @ points_b.T).T + t_ab[:3, 3]


def rotation_from_config(node: Optional[Mapping[str, Any]]) -> np.ndarray:
    if not node:
        return np.eye(3)
    if "matrix" in node:
        return mat3(node["matrix"], default_identity=True)
    if "rotvec" in node:
        return Rotation.from_rotvec(vec3(node["rotvec"])).as_matrix()
    if "rpy_deg" in node:
        return Rotation.from_euler("xyz", vec3(node["rpy_deg"]), degrees=True).as_matrix()
    if "rpy_rad" in node:
        return Rotation.from_euler("xyz", vec3(node["rpy_rad"]), degrees=False).as_matrix()
    if "quaternion_wxyz" in node:
        q = np.asarray(node["quaternion_wxyz"], dtype=float).reshape(4)
        return Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    if "quaternion_xyzw" in node:
        return Rotation.from_quat(np.asarray(node["quaternion_xyzw"], dtype=float).reshape(4)).as_matrix()
    return np.eye(3)


def parse_transform(node: Optional[Mapping[str, Any]]) -> np.ndarray:
    if node is None:
        return np.eye(4)
    if "matrix" in node:
        arr = np.asarray(node["matrix"], dtype=float)
        if arr.size != 16:
            raise ValueError("SE(3) matrix must contain 16 values")
        return arr.reshape(4, 4)
    rotation = rotation_from_config(node.get("rotation", {}))
    translation = vec3(node.get("translation", [0.0, 0.0, 0.0]))
    return transform_matrix(rotation, translation)


def perturb_transform(
    base: np.ndarray,
    node: Optional[Mapping[str, Any]],
    rng: np.random.Generator,
) -> np.ndarray:
    if not node:
        return base.copy()
    trans_std = vec3(node.get("translation_std_m", [0.0, 0.0, 0.0]))
    rot_std = vec3(node.get("rotation_std_rad", [0.0, 0.0, 0.0]))
    deterministic_trans = vec3(node.get("translation_m", [0.0, 0.0, 0.0]))
    deterministic_rot = vec3(node.get("rotation_rotvec_rad", [0.0, 0.0, 0.0]))
    delta_t = deterministic_trans + rng.normal(0.0, trans_std)
    delta_r = deterministic_rot + rng.normal(0.0, rot_std)
    delta = transform_matrix(Rotation.from_rotvec(delta_r).as_matrix(), delta_t)
    side = str(node.get("side", "left")).lower()
    return delta @ base if side == "left" else base @ delta


def matrix_rows(matrix: np.ndarray) -> list[list[float]]:
    return [[float(matrix[r, c]) for c in range(matrix.shape[1])] for r in range(matrix.shape[0])]


def normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        raise ValueError("cannot normalize near-zero vector")
    return v / n


def look_at_rotation(
    position_w: np.ndarray,
    target_w: np.ndarray,
    up_w: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> np.ndarray:
    z_axis = normalize(target_w - position_w)
    x_axis = np.cross(up_w, z_axis)
    if np.linalg.norm(x_axis) < 1e-9:
        x_axis = np.cross(np.array([0.0, 1.0, 0.0]), z_axis)
    x_axis = normalize(x_axis)
    y_axis = normalize(np.cross(z_axis, x_axis))
    return np.column_stack([x_axis, y_axis, z_axis])


def ensure_rotvec_continuity(rotvecs: np.ndarray) -> np.ndarray:
    out = np.asarray(rotvecs, dtype=float).copy()
    for i in range(1, len(out)):
        if np.linalg.norm(out[i] - out[i - 1]) > np.linalg.norm(-out[i] - out[i - 1]):
            out[i] *= -1.0
    return out
