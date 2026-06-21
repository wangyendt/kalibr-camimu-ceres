#!/usr/bin/env python3
"""Convert simulator corner CSV into Kalibr GridCalibrationTargetObservation pickle.

This script must run inside a Kalibr Python environment because the pickle
contains Boost.Python observation objects.
"""

import argparse
import csv
import pickle
from collections import OrderedDict
from pathlib import Path

import numpy as np
import yaml


def _load_kalibr_modules():
    import aslam_cv
    import aslam_cameras_april
    import sm

    return aslam_cv, aslam_cameras_april, sm


def _read_target(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return {
        "tag_rows": int(data.get("tagRows", data.get("tag_rows"))),
        "tag_cols": int(data.get("tagCols", data.get("tag_cols"))),
        "tag_size": float(data.get("tagSize", data.get("tag_size_m"))),
        "tag_spacing": float(data.get("tagSpacing", data.get("tag_spacing_ratio"))),
    }


def _read_resolution(camchain_path, camera_index):
    with open(camchain_path) as f:
        data = yaml.safe_load(f)
    node = data[f"cam{camera_index}"]
    width, height = node.get("resolution", [640, 480])
    return int(width), int(height)


def _read_pose_csv(path):
    poses = {}
    if not path:
        return poses
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ts = int(row.get("timestamp_ns") or row.get("#timestamp_ns"))
            if all(f"T{r}{c}" in row for r in range(4) for c in range(4)):
                values = [float(row[f"T{r}{c}"]) for r in range(4) for c in range(4)]
            else:
                fields = list(row.values())
                values = [float(v) for v in fields[1:17]]
            poses[ts] = np.asarray(values, dtype=float).reshape(4, 4)
    return poses


def _make_time(acv, timestamp_ns):
    sec = int(timestamp_ns // 1_000_000_000)
    nsec = int(timestamp_ns % 1_000_000_000)
    return acv.Time(sec, nsec)


def _make_transform(sm, T):
    R = np.asarray(T[:3, :3], dtype=float)
    t = np.asarray(T[:3, 3], dtype=float)
    return sm.Transformation(sm.rt2Transform(R, t))


def _sim_local_id_to_kalibr_id(local_id, tag_cols):
    tag_index, corner_index = divmod(local_id, 4)
    tag_row, tag_col = divmod(tag_index, tag_cols)
    row_base = tag_row * 4 * tag_cols
    top = row_base + 2 * tag_col
    bottom = row_base + 2 * tag_cols + 2 * tag_col
    if corner_index == 0:
        return top
    if corner_index == 1:
        return top + 1
    if corner_index == 2:
        return bottom + 1
    if corner_index == 3:
        return bottom
    raise ValueError(f"invalid local corner index: {local_id}")


def export(args):
    acv, acv_april, sm = _load_kalibr_modules()
    target_cfg = _read_target(args.target_yaml)
    width, height = _read_resolution(args.camchain_yaml, args.camera_index)
    options = acv_april.AprilgridOptions()
    target = acv_april.GridCalibrationTargetAprilgrid(
        target_cfg["tag_rows"],
        target_cfg["tag_cols"],
        target_cfg["tag_size"],
        target_cfg["tag_spacing"],
        options,
    )
    target_size = int(target.size())
    poses = _read_pose_csv(args.poses_csv)

    grouped = OrderedDict()
    with open(args.corners_csv, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if "target_index" in row and int(row["target_index"]) != args.target_index:
                continue
            timestamp_ns = int(row["timestamp_ns"])
            local_id = int(row.get("local_corner_id") or row["corner_id"])
            if local_id < 0 or local_id >= target_size:
                continue
            kalibr_id = _sim_local_id_to_kalibr_id(local_id, target_cfg["tag_cols"])
            grouped.setdefault(timestamp_ns, []).append(
                (kalibr_id, float(row["u_px"]), float(row["v_px"]))
            )

    observations = []
    timestamps = []
    _ = (width, height)
    for timestamp_ns, corners in grouped.items():
        if len(corners) < args.min_corners:
            continue
        obs = acv.GridCalibrationTargetObservation(target)
        obs.setTime(_make_time(acv, timestamp_ns))
        for local_id, u, v in corners:
            obs.updateImagePoint(local_id, np.array([u, v], dtype=float))
        if timestamp_ns in poses:
            obs.set_T_t_c(_make_transform(sm, poses[timestamp_ns]))
        observations.append(obs)
        timestamps.append(timestamp_ns)

    args.output_corner_pkl.parent.mkdir(parents=True, exist_ok=True)
    args.output_timestamps.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output_corner_pkl, "wb") as f:
        pickle.dump(observations, f)
    with open(args.output_timestamps, "w") as f:
        f.write("\n".join(str(ts) for ts in timestamps))
        f.write("\n")
    print(
        f"wrote {len(observations)} observations, target_index={args.target_index}, "
        f"corner_pkl={args.output_corner_pkl}, timestamps={args.output_timestamps}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--corners-csv", type=Path, required=True)
    parser.add_argument("--poses-csv", type=Path)
    parser.add_argument("--target-yaml", type=Path, required=True)
    parser.add_argument("--camchain-yaml", type=Path, required=True)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--target-index", type=int, default=0)
    parser.add_argument("--min-corners", type=int, default=12)
    parser.add_argument("--output-corner-pkl", type=Path, required=True)
    parser.add_argument("--output-timestamps", type=Path, required=True)
    args = parser.parse_args()
    export(args)


if __name__ == "__main__":
    main()
