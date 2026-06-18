#!/usr/bin/env python3
"""Export Kalibr target-observation pickle files to neutral CSV files.

Run this inside the Kalibr Python environment because the pickle contains
extension-backed observation objects.
"""

import argparse
import csv
import pickle

import numpy as np


def _register_kalibr_pickle_types():
    # Kalibr observations are Boost.Python objects. Importing these modules
    # registers their classes before pickle.load reconstructs the objects.
    import aslam_cv  # noqa: F401
    import aslam_cameras_april  # noqa: F401
    try:
        import kalibr_common  # noqa: F401
    except ImportError:
        pass


def _as_rows(array, cols):
    data = np.asarray(array, dtype=float)
    if data.ndim != 2:
        raise ValueError(f"expected 2D corner array, got shape {data.shape}")
    if data.shape[1] == cols:
        return data
    if data.shape[0] == cols:
        return data.T
    raise ValueError(f"cannot interpret shape {data.shape} as rows with {cols} columns")


def _corner_ids(obs, count):
    if hasattr(obs, "getCornersIdx"):
        ids = list(obs.getCornersIdx())
        if len(ids) == count:
            return [int(x) for x in ids]
    return list(range(count))


def _timestamp_ns(obs):
    return int(round(float(obs.time().toSec()) * 1_000_000_000.0))


def export(corner_pkl, corners_csv, poses_csv=None):
    _register_kalibr_pickle_types()
    with open(corner_pkl, "rb") as f:
        observations = pickle.load(f)

    with open(corners_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["timestamp_ns", "corner_id", "pixel_x", "pixel_y", "target_x", "target_y", "target_z"]
        )
        for obs in observations:
            image = _as_rows(obs.getCornersImageFrame(), 2)
            target = _as_rows(obs.getCornersTargetFrame(), 3)
            if image.shape[0] != target.shape[0]:
                raise ValueError("image and target corner counts differ")
            ids = _corner_ids(obs, image.shape[0])
            ts = _timestamp_ns(obs)
            for idx, pixel, point in zip(ids, image, target):
                writer.writerow([ts, idx, pixel[0], pixel[1], point[0], point[1], point[2]])

    if poses_csv:
        with open(poses_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_ns"] + [f"T_t_c_{r}{c}" for r in range(4) for c in range(4)])
            for obs in observations:
                T = np.asarray(obs.T_t_c().T(), dtype=float).reshape(4, 4)
                writer.writerow([_timestamp_ns(obs)] + T.reshape(-1).tolist())

    print(f"exported {len(observations)} observations to {corners_csv}")
    if poses_csv:
        print(f"exported per-observation target/camera poses to {poses_csv}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--corner-pkl", required=True)
    parser.add_argument("--corners-csv", required=True)
    parser.add_argument("--poses-csv")
    args = parser.parse_args()
    export(args.corner_pkl, args.corners_csv, args.poses_csv)


if __name__ == "__main__":
    main()
