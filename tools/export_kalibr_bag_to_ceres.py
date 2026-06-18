#!/usr/bin/env python
"""Export Kalibr bag observations to ceres_cam_imu CSV inputs.

Run inside the Kalibr Python environment.  The script intentionally reuses
Kalibr's camera-chain detector so a Ceres comparison consumes the same target
observations that Kalibr would add to its cam-IMU problem.
"""

from __future__ import print_function

import argparse
import csv
import os

import kalibr_common as kc
from kalibr_imu_camera_calibration import IccSensors as sens
import numpy as np


class Parsed(object):
    pass


def _as_rows(array, cols):
    data = np.asarray(array, dtype=float)
    if data.ndim != 2:
        raise ValueError("expected 2D corner array, got shape {0}".format(data.shape))
    if data.shape[1] == cols:
        return data
    if data.shape[0] == cols:
        return data.T
    raise ValueError("cannot interpret shape {0} as rows with {1} columns".format(data.shape, cols))


def _corner_ids(obs, count):
    if hasattr(obs, "getCornersIdx"):
        ids = list(obs.getCornersIdx())
        if len(ids) == count:
            return [int(x) for x in ids]
    return list(range(count))


def _timestamp_ns(timestamp):
    return int(round(float(timestamp.toSec()) * 1000000000.0))


def _write_corners(observations, csv_path):
    with open(csv_path, "w") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_ns",
            "corner_id",
            "pixel_x",
            "pixel_y",
            "target_x",
            "target_y",
            "target_z",
        ])
        for obs in observations:
            image = _as_rows(obs.getCornersImageFrame(), 2)
            target = _as_rows(obs.getCornersTargetFrame(), 3)
            if image.shape[0] != target.shape[0]:
                raise ValueError("image and target corner counts differ")
            ids = _corner_ids(obs, image.shape[0])
            ts = _timestamp_ns(obs.time())
            for corner_id, pixel, point in zip(ids, image, target):
                writer.writerow([
                    ts,
                    corner_id,
                    pixel[0],
                    pixel[1],
                    point[0],
                    point[1],
                    point[2],
                ])


def _write_poses(observations, csv_path):
    with open(csv_path, "w") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns"] + [
            "T_t_c_{0}{1}".format(r, c) for r in range(4) for c in range(4)
        ])
        for obs in observations:
            T = np.asarray(obs.T_t_c().T(), dtype=float).reshape(4, 4)
            writer.writerow([_timestamp_ns(obs.time())] + T.reshape(-1).tolist())


def _write_imu(bag_path, imu_yaml, csv_path, bag_from_to, perform_synchronization):
    imu_config = kc.ImuParameters(imu_yaml)
    reader = kc.BagImuDatasetReader(
        bag_path,
        imu_config.getRosTopic(),
        bag_from_to=bag_from_to,
        perform_synchronization=perform_synchronization,
    )
    with open(csv_path, "w") as f:
        writer = csv.writer(f)
        for timestamp, omega, accel in reader.readDataset():
            writer.writerow([
                _timestamp_ns(timestamp),
                omega[0],
                omega[1],
                omega[2],
                accel[0],
                accel[1],
                accel[2],
            ])
    print("exported {0} IMU samples to {1}".format(reader.numMessages(), csv_path))


def _make_parsed(args):
    parsed = Parsed()
    parsed.bagfile = [args.bag]
    parsed.corner_file = None
    parsed.image_timestamp_file = None
    parsed.h5file = None
    parsed.h5timestampfile = None
    parsed.bag_from_to = args.bag_from_to
    parsed.bag_freq = args.bag_freq
    parsed.perform_synchronization = args.perform_synchronization
    parsed.reprojection_sigma = args.reprojection_sigma
    parsed.showextraction = False
    parsed.extractionstepping = False
    return parsed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--cams", required=True)
    parser.add_argument("--imu", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--bag-from-to", type=float, nargs=2)
    parser.add_argument("--bag-freq", type=float)
    parser.add_argument("--perform-synchronization", action="store_true")
    parser.add_argument("--reprojection-sigma", type=float, default=1.0)
    args = parser.parse_args()

    if not os.path.isdir(args.out_dir):
        os.makedirs(args.out_dir)

    target_config = kc.CalibrationTargetParameters(args.target)
    chain_config = kc.CameraChainParameters(args.cams)
    parsed = _make_parsed(args)
    cam_chain = sens.IccCameraChain(chain_config, target_config, parsed)

    for camera_index, camera in enumerate(cam_chain.camList):
        corners_csv = os.path.join(args.out_dir, "cam{0}_corners.csv".format(camera_index))
        _write_corners(camera.targetObservations, corners_csv)
        print("exported {0} observations to {1}".format(
            len(camera.targetObservations), corners_csv))
        if camera_index == 0:
            poses_csv = os.path.join(args.out_dir, "cam0_corner_poses.csv")
            _write_poses(camera.targetObservations, poses_csv)
            print("exported cam0 poses to {0}".format(poses_csv))

    imu_csv = os.path.join(args.out_dir, "imu.csv")
    _write_imu(args.bag, args.imu, imu_csv, args.bag_from_to, args.perform_synchronization)


if __name__ == "__main__":
    main()
