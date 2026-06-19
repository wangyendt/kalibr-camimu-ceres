#!/usr/bin/env python3
"""Export Kalibr cam-IMU initialization splines as neutral CSV samples.

Run inside the Kalibr Python environment. The script mirrors the initialization
part of ``kalibr_calibrate_imu_camera`` and stops before adding residuals or
running the optimizer. It is intended for diagnosing whether this project's
cold-start pose/bias splines numerically match Kalibr's initialization.
"""

import argparse
import csv
import json
import math
import os

import kalibr_common as kc
from kalibr_imu_camera_calibration import IccSensors as sens
import numpy as np


class Parsed(object):
    pass


def rotation_matrix_to_vector(R):
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cos_angle = (float(np.trace(R)) - 1.0) * 0.5
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.acos(cos_angle)
    if angle < 1e-12:
        return np.zeros(3)
    if math.pi - angle < 1e-7:
        axis = np.empty(3)
        axis[0] = math.sqrt(max(0.0, (R[0, 0] + 1.0) * 0.5))
        axis[1] = math.sqrt(max(0.0, (R[1, 1] + 1.0) * 0.5))
        axis[2] = math.sqrt(max(0.0, (R[2, 2] + 1.0) * 0.5))
        if R[0, 1] < 0.0:
            axis[1] = -axis[1]
        if R[0, 2] < 0.0:
            axis[2] = -axis[2]
        norm = np.linalg.norm(axis)
        if norm < 1e-12:
            return np.zeros(3)
        return axis / norm * angle
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2.0 * math.sin(angle))
    return axis * angle


def array3(value):
    return np.asarray(value, dtype=float).reshape(-1)[:3]


def transform_to_dict(T):
    matrix = np.asarray(T.T(), dtype=float).reshape(4, 4)
    return {
        "matrix": matrix.tolist(),
        "translation": matrix[:3, 3].tolist(),
        "rotation_vector": rotation_matrix_to_vector(matrix[:3, :3]).tolist(),
    }


def make_parsed(args):
    parsed = Parsed()
    parsed.bagfile = None
    parsed.h5file = None
    parsed.h5timestampfile = None
    parsed.imufile = None
    parsed.corner_file = [args.corner_file]
    parsed.image_timestamp_file = [args.image_timestamp_file]
    parsed.imu_data_file = args.imu_data_file
    parsed.bag_from_to = args.bag_from_to
    parsed.bag_freq = args.bag_freq
    parsed.perform_synchronization = args.perform_synchronization
    parsed.fixture_id = ["fixture"]
    parsed.trim_imu_edge_count = args.trim_imu_edge_count
    parsed.chain_yaml = args.cams
    parsed.recompute_chain_extrinsics = False
    parsed.reprojection_sigma = args.reprojection_sigma
    parsed.imu_yamls = args.imu
    parsed.estimate_imu_delay = args.imu_delay_by_correlation
    parsed.imu_models = args.imu_models
    parsed.target_yaml = args.target
    parsed.no_time = args.no_time_calibration
    parsed.max_iter = 0
    parsed.recover_cov = False
    parsed.timeoffset_padding = args.timeoffset_padding
    parsed.pose_knots_per_second = args.pose_knots_per_second
    parsed.bias_knots_per_second = args.bias_knots_per_second
    parsed.showextraction = False
    parsed.extractionstepping = False
    parsed.verbose = args.verbose
    parsed.dontShowReport = True
    parsed.exportPoses = False
    return parsed


def instantiate_imu(imu_config, parsed, imu_model, imu_index, have_reference):
    is_reference = not have_reference
    if imu_model == "calibrated":
        return sens.IccImu(imu_config, parsed, isReferenceImu=is_reference,
                           estimateTimedelay=parsed.estimate_imu_delay,
                           imuNr=imu_index)
    if imu_model == "scale-misalignment":
        return sens.IccScaledMisalignedImu(
            imu_config, parsed, isReferenceImu=is_reference,
            estimateTimedelay=parsed.estimate_imu_delay, imuNr=imu_index)
    if imu_model == "scale-misalignment-size-effect":
        return sens.IccScaledMisalignedSizeEffectImu(
            imu_config, parsed, isReferenceImu=is_reference,
            estimateTimedelay=parsed.estimate_imu_delay, imuNr=imu_index)
    raise ValueError("unsupported IMU model: {0}".format(imu_model))


def sample_times(t_min, t_max, sample_hz):
    if sample_hz <= 0.0:
        raise ValueError("--sample-hz must be positive")
    step = 1.0 / sample_hz
    count = int(math.floor((t_max - t_min) / step)) + 1
    return [t_min + i * step for i in range(count + 1)
            if t_min <= t_min + i * step <= t_max]


def write_pose_samples(path, pose_spline, times):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_s",
            "p_x", "p_y", "p_z",
            "r_x", "r_y", "r_z",
            "omega_b_x", "omega_b_y", "omega_b_z",
            "alpha_b_x", "alpha_b_y", "alpha_b_z",
            "accel_w_x", "accel_w_y", "accel_w_z",
        ])
        for t in times:
            R = np.asarray(pose_spline.orientation(t), dtype=float).reshape(3, 3)
            row = [t]
            row.extend(array3(pose_spline.position(t)).tolist())
            row.extend(rotation_matrix_to_vector(R).tolist())
            row.extend(array3(pose_spline.angularVelocityBodyFrame(t)).tolist())
            row.extend(array3(pose_spline.angularAccelerationBodyFrame(t)).tolist())
            row.extend(array3(pose_spline.linearAcceleration(t)).tolist())
            writer.writerow(row)


def write_bias_samples(path, imu, times):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_s",
            "gyro_bias_x", "gyro_bias_y", "gyro_bias_z",
            "accel_bias_x", "accel_bias_y", "accel_bias_z",
        ])
        for t in times:
            row = [t]
            row.extend(array3(imu.gyroBias.evalD(t, 0)).tolist())
            row.extend(array3(imu.accelBias.evalD(t, 0)).tolist())
            writer.writerow(row)


def export(args):
    if len(args.imu) != len(args.imu_data_file):
        raise ValueError("--imu and --imu-data-file counts must match")
    if args.imu_models and len(args.imu_models) != len(args.imu):
        raise ValueError("--imu-models count must match --imu")
    imu_models = args.imu_models or ["calibrated"] * len(args.imu)
    args.imu_models = imu_models

    os.makedirs(args.output_dir, exist_ok=True)
    parsed = make_parsed(args)

    imus = []
    for imu_index, (imu_yaml, imu_model) in enumerate(zip(args.imu, imu_models)):
        imu_config = kc.ImuParameters(imu_yaml)
        imus.append(instantiate_imu(imu_config, parsed, imu_model, imu_index,
                                    have_reference=bool(imus)))

    target_config = kc.CalibrationTargetParameters(args.target)
    chain_config = kc.CameraChainParameters(args.cams)
    camera_chain = sens.IccCameraChain(chain_config, target_config, parsed)

    for imu in imus[1:]:
        imu.findOrientationPrior(imus[0])

    if not args.no_time_calibration:
        for cam in camera_chain.camList:
            cam.findTimeshiftCameraImuPrior(imus[0], args.verbose)

    camera_chain.findOrientationPriorCameraChainToImu(imus[0])
    gravity = np.asarray(camera_chain.getEstimatedGravity(), dtype=float).reshape(3)

    pose_spline = camera_chain.initializePoseSplineFromCameraChain(
        args.spline_order, args.pose_knots_per_second, args.timeoffset_padding)
    for imu in imus:
        imu.initBiasSplines(pose_spline, args.spline_order,
                            args.bias_knots_per_second)

    times = sample_times(pose_spline.t_min(), pose_spline.t_max(), args.sample_hz)
    write_pose_samples(os.path.join(args.output_dir, "kalibr_pose_samples.csv"),
                       pose_spline, times)
    for imu_index, imu in enumerate(imus):
        write_bias_samples(
            os.path.join(args.output_dir,
                         "kalibr_imu{0}_bias_samples.csv".format(imu_index)),
            imu, times)

    meta = {
        "spline_order": args.spline_order,
        "pose_knots_per_second": args.pose_knots_per_second,
        "bias_knots_per_second": args.bias_knots_per_second,
        "timeoffset_padding_s": args.timeoffset_padding,
        "sample_hz": args.sample_hz,
        "pose_t_min_s": float(pose_spline.t_min()),
        "pose_t_max_s": float(pose_spline.t_max()),
        "num_pose_samples": len(times),
        "gravity": gravity.tolist(),
        "cameras": [],
        "imus": [],
    }
    for camera_index, cam in enumerate(camera_chain.camList):
        meta["cameras"].append({
            "camera_index": camera_index,
            "timeshift_cam_to_imu_prior_s":
                float(cam.timeshiftCamToImuPrior),
            "T_extrinsic": transform_to_dict(cam.T_extrinsic),
        })
    for imu_index, imu in enumerate(imus):
        meta["imus"].append({
            "imu_index": imu_index,
            "time_offset_s": float(imu.timeOffset),
            "gyro_bias_prior": np.asarray(imu.GyroBiasPrior,
                                          dtype=float).reshape(3).tolist(),
            "q_i_b_prior": np.asarray(imu.q_i_b_prior,
                                      dtype=float).reshape(4).tolist(),
        })

    with open(os.path.join(args.output_dir, "kalibr_initial_meta.json"), "w") as f:
        json.dump(meta, f, indent=2, sort_keys=True)


def main():
    parser = argparse.ArgumentParser(
        description="Export Kalibr cam-IMU initialization spline samples.")
    parser.add_argument("--corner-file", required=True)
    parser.add_argument("--image-timestamp-file", required=True)
    parser.add_argument("--cams", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--imu", nargs="+", required=True)
    parser.add_argument("--imu-data-file", nargs="+", required=True)
    parser.add_argument("--imu-models", nargs="+")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--spline-order", type=int, default=6)
    parser.add_argument("--pose-knots-per-second", type=int, default=100)
    parser.add_argument("--bias-knots-per-second", type=int, default=50)
    parser.add_argument("--timeoffset-padding", type=float, default=0.04)
    parser.add_argument("--trim-imu-edge-count", type=int, default=1000)
    parser.add_argument("--reprojection-sigma", type=float, default=1.0)
    parser.add_argument("--bag-from-to", metavar="T", type=float, nargs=2)
    parser.add_argument("--bag-freq", type=float)
    parser.add_argument("--perform-synchronization", action="store_true")
    parser.add_argument("--imu-delay-by-correlation", action="store_true")
    parser.add_argument("--no-time-calibration", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    export(parser.parse_args())


if __name__ == "__main__":
    main()
