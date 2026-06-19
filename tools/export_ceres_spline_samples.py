#!/usr/bin/env python3
"""Export and compare Ceres pose/bias spline samples from a result YAML."""

import argparse
import csv
import json
import math
import os
from functools import lru_cache

import numpy as np
import yaml


def derivative_multiplier(power, derivative_order):
    if derivative_order == 0:
        return 1.0
    value = 1.0
    for i in range(derivative_order):
        value *= power - i
    return value


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def left_jacobian_so3(r):
    theta2 = float(np.dot(r, r))
    k = skew(r)
    if theta2 < 1e-12:
        return np.eye(3) + 0.5 * k + (1.0 / 6.0) * k.dot(k)
    theta = math.sqrt(theta2)
    a = (1.0 - math.cos(theta)) / theta2
    b = (theta - math.sin(theta)) / (theta2 * theta)
    return np.eye(3) + a * k + b * k.dot(k)


class UniformBSpline:
    def __init__(self, metadata):
        self.order = int(metadata["order"])
        self.t_min = float(metadata["t_min_s"])
        self.t_max = float(metadata["t_max_s"])
        self.dt = float(metadata["dt_s"])
        self.num_segments = int(metadata["num_segments"])
        self.num_coefficients = int(metadata["num_coefficients"])
        self.knots = [
            self.t_min + float(i - self.order + 1) * self.dt
            for i in range(self.num_coefficients + self.order)
        ]

    def is_valid_time(self, timestamp_s):
        return self.t_min <= timestamp_s <= self.t_max

    def segment_index(self, timestamp_s):
        if not self.is_valid_time(timestamp_s):
            raise ValueError("timestamp outside spline interval")
        if timestamp_s == self.t_max:
            return self.num_segments - 1
        s = (timestamp_s - self.t_min) / self.dt
        return min(max(int(math.floor(s)), 0), self.num_segments - 1)

    def d0(self, k, i, j):
        denom = self.knots[j + k - 1] - self.knots[j]
        if denom <= 0.0:
            return 0.0
        return (self.knots[i] - self.knots[j]) / denom

    def d1(self, k, i, j):
        denom = self.knots[j + k - 1] - self.knots[j]
        if denom <= 0.0:
            return 0.0
        return (self.knots[i + 1] - self.knots[i]) / denom

    @lru_cache(maxsize=None)
    def basis_matrix_recursive(self, k, i):
        if k == 1:
            return np.ones((1, 1), dtype=float)

        previous = self.basis_matrix_recursive(k - 1, i)
        m1 = np.zeros((previous.shape[0] + 1, previous.shape[1]))
        m2 = np.zeros((previous.shape[0] + 1, previous.shape[1]))
        m1[: previous.shape[0], :] = previous
        m2[1 : 1 + previous.shape[0], :] = previous

        a = np.zeros((k - 1, k))
        b = np.zeros((k - 1, k))
        for row in range(a.shape[0]):
            j = i - k + 2 + row
            d_0 = self.d0(k, i, j)
            d_1 = self.d1(k, i, j)
            a[row, row] = 1.0 - d_0
            a[row, row + 1] = d_0
            b[row, row] = -d_1
            b[row, row + 1] = d_1
        return m1.dot(a) + m2.dot(b)

    def basis_matrix(self, segment_index):
        return self.basis_matrix_recursive(self.order, segment_index + self.order - 1)

    def weights(self, timestamp_s, derivative_order):
        segment = self.segment_index(timestamp_s)
        segment_start = self.t_min + float(segment) * self.dt
        normalized_u = (timestamp_s - segment_start) / self.dt
        scale = self.dt ** (-derivative_order)
        u = np.zeros(self.order)
        power = 1.0
        for i in range(self.order):
            if i >= derivative_order:
                u[i] = scale * derivative_multiplier(i, derivative_order) * power
                power *= normalized_u
        return segment, self.basis_matrix(segment).T.dot(u)

    def evaluate(self, controls, timestamp_s, derivative_order):
        if len(controls) != self.num_coefficients:
            raise ValueError("coefficient count does not match spline")
        segment, weights = self.weights(timestamp_s, derivative_order)
        value = np.zeros(len(controls[0]), dtype=float)
        for i, weight in enumerate(weights):
            value += weight * np.asarray(controls[segment + i], dtype=float)
        return value


def load_ceres_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def sample_times(t_min, t_max, sample_hz):
    step = 1.0 / sample_hz
    count = int(math.floor((t_max - t_min) / step)) + 1
    return [t_min + i * step for i in range(count + 1)
            if t_min <= t_min + i * step <= t_max]


def read_csv_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def read_csv_times(path):
    return [float(row["timestamp_s"]) for row in read_csv_rows(path)]


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def pose_sample_row(pose_spline, pose_controls, timestamp_s):
    curve = pose_spline.evaluate(pose_controls, timestamp_s, 0)
    curve_dot = pose_spline.evaluate(pose_controls, timestamp_s, 1)
    curve_ddot = pose_spline.evaluate(pose_controls, timestamp_s, 2)
    r_w_b = curve[3:6]
    omega_b = -left_jacobian_so3(r_w_b).dot(curve_dot[3:6])
    alpha_b = -left_jacobian_so3(r_w_b).dot(curve_ddot[3:6])
    return [
        timestamp_s,
        *curve[0:3],
        *r_w_b,
        *omega_b,
        *alpha_b,
        *curve_ddot[0:3],
    ]


def bias_sample_row(gyro_spline, accel_spline, gyro_controls, accel_controls,
                    timestamp_s):
    gyro_bias = gyro_spline.evaluate(gyro_controls, timestamp_s, 0)
    accel_bias = accel_spline.evaluate(accel_controls, timestamp_s, 0)
    return [timestamp_s, *gyro_bias, *accel_bias]


def controls_from(node):
    controls = node.get("controls")
    if controls is None:
        raise ValueError("result YAML does not contain exported spline controls")
    return [list(map(float, row)) for row in controls]


def imu_bias_controls(data, imu_count):
    bias = data["bias_splines"]
    base_gyro = controls_from(bias["gyro"])
    base_accel = controls_from(bias["accel"])
    gyro_by_imu = [base_gyro for _ in range(imu_count)]
    accel_by_imu = [base_accel for _ in range(imu_count)]

    for item in bias.get("by_imu", []) or []:
        imu_index = int(item["imu_index"])
        if imu_index >= imu_count:
            continue
        if "gyro" in item and "controls" in item["gyro"]:
            gyro_by_imu[imu_index] = controls_from(item["gyro"])
        if "accel" in item and "controls" in item["accel"]:
            accel_by_imu[imu_index] = controls_from(item["accel"])
    return gyro_by_imu, accel_by_imu


def stats(values):
    if not values:
        return {"count": 0}
    arr = np.asarray(values, dtype=float)
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "rms": float(math.sqrt(np.mean(arr * arr))),
        "max": float(np.max(arr)),
    }


def compare_csv(reference_path, generated_path, columns):
    reference = read_csv_rows(reference_path)
    generated = read_csv_rows(generated_path)
    generated_by_time = {round(float(row["timestamp_s"]), 9): row
                         for row in generated}
    diffs = {column: [] for column in columns}
    for ref_row in reference:
        key = round(float(ref_row["timestamp_s"]), 9)
        gen_row = generated_by_time.get(key)
        if gen_row is None:
            continue
        for column in columns:
            diffs[column].append(float(gen_row[column]) - float(ref_row[column]))
    return {column: stats(values) for column, values in diffs.items()}


def write_summary(path, summary):
    with open(path, "w") as f:
        json.dump(summary, f, indent=2, sort_keys=True)


def export(args):
    data = load_ceres_yaml(args.input)
    os.makedirs(args.output_dir, exist_ok=True)

    pose_spline = UniformBSpline(data["pose_spline"])
    gyro_spline = UniformBSpline(data["bias_splines"]["gyro"])
    accel_spline = UniformBSpline(data["bias_splines"]["accel"])
    pose_controls = controls_from(data["pose_spline"])

    imu_count = args.imu_count
    if imu_count is None:
        imu_count = max(1, len(data.get("imu_chain", []) or []))
    gyro_by_imu, accel_by_imu = imu_bias_controls(data, imu_count)

    times = None
    if args.kalibr_dir:
        kalibr_pose_path = os.path.join(args.kalibr_dir, "kalibr_pose_samples.csv")
        if os.path.exists(kalibr_pose_path):
            times = [
                t for t in read_csv_times(kalibr_pose_path)
                if pose_spline.is_valid_time(t)
            ]
    if times is None:
        times = sample_times(pose_spline.t_min, pose_spline.t_max, args.sample_hz)

    pose_header = [
        "timestamp_s",
        "p_x", "p_y", "p_z",
        "r_x", "r_y", "r_z",
        "omega_b_x", "omega_b_y", "omega_b_z",
        "alpha_b_x", "alpha_b_y", "alpha_b_z",
        "accel_w_x", "accel_w_y", "accel_w_z",
    ]
    pose_rows = [pose_sample_row(pose_spline, pose_controls, t) for t in times]
    ceres_pose_path = os.path.join(args.output_dir, "ceres_pose_samples.csv")
    write_csv(ceres_pose_path, pose_header, pose_rows)

    bias_header = [
        "timestamp_s",
        "gyro_bias_x", "gyro_bias_y", "gyro_bias_z",
        "accel_bias_x", "accel_bias_y", "accel_bias_z",
    ]
    for imu_index in range(imu_count):
        rows = [
            bias_sample_row(gyro_spline, accel_spline, gyro_by_imu[imu_index],
                            accel_by_imu[imu_index], t)
            for t in times
        ]
        write_csv(os.path.join(args.output_dir,
                               "ceres_imu{0}_bias_samples.csv".format(imu_index)),
                  bias_header, rows)

    summary = {
        "input": args.input,
        "sample_count": len(times),
        "sample_t_min_s": min(times) if times else None,
        "sample_t_max_s": max(times) if times else None,
        "imu_count": imu_count,
    }

    if args.kalibr_dir:
        kalibr_pose_path = os.path.join(args.kalibr_dir, "kalibr_pose_samples.csv")
        if os.path.exists(kalibr_pose_path):
            summary["pose_delta_ceres_minus_kalibr"] = compare_csv(
                kalibr_pose_path, ceres_pose_path, pose_header[1:])
        bias_summary = {}
        for imu_index in range(imu_count):
            ref = os.path.join(args.kalibr_dir,
                               "kalibr_imu{0}_bias_samples.csv".format(imu_index))
            gen = os.path.join(args.output_dir,
                               "ceres_imu{0}_bias_samples.csv".format(imu_index))
            if os.path.exists(ref):
                bias_summary["imu{0}".format(imu_index)] = compare_csv(
                    ref, gen, bias_header[1:])
        if bias_summary:
            summary["bias_delta_ceres_minus_kalibr"] = bias_summary

    summary_path = os.path.join(args.output_dir, "ceres_spline_sample_summary.json")
    write_summary(summary_path, summary)
    print(json.dumps(summary, indent=2, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(
        description="Export Ceres result YAML spline samples and optionally compare to Kalibr samples.")
    parser.add_argument("--input", required=True,
                        help="Ceres result YAML produced with --export-spline-controls.")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--imu-count", type=int)
    parser.add_argument("--kalibr-dir")
    export(parser.parse_args())


if __name__ == "__main__":
    main()
