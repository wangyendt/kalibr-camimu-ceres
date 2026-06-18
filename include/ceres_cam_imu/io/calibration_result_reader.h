#pragma once

#include <string>
#include <vector>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

struct CalibrationResultDelta {
  double rotation_deg = 0.0;
  double translation_m = 0.0;
  double time_shift_s = 0.0;
  double gravity_norm = 0.0;
};

struct CalibrationResultFile {
  int format_version = 0;
  Mat4 T_c_b = Mat4::Identity();
  Mat4 T_b_c = Mat4::Identity();
  double time_shift_s = 0.0;
  Vec3 gravity = Vec3::Zero();
  KalibrResidualStats residuals;
  std::vector<Mat4> camera_T_c_b;
  std::vector<double> camera_time_shift_s;
  bool has_accel_M = false;
  bool has_gyro_M = false;
  bool has_gyro_accel_sensitivity = false;
  bool has_gyro_sensing_rotation = false;
  bool has_accel_axis_rx_i = false;
  bool has_accel_axis_ry_i = false;
  bool has_accel_axis_rz_i = false;
  Mat3 accel_M = Mat3::Identity();
  Mat3 gyro_M = Mat3::Identity();
  Mat3 gyro_accel_sensitivity = Mat3::Zero();
  Mat3 gyro_sensing_rotation = Mat3::Identity();
  Vec3 accel_axis_rx_i = Vec3::Zero();
  Vec3 accel_axis_ry_i = Vec3::Zero();
  Vec3 accel_axis_rz_i = Vec3::Zero();
  bool has_kalibr_delta = false;
  CalibrationResultDelta kalibr_delta;
};

CalibrationResultFile readCalibrationResultYaml(
    const std::string& result_yaml_path);

}  // namespace ceres_cam_imu
