#pragma once

#include <array>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

struct LowerTriangularMatrixBlock {
  static constexpr int kSize = 6;
  // Row-major lower-triangular active entries:
  // [m00, m10, m11, m20, m21, m22].
  std::array<double, kSize> values = {1.0, 0.0, 1.0, 0.0, 0.0, 1.0};

  double *data() { return values.data(); }
  const double *data() const { return values.data(); }
};

struct Matrix3Block {
  static constexpr int kSize = 9;
  std::array<double, kSize> values = {0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0};

  double *data() { return values.data(); }
  const double *data() const { return values.data(); }
};

struct Vector3Block {
  static constexpr int kSize = 3;
  std::array<double, kSize> values = {0.0, 0.0, 0.0};

  double *data() { return values.data(); }
  const double *data() const { return values.data(); }
};

struct ImuIntrinsicBlocks {
  LowerTriangularMatrixBlock accel_M;
  LowerTriangularMatrixBlock gyro_M;
  Matrix3Block gyro_accel_sensitivity;
  Vector3Block gyro_sensing_rotation;
  Vector3Block accel_axis_rx_i;
  Vector3Block accel_axis_ry_i;
  Vector3Block accel_axis_rz_i;
};

Mat3 lowerTriangularMatrix(const double *block);
Mat3 matrix3Block(const double *block);
Vec3 vector3Block(const double *block);

const char *imuCalibrationModelName(ImuCalibrationModel model);
ImuCalibrationModel parseImuCalibrationModel(const std::string &model);

} // namespace ceres_cam_imu
