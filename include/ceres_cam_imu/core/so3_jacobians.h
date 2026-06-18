#pragma once

#include <cmath>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

inline Mat3 leftJacobianSO3(const Vec3& r) {
  const double theta2 = r.squaredNorm();
  const Mat3 K = skew(r);
  const Mat3 K2 = K * K;
  double A = 0.5;
  double B = 1.0 / 6.0;
  if (theta2 < 1e-12) {
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    A = 0.5 - theta2 / 24.0 + theta4 / 720.0 - theta6 / 40320.0;
    B = 1.0 / 6.0 - theta2 / 120.0 + theta4 / 5040.0
      - theta6 / 362880.0;
  } else {
    const double theta = std::sqrt(theta2);
    A = (1.0 - std::cos(theta)) / theta2;
    B = (theta - std::sin(theta)) / (theta2 * theta);
  }
  return Mat3::Identity() + A * K + B * K2;
}

inline Mat3 leftJacobianTimesVectorDerivative(const Vec3& r, const Vec3& v) {
  const double theta2 = r.squaredNorm();
  const Mat3 K = skew(r);
  const Vec3 Kv = K * v;
  const Vec3 K2v = K * Kv;

  double A = 0.5;
  double B = 1.0 / 6.0;
  double dA_dtheta_over_theta = -1.0 / 12.0;
  double dB_dtheta_over_theta = -1.0 / 60.0;
  if (theta2 < 1e-12) {
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    A = 0.5 - theta2 / 24.0 + theta4 / 720.0 - theta6 / 40320.0;
    B = 1.0 / 6.0 - theta2 / 120.0 + theta4 / 5040.0
      - theta6 / 362880.0;
    dA_dtheta_over_theta =
        -1.0 / 12.0 + theta2 / 180.0 - theta4 / 6720.0
        + theta6 / 453600.0;
    dB_dtheta_over_theta =
        -1.0 / 60.0 + theta2 / 1260.0 - theta4 / 60480.0
        + theta6 / 3991680.0;
  } else {
    const double theta = std::sqrt(theta2);
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    A = (1.0 - cos_theta) / theta2;
    B = (theta - sin_theta) / (theta2 * theta);
    dA_dtheta_over_theta =
        (theta * sin_theta - 2.0 * (1.0 - cos_theta))
        / (theta2 * theta2);
    dB_dtheta_over_theta =
        (theta * (1.0 - cos_theta) - 3.0 * (theta - sin_theta))
        / (theta2 * theta2 * theta);
  }

  Mat3 derivative = Mat3::Zero();
  for (int col = 0; col < 3; ++col) {
    Vec3 basis = Vec3::Zero();
    basis(col) = 1.0;
    const Mat3 dK = skew(basis);
    const Vec3 dKv = dK * v;
    const Vec3 dK2v = dK * Kv + K * dKv;
    derivative.col(col) =
        dA_dtheta_over_theta * r(col) * Kv + A * dKv
        + dB_dtheta_over_theta * r(col) * K2v + B * dK2v;
  }
  return derivative;
}

inline Mat3 rotationTransposeTimesVectorDerivative(const Vec3& r,
                                                   const Vec3& v) {
  const double theta2 = r.squaredNorm();
  const Mat3 K = skew(r);
  const Vec3 Kv = K * v;
  const Vec3 K2v = K * Kv;

  double A = 1.0;
  double B = 0.5;
  double dA_dtheta_over_theta = -1.0 / 3.0;
  double dB_dtheta_over_theta = -1.0 / 12.0;
  if (theta2 < 1e-12) {
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    A = 1.0 - theta2 / 6.0 + theta4 / 120.0 - theta6 / 5040.0;
    B = 0.5 - theta2 / 24.0 + theta4 / 720.0 - theta6 / 40320.0;
    dA_dtheta_over_theta =
        -1.0 / 3.0 + theta2 / 30.0 - theta4 / 840.0
        + theta6 / 45360.0;
    dB_dtheta_over_theta =
        -1.0 / 12.0 + theta2 / 180.0 - theta4 / 6720.0
        + theta6 / 453600.0;
  } else {
    const double theta = std::sqrt(theta2);
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    A = sin_theta / theta;
    B = (1.0 - cos_theta) / theta2;
    dA_dtheta_over_theta = (theta * cos_theta - sin_theta)
                         / (theta2 * theta);
    dB_dtheta_over_theta =
        (theta * sin_theta - 2.0 * (1.0 - cos_theta))
        / (theta2 * theta2);
  }

  Mat3 derivative = Mat3::Zero();
  for (int col = 0; col < 3; ++col) {
    Vec3 basis = Vec3::Zero();
    basis(col) = 1.0;
    const Mat3 dK = skew(basis);
    const Vec3 dKv = dK * v;
    const Vec3 dK2v = dK * Kv + K * dKv;
    derivative.col(col) =
        dA_dtheta_over_theta * r(col) * Kv + A * dKv
        + dB_dtheta_over_theta * r(col) * K2v + B * dK2v;
  }
  return derivative;
}

}  // namespace ceres_cam_imu
