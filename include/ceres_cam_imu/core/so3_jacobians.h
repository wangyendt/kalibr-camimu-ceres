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

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> leftJacobianSO3Generic(
    const Eigen::MatrixBase<Derived>& r_expr) {
  using T = typename Derived::Scalar;
  const Eigen::Matrix<T, 3, 1> r = r_expr;
  const T theta2 = r.squaredNorm();
  const Eigen::Matrix<T, 3, 3> K = skew(r);
  const Eigen::Matrix<T, 3, 3> K2 = K * K;
  T A = T(0.5);
  T B = T(1.0 / 6.0);
  if (theta2 < T(1e-12)) {
    const T theta4 = theta2 * theta2;
    const T theta6 = theta4 * theta2;
    A = T(0.5) - theta2 / T(24.0) + theta4 / T(720.0) -
        theta6 / T(40320.0);
    B = T(1.0 / 6.0) - theta2 / T(120.0) + theta4 / T(5040.0) -
        theta6 / T(362880.0);
  } else {
    const T theta = sqrt(theta2);
    A = (T(1.0) - cos(theta)) / theta2;
    B = (theta - sin(theta)) / (theta2 * theta);
  }
  return Eigen::Matrix<T, 3, 3>::Identity() + A * K + B * K2;
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

template <typename DerivedR, typename DerivedV>
inline Eigen::Matrix<typename DerivedR::Scalar, 3, 3>
leftJacobianTimesVectorDerivativeGeneric(
    const Eigen::MatrixBase<DerivedR>& r_expr,
    const Eigen::MatrixBase<DerivedV>& v_expr) {
  using T = typename DerivedR::Scalar;
  const Eigen::Matrix<T, 3, 1> r = r_expr;
  const Eigen::Matrix<T, 3, 1> v = v_expr.template cast<T>();
  const T theta2 = r.squaredNorm();
  const Eigen::Matrix<T, 3, 3> K = skew(r);
  const Eigen::Matrix<T, 3, 1> Kv = K * v;
  const Eigen::Matrix<T, 3, 1> K2v = K * Kv;

  T A = T(0.5);
  T B = T(1.0 / 6.0);
  T dA_dtheta_over_theta = T(-1.0 / 12.0);
  T dB_dtheta_over_theta = T(-1.0 / 60.0);
  if (theta2 < T(1e-12)) {
    const T theta4 = theta2 * theta2;
    const T theta6 = theta4 * theta2;
    A = T(0.5) - theta2 / T(24.0) + theta4 / T(720.0) -
        theta6 / T(40320.0);
    B = T(1.0 / 6.0) - theta2 / T(120.0) + theta4 / T(5040.0) -
        theta6 / T(362880.0);
    dA_dtheta_over_theta =
        T(-1.0 / 12.0) + theta2 / T(180.0) - theta4 / T(6720.0) +
        theta6 / T(453600.0);
    dB_dtheta_over_theta =
        T(-1.0 / 60.0) + theta2 / T(1260.0) - theta4 / T(60480.0) +
        theta6 / T(3991680.0);
  } else {
    const T theta = sqrt(theta2);
    const T sin_theta = sin(theta);
    const T cos_theta = cos(theta);
    A = (T(1.0) - cos_theta) / theta2;
    B = (theta - sin_theta) / (theta2 * theta);
    dA_dtheta_over_theta =
        (theta * sin_theta - T(2.0) * (T(1.0) - cos_theta)) /
        (theta2 * theta2);
    dB_dtheta_over_theta =
        (theta * (T(1.0) - cos_theta) - T(3.0) * (theta - sin_theta)) /
        (theta2 * theta2 * theta);
  }

  Eigen::Matrix<T, 3, 3> derivative = Eigen::Matrix<T, 3, 3>::Zero();
  for (int col = 0; col < 3; ++col) {
    Eigen::Matrix<T, 3, 1> basis = Eigen::Matrix<T, 3, 1>::Zero();
    basis(col) = T(1.0);
    const Eigen::Matrix<T, 3, 3> dK = skew(basis);
    const Eigen::Matrix<T, 3, 1> dKv = dK * v;
    const Eigen::Matrix<T, 3, 1> dK2v = dK * Kv + K * dKv;
    derivative.col(col) =
        dA_dtheta_over_theta * r(col) * Kv + A * dKv +
        dB_dtheta_over_theta * r(col) * K2v + B * dK2v;
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
