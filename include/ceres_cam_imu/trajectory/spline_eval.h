#pragma once

#include <array>

#include <Eigen/Core>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"

namespace ceres_cam_imu {

template <typename T>
inline Eigen::Matrix<T, 6, 1> evalPoseCurve6(
    const SplineSegmentMeta6& meta, const T& timestamp_s,
    const std::array<const T*, 6>& controls, const int derivative_order) {
  const std::array<T, 6> weights = meta.weights(timestamp_s, derivative_order);
  Eigen::Matrix<T, 6, 1> value = Eigen::Matrix<T, 6, 1>::Zero();
  for (int i = 0; i < 6; ++i) {
    const Eigen::Map<const Eigen::Matrix<T, 6, 1>> control(controls[i]);
    value += weights[static_cast<std::size_t>(i)] * control;
  }
  return value;
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> evalBiasCurve6(
    const SplineSegmentMeta6& meta, const T& timestamp_s,
    const std::array<const T*, 6>& controls, const int derivative_order) {
  const std::array<T, 6> weights = meta.weights(timestamp_s, derivative_order);
  Eigen::Matrix<T, 3, 1> value = Eigen::Matrix<T, 3, 1>::Zero();
  for (int i = 0; i < 6; ++i) {
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> control(controls[i]);
    value += weights[static_cast<std::size_t>(i)] * control;
  }
  return value;
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> bodyAngularVelocityFromCurve(
    const Eigen::Matrix<T, 6, 1>& curve,
    const Eigen::Matrix<T, 6, 1>& curve_dot) {
  const Eigen::Matrix<T, 3, 1> r = curve.template tail<3>();
  const Eigen::Matrix<T, 3, 1> r_dot = curve_dot.template tail<3>();
  return -rotationVectorToMatrix(r).transpose() * rotationVectorSMatrix(r) * r_dot;
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> bodyAngularAccelerationFromCurve(
    const Eigen::Matrix<T, 6, 1>& curve,
    const Eigen::Matrix<T, 6, 1>& curve_ddot) {
  const Eigen::Matrix<T, 3, 1> r = curve.template tail<3>();
  const Eigen::Matrix<T, 3, 1> r_ddot = curve_ddot.template tail<3>();
  return -rotationVectorToMatrix(r).transpose() * rotationVectorSMatrix(r) * r_ddot;
}

}  // namespace ceres_cam_imu
