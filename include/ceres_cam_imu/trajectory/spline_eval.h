#pragma once

#include <array>

#include <Eigen/Core>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/so3_jacobians.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"

namespace ceres_cam_imu {

template <typename T>
struct PoseSplineKinematics {
  Eigen::Matrix<T, 3, 1> position = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> velocity = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> acceleration = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 3> R_w_b = Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, 3, 3> R_b_w = Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, 3, 3> R_dot_w_b = Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, 3, 3> R_ddot_w_b = Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, 3, 1> omega_b = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> alpha_b = Eigen::Matrix<T, 3, 1>::Zero();
};

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> veeSkew(
    const Eigen::MatrixBase<Derived>& skew_matrix) {
  using T = typename Derived::Scalar;
  return Eigen::Matrix<T, 3, 1>(skew_matrix(2, 1), skew_matrix(0, 2),
                                skew_matrix(1, 0));
}

template <typename T>
inline std::array<T, SplineSegmentMeta6::kOrder> cumulativeWeights(
    const std::array<T, SplineSegmentMeta6::kOrder>& weights) {
  std::array<T, SplineSegmentMeta6::kOrder> cumulative{};
  T sum = T(0);
  for (int i = SplineSegmentMeta6::kOrder - 1; i >= 0; --i) {
    sum += weights[static_cast<std::size_t>(i)];
    cumulative[static_cast<std::size_t>(i)] = sum;
  }
  return cumulative;
}

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
inline PoseSplineKinematics<T> evalPoseSplineKinematicsSO3(
    const SplineSegmentMeta6& meta, const T& timestamp_s,
    const std::array<const T*, SplineSegmentMeta6::kOrder>& controls) {
  const std::array<T, SplineSegmentMeta6::kOrder> weights =
      meta.weights(timestamp_s, 0);
  const std::array<T, SplineSegmentMeta6::kOrder> dot_weights =
      meta.weights(timestamp_s, 1);
  const std::array<T, SplineSegmentMeta6::kOrder> ddot_weights =
      meta.weights(timestamp_s, 2);
  const std::array<T, SplineSegmentMeta6::kOrder> cumulative =
      cumulativeWeights(weights);
  const std::array<T, SplineSegmentMeta6::kOrder> cumulative_dot =
      cumulativeWeights(dot_weights);
  const std::array<T, SplineSegmentMeta6::kOrder> cumulative_ddot =
      cumulativeWeights(ddot_weights);

  PoseSplineKinematics<T> kinematics;
  std::array<Eigen::Matrix<T, 3, 3>, SplineSegmentMeta6::kOrder>
      control_rotations{};

  for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
    const Eigen::Map<const Eigen::Matrix<T, 6, 1>> control(controls[i]);
    kinematics.position += weights[static_cast<std::size_t>(i)] *
                           control.template head<3>();
    kinematics.velocity += dot_weights[static_cast<std::size_t>(i)] *
                           control.template head<3>();
    kinematics.acceleration += ddot_weights[static_cast<std::size_t>(i)] *
                               control.template head<3>();
    control_rotations[static_cast<std::size_t>(i)] =
        rotationVectorToMatrix(control.template tail<3>());
  }

  Eigen::Matrix<T, 3, 3> R = control_rotations[0];
  Eigen::Matrix<T, 3, 3> R_dot = Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, 3, 3> R_ddot = Eigen::Matrix<T, 3, 3>::Zero();

  for (int i = 1; i < SplineSegmentMeta6::kOrder; ++i) {
    const Eigen::Matrix<T, 3, 3> R_delta =
        control_rotations[static_cast<std::size_t>(i - 1)].transpose() *
        control_rotations[static_cast<std::size_t>(i)];
    const Eigen::Matrix<T, 3, 1> delta = rotationMatrixToVector(R_delta);
    const T beta = cumulative[static_cast<std::size_t>(i)];
    const T beta_dot = cumulative_dot[static_cast<std::size_t>(i)];
    const T beta_ddot = cumulative_ddot[static_cast<std::size_t>(i)];
    const Eigen::Matrix<T, 3, 1> phi = beta * delta;
    const Eigen::Matrix<T, 3, 1> phi_dot = beta_dot * delta;
    const Eigen::Matrix<T, 3, 1> phi_ddot = beta_ddot * delta;

    const Eigen::Matrix<T, 3, 3> A = rotationVectorToMatrix(phi);
    const Eigen::Matrix<T, 3, 3> J_left = leftJacobianSO3Generic(phi);
    const Eigen::Matrix<T, 3, 1> omega_A = -J_left * phi_dot;
    const Eigen::Matrix<T, 3, 1> alpha_A =
        -(leftJacobianTimesVectorDerivativeGeneric(phi, phi_dot) * phi_dot +
          J_left * phi_ddot);
    const Eigen::Matrix<T, 3, 3> omega_A_x = skew(omega_A);
    const Eigen::Matrix<T, 3, 3> alpha_A_x = skew(alpha_A);
    const Eigen::Matrix<T, 3, 3> A_dot = A * omega_A_x;
    const Eigen::Matrix<T, 3, 3> A_ddot =
        A * (omega_A_x * omega_A_x + alpha_A_x);

    const Eigen::Matrix<T, 3, 3> next_R = R * A;
    const Eigen::Matrix<T, 3, 3> next_R_dot = R_dot * A + R * A_dot;
    const Eigen::Matrix<T, 3, 3> next_R_ddot =
        R_ddot * A + T(2) * R_dot * A_dot + R * A_ddot;
    R = next_R;
    R_dot = next_R_dot;
    R_ddot = next_R_ddot;
  }

  kinematics.R_w_b = R;
  kinematics.R_b_w = R.transpose();
  kinematics.R_dot_w_b = R_dot;
  kinematics.R_ddot_w_b = R_ddot;
  const Eigen::Matrix<T, 3, 3> omega_x = R.transpose() * R_dot;
  kinematics.omega_b = veeSkew(omega_x);
  kinematics.alpha_b =
      veeSkew(R.transpose() * R_ddot - omega_x * omega_x);
  return kinematics;
}

template <typename T>
inline Eigen::Matrix<T, 6, 1> evalPoseSO3(
    const SplineSegmentMeta6& meta, const T& timestamp_s,
    const std::array<const T*, SplineSegmentMeta6::kOrder>& controls) {
  const PoseSplineKinematics<T> kinematics =
      evalPoseSplineKinematicsSO3(meta, timestamp_s, controls);
  Eigen::Matrix<T, 6, 1> pose;
  pose.template head<3>() = kinematics.position;
  pose.template tail<3>() = rotationMatrixToVector(kinematics.R_w_b);
  return pose;
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
