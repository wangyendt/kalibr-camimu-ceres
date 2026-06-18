#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ceres_cam_imu {

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(
    const Eigen::MatrixBase<Derived>& v_expr) {
  using T = typename Derived::Scalar;
  const Eigen::Matrix<T, 3, 1> v = v_expr;
  Eigen::Matrix<T, 3, 3> m;
  m << T(0), -v.z(), v.y(),
       v.z(), T(0), -v.x(),
       -v.y(), v.x(), T(0);
  return m;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> rotationVectorToMatrix(
    const Eigen::MatrixBase<Derived>& r_expr) {
  using T = typename Derived::Scalar;
  const Eigen::Matrix<T, 3, 1> r = r_expr;
  const T theta2 = r.squaredNorm();
  const T theta = sqrt(theta2);
  Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity();

  if (theta2 < T(1e-16)) {
    const Eigen::Matrix<T, 3, 3> K = skew(r);
    return R - K + T(0.5) * K * K;
  }

  const Eigen::Matrix<T, 3, 1> axis = r / theta;
  const Eigen::Matrix<T, 3, 3> K = skew(axis);
  return R - sin(theta) * K + (T(1) - cos(theta)) * K * K;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> rotationVectorSMatrix(
    const Eigen::MatrixBase<Derived>& r_expr) {
  using T = typename Derived::Scalar;
  const Eigen::Matrix<T, 3, 1> r = r_expr;
  const T theta2 = r.squaredNorm();
  if (theta2 < T(1e-16)) {
    return Eigen::Matrix<T, 3, 3>::Identity();
  }

  const T theta = sqrt(theta2);
  const Eigen::Matrix<T, 3, 1> axis = r / theta;
  const T c1 = T(-2) * sin(theta * T(0.5)) * sin(theta * T(0.5)) / theta;
  const T c2 = (theta - sin(theta)) / theta;
  const Eigen::Matrix<T, 3, 3> A = skew(axis);
  return Eigen::Matrix<T, 3, 3>::Identity() + c1 * A + c2 * A * A;
}

template <typename RotationDerived, typename PointDerived>
inline Eigen::Matrix<typename RotationDerived::Scalar, 3, 1> rotate(
    const Eigen::MatrixBase<RotationDerived>& r_w_b,
    const Eigen::MatrixBase<PointDerived>& p_b) {
  return rotationVectorToMatrix(r_w_b) * p_b;
}

template <typename RotationDerived, typename PointDerived>
inline Eigen::Matrix<typename RotationDerived::Scalar, 3, 1> inverseRotate(
    const Eigen::MatrixBase<RotationDerived>& r_w_b,
    const Eigen::MatrixBase<PointDerived>& p_w) {
  return rotationVectorToMatrix(r_w_b).transpose() * p_w;
}

Eigen::Vector3d rotationMatrixToVector(const Eigen::Matrix3d& R);

}  // namespace ceres_cam_imu
