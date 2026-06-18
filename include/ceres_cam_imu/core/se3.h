#pragma once

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

inline Mat4 rtToMatrix(const Mat3& R, const Vec3& t) {
  Mat4 T = Mat4::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

inline Mat4 pose6ToMatrix(const Vec6& pose) {
  return rtToMatrix(rotationVectorToMatrix(pose.tail<3>()), pose.head<3>());
}

inline Vec3 transformPoint(const Vec6& T_ab, const Vec3& p_b) {
  return rotationVectorToMatrix(T_ab.tail<3>()) * p_b + T_ab.head<3>();
}

inline Vec3 inverseTransformPoint(const Vec6& T_ab, const Vec3& p_a) {
  return rotationVectorToMatrix(T_ab.tail<3>()).transpose() * (p_a - T_ab.head<3>());
}

}  // namespace ceres_cam_imu
