#include "ceres_cam_imu/core/so3.h"

#include <algorithm>
#include <cmath>

namespace ceres_cam_imu {

Eigen::Vector3d rotationMatrixToVector(const Eigen::Matrix3d& R) {
  const double trace_value = (R.trace() - 1.0) * 0.5;
  const double c = std::max(-1.0, std::min(1.0, trace_value));
  const double angle = std::acos(c);
  if (std::abs(angle) < 1e-14) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d v;
  v << R(2, 1) - R(1, 2),
       R(0, 2) - R(2, 0),
       R(1, 0) - R(0, 1);
  const double n = v.norm();
  if (n < 1e-14) {
    return Eigen::Vector3d::Zero();
  }
  return -angle / n * v;
}

}  // namespace ceres_cam_imu
