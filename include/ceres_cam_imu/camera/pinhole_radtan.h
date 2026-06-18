#pragma once

#include <Eigen/Core>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

class PinholeRadtanCamera {
 public:
  explicit PinholeRadtanCamera(CameraIntrinsics intrinsics);

  const CameraIntrinsics& intrinsics() const { return intrinsics_; }

  Vec2 project(const Vec3& p_c) const;
  bool projectWithJacobian(const Vec3& p_c, Vec2* pixel, Mat23* d_pixel_d_point) const;

  template <typename T>
  bool project(const Eigen::Matrix<T, 3, 1>& p_c,
               Eigen::Matrix<T, 2, 1>* pixel) const {
    const T x = p_c.x() / p_c.z();
    const T y = p_c.y() / p_c.z();
    const T r2 = x * x + y * y;
    const T r4 = r2 * r2;
    const T radial = T(1) + T(intrinsics_.k1) * r2 + T(intrinsics_.k2) * r4;
    const T xy = x * y;
    const T xd = x * radial + T(2.0 * intrinsics_.p1) * xy
               + T(intrinsics_.p2) * (r2 + T(2) * x * x);
    const T yd = y * radial + T(intrinsics_.p1) * (r2 + T(2) * y * y)
               + T(2.0 * intrinsics_.p2) * xy;
    (*pixel).x() = T(intrinsics_.fx) * xd + T(intrinsics_.cx);
    (*pixel).y() = T(intrinsics_.fy) * yd + T(intrinsics_.cy);
    return true;
  }

 private:
  CameraIntrinsics intrinsics_;
};

}  // namespace ceres_cam_imu
