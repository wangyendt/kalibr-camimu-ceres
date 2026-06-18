#pragma once

#include <string>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

std::string canonicalCameraModelName(const std::string &model);
std::string canonicalDistortionModelName(const std::string &model);

class CameraModel {
public:
  explicit CameraModel(CameraIntrinsics intrinsics);

  const CameraIntrinsics &intrinsics() const { return intrinsics_; }

  Vec2 project(const Vec3 &p_c) const;
  bool projectWithJacobian(const Vec3 &p_c, Vec2 *pixel,
                           Mat23 *d_pixel_d_point) const;

private:
  CameraIntrinsics intrinsics_;
};

} // namespace ceres_cam_imu
