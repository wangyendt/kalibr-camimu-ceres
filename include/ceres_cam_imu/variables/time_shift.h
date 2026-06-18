#pragma once

namespace ceres_cam_imu {

struct TimeShiftBlock {
  static constexpr int kSize = 1;
  double value = 0.0;

  double* data() { return &value; }
  const double* data() const { return &value; }
};

}  // namespace ceres_cam_imu
