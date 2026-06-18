#pragma once

#include <array>

namespace ceres_cam_imu {

struct PoseControlBlock {
  static constexpr int kSize = 6;
  std::array<double, kSize> values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double* data() { return values.data(); }
  const double* data() const { return values.data(); }
};

}  // namespace ceres_cam_imu
