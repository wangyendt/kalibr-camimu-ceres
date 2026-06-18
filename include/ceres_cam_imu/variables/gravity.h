#pragma once

#include <array>

namespace ceres_cam_imu {

struct GravityBlock {
  static constexpr int kSize = 3;
  std::array<double, kSize> values = {0.0, -9.80655, 0.0};

  double* data() { return values.data(); }
  const double* data() const { return values.data(); }
};

}  // namespace ceres_cam_imu
