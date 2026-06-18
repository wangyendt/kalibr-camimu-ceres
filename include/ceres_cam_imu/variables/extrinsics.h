#pragma once

#include <array>

namespace ceres_cam_imu {

struct CameraExtrinsicBlock {
  static constexpr int kSize = 6;
  // T_c_b stores translation first, then Kalibr-compatible rotation vector.
  std::array<double, kSize> values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double* data() { return values.data(); }
  const double* data() const { return values.data(); }
};

struct ImuExtrinsicBlock {
  static constexpr int kSize = 6;
  // First three values are lever arm r_b; last three are R_i_b rotation vector.
  std::array<double, kSize> values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double* data() { return values.data(); }
  const double* data() const { return values.data(); }
};

}  // namespace ceres_cam_imu
