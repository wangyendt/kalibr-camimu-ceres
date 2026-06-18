#pragma once

#include <vector>

#include <ceres/ceres.h>

namespace ceres_cam_imu {

class ParameterDeltaTracker {
public:
  ParameterDeltaTracker() = default;
  explicit ParameterDeltaTracker(const ceres::Problem &problem);

  void reset(const ceres::Problem &problem);
  double updateAndReturnMaxDelta();

  int trackedBlocks() const;
  int trackedScalars() const;
  bool empty() const;

private:
  struct ParameterBlockSnapshot {
    double *data = nullptr;
    std::vector<double> values;
  };

  std::vector<ParameterBlockSnapshot> blocks_;
  int tracked_scalars_ = 0;
};

} // namespace ceres_cam_imu
