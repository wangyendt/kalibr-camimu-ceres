#include "ceres_cam_imu/optimizer/parameter_delta_tracker.h"

#include <algorithm>
#include <cmath>

namespace ceres_cam_imu {

ParameterDeltaTracker::ParameterDeltaTracker(const ceres::Problem &problem) {
  reset(problem);
}

void ParameterDeltaTracker::reset(const ceres::Problem &problem) {
  blocks_.clear();
  tracked_scalars_ = 0;

  std::vector<double *> parameter_blocks;
  problem.GetParameterBlocks(&parameter_blocks);
  blocks_.reserve(parameter_blocks.size());
  for (double *parameter_block : parameter_blocks) {
    if (problem.IsParameterBlockConstant(parameter_block) ||
        problem.ParameterBlockTangentSize(parameter_block) <= 0) {
      continue;
    }
    ParameterBlockSnapshot snapshot;
    snapshot.data = parameter_block;
    snapshot.values.resize(
        static_cast<std::size_t>(problem.ParameterBlockSize(parameter_block)));
    std::copy(parameter_block, parameter_block + snapshot.values.size(),
              snapshot.values.begin());
    tracked_scalars_ += static_cast<int>(snapshot.values.size());
    blocks_.push_back(snapshot);
  }
}

double ParameterDeltaTracker::updateAndReturnMaxDelta() {
  double max_delta = 0.0;
  for (ParameterBlockSnapshot &block : blocks_) {
    for (std::size_t i = 0; i < block.values.size(); ++i) {
      const double current = block.data[i];
      max_delta = std::max(max_delta, std::abs(current - block.values[i]));
      block.values[i] = current;
    }
  }
  return max_delta;
}

int ParameterDeltaTracker::trackedBlocks() const {
  return static_cast<int>(blocks_.size());
}

int ParameterDeltaTracker::trackedScalars() const { return tracked_scalars_; }

bool ParameterDeltaTracker::empty() const { return blocks_.empty(); }

} // namespace ceres_cam_imu
