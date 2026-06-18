#pragma once

#include <vector>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

class AprilGrid {
 public:
  explicit AprilGrid(AprilGridConfig config);

  const AprilGridConfig& config() const { return config_; }
  int numTags() const { return config_.tag_cols * config_.tag_rows; }
  int numCorners() const { return numTags() * 4; }

  Vec3 cornerPoint(int corner_id) const;
  std::vector<Vec3> allCornerPoints() const;

 private:
  AprilGridConfig config_;
};

}  // namespace ceres_cam_imu
