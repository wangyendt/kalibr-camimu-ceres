#include "ceres_cam_imu/target/aprilgrid.h"

#include <stdexcept>

namespace ceres_cam_imu {

AprilGrid::AprilGrid(AprilGridConfig config) : config_(config) {}

Vec3 AprilGrid::cornerPoint(const int corner_id) const {
  if (corner_id < 0 || corner_id >= numCorners()) {
    throw std::out_of_range("AprilGrid corner id out of range");
  }

  const int tag_id = corner_id / 4;
  const int corner_in_tag = corner_id % 4;
  const int row = tag_id / config_.tag_cols;
  const int col = tag_id % config_.tag_cols;
  const double step = config_.tag_size_m * (1.0 + config_.tag_spacing_ratio);
  const double x0 = static_cast<double>(col) * step;
  const double y0 = static_cast<double>(row) * step;
  const double s = config_.tag_size_m;

  switch (corner_in_tag) {
    case 0:
      return Vec3(x0, y0, 0.0);
    case 1:
      return Vec3(x0 + s, y0, 0.0);
    case 2:
      return Vec3(x0 + s, y0 + s, 0.0);
    case 3:
      return Vec3(x0, y0 + s, 0.0);
    default:
      throw std::out_of_range("invalid AprilGrid corner slot");
  }
}

std::vector<Vec3> AprilGrid::allCornerPoints() const {
  std::vector<Vec3> points;
  points.reserve(static_cast<std::size_t>(numCorners()));
  for (int i = 0; i < numCorners(); ++i) {
    points.push_back(cornerPoint(i));
  }
  return points;
}

}  // namespace ceres_cam_imu
