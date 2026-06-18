#pragma once

#include <string>
#include <vector>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

std::vector<ImageObservation> readCornerCsv(const std::string& csv_path,
                                            int max_frames = 0);

}  // namespace ceres_cam_imu
