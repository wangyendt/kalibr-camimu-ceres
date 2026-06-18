#pragma once

#include <string>
#include <vector>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

std::vector<PoseObservation> readPoseCsv(const std::string& csv_path);

}  // namespace ceres_cam_imu
