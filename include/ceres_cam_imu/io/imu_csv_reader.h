#pragma once

#include <string>
#include <vector>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

std::vector<ImuSample> readImuCsv(const std::string& csv_path,
                                  int trim_edge_count = 0);

}  // namespace ceres_cam_imu
