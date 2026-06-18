#pragma once

#include <string>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

KalibrResult readKalibrResult(const std::string& result_txt_path);

}  // namespace ceres_cam_imu
