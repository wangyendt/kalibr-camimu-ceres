#pragma once

#include <string>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/optimizer/calibration_problem.h"
#include "ceres_cam_imu/optimizer/residual_statistics.h"

namespace ceres_cam_imu {

struct CalibrationResultWriterOptions {
  bool include_kalibr_comparison = false;
  bool include_spline_controls = false;
  KalibrResult kalibr_result;
};

void writeCalibrationResultYaml(
    const std::string& output_path, const CalibrationState& state,
    const CalibrationResidualStatistics& residual_stats,
    const CalibrationResultWriterOptions& options = {});

}  // namespace ceres_cam_imu
