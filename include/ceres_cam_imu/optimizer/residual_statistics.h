#pragma once

#include <string>
#include <vector>

#include "ceres_cam_imu/optimizer/calibration_problem.h"

namespace ceres_cam_imu {

struct ResidualMagnitudeStats {
  int count = 0;
  double mean = 0.0;
  double median = 0.0;
  double stddev = 0.0;
  double rms = 0.0;
  double max = 0.0;
};

struct ImuResidualOutlier {
  int sample_index = -1;
  double timestamp_s = 0.0;
  double accel_error_m_s2 = 0.0;
  double accel_normalized = 0.0;
  double gyro_error_rad_s = 0.0;
  double gyro_normalized = 0.0;
  double measured_accel_norm = 0.0;
  double predicted_accel_norm = 0.0;
  double pose_accel_world_norm = 0.0;
  double gravity_corrected_body_accel_norm = 0.0;
  double angular_accel_lever_norm = 0.0;
  double centripetal_lever_norm = 0.0;
  double omega_body_norm = 0.0;
  double alpha_body_norm = 0.0;
};

struct CalibrationResidualStatistics {
  ResidualMagnitudeStats reprojection_px;
  ResidualMagnitudeStats reprojection_normalized;
  ResidualMagnitudeStats gyro_rad_s;
  ResidualMagnitudeStats gyro_normalized;
  ResidualMagnitudeStats accel_m_s2;
  ResidualMagnitudeStats accel_normalized;
  std::vector<ImuResidualOutlier> top_accel_outliers;
  int skipped_camera_frames = 0;
  int skipped_camera_projections = 0;
  int skipped_imu_samples = 0;
};

CalibrationResidualStatistics evaluateCalibrationResidualStatistics(
    const CameraIntrinsics& intrinsics, const ImuNoise& imu_noise,
    const std::vector<ImageObservation>& images,
    const std::vector<ImuSample>& imu_samples,
    const CalibrationOptions& options, const CalibrationState& state);

CalibrationResidualStatistics evaluateCalibrationResidualStatistics(
    const std::vector<CameraObservationDataset>& cameras,
    const ImuNoise& imu_noise, const std::vector<ImuSample>& imu_samples,
    const CalibrationOptions& options, const CalibrationState& state);

void writeImuDiagnosticsCsv(const std::string& output_path,
                            const std::vector<ImuSample>& imu_samples,
                            const CalibrationOptions& options,
                            const CalibrationState& state);

}  // namespace ceres_cam_imu
