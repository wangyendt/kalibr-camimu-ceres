#pragma once

#include <vector>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/variables/extrinsics.h"

namespace ceres_cam_imu {

struct TimeShiftPriorOptions {
  int spline_order = 6;
  // Kalibr's findTimeshiftCameraImuPrior() calls initPoseSplineFromCamera()
  // with the default poseKnotsPerSecond=100 and timeOffsetPadding=0.
  double pose_knots_per_second = 100.0;
  // Matches the smoothness lambda passed to Kalibr's initPoseSplineSparse().
  double pose_fit_regularization = 1e-4;
  // Prevent normalized correlation from selecting tiny-overlap edge lags.
  double min_overlap_fraction = 0.5;
  // Camera/IMU time shift is expected to be small; keep IMU/IMU full-overlap
  // search separate from this camera prior.
  double max_search_s = 0.05;
};

struct TimeShiftPriorEstimate {
  double shift_s = 0.0;
  int discrete_shift_samples = 0;
  double sample_dt_s = 0.0;
  int num_samples = 0;
  double peak_correlation = 0.0;
  int second_best_discrete_shift_samples = 0;
  double second_best_correlation = 0.0;
  double zero_lag_correlation = 0.0;
  double predicted_norm_rms = 0.0;
  double measured_norm_rms = 0.0;
  bool boundary_peak_rejected = false;
};

TimeShiftPriorEstimate estimateCameraImuTimeShiftPrior(
    const std::vector<PoseObservation>& pose_observations,
    const std::vector<ImuSample>& imu_samples, const CameraExtrinsicBlock& T_c_b,
    const TimeShiftPriorOptions& options);

}  // namespace ceres_cam_imu
