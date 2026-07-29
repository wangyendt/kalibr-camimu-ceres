#pragma once

#include <cstdint>
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
  // The default permits every lag that retains at least half of the shorter
  // stream, so independently-originated timestamps do not need to overlap.
  double min_overlap_fraction = 0.5;
  // Optional absolute bound on the reported t_imu = t_cam + shift. Zero means
  // no fixed seconds bound: min_overlap_fraction defines the search extent.
  // A positive value preserves the legacy explicitly bounded behavior.
  double max_search_s = 0.0;
  // Full-range correlation is evaluated on a cheap uniform coarse grid, then
  // refined at the native mean IMU period around the winning coarse peak.
  double coarse_sample_dt_s = 0.01;
  double fine_search_half_width_s = 0.05;
};

// Sign convention, once, for every field below.
//
// The estimator maximizes the raw cross-correlation
//   sum_i predicted[i] * measured[i - lag]
// over lag. That raw argmax ("best_lag") is NOT what this struct reports.
// Kalibr's findTimeshiftCameraImuPrior() negates it -- "shift = -discrete_shift
// * dT", IccSensors.py:283 -- to reach the t_imu = t_cam + shift convention
// (IccSensors.py:300) that camera_reprojection_residual.cpp consumes as
// query_time_s = timestamp_s_ + camera_time_shift_s. We report in that same
// applied-shift sign.
//
// So: every *_discrete_shift_samples field is the nearest APPLIED total shift
// in native-IMU samples. With unrelated timestamp origins the exact origin
// difference need not be an integer number of samples. In that case shift_s
// retains the exact origin term and discrete_shift_residual_s closes the
// diagnostic identity
//   shift_s == discrete_shift_samples * sample_dt_s
//              + discrete_shift_residual_s.
// The mirrored case catches sign-asymmetric handling of the negation; either
// signed case alone already catches a global flip.
//
// Not covered by that invariant: second_best_discrete_shift_samples has no
// paired seconds-valued field, so nothing cross-checks its sign at runtime.
//
// Do not copy this negation into the IMU-to-IMU path: Kalibr's
// findOrientationPrior() (IccSensors.py:920-925) uses the OPPOSITE convention,
// which is why estimateTimeOffsetByGyroNorm() in multi_imu_initializer.cpp
// correctly has no minus sign.
struct TimeShiftPriorEstimate {
  // Applied shift, seconds, in the t_imu = t_cam + shift_s convention.
  double shift_s = 0.0;
  // Applied total shift in samples. Same sign as shift_s. This is 64-bit so a
  // Unix-epoch clock can be aligned with a device-uptime clock.
  std::int64_t discrete_shift_samples = 0;
  double sample_dt_s = 0.0;
  int num_samples = 0;
  double peak_correlation = 0.0;
  // Also an applied shift, not a raw lag. Same sign convention as above.
  std::int64_t second_best_discrete_shift_samples = 0;
  double second_best_correlation = 0.0;
  double zero_lag_correlation = 0.0;
  double predicted_norm_rms = 0.0;
  double measured_norm_rms = 0.0;
  bool boundary_peak_rejected = false;
  // Appended to preserve positional aggregate initialization of the older
  // fields. Exact shift = discrete samples * sample_dt_s + this residual.
  double discrete_shift_residual_s = 0.0;
};

// Resolves a correlation estimate against an already available initialization.
// A boundary peak means that the estimator does not know the shift: it must not
// replace a valid fallback with the estimator's sentinel zero.
struct TimeShiftInitializationResolution {
  double shift_s = 0.0;
  bool used_estimate = false;
  bool kept_fallback = false;
  bool rejected_without_fallback = false;
};

TimeShiftInitializationResolution resolveCameraImuTimeShiftInitialization(
    const TimeShiftPriorEstimate& estimate, double fallback_shift_s,
    bool have_fallback);

TimeShiftPriorEstimate estimateCameraImuTimeShiftPrior(
    const std::vector<PoseObservation>& pose_observations,
    const std::vector<ImuSample>& imu_samples, const CameraExtrinsicBlock& T_c_b,
    const TimeShiftPriorOptions& options);

}  // namespace ceres_cam_imu
