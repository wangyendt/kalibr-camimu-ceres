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
// So: every *_discrete_shift_samples field is the APPLIED discrete shift
// (-best_lag), not the raw correlation lag. The upshot is the invariant
//   shift_s == discrete_shift_samples * sample_dt_s
// which holds exactly for shift_s / discrete_shift_samples. tests/test_math.cpp
// asserts it on a positive shift, a negative shift, and on the
// boundary_peak_rejected path (where both are forced to zero). The mirrored
// case is there to catch sign-asymmetric handling of the negation, not to
// disambiguate the convention -- either signed case alone would already catch
// a global flip.
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
  // Applied shift in samples (= -best_lag). Same sign as shift_s.
  int discrete_shift_samples = 0;
  double sample_dt_s = 0.0;
  int num_samples = 0;
  double peak_correlation = 0.0;
  // Also an applied shift, not a raw lag. Same sign convention as above.
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
