#include "ceres_cam_imu/initialization/time_shift_initializer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ceres_cam_imu/initialization/pose_spline_fit.h"
#include "ceres_cam_imu/trajectory/spline_eval.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"
#include "ceres_cam_imu/variables/pose_control.h"
#include "uniform_signal_correlation.h"

namespace ceres_cam_imu {
namespace {

std::pair<double, double>
poseTimeSpan(const std::vector<PoseObservation> &pose_observations) {
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  for (const PoseObservation &observation : pose_observations) {
    first = std::min(first, observation.timestamp_s);
    last = std::max(last, observation.timestamp_s);
  }
  if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
    throw std::runtime_error(
        "cannot estimate time shift from degenerate pose times");
  }
  return {first, last};
}

double meanImuDt(const std::vector<ImuSample> &imu_samples) {
  if (imu_samples.size() < 2) {
    throw std::runtime_error(
        "at least two IMU samples are required for time shift estimation");
  }
  double total_dt = 0.0;
  int count = 0;
  for (std::size_t i = 1; i < imu_samples.size(); ++i) {
    const double dt =
        imu_samples[i].timestamp_s - imu_samples[i - 1].timestamp_s;
    if (dt > 0.0) {
      total_dt += dt;
      ++count;
    }
  }
  if (count == 0) {
    throw std::runtime_error("IMU timestamps are not increasing");
  }
  return total_dt / static_cast<double>(count);
}

std::vector<ImuSample>
deduplicateImuTimestamps(const std::vector<ImuSample> &imu_samples) {
  std::vector<ImuSample> unique;
  unique.reserve(imu_samples.size());
  for (const ImuSample &sample : imu_samples) {
    if (!unique.empty() && sample.timestamp_s < unique.back().timestamp_s) {
      throw std::runtime_error(
          "IMU timestamps must be nondecreasing for time shift estimation");
    }
    if (!unique.empty() && sample.timestamp_s == unique.back().timestamp_s) {
      unique.back() = sample;
      continue;
    }
    unique.push_back(sample);
  }
  if (unique.size() < 2) {
    throw std::runtime_error(
        "at least two distinct IMU timestamps are required for time shift "
        "estimation");
  }
  return unique;
}

Vec3 angularVelocityAt(const UniformBSpline &spline,
                       const std::vector<PoseControlBlock> &controls,
                       const double timestamp_s) {
  const SplineSegmentMeta6 meta = spline.segmentMeta6(timestamp_s);
  std::array<const double *, SplineSegmentMeta6::kOrder> active{};
  for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
    active[static_cast<std::size_t>(i)] =
        controls.at(static_cast<std::size_t>(meta.coeff_start + i)).data();
  }
  const Vec6 curve = evalPoseCurve6(meta, timestamp_s, active, 0);
  const Vec6 curve_dot = evalPoseCurve6(meta, timestamp_s, active, 1);
  return bodyAngularVelocityFromCurve(curve, curve_dot);
}

double rms(const std::vector<double> &values) {
  if (values.empty()) {
    return 0.0;
  }
  double sum_sq = 0.0;
  for (const double value : values) {
    sum_sq += value * value;
  }
  return std::sqrt(sum_sq / static_cast<double>(values.size()));
}

internal::UniformScalarSignal
makePredictedNormSignal(const UniformBSpline &spline,
                        const std::vector<PoseControlBlock> &controls,
                        const double sample_dt_s) {
  internal::UniformScalarSignal signal;
  signal.sample_dt_s = sample_dt_s;
  // angularVelocityAt() requires a proper interior spline segment. Removing
  // one grid interval at each edge is negligible for the overlap calculation
  // and avoids turning endpoint conventions into a clock-alignment signal.
  signal.start_time_s = spline.tMin() + sample_dt_s;
  const double end_time_s = spline.tMax() - sample_dt_s;
  const int count = internal::uniformSampleCount(signal.start_time_s,
                                                 end_time_s, sample_dt_s);
  signal.values.reserve(static_cast<std::size_t>(std::max(0, count)));
  for (int i = 0; i < count; ++i) {
    const double timestamp_s =
        signal.start_time_s + static_cast<double>(i) * sample_dt_s;
    signal.values.push_back(
        angularVelocityAt(spline, controls, timestamp_s).norm());
  }
  return signal;
}

internal::UniformScalarSignal
makeMeasuredNormSignal(const std::vector<ImuSample> &imu_samples,
                       const double sample_dt_s) {
  if (imu_samples.size() < 2) {
    throw std::runtime_error(
        "at least two IMU samples are required for time shift estimation");
  }
  internal::UniformScalarSignal signal;
  signal.start_time_s = imu_samples.front().timestamp_s;
  signal.sample_dt_s = sample_dt_s;
  const double end_time_s = imu_samples.back().timestamp_s;
  const int count = internal::uniformSampleCount(signal.start_time_s,
                                                 end_time_s, sample_dt_s);
  signal.values.reserve(static_cast<std::size_t>(std::max(0, count)));

  std::size_t upper = 1;
  for (int i = 0; i < count; ++i) {
    const double timestamp_s =
        signal.start_time_s + static_cast<double>(i) * sample_dt_s;
    while (upper + 1 < imu_samples.size() &&
           imu_samples[upper].timestamp_s < timestamp_s) {
      ++upper;
    }
    const ImuSample &before = imu_samples[upper - 1];
    const ImuSample &after = imu_samples[upper];
    const double interval_s = after.timestamp_s - before.timestamp_s;
    const double alpha =
        std::clamp((timestamp_s - before.timestamp_s) / interval_s, 0.0, 1.0);
    signal.values.push_back((1.0 - alpha) * before.gyro_rad_s.norm() +
                            alpha * after.gyro_rad_s.norm());
  }
  return signal;
}

} // namespace

TimeShiftInitializationResolution
resolveCameraImuTimeShiftInitialization(const TimeShiftPriorEstimate &estimate,
                                        const double fallback_shift_s,
                                        const bool have_fallback) {
  TimeShiftInitializationResolution resolution;
  if (!estimate.boundary_peak_rejected) {
    resolution.shift_s = estimate.shift_s;
    resolution.used_estimate = true;
    return resolution;
  }
  if (have_fallback) {
    resolution.shift_s = fallback_shift_s;
    resolution.kept_fallback = true;
    return resolution;
  }
  resolution.rejected_without_fallback = true;
  return resolution;
}

TimeShiftPriorEstimate estimateCameraImuTimeShiftPrior(
    const std::vector<PoseObservation> &pose_observations,
    const std::vector<ImuSample> &imu_samples,
    const CameraExtrinsicBlock &T_c_b, const TimeShiftPriorOptions &options) {
  if (options.spline_order != SplineSegmentMeta6::kOrder) {
    throw std::runtime_error(
        "time shift estimator currently requires order-6 splines");
  }
  if (pose_observations.empty()) {
    throw std::runtime_error(
        "pose observations are required for time shift estimation");
  }
  if (!(std::isfinite(options.min_overlap_fraction) &&
        options.min_overlap_fraction > 0.0 &&
        options.min_overlap_fraction <= 1.0) ||
      !(std::isfinite(options.max_search_s) && options.max_search_s >= 0.0) ||
      !(std::isfinite(options.coarse_sample_dt_s) &&
        options.coarse_sample_dt_s > 0.0) ||
      !(std::isfinite(options.fine_search_half_width_s) &&
        options.fine_search_half_width_s > 0.0)) {
    throw std::invalid_argument(
        "time shift estimator options are out of range");
  }

  const auto [first_pose_time, last_pose_time] =
      poseTimeSpan(pose_observations);
  const UniformBSpline pose_spline =
      makeSplineForTimes(6, options.spline_order, first_pose_time,
                         last_pose_time, options.pose_knots_per_second, 0.0);

  std::vector<PoseControlBlock> pose_controls;
  PoseSplineFitOptions fit_options;
  // Kalibr's sparse pose-spline initialization adds a second-derivative
  // smoothness term for order > 2 splines, without diagonal damping.
  fit_options.motion_regularization = options.pose_fit_regularization;
  fit_options.motion_regularization_order = 2;
  fit_options.unwrap_rotation_vectors = true;
  const PoseSplineFitSummary fit_summary = fitPoseSplineControlsFromCameraPoses(
      pose_observations, T_c_b, 0.0, pose_spline, fit_options, &pose_controls);
  if (fit_summary.used_observations == 0) {
    throw std::runtime_error(
        "no pose observations overlap the time shift spline");
  }

  const std::vector<ImuSample> correlation_imu_samples =
      deduplicateImuTimestamps(imu_samples);
  const double fine_sample_dt_s = meanImuDt(correlation_imu_samples);
  const internal::UniformScalarSignal coarse_predicted =
      makePredictedNormSignal(pose_spline, pose_controls,
                              options.coarse_sample_dt_s);
  const internal::UniformScalarSignal coarse_measured = makeMeasuredNormSignal(
      correlation_imu_samples, options.coarse_sample_dt_s);
  if (coarse_predicted.values.size() < 10 ||
      coarse_measured.values.size() < 10) {
    throw std::runtime_error(
        "not enough camera or IMU duration for time shift estimation");
  }
  const internal::UniformScalarSignal fine_predicted =
      makePredictedNormSignal(pose_spline, pose_controls, fine_sample_dt_s);
  const internal::UniformScalarSignal fine_measured =
      makeMeasuredNormSignal(correlation_imu_samples, fine_sample_dt_s);
  constexpr internal::AppliedShiftConvention kCameraShiftConvention =
      internal::AppliedShiftConvention::kRightMinusLeftMinusLag;
  const internal::CoarseToFineCorrelationResult correlation =
      internal::searchCoarseToFineCorrelation(
          coarse_predicted, coarse_measured, fine_predicted, fine_measured,
          options.min_overlap_fraction, options.max_search_s,
          options.fine_search_half_width_s, kCameraShiftConvention);
  const auto &fine = correlation.fine;
  const double best_shift_s = correlation.best_shift_s;
  const double second_best_shift_s = correlation.second_best_shift_s;
  const std::int64_t best_shift_samples =
      std::llround(best_shift_s / fine_sample_dt_s);
  const std::int64_t second_best_shift_samples =
      std::llround(second_best_shift_s / fine_sample_dt_s);

  const int zero_shift_lag = internal::lagForAppliedShift(
      fine_predicted, fine_measured, 0.0, kCameraShiftConvention);
  internal::CorrelationAtLag zero_shift_stats;
  if (zero_shift_lag >= correlation.fine_global_first_lag &&
      zero_shift_lag <= correlation.fine_global_last_lag) {
    zero_shift_stats = internal::normalizedCorrelationAtLag(
        fine_predicted, fine_measured, static_cast<int>(zero_shift_lag));
  }

  // best_lag is the raw correlation lag on independently rebased uniform
  // signals: it maximizes
  //   sum_i predicted[i] * measured[i - lag],
  // i.e. it satisfies measured[n] ~ predicted[n + best_lag]. The reported
  // total shift is measured_start - predicted_start - best_lag * dt. When the
  // timestamp origins already agree, this reduces to the opposite-lag sign
  // used by Kalibr's
  // findTimeshiftCameraImuPrior() ("shift = -discrete_shift*dT",
  // IccSensors.py:283) and the t_imu = t_cam + shift convention that
  // camera_reprojection_residual.cpp applies as
  // query_time_s = timestamp_s_ + camera_time_shift_s. The
  // *_discrete_shift_samples fields contain the nearest total applied shift in
  // native-IMU samples. A non-grid-aligned epoch difference is retained in
  // shift_s and exposed separately as discrete_shift_residual_s.
  //
  // Kalibr's two cross-correlation paths use OPPOSITE conventions and must not
  // be copied from one another: the cam-IMU path negates, the IMU-IMU path
  // (findOrientationPrior, IccSensors.py:920-925) does not -- which is why
  // estimateTimeOffsetByGyroNorm() in multi_imu_initializer.cpp correctly has
  // no minus sign. See docs/books/kalibr_cam_imu_from_equations_to_ceres/
  // 13_多相机与多IMU因子图.md, the "符号陷阱" callout in 13.9.
  TimeShiftPriorEstimate estimate;
  estimate.sample_dt_s = fine_sample_dt_s;
  if (correlation.global_boundary) {
    estimate.boundary_peak_rejected = true;
    estimate.discrete_shift_samples = 0;
    estimate.shift_s = 0.0;
    estimate.discrete_shift_residual_s = 0.0;
  } else {
    estimate.discrete_shift_samples = best_shift_samples;
    estimate.shift_s = best_shift_s;
    estimate.discrete_shift_residual_s =
        estimate.shift_s -
        static_cast<double>(estimate.discrete_shift_samples) *
            estimate.sample_dt_s;
  }
  estimate.num_samples = static_cast<int>(fine_measured.values.size());
  estimate.peak_correlation = fine.best_correlation;
  estimate.second_best_discrete_shift_samples = second_best_shift_samples;
  estimate.second_best_correlation = std::isfinite(fine.second_best_correlation)
                                         ? fine.second_best_correlation
                                         : 0.0;
  estimate.zero_lag_correlation =
      std::isfinite(zero_shift_stats.value) ? zero_shift_stats.value : 0.0;
  estimate.predicted_norm_rms = rms(fine_predicted.values);
  estimate.measured_norm_rms = rms(fine_measured.values);
  return estimate;
}

} // namespace ceres_cam_imu
