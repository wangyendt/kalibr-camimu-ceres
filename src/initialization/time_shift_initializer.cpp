#include "ceres_cam_imu/initialization/time_shift_initializer.h"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ceres_cam_imu/initialization/pose_spline_fit.h"
#include "ceres_cam_imu/trajectory/spline_eval.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"
#include "ceres_cam_imu/variables/pose_control.h"

namespace ceres_cam_imu {
namespace {

std::pair<double, double> poseTimeSpan(
    const std::vector<PoseObservation>& pose_observations) {
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  for (const PoseObservation& observation : pose_observations) {
    first = std::min(first, observation.timestamp_s);
    last = std::max(last, observation.timestamp_s);
  }
  if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
    throw std::runtime_error("cannot estimate time shift from degenerate pose times");
  }
  return {first, last};
}

double meanImuDt(const std::vector<ImuSample>& imu_samples) {
  if (imu_samples.size() < 2) {
    throw std::runtime_error("at least two IMU samples are required for time shift estimation");
  }
  double total_dt = 0.0;
  int count = 0;
  for (std::size_t i = 1; i < imu_samples.size(); ++i) {
    const double dt = imu_samples[i].timestamp_s - imu_samples[i - 1].timestamp_s;
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

Vec3 angularVelocityAt(const UniformBSpline& spline,
                       const std::vector<PoseControlBlock>& controls,
                       const double timestamp_s) {
  const SplineSegmentMeta6 meta = spline.segmentMeta6(timestamp_s);
  std::array<const double*, SplineSegmentMeta6::kOrder> active{};
  for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
    active[static_cast<std::size_t>(i)] =
        controls.at(static_cast<std::size_t>(meta.coeff_start + i)).data();
  }
  const Vec6 curve = evalPoseCurve6(meta, timestamp_s, active, 0);
  const Vec6 curve_dot = evalPoseCurve6(meta, timestamp_s, active, 1);
  return bodyAngularVelocityFromCurve(curve, curve_dot);
}

double rms(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  double sum_sq = 0.0;
  for (const double value : values) {
    sum_sq += value * value;
  }
  return std::sqrt(sum_sq / static_cast<double>(values.size()));
}

}  // namespace

TimeShiftPriorEstimate estimateCameraImuTimeShiftPrior(
    const std::vector<PoseObservation>& pose_observations,
    const std::vector<ImuSample>& imu_samples, const CameraExtrinsicBlock& T_c_b,
    const TimeShiftPriorOptions& options) {
  if (options.spline_order != SplineSegmentMeta6::kOrder) {
    throw std::runtime_error("time shift estimator currently requires order-6 splines");
  }
  if (pose_observations.empty()) {
    throw std::runtime_error("pose observations are required for time shift estimation");
  }

  const auto [first_pose_time, last_pose_time] = poseTimeSpan(pose_observations);
  const UniformBSpline pose_spline = makeSplineForTimes(
      6, options.spline_order, first_pose_time, last_pose_time,
      options.pose_knots_per_second, 0.0);

  std::vector<PoseControlBlock> pose_controls;
  PoseSplineFitOptions fit_options;
  fit_options.regularization = 1e-12;
  // Kalibr's sparse pose-spline initialization adds a second-derivative
  // smoothness term for order > 2 splines. Keep the diagonal damping tiny and
  // expose the Kalibr lambda through TimeShiftPriorOptions.
  fit_options.motion_regularization = options.pose_fit_regularization;
  fit_options.motion_regularization_order = 2;
  fit_options.unwrap_rotation_vectors = true;
  const PoseSplineFitSummary fit_summary = fitPoseSplineControlsFromCameraPoses(
      pose_observations, T_c_b, 0.0, pose_spline, fit_options, &pose_controls);
  if (fit_summary.used_observations == 0) {
    throw std::runtime_error("no pose observations overlap the time shift spline");
  }

  std::vector<double> measured_norms;
  std::vector<double> predicted_norms;
  measured_norms.reserve(imu_samples.size());
  predicted_norms.reserve(imu_samples.size());
  for (const ImuSample& sample : imu_samples) {
    const double t = sample.timestamp_s;
    if (t <= pose_spline.tMin() || t >= pose_spline.tMax()) {
      continue;
    }
    measured_norms.push_back(sample.gyro_rad_s.norm());
    predicted_norms.push_back(angularVelocityAt(pose_spline, pose_controls, t).norm());
  }

  if (predicted_norms.empty() || measured_norms.empty()) {
    throw std::runtime_error(
        "the camera pose and IMU time ranges do not overlap for time shift estimation");
  }
  if (predicted_norms.size() != measured_norms.size()) {
    throw std::logic_error("time shift norm vectors must have equal size");
  }

  const int n = static_cast<int>(predicted_norms.size());
  int best_lag = 0;
  double best_correlation = -std::numeric_limits<double>::infinity();
  for (int lag = -(n - 1); lag <= n - 1; ++lag) {
    double correlation = 0.0;
    for (int i = 0; i < n; ++i) {
      const int j = i - lag;
      if (j >= 0 && j < n) {
        correlation += predicted_norms[static_cast<std::size_t>(i)]
                     * measured_norms[static_cast<std::size_t>(j)];
      }
    }
    if (correlation > best_correlation) {
      best_correlation = correlation;
      best_lag = lag;
    }
  }

  TimeShiftPriorEstimate estimate;
  estimate.discrete_shift_samples = best_lag;
  estimate.sample_dt_s = meanImuDt(imu_samples);
  estimate.shift_s = -static_cast<double>(best_lag) * estimate.sample_dt_s;
  estimate.num_samples = n;
  estimate.peak_correlation = best_correlation;
  estimate.predicted_norm_rms = rms(predicted_norms);
  estimate.measured_norm_rms = rms(measured_norms);
  return estimate;
}

}  // namespace ceres_cam_imu
