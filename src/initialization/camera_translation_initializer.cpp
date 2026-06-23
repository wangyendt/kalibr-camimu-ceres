#include "ceres_cam_imu/initialization/camera_translation_initializer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/initialization/pose_spline_fit.h"
#include "ceres_cam_imu/trajectory/spline_eval.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"
#include "ceres_cam_imu/variables/pose_control.h"

namespace ceres_cam_imu {
namespace {

std::pair<double, double>
shiftedPoseTimeSpan(const std::vector<PoseObservation> &pose_observations,
                    const double camera_time_shift_s) {
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  for (const PoseObservation &observation : pose_observations) {
    const double t = observation.timestamp_s + camera_time_shift_s;
    first = std::min(first, t);
    last = std::max(last, t);
  }
  if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
    throw std::runtime_error(
        "cannot estimate camera translation from degenerate pose times");
  }
  return {first, last};
}

Vec6 poseCurveAt(const UniformBSpline &spline,
                 const std::vector<PoseControlBlock> &controls,
                 const double timestamp_s, const int derivative_order) {
  const SplineSegmentMeta6 meta = spline.segmentMeta6(timestamp_s);
  std::array<const double *, SplineSegmentMeta6::kOrder> active{};
  for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
    active[static_cast<std::size_t>(i)] =
        controls.at(static_cast<std::size_t>(meta.coeff_start + i)).data();
  }
  return evalPoseCurve6(meta, timestamp_s, active, derivative_order);
}

void appendLeastSquaresRows(const Mat3 &A_r, const Mat3 &A_bias,
                            const Vec3 &rhs,
                            Eigen::MatrixXd *normal_matrix,
                            Eigen::VectorXd *normal_rhs) {
  Eigen::Matrix<double, 3, 6> A;
  A.block<3, 3>(0, 0) = A_r;
  A.block<3, 3>(0, 3) = A_bias;
  *normal_matrix += A.transpose() * A;
  *normal_rhs += A.transpose() * rhs;
}

double rmsAccelError(const Eigen::MatrixXd &rows, const Eigen::VectorXd &rhs,
                     const Eigen::VectorXd &solution) {
  if (rows.rows() == 0) {
    return 0.0;
  }
  const Eigen::VectorXd residual = rows * solution - rhs;
  return std::sqrt(residual.squaredNorm() /
                   static_cast<double>(rows.rows() / 3));
}

} // namespace

CameraTranslationInitializerResult estimateCameraTranslationAndAccelBiasPrior(
    const std::vector<PoseObservation> &pose_observations,
    const std::vector<ImuSample> &imu_samples,
    const CameraExtrinsicBlock &initial_T_c_b, const Vec3 &gravity_m_s2,
    const double camera_time_shift_s,
    const CameraTranslationInitializerOptions &options) {
  if (options.spline_order != SplineSegmentMeta6::kOrder) {
    throw std::runtime_error(
        "camera translation initializer currently requires order-6 splines");
  }
  if (pose_observations.empty()) {
    throw std::runtime_error(
        "pose observations are required for camera translation initialization");
  }
  if (imu_samples.empty()) {
    throw std::runtime_error(
        "IMU samples are required for camera translation initialization");
  }
  if (options.pose_knots_per_second <= 0.0 ||
      options.pose_fit_regularization < 0.0 || options.sample_stride <= 0 ||
      options.min_samples <= 0 || options.min_excitation < 0.0 ||
      options.min_lever_jacobian_norm < 0.0 ||
      options.max_translation_norm_m <= 0.0) {
    throw std::invalid_argument(
        "camera translation initializer options are out of range");
  }
  if (gravity_m_s2.norm() <= 0.0) {
    throw std::runtime_error(
        "camera translation initializer requires non-zero gravity");
  }

  const auto [first_pose_time, last_pose_time] =
      shiftedPoseTimeSpan(pose_observations, camera_time_shift_s);
  const UniformBSpline camera_pose_spline = makeSplineForTimes(
      6, options.spline_order, first_pose_time, last_pose_time,
      options.pose_knots_per_second, 0.0);

  CameraExtrinsicBlock identity_T_c_b;
  identity_T_c_b.values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  PoseSplineFitOptions fit_options;
  fit_options.motion_regularization = options.pose_fit_regularization;
  fit_options.motion_regularization_order = 2;
  fit_options.add_boundary_anchors = options.pose_fit_boundary_anchors;
  fit_options.unwrap_rotation_vectors = true;

  std::vector<PoseControlBlock> camera_pose_controls;
  const PoseSplineFitSummary fit_summary = fitPoseSplineControlsFromCameraPoses(
      pose_observations, identity_T_c_b, camera_time_shift_s,
      camera_pose_spline, fit_options, &camera_pose_controls);
  if (fit_summary.used_observations == 0) {
    throw std::runtime_error(
        "no pose observations overlap the camera translation initializer spline");
  }

  const Vec3 r_c_b(initial_T_c_b.values[3], initial_T_c_b.values[4],
                  initial_T_c_b.values[5]);
  const Mat3 R_c_b = rotationVectorToMatrix(r_c_b);
  const Mat3 R_i_c = R_c_b.transpose();

  std::vector<Eigen::Matrix<double, 3, 6>> row_blocks;
  std::vector<Vec3> rhs_blocks;
  row_blocks.reserve(imu_samples.size());
  rhs_blocks.reserve(imu_samples.size());
  Eigen::MatrixXd normal_matrix = Eigen::MatrixXd::Zero(6, 6);
  Eigen::VectorXd normal_rhs = Eigen::VectorXd::Zero(6);
  int used_samples = 0;
  const int stride = std::max(1, options.sample_stride);
  for (std::size_t sample_index = 0; sample_index < imu_samples.size();
       sample_index += static_cast<std::size_t>(stride)) {
    const ImuSample &sample = imu_samples[sample_index];
    const double t = sample.timestamp_s;
    if (t <= camera_pose_spline.tMin() || t >= camera_pose_spline.tMax()) {
      continue;
    }
    const Vec6 curve = poseCurveAt(camera_pose_spline, camera_pose_controls, t, 0);
    const Vec6 curve_dot =
        poseCurveAt(camera_pose_spline, camera_pose_controls, t, 1);
    const Vec6 curve_ddot =
        poseCurveAt(camera_pose_spline, camera_pose_controls, t, 2);
    const Mat3 R_w_c = rotationVectorToMatrix(curve.tail<3>());
    const Mat3 R_c_w = R_w_c.transpose();
    const Vec3 camera_specific_force_c =
        R_c_w * (curve_ddot.head<3>() - gravity_m_s2);
    const Vec3 omega_c = bodyAngularVelocityFromCurve(curve, curve_dot);
    const Vec3 alpha_c = bodyAngularAccelerationFromCurve(curve, curve_ddot);
    const Mat3 lever_jacobian =
        skew(alpha_c) + skew(omega_c) * skew(omega_c);
    if (lever_jacobian.norm() < options.min_lever_jacobian_norm) {
      continue;
    }
    const Mat3 A_r = R_i_c * lever_jacobian;
    const Mat3 A_bias = Mat3::Identity();
    const Vec3 rhs = sample.accel_m_s2 - R_i_c * camera_specific_force_c;

    Eigen::Matrix<double, 3, 6> A;
    A.block<3, 3>(0, 0) = A_r;
    A.block<3, 3>(0, 3) = A_bias;
    row_blocks.push_back(A);
    rhs_blocks.push_back(rhs);
    appendLeastSquaresRows(A_r, A_bias, rhs, &normal_matrix, &normal_rhs);
    ++used_samples;
  }

  if (used_samples < options.min_samples) {
    throw std::runtime_error(
        "not enough overlapping samples for camera translation initialization");
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> normal_svd(
      normal_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::VectorXd singular = normal_svd.singularValues();
  if (singular.size() < 6 ||
      singular(2) <= options.min_excitation ||
      singular(5) <= 0.0) {
    throw std::runtime_error(
        "camera translation excitation is too weak for initialization");
  }
  const Eigen::VectorXd solution = normal_svd.solve(normal_rhs);
  const Vec3 t_c_b = solution.head<3>();
  if (!t_c_b.allFinite() || t_c_b.norm() > options.max_translation_norm_m) {
    throw std::runtime_error(
        "camera translation initializer produced an invalid translation");
  }

  Eigen::MatrixXd rows(3 * used_samples, 6);
  Eigen::VectorXd rhs(3 * used_samples);
  for (int i = 0; i < used_samples; ++i) {
    rows.block<3, 6>(3 * i, 0) = row_blocks[static_cast<std::size_t>(i)];
    rhs.segment<3>(3 * i) = rhs_blocks[static_cast<std::size_t>(i)];
  }

  CameraTranslationInitializerResult result;
  result.t_c_b_m = t_c_b;
  result.accel_bias_m_s2 = solution.tail<3>();
  result.num_samples = used_samples;
  result.accel_rms_m_s2 = rmsAccelError(rows, rhs, solution);
  result.singular_values =
      Vec3(singular(0), singular(1), singular(2));
  result.pose_fit_rms_translation_m = fit_summary.rms_translation_m;
  result.pose_fit_rms_rotation_rad = fit_summary.rms_rotation_rad;
  result.pose_fit_boundary_anchor_observations =
      fit_summary.boundary_anchor_observations;
  return result;
}

} // namespace ceres_cam_imu
