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

MultiImuTranslationInitializerResult
estimateMultiImuCameraTranslationAndLeverPrior(
    const std::vector<PoseObservation> &pose_observations,
    const std::vector<ImuObservationDataset> &imus,
    const std::vector<ImuExtrinsicBlock> &initial_imu_extrinsics,
    const std::vector<double> &imu_time_offsets_s,
    const CameraExtrinsicBlock &initial_T_c_b,
    const Vec3 &gravity_m_s2,
    const double camera_time_shift_s,
    const MultiImuTranslationInitializerOptions &options) {
  if (options.spline_order != SplineSegmentMeta6::kOrder) {
    throw std::runtime_error(
        "multi-IMU translation initializer currently requires order-6 splines");
  }
  if (pose_observations.empty()) {
    throw std::runtime_error(
        "pose observations are required for multi-IMU translation initialization");
  }
  if (imus.empty()) {
    throw std::runtime_error(
        "IMU samples are required for multi-IMU translation initialization");
  }
  if (initial_imu_extrinsics.size() < imus.size()) {
    throw std::runtime_error(
        "multi-IMU translation initialization requires IMU chain rotations");
  }
  if (options.pose_knots_per_second <= 0.0 ||
      options.pose_fit_regularization < 0.0 || options.sample_stride <= 0 ||
      options.min_samples <= 0 || options.min_excitation < 0.0 ||
      options.min_lever_jacobian_norm < 0.0 ||
      options.max_translation_norm_m <= 0.0 ||
      options.max_lever_arm_norm_m <= 0.0 ||
      (options.camera_prior_sigma_m != -1.0 &&
       options.camera_prior_sigma_m <= 0.0) ||
      (options.lever_prior_sigma_m != -1.0 &&
       options.lever_prior_sigma_m <= 0.0) ||
      (options.accel_bias_prior_sigma_m_s2 != -1.0 &&
       options.accel_bias_prior_sigma_m_s2 <= 0.0)) {
    throw std::invalid_argument(
        "multi-IMU translation initializer options are out of range");
  }
  if (gravity_m_s2.norm() <= 0.0) {
    throw std::runtime_error(
        "multi-IMU translation initializer requires non-zero gravity");
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
        "no pose observations overlap the multi-IMU translation initializer spline");
  }

  const std::size_t imu_count = imus.size();
  const int variable_count =
      3 + 3 * static_cast<int>(imu_count - 1) +
      3 * static_cast<int>(imu_count);
  const auto lever_index = [](const std::size_t imu_index) {
    return 3 + 3 * static_cast<int>(imu_index - 1);
  };
  const auto bias_index = [imu_count](const std::size_t imu_index) {
    return 3 + 3 * static_cast<int>(imu_count - 1) +
           3 * static_cast<int>(imu_index);
  };

  const Vec3 r_c_b(initial_T_c_b.values[3], initial_T_c_b.values[4],
                  initial_T_c_b.values[5]);
  const Mat3 R_c_b = rotationVectorToMatrix(r_c_b);
  const Mat3 R_b_c = R_c_b.transpose();

  std::vector<Mat3> R_i_c_by_imu;
  R_i_c_by_imu.reserve(imu_count);
  for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
    const ImuExtrinsicBlock &extrinsic = initial_imu_extrinsics[imu_index];
    const Vec3 r_i_b(extrinsic.values[3], extrinsic.values[4],
                     extrinsic.values[5]);
    const Mat3 R_i_b = rotationVectorToMatrix(r_i_b);
    R_i_c_by_imu.push_back(R_i_b * R_b_c);
  }

  Eigen::MatrixXd normal_matrix =
      Eigen::MatrixXd::Zero(variable_count, variable_count);
  Eigen::VectorXd normal_rhs = Eigen::VectorXd::Zero(variable_count);
  int used_samples = 0;
  const int stride = std::max(1, options.sample_stride);

  const auto fillRow = [&](const std::size_t imu_index,
                           const ImuSample &raw_sample,
                           Eigen::MatrixXd *A,
                           Vec3 *rhs) -> bool {
    const double imu_time_offset_s =
        imu_index < imu_time_offsets_s.size()
            ? imu_time_offsets_s[imu_index]
            : 0.0;
    const double t = raw_sample.timestamp_s + imu_time_offset_s;
    if (t <= camera_pose_spline.tMin() || t >= camera_pose_spline.tMax()) {
      return false;
    }
    const Vec6 curve = poseCurveAt(camera_pose_spline, camera_pose_controls,
                                   t, 0);
    const Vec6 curve_dot = poseCurveAt(camera_pose_spline,
                                       camera_pose_controls, t, 1);
    const Vec6 curve_ddot = poseCurveAt(camera_pose_spline,
                                        camera_pose_controls, t, 2);
    const Mat3 R_w_c = rotationVectorToMatrix(curve.tail<3>());
    const Mat3 R_c_w = R_w_c.transpose();
    const Vec3 camera_specific_force_c =
        R_c_w * (curve_ddot.head<3>() - gravity_m_s2);
    const Vec3 omega_c = bodyAngularVelocityFromCurve(curve, curve_dot);
    const Vec3 alpha_c = bodyAngularAccelerationFromCurve(curve, curve_ddot);
    const Mat3 lever_jacobian_c =
        skew(alpha_c) + skew(omega_c) * skew(omega_c);
    if (lever_jacobian_c.norm() < options.min_lever_jacobian_norm) {
      return false;
    }

    A->setZero(3, variable_count);
    const Mat3 &R_i_c = R_i_c_by_imu[imu_index];
    A->block<3, 3>(0, 0) = R_i_c * lever_jacobian_c;
    if (imu_index > 0) {
      A->block<3, 3>(0, lever_index(imu_index)) =
          R_i_c * lever_jacobian_c * R_c_b;
    }
    A->block<3, 3>(0, bias_index(imu_index)) = Mat3::Identity();
    *rhs = raw_sample.accel_m_s2 - R_i_c * camera_specific_force_c;
    return true;
  };

  for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
    const std::vector<ImuSample> &samples = imus[imu_index].samples;
    for (std::size_t sample_index = 0; sample_index < samples.size();
         sample_index += static_cast<std::size_t>(stride)) {
      Eigen::MatrixXd A;
      Vec3 rhs = Vec3::Zero();
      if (!fillRow(imu_index, samples[sample_index], &A, &rhs)) {
        continue;
      }
      normal_matrix += A.transpose() * A;
      normal_rhs += A.transpose() * rhs;
      ++used_samples;
    }
  }

  if (options.camera_prior_sigma_m > 0.0) {
    const double information =
        1.0 / (options.camera_prior_sigma_m * options.camera_prior_sigma_m);
    const Vec3 prior(initial_T_c_b.values[0], initial_T_c_b.values[1],
                     initial_T_c_b.values[2]);
    normal_matrix.block<3, 3>(0, 0) += information * Mat3::Identity();
    normal_rhs.segment<3>(0) += information * prior;
  }

  if (options.lever_prior_sigma_m > 0.0) {
    const double information =
        1.0 / (options.lever_prior_sigma_m * options.lever_prior_sigma_m);
    for (std::size_t imu_index = 1; imu_index < imu_count; ++imu_index) {
      const int index = lever_index(imu_index);
      Vec3 prior = Vec3::Zero();
      if (imu_index < initial_imu_extrinsics.size()) {
        const ImuExtrinsicBlock &extrinsic =
            initial_imu_extrinsics[imu_index];
        prior = Vec3(extrinsic.values[0], extrinsic.values[1],
                     extrinsic.values[2]);
      }
      normal_matrix.block<3, 3>(index, index) +=
          information * Mat3::Identity();
      normal_rhs.segment<3>(index) += information * prior;
    }
  }

  if (options.accel_bias_prior_sigma_m_s2 > 0.0) {
    const double information =
        1.0 / (options.accel_bias_prior_sigma_m_s2 *
               options.accel_bias_prior_sigma_m_s2);
    for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
      const int index = bias_index(imu_index);
      normal_matrix.block<3, 3>(index, index) +=
          information * Mat3::Identity();
    }
  }

  if (used_samples < options.min_samples) {
    throw std::runtime_error(
        "not enough overlapping samples for multi-IMU translation initialization");
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> normal_svd(
      normal_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::VectorXd singular = normal_svd.singularValues();
  if (singular.size() < variable_count ||
      singular(std::min<int>(2, singular.size() - 1)) <=
          options.min_excitation ||
      singular(variable_count - 1) <= 0.0) {
    throw std::runtime_error(
        "multi-IMU translation excitation is too weak for initialization");
  }
  const Eigen::VectorXd solution = normal_svd.solve(normal_rhs);
  const Vec3 t_c_b = solution.segment<3>(0);
  if (!t_c_b.allFinite() || t_c_b.norm() > options.max_translation_norm_m) {
    throw std::runtime_error(
        "multi-IMU translation initializer produced an invalid camera translation");
  }

  MultiImuTranslationInitializerResult result;
  result.t_c_b_m = t_c_b;
  result.r_b_m.assign(imu_count, Vec3::Zero());
  result.accel_bias_m_s2.assign(imu_count, Vec3::Zero());
  for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
    if (imu_index > 0) {
      const Vec3 r_b = solution.segment<3>(lever_index(imu_index));
      if (!r_b.allFinite() || r_b.norm() > options.max_lever_arm_norm_m) {
        throw std::runtime_error(
            "multi-IMU translation initializer produced an invalid lever arm");
      }
      result.r_b_m[imu_index] = r_b;
    }
    const Vec3 bias = solution.segment<3>(bias_index(imu_index));
    if (!bias.allFinite()) {
      throw std::runtime_error(
          "multi-IMU translation initializer produced an invalid accel bias");
    }
    result.accel_bias_m_s2[imu_index] = bias;
  }

  double sum_sq = 0.0;
  int residual_samples = 0;
  for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
    const std::vector<ImuSample> &samples = imus[imu_index].samples;
    for (std::size_t sample_index = 0; sample_index < samples.size();
         sample_index += static_cast<std::size_t>(stride)) {
      Eigen::MatrixXd A;
      Vec3 rhs = Vec3::Zero();
      if (!fillRow(imu_index, samples[sample_index], &A, &rhs)) {
        continue;
      }
      const Vec3 residual = A * solution - rhs;
      sum_sq += residual.squaredNorm();
      ++residual_samples;
    }
  }
  result.num_samples = used_samples;
  result.accel_rms_m_s2 =
      residual_samples > 0
          ? std::sqrt(sum_sq / static_cast<double>(residual_samples))
          : 0.0;
  result.singular_values =
      Vec3(singular(0),
           singular(std::min<int>(1, singular.size() - 1)),
           singular(std::min<int>(2, singular.size() - 1)));
  result.pose_fit_rms_translation_m = fit_summary.rms_translation_m;
  result.pose_fit_rms_rotation_rad = fit_summary.rms_rotation_rad;
  result.pose_fit_boundary_anchor_observations =
      fit_summary.boundary_anchor_observations;
  return result;
}

} // namespace ceres_cam_imu
