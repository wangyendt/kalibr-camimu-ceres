#include "ceres_cam_imu/initialization/orientation_gravity_initializer.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ceres/problem.h>
#include <ceres/sized_cost_function.h>
#include <ceres/solver.h>
#include <Eigen/SVD>

#include "ceres_cam_imu/core/se3.h"
#include "ceres_cam_imu/initialization/pose_spline_fit.h"
#include "ceres_cam_imu/core/so3_jacobians.h"
#include "ceres_cam_imu/trajectory/spline_eval.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"
#include "ceres_cam_imu/variables/pose_control.h"

namespace ceres_cam_imu {
namespace {

void writeMat3RowMajor(const Mat3& matrix, double* jacobian) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      jacobian[row * 3 + col] = matrix(row, col);
    }
  }
}

class OrientationGyroBiasCost final
    : public ceres::SizedCostFunction<3, 3, 3> {
 public:
  OrientationGyroBiasCost(Vec3 omega_camera, Vec3 omega_imu)
      : omega_camera_(std::move(omega_camera)),
        omega_imu_(std::move(omega_imu)) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const Eigen::Map<const Vec3> r_i_c(parameters[0]);
    const Eigen::Map<const Vec3> bias(parameters[1]);
    const Mat3 R_i_c = rotationVectorToMatrix(r_i_c);
    const Vec3 residual = R_i_c * omega_camera_ + bias - omega_imu_;
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (!jacobians) {
      return true;
    }
    if (jacobians[0]) {
      const Mat3 J =
          R_i_c * skew(omega_camera_) * leftJacobianSO3(r_i_c);
      writeMat3RowMajor(J, jacobians[0]);
    }
    if (jacobians[1]) {
      writeMat3RowMajor(Mat3::Identity(), jacobians[1]);
    }
    return true;
  }

 private:
  Vec3 omega_camera_;
  Vec3 omega_imu_;
};

std::pair<double, double> shiftedPoseTimeSpan(
    const std::vector<PoseObservation>& pose_observations,
    const double camera_time_shift_s) {
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  for (const PoseObservation& observation : pose_observations) {
    const double t = observation.timestamp_s + camera_time_shift_s;
    first = std::min(first, t);
    last = std::max(last, t);
  }
  if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
    throw std::runtime_error(
        "cannot estimate orientation prior from degenerate pose times");
  }
  return {first, last};
}

Vec6 poseCurveAt(const UniformBSpline& spline,
                 const std::vector<PoseControlBlock>& controls,
                 const double timestamp_s, const int derivative_order) {
  const SplineSegmentMeta6 meta = spline.segmentMeta6(timestamp_s);
  std::array<const double*, SplineSegmentMeta6::kOrder> active{};
  for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
    active[static_cast<std::size_t>(i)] =
        controls.at(static_cast<std::size_t>(meta.coeff_start + i)).data();
  }
  return evalPoseCurve6(meta, timestamp_s, active, derivative_order);
}

Vec3 angularVelocityAt(const UniformBSpline& spline,
                       const std::vector<PoseControlBlock>& controls,
                       const double timestamp_s) {
  const Vec6 curve = poseCurveAt(spline, controls, timestamp_s, 0);
  const Vec6 curve_dot = poseCurveAt(spline, controls, timestamp_s, 1);
  return bodyAngularVelocityFromCurve(curve, curve_dot);
}

Mat3 wahbaRotationCameraToImu(const std::vector<Vec3>& omega_camera,
                              const std::vector<Vec3>& omega_imu,
                              Vec3* camera_mean, Vec3* imu_mean,
                              Vec3* singular_values) {
  Vec3 mean_camera = Vec3::Zero();
  Vec3 mean_imu = Vec3::Zero();
  for (std::size_t i = 0; i < omega_camera.size(); ++i) {
    mean_camera += omega_camera[i];
    mean_imu += omega_imu[i];
  }
  const double inv_n = 1.0 / static_cast<double>(omega_camera.size());
  mean_camera *= inv_n;
  mean_imu *= inv_n;

  Mat3 correlation = Mat3::Zero();
  for (std::size_t i = 0; i < omega_camera.size(); ++i) {
    correlation += (omega_camera[i] - mean_camera)
                 * (omega_imu[i] - mean_imu).transpose();
  }

  Eigen::JacobiSVD<Mat3> svd(
      correlation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 fix = Mat3::Identity();
  if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0) {
    fix(2, 2) = -1.0;
  }
  if (camera_mean) {
    *camera_mean = mean_camera;
  }
  if (imu_mean) {
    *imu_mean = mean_imu;
  }
  if (singular_values) {
    *singular_values = svd.singularValues();
  }
  return svd.matrixV() * fix * svd.matrixU().transpose();
}

double rmsGyroError(const std::vector<Vec3>& omega_camera,
                    const std::vector<Vec3>& omega_imu,
                    const Mat3& R_i_c, const Vec3& bias) {
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < omega_camera.size(); ++i) {
    sum_sq += (R_i_c * omega_camera[i] + bias - omega_imu[i]).squaredNorm();
  }
  return std::sqrt(sum_sq / static_cast<double>(omega_camera.size()));
}

ceres::Solver::Summary refineOrientationAndGyroBias(
    const std::vector<Vec3>& omega_camera, const std::vector<Vec3>& omega_imu,
    const OrientationGravityInitializerOptions& options,
    Mat3* R_i_c, Vec3* gyro_bias) {
  if (!R_i_c || !gyro_bias) {
    throw std::invalid_argument("orientation refinement outputs must be non-null");
  }
  Vec3 r_i_c = rotationMatrixToVector(*R_i_c);
  Vec3 bias = *gyro_bias;

  ceres::Problem problem;
  problem.AddParameterBlock(r_i_c.data(), 3);
  problem.AddParameterBlock(bias.data(), 3);
  for (std::size_t i = 0; i < omega_camera.size(); ++i) {
    problem.AddResidualBlock(
        new OrientationGyroBiasCost(omega_camera[i], omega_imu[i]),
        nullptr, r_i_c.data(), bias.data());
  }

  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = std::max(0, options.refine_max_iterations);
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.num_threads = 2;
  solver_options.minimizer_progress_to_stdout = false;
  solver_options.logging_type = ceres::SILENT;
  solver_options.parameter_tolerance = 1e-4;
  solver_options.function_tolerance = 1e-12;
  solver_options.gradient_tolerance = 1e-12;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  *R_i_c = rotationVectorToMatrix(r_i_c);
  *gyro_bias = bias;
  return summary;
}

}  // namespace

OrientationGravityInitializerResult
estimateOrientationGravityAndGyroBiasPrior(
    const std::vector<PoseObservation>& pose_observations,
    const std::vector<ImuSample>& imu_samples,
    const CameraExtrinsicBlock& initial_T_c_b,
    const double camera_time_shift_s,
    const OrientationGravityInitializerOptions& options) {
  if (options.spline_order != SplineSegmentMeta6::kOrder) {
    throw std::runtime_error(
        "orientation/gravity initializer currently requires order-6 splines");
  }
  if (pose_observations.empty()) {
    throw std::runtime_error(
        "pose observations are required for orientation/gravity initialization");
  }
  if (imu_samples.empty()) {
    throw std::runtime_error(
        "IMU samples are required for orientation/gravity initialization");
  }

  const auto [first_pose_time, last_pose_time] =
      shiftedPoseTimeSpan(pose_observations, camera_time_shift_s);
  const UniformBSpline camera_pose_spline = makeSplineForTimes(
      6, options.spline_order, first_pose_time, last_pose_time,
      options.pose_knots_per_second, 0.0);

  CameraExtrinsicBlock identity_T_c_b;
  identity_T_c_b.values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  PoseSplineFitOptions fit_options;
  fit_options.regularization = 1e-12;
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
        "no pose observations overlap the orientation initializer spline");
  }

  std::vector<Vec3> omega_camera;
  std::vector<Vec3> omega_imu;
  omega_camera.reserve(imu_samples.size());
  omega_imu.reserve(imu_samples.size());
  for (const ImuSample& sample : imu_samples) {
    const double t = sample.timestamp_s;
    if (t <= camera_pose_spline.tMin() || t >= camera_pose_spline.tMax()) {
      continue;
    }
    omega_camera.push_back(
        angularVelocityAt(camera_pose_spline, camera_pose_controls, t));
    omega_imu.push_back(sample.gyro_rad_s);
  }

  if (omega_camera.size() < static_cast<std::size_t>(options.min_samples)) {
    throw std::runtime_error(
        "not enough overlapping samples for orientation/gravity initialization");
  }

  Vec3 camera_mean = Vec3::Zero();
  Vec3 imu_mean = Vec3::Zero();
  Vec3 singular_values = Vec3::Zero();
  const Mat3 R_i_c = wahbaRotationCameraToImu(
      omega_camera, omega_imu, &camera_mean, &imu_mean, &singular_values);
  if (singular_values.sum() <= options.min_rotation_excitation) {
    throw std::runtime_error(
        "gyro motion is too weak for orientation prior initialization");
  }
  Vec3 gyro_bias = imu_mean - R_i_c * camera_mean;
  Mat3 refined_R_i_c = R_i_c;
  ceres::Solver::Summary refine_summary;
  if (options.refine_with_ceres && options.refine_max_iterations > 0) {
    refine_summary = refineOrientationAndGyroBias(
        omega_camera, omega_imu, options, &refined_R_i_c, &gyro_bias);
  }
  const Mat3 R_c_i = refined_R_i_c.transpose();

  Vec3 gravity_sum = Vec3::Zero();
  int gravity_samples = 0;
  for (const ImuSample& sample : imu_samples) {
    const double t = sample.timestamp_s;
    if (t <= camera_pose_spline.tMin() || t >= camera_pose_spline.tMax()) {
      continue;
    }
    const Vec6 pose = poseCurveAt(camera_pose_spline, camera_pose_controls, t, 0);
    const Mat3 R_w_c = rotationVectorToMatrix(pose.tail<3>());
    gravity_sum += R_w_c * R_c_i * (-sample.accel_m_s2);
    ++gravity_samples;
  }
  if (gravity_samples == 0) {
    throw std::runtime_error(
        "no accelerometer samples overlap the orientation initializer spline");
  }
  const Vec3 gravity_mean =
      gravity_sum / static_cast<double>(gravity_samples);
  if (gravity_mean.norm() <= 0.0) {
    throw std::runtime_error("gravity prior initialization produced zero norm");
  }

  OrientationGravityInitializerResult result;
  result.T_c_b = initial_T_c_b;
  const Vec3 r_c_i = rotationMatrixToVector(R_c_i);
  for (int i = 0; i < 3; ++i) {
    result.T_c_b.values[static_cast<std::size_t>(3 + i)] = r_c_i(i);
  }
  result.gyro_bias_rad_s = gyro_bias;
  result.gravity_mean_norm_m_s2 = gravity_mean.norm();
  result.gravity_m_s2 =
      gravity_mean / gravity_mean.norm() * options.gravity_norm_m_s2;
  result.singular_values = singular_values;
  result.num_samples = static_cast<int>(omega_camera.size());
  result.gyro_rms_rad_s =
      rmsGyroError(omega_camera, omega_imu, refined_R_i_c, gyro_bias);
  result.pose_fit_rms_translation_m = fit_summary.rms_translation_m;
  result.pose_fit_rms_rotation_rad = fit_summary.rms_rotation_rad;
  result.pose_fit_boundary_anchor_observations =
      fit_summary.boundary_anchor_observations;
  result.refine_iterations = refine_summary.iterations.size();
  result.refine_final_cost = refine_summary.final_cost;
  return result;
}

}  // namespace ceres_cam_imu
