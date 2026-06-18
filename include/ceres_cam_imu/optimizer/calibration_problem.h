#pragma once

#include <array>
#include <string>
#include <vector>

#include <ceres/ceres.h>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"
#include "ceres_cam_imu/variables/bias_control.h"
#include "ceres_cam_imu/variables/extrinsics.h"
#include "ceres_cam_imu/variables/gravity.h"
#include "ceres_cam_imu/variables/imu_intrinsics.h"
#include "ceres_cam_imu/variables/pose_control.h"
#include "ceres_cam_imu/variables/time_shift.h"

namespace ceres_cam_imu {

enum class RobustLossType {
  kNone,
  kCauchy,
  kHuber,
};

struct CalibrationOptions {
  int spline_order = 6;
  double pose_knots_per_second = 20.0;
  double bias_knots_per_second = 10.0;
  double pose_fit_diagonal_regularization = 1e-9;
  double pose_fit_motion_regularization = 0.0;
  bool pose_fit_add_boundary_anchors = false;
  // Matches Kalibr's --timeoffset-padding. The spline range pads both sides
  // by 2 * time_padding_s to keep camera time-shift changes inside the spline.
  double time_padding_s = 0.04;
  double initial_camera_time_shift_s = 0.0;
  Vec3 initial_gyro_bias_rad_s = Vec3::Zero();
  Vec3 initial_accel_bias_m_s2 = Vec3::Zero();
  bool add_time_shift_prior = false;
  double time_shift_prior_s = 0.0;
  double time_shift_prior_sigma_s = 0.0;
  double reprojection_sigma_px = 1.0;
  int max_frames = 0;
  int imu_stride = 1;
  int max_imu_residuals = 0;
  bool add_bias_motion_prior = true;
  bool add_pose_motion_prior = false;
  bool pose_motion_all_segments = false;
  int pose_motion_derivative_order = 2;
  double pose_motion_translation_variance = 1e6;
  double pose_motion_rotation_variance = 1e5;
  bool add_pose_motion_local_scaling = false;
  double pose_motion_local_center_s = 0.0;
  double pose_motion_local_half_window_s = 0.0;
  double pose_motion_local_translation_variance_scale = 1.0;
  double pose_motion_local_rotation_variance_scale = 1.0;
  bool fix_reference_imu_extrinsic = true;
  bool fix_pose_controls = false;
  bool fix_bias_controls = false;
  bool fix_camera_extrinsic = false;
  bool fix_time_shift = false;
  bool fix_gravity = false;
  ImuCalibrationModel imu_model = ImuCalibrationModel::kCalibrated;
  bool fix_imu_intrinsics = false;
  bool fix_accel_size_effect_rx = true;
  bool estimate_gravity_length = false;
  bool fix_camera_intrinsics = true;
  double robust_loss_width = 10.0;
  RobustLossType camera_loss_type = RobustLossType::kCauchy;
  RobustLossType gyro_loss_type = RobustLossType::kCauchy;
  RobustLossType accel_loss_type = RobustLossType::kCauchy;
  double camera_loss_width = 10.0;
  double gyro_loss_width = 10.0;
  double accel_loss_width = 10.0;
  int max_iterations = 10;
  double solver_function_tolerance = 1e-6;
  double solver_gradient_tolerance = 1e-10;
  double solver_parameter_tolerance = 1e-8;
  double solver_initial_trust_region_radius = 1e4;
  double solver_max_trust_region_radius = 1e16;
  double solver_min_trust_region_radius = 1e-32;
  double solver_min_relative_decrease = 1e-3;
  double solver_absolute_cost_change_tolerance = -1.0;
  double solver_absolute_step_tolerance = -1.0;
  double solver_absolute_parameter_tolerance = -1.0;
  int solver_num_threads = 4;
  int solver_max_consecutive_nonmonotonic_steps = 5;
  bool solver_use_nonmonotonic_steps = false;
  ceres::LinearSolverType solver_linear_solver_type =
      ceres::SPARSE_NORMAL_CHOLESKY;
  bool trace_iteration_state = false;
  std::string trace_label;
  bool trace_has_reference_state = false;
  Mat4 trace_reference_T_c_b = Mat4::Identity();
  double trace_reference_time_shift_s = 0.0;
  Vec3 trace_reference_gravity = Vec3::Zero();
  int top_residuals = 5;
};

struct CalibrationState {
  UniformBSpline pose_spline;
  UniformBSpline gyro_bias_spline;
  UniformBSpline accel_bias_spline;

  std::vector<PoseControlBlock> pose_controls;
  std::vector<BiasControlBlock> gyro_bias_controls;
  std::vector<BiasControlBlock> accel_bias_controls;

  CameraExtrinsicBlock T_c_b;
  ImuExtrinsicBlock imu_extrinsic;
  ImuIntrinsicBlocks imu_intrinsics;
  GravityBlock gravity;
  TimeShiftBlock camera_time_shift_s;
  std::vector<CameraExtrinsicBlock> camera_extrinsics;
  std::vector<TimeShiftBlock> camera_time_shifts;
};

struct CalibrationBuildSummary {
  int camera_residuals = 0;
  int gyro_residuals = 0;
  int accel_residuals = 0;
  int gyro_bias_priors = 0;
  int accel_bias_priors = 0;
  int pose_motion_priors = 0;
  int local_pose_motion_priors = 0;
  int time_shift_priors = 0;
  int gravity_tangent_size = 3;
  int residual_blocks = 0;
  int scalar_residuals = 0;
  int parameter_blocks = 0;
  int active_parameter_blocks = 0;
  int ambient_parameters = 0;
  int tangent_parameters = 0;
  int kalibr_style_error_terms = 0;
  int skipped_camera_frames = 0;
  int skipped_imu_samples = 0;
};

struct PoseInitializationSummary {
  int used_observations = 0;
  int skipped_observations = 0;
  int boundary_anchor_observations = 0;
  int num_coefficients = 0;
  double rms_translation_m = 0.0;
  double rms_rotation_rad = 0.0;
};

struct CameraObservationDataset {
  CameraIntrinsics intrinsics;
  std::vector<ImageObservation> images;
};

CalibrationState
initializeCalibrationState(const std::vector<ImageObservation> &images,
                           const std::vector<ImuSample> &imu_samples,
                           const CalibrationOptions &options);

CalibrationState
initializeCalibrationState(const std::vector<CameraObservationDataset> &cameras,
                           const std::vector<ImuSample> &imu_samples,
                           const CalibrationOptions &options);

CalibrationBuildSummary
buildCalibrationProblem(const CameraIntrinsics &intrinsics,
                        const ImuNoise &imu_noise,
                        const std::vector<ImageObservation> &images,
                        const std::vector<ImuSample> &imu_samples,
                        const CalibrationOptions &options,
                        CalibrationState *state, ceres::Problem *problem);

CalibrationBuildSummary
buildCalibrationProblem(const std::vector<CameraObservationDataset> &cameras,
                        const ImuNoise &imu_noise,
                        const std::vector<ImuSample> &imu_samples,
                        const CalibrationOptions &options,
                        CalibrationState *state, ceres::Problem *problem);

ceres::Solver::Summary
solveCalibrationProblem(const CalibrationOptions &options,
                        ceres::Problem *problem);

ceres::Solver::Summary
solveCalibrationProblem(const CalibrationOptions &options,
                        const CalibrationState *state, ceres::Problem *problem);

PoseInitializationSummary initializePoseControlsFromCameraPoses(
    const std::vector<PoseObservation> &pose_observations,
    const CameraExtrinsicBlock &T_c_b, CalibrationState *state);

PoseInitializationSummary initializePoseControlsFromCameraPoses(
    const std::vector<PoseObservation> &pose_observations,
    const CameraExtrinsicBlock &T_c_b, const CalibrationOptions &options,
    CalibrationState *state);

Vec6 matrixToPose6(const Mat4 &T);
Mat4 pose6ToMatrix(const CameraExtrinsicBlock &pose);

} // namespace ceres_cam_imu
