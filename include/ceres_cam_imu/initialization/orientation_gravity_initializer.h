#pragma once

#include <vector>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/variables/extrinsics.h"

namespace ceres_cam_imu {

struct OrientationGravityInitializerOptions {
  int spline_order = 6;
  double pose_knots_per_second = 100.0;
  double pose_fit_regularization = 1e-4;
  bool pose_fit_boundary_anchors = true;
  bool refine_with_ceres = true;
  int refine_max_iterations = 50;
  double gravity_norm_m_s2 = 9.80655;
  int min_samples = 20;
  double min_rotation_excitation = 1e-10;
};

struct OrientationGravityInitializerResult {
  CameraExtrinsicBlock T_c_b;
  Vec3 gyro_bias_rad_s = Vec3::Zero();
  Vec3 gravity_m_s2 = Vec3::Zero();
  Vec3 singular_values = Vec3::Zero();
  int num_samples = 0;
  double gyro_rms_rad_s = 0.0;
  double gravity_mean_norm_m_s2 = 0.0;
  double pose_fit_rms_translation_m = 0.0;
  double pose_fit_rms_rotation_rad = 0.0;
  int pose_fit_boundary_anchor_observations = 0;
  int refine_iterations = 0;
  double refine_final_cost = 0.0;
};

OrientationGravityInitializerResult
estimateOrientationGravityAndGyroBiasPrior(
    const std::vector<PoseObservation>& pose_observations,
    const std::vector<ImuSample>& imu_samples,
    const CameraExtrinsicBlock& initial_T_c_b,
    double camera_time_shift_s,
    const OrientationGravityInitializerOptions& options);

}  // namespace ceres_cam_imu
