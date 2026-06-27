#pragma once

#include <vector>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/variables/extrinsics.h"

namespace ceres_cam_imu {

struct CameraTranslationInitializerOptions {
  int spline_order = 6;
  double pose_knots_per_second = 100.0;
  double pose_fit_regularization = 1e-4;
  bool pose_fit_boundary_anchors = true;
  int sample_stride = 1;
  int min_samples = 20;
  double min_excitation = 1e-10;
  double min_lever_jacobian_norm = 0.0;
  double max_translation_norm_m = 1.0;
};

struct CameraTranslationInitializerResult {
  Vec3 t_c_b_m = Vec3::Zero();
  Vec3 accel_bias_m_s2 = Vec3::Zero();
  Vec3 singular_values = Vec3::Zero();
  int num_samples = 0;
  double accel_rms_m_s2 = 0.0;
  double pose_fit_rms_translation_m = 0.0;
  double pose_fit_rms_rotation_rad = 0.0;
  int pose_fit_boundary_anchor_observations = 0;
};

struct MultiImuTranslationInitializerOptions
    : public CameraTranslationInitializerOptions {
  double max_lever_arm_norm_m = 1.0;
  // Negative disables the corresponding Tikhonov prior. Positive values add a
  // weak LS prior around the current IMU lever initialization or zero bias.
  double camera_prior_sigma_m = -1.0;
  double lever_prior_sigma_m = -1.0;
  double accel_bias_prior_sigma_m_s2 = -1.0;
};

struct MultiImuTranslationInitializerResult {
  Vec3 t_c_b_m = Vec3::Zero();
  std::vector<Vec3> r_b_m;
  std::vector<Vec3> accel_bias_m_s2;
  Vec3 singular_values = Vec3::Zero();
  int num_samples = 0;
  double accel_rms_m_s2 = 0.0;
  double pose_fit_rms_translation_m = 0.0;
  double pose_fit_rms_rotation_rad = 0.0;
  int pose_fit_boundary_anchor_observations = 0;
};

CameraTranslationInitializerResult estimateCameraTranslationAndAccelBiasPrior(
    const std::vector<PoseObservation> &pose_observations,
    const std::vector<ImuSample> &imu_samples,
    const CameraExtrinsicBlock &initial_T_c_b,
    const Vec3 &gravity_m_s2,
    double camera_time_shift_s,
    const CameraTranslationInitializerOptions &options);

MultiImuTranslationInitializerResult
estimateMultiImuCameraTranslationAndLeverPrior(
    const std::vector<PoseObservation> &pose_observations,
    const std::vector<ImuObservationDataset> &imus,
    const std::vector<ImuExtrinsicBlock> &initial_imu_extrinsics,
    const std::vector<double> &imu_time_offsets_s,
    const CameraExtrinsicBlock &initial_T_c_b,
    const Vec3 &gravity_m_s2,
    double camera_time_shift_s,
    const MultiImuTranslationInitializerOptions &options);

} // namespace ceres_cam_imu
