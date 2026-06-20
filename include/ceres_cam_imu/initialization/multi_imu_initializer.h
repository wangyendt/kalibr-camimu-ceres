#pragma once

#include <cstddef>
#include <vector>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/variables/extrinsics.h"

namespace ceres_cam_imu {

struct ImuChainInitializerOptions {
  double max_time_offset_search_s = 0.2;
  int sample_stride = 1;
  int min_samples = 200;
  double min_rotation_excitation = 1e-8;
  bool estimate_lever_arms = false;
  double min_lever_excitation = 1e-8;
  double max_lever_arm_norm_m = 1.0;
  bool refine_with_ceres = true;
  int refine_max_iterations = 50;
};

struct ImuChainInitializerPairResult {
  std::size_t imu_index = 0;
  double time_offset_s = 0.0;
  int discrete_shift_samples = 0;
  double sample_dt_s = 0.0;
  int matched_samples = 0;
  double peak_correlation = 0.0;
  Mat3 R_i_b = Mat3::Identity();
  Vec3 r_b = Vec3::Zero();
  Vec3 r_i_b = Vec3::Zero();
  Vec3 gyro_bias_rad_s = Vec3::Zero();
  Vec3 singular_values = Vec3::Zero();
  double gyro_rms_rad_s = 0.0;
  bool lever_arm_estimated = false;
  Vec3 accel_bias_delta_body_m_s2 = Vec3::Zero();
  Vec3 lever_singular_values = Vec3::Zero();
  double accel_rms_m_s2 = 0.0;
  int refine_iterations = 0;
  double refine_final_cost = 0.0;
};

struct ImuChainInitializerResult {
  std::vector<ImuChainInitializerPairResult> imu_results;
};

ImuChainInitializerPairResult estimateImuChainPairPrior(
    const ImuObservationDataset& reference_imu,
    const ImuObservationDataset& target_imu, std::size_t imu_index,
    const ImuChainInitializerOptions& options);

ImuChainInitializerResult estimateImuChainPrior(
    const std::vector<ImuObservationDataset>& imus,
    const ImuChainInitializerOptions& options);

ImuExtrinsicBlock imuExtrinsicFromRotation(const Mat3& R_i_b);
ImuExtrinsicBlock imuExtrinsicFromRotationAndLever(const Mat3& R_i_b,
                                                   const Vec3& r_b);

}  // namespace ceres_cam_imu
