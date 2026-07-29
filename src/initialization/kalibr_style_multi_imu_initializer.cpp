#include "ceres_cam_imu/initialization/kalibr_style_multi_imu_initializer.h"

#include <algorithm>
#include <stdexcept>

#include "ceres_cam_imu/core/se3.h"
#include "ceres_cam_imu/core/so3.h"

namespace ceres_cam_imu {

KalibrStyleMultiImuInitializerResult estimateKalibrStyleMultiImuInitialization(
    const std::vector<std::vector<PoseObservation>> &camera_pose_observations,
    const std::vector<ImuObservationDataset> &imus,
    const std::vector<CameraExtrinsicBlock> &initial_camera_extrinsics,
    const KalibrStyleMultiImuInitializerOptions &options) {
  if (imus.size() < 2) {
    throw std::runtime_error(
        "Kalibr-style multi-IMU initialization requires at least two IMUs");
  }
  if (camera_pose_observations.empty() ||
      camera_pose_observations.front().empty()) {
    throw std::runtime_error(
        "Kalibr-style multi-IMU initialization requires camera-0 poses");
  }
  if (initial_camera_extrinsics.size() != camera_pose_observations.size()) {
    throw std::runtime_error(
        "Kalibr-style initialization requires one initial extrinsic per "
        "camera pose stream");
  }
  if (!options.camera_time_shift_seeds.empty() &&
      options.camera_time_shift_seeds.size() !=
          camera_pose_observations.size()) {
    throw std::runtime_error(
        "Kalibr-style initialization requires either zero camera time seeds "
        "or exactly one seed per camera");
  }
  for (std::size_t camera_index = 0;
       camera_index < camera_pose_observations.size(); ++camera_index) {
    const bool estimate_this_camera =
        options.camera_time_shift_seeds.empty() ||
        options.camera_time_shift_seeds[camera_index].estimate;
    if (estimate_this_camera &&
        camera_pose_observations[camera_index].empty()) {
      throw std::runtime_error("Kalibr-style initialization requires camera-" +
                               std::to_string(camera_index) + " poses");
    }
    if (!estimate_this_camera && !options.camera_time_shift_seeds[camera_index]
                                      .initial_time_shift_valid) {
      throw std::runtime_error(
          "Kalibr-style initialization requires a valid fixed time shift for "
          "camera-" +
          std::to_string(camera_index));
    }
  }

  KalibrStyleMultiImuInitializerResult result;
  result.cameras.resize(camera_pose_observations.size());
  for (std::size_t camera_index = 0;
       camera_index < camera_pose_observations.size(); ++camera_index) {
    KalibrStyleCameraInitializationResult &camera =
        result.cameras[camera_index];
    camera.T_c_b = initial_camera_extrinsics[camera_index];
    if (camera_index < options.camera_time_shift_seeds.size()) {
      const KalibrStyleCameraTimeShiftSeed &seed =
          options.camera_time_shift_seeds[camera_index];
      camera.time_shift_s = seed.initial_time_shift_s;
      camera.time_shift_valid = seed.initial_time_shift_valid;
    }
  }

  CameraExtrinsicBlock camera0_T_c_b = initial_camera_extrinsics.front();
  for (int i = 0; i < 3; ++i) {
    camera0_T_c_b.values[static_cast<std::size_t>(i)] = 0.0;
  }

  for (std::size_t camera_index = 0;
       camera_index < camera_pose_observations.size(); ++camera_index) {
    const bool estimate_this_camera =
        options.camera_time_shift_seeds.empty() ||
        options.camera_time_shift_seeds[camera_index].estimate;
    if (!estimate_this_camera) {
      continue;
    }
    KalibrStyleCameraInitializationResult &camera =
        result.cameras[camera_index];
    camera.time_shift_estimate = estimateCameraImuTimeShiftPrior(
        camera_pose_observations[camera_index], imus.front().samples,
        initial_camera_extrinsics[camera_index], options.camera_time_shift);
    if (!camera.time_shift_estimate.boundary_peak_rejected) {
      camera.time_shift_s = camera.time_shift_estimate.shift_s;
      camera.time_shift_valid = true;
    }
  }
  result.orientation_gravity = estimateOrientationGravityAndGyroBiasPrior(
      camera_pose_observations.front(), imus.front().samples, camera0_T_c_b,
      result.cameras.front().time_shift_s, options.orientation_gravity);
  camera0_T_c_b = result.orientation_gravity.T_c_b;
  for (int i = 0; i < 3; ++i) {
    camera0_T_c_b.values[static_cast<std::size_t>(i)] = 0.0;
  }

  // Kalibr estimates cam0-to-body first, then applies the configured camera
  // chain. Preserve those relative camera baselines instead of treating each
  // camera as an unrelated body sensor.
  const Mat4 initial_T_c0_b = pose6ToMatrix(
      Eigen::Map<const Vec6>(initial_camera_extrinsics.front().data()));
  const Mat4 estimated_T_c0_b =
      pose6ToMatrix(Eigen::Map<const Vec6>(camera0_T_c_b.data()));
  result.cameras.front().T_c_b = camera0_T_c_b;
  for (std::size_t camera_index = 1;
       camera_index < initial_camera_extrinsics.size(); ++camera_index) {
    const Mat4 initial_T_ci_b = pose6ToMatrix(
        Eigen::Map<const Vec6>(initial_camera_extrinsics[camera_index].data()));
    const Mat4 estimated_T_ci_b =
        initial_T_ci_b * initial_T_c0_b.inverse() * estimated_T_c0_b;
    Vec6 pose;
    pose.head<3>() = estimated_T_ci_b.block<3, 1>(0, 3);
    pose.tail<3>() = rotationMatrixToVector(estimated_T_ci_b.block<3, 3>(0, 0));
    std::copy(pose.data(), pose.data() + 6,
              result.cameras[camera_index].T_c_b.data());
  }

  ImuChainInitializerOptions chain_options = options.imu_chain;
  // Kalibr's initialization snapshot gives the joint optimizer zero
  // translations. Do not let the older Ceres-only lever-arm experiments leak
  // into this deterministic seed.
  chain_options.estimate_lever_arms = false;
  chain_options.refine_with_accel = false;
  result.imu_chain = estimateImuChainPrior(imus, chain_options);

  result.imu_extrinsics.assign(imus.size(), ImuExtrinsicBlock{});
  result.imu_time_offsets_s.assign(imus.size(), 0.0);

  for (const ImuChainInitializerPairResult &pair :
       result.imu_chain.imu_results) {
    if (pair.imu_index >= imus.size()) {
      throw std::runtime_error(
          "IMU chain initializer returned an out-of-range IMU index");
    }
    if (pair.time_offset_boundary_peak_rejected) {
      throw std::runtime_error(
          "Kalibr-style IMU-chain initialization rejected a boundary time "
          "offset peak for IMU " +
          std::to_string(pair.imu_index));
    }
    result.imu_extrinsics[pair.imu_index] =
        imuExtrinsicFromRotation(pair.R_i_b);
    result.imu_time_offsets_s[pair.imu_index] = pair.time_offset_s;
  }
  return result;
}

} // namespace ceres_cam_imu
