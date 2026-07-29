#pragma once

#include <vector>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/initialization/multi_imu_initializer.h"
#include "ceres_cam_imu/initialization/orientation_gravity_initializer.h"
#include "ceres_cam_imu/initialization/time_shift_initializer.h"
#include "ceres_cam_imu/variables/extrinsics.h"

namespace ceres_cam_imu {

struct KalibrStyleCameraTimeShiftSeed {
  bool estimate = true;
  double initial_time_shift_s = 0.0;
  bool initial_time_shift_valid = false;
};

// A deterministic cold-start state assembled in Kalibr's initialization order:
// per-camera/reference-IMU time, camera-0/reference-IMU rotation and gravity,
// then each non-reference IMU's relative time and rotation. Camera-0 and IMU
// translations start at zero; configured inter-camera baselines are preserved.
struct KalibrStyleMultiImuInitializerOptions {
  KalibrStyleMultiImuInitializerOptions() {
    imu_chain.use_full_overlap_time_offset_search = true;
  }

  std::vector<KalibrStyleCameraTimeShiftSeed> camera_time_shift_seeds;
  TimeShiftPriorOptions camera_time_shift;
  OrientationGravityInitializerOptions orientation_gravity;
  ImuChainInitializerOptions imu_chain;
};

struct KalibrStyleCameraInitializationResult {
  CameraExtrinsicBlock T_c_b;
  double time_shift_s = 0.0;
  bool time_shift_valid = false;
  TimeShiftPriorEstimate time_shift_estimate;
};

struct KalibrStyleMultiImuInitializerResult {
  std::vector<KalibrStyleCameraInitializationResult> cameras;
  std::vector<ImuExtrinsicBlock> imu_extrinsics;
  std::vector<double> imu_time_offsets_s;

  OrientationGravityInitializerResult orientation_gravity;
  ImuChainInitializerResult imu_chain;
};

// Boundary behavior is deliberately asymmetric. A camera correlation boundary
// is returned in cameras[i].time_shift_estimate; a valid caller-provided seed
// is retained, otherwise time_shift_valid is false so the CLI can continue only
// without anchoring it. A non-reference IMU boundary throws because its time
// mapping is required to construct every downstream joint residual safely.
KalibrStyleMultiImuInitializerResult estimateKalibrStyleMultiImuInitialization(
    const std::vector<std::vector<PoseObservation>> &camera_pose_observations,
    const std::vector<ImuObservationDataset> &imus,
    const std::vector<CameraExtrinsicBlock> &initial_camera_extrinsics,
    const KalibrStyleMultiImuInitializerOptions &options);

} // namespace ceres_cam_imu
