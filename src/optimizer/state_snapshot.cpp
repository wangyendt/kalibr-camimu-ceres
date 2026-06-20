#include "ceres_cam_imu/optimizer/state_snapshot.h"

#include <stdexcept>

namespace ceres_cam_imu {

CalibrationStateSnapshot
snapshotCalibrationState(const CalibrationState &state) {
  CalibrationStateSnapshot snapshot;
  snapshot.pose_controls = state.pose_controls;
  snapshot.gyro_bias_controls = state.gyro_bias_controls;
  snapshot.accel_bias_controls = state.accel_bias_controls;
  snapshot.T_c_b = state.T_c_b;
  snapshot.imu_extrinsic = state.imu_extrinsic;
  snapshot.imu_intrinsics = state.imu_intrinsics;
  snapshot.gravity = state.gravity;
  snapshot.camera_time_shift_s = state.camera_time_shift_s;
  snapshot.camera_extrinsics = state.camera_extrinsics;
  snapshot.camera_time_shifts = state.camera_time_shifts;
  snapshot.imu_extrinsics = state.imu_extrinsics;
  snapshot.imu_intrinsics_by_imu = state.imu_intrinsics_by_imu;
  snapshot.imu_time_offsets_s = state.imu_time_offsets_s;
  snapshot.gyro_bias_controls_by_imu = state.gyro_bias_controls_by_imu;
  snapshot.accel_bias_controls_by_imu = state.accel_bias_controls_by_imu;
  return snapshot;
}

bool isCompatibleStateSnapshot(const CalibrationStateSnapshot &snapshot,
                               const CalibrationState &state) {
  return snapshot.pose_controls.size() == state.pose_controls.size() &&
         snapshot.gyro_bias_controls.size() ==
             state.gyro_bias_controls.size() &&
         snapshot.accel_bias_controls.size() ==
             state.accel_bias_controls.size() &&
         snapshot.camera_extrinsics.size() == state.camera_extrinsics.size() &&
         snapshot.camera_time_shifts.size() == state.camera_time_shifts.size() &&
         snapshot.imu_extrinsics.size() == state.imu_extrinsics.size() &&
         snapshot.imu_intrinsics_by_imu.size() ==
             state.imu_intrinsics_by_imu.size() &&
         snapshot.imu_time_offsets_s.size() ==
             state.imu_time_offsets_s.size() &&
         snapshot.gyro_bias_controls_by_imu.size() ==
             state.gyro_bias_controls_by_imu.size() &&
         snapshot.accel_bias_controls_by_imu.size() ==
             state.accel_bias_controls_by_imu.size();
}

void restoreCalibrationState(const CalibrationStateSnapshot &snapshot,
                             CalibrationState *state) {
  if (!state) {
    throw std::invalid_argument("state must be non-null");
  }
  if (!isCompatibleStateSnapshot(snapshot, *state)) {
    throw std::invalid_argument(
        "calibration state snapshot is incompatible with current state");
  }
  state->pose_controls = snapshot.pose_controls;
  state->gyro_bias_controls = snapshot.gyro_bias_controls;
  state->accel_bias_controls = snapshot.accel_bias_controls;
  state->T_c_b = snapshot.T_c_b;
  state->imu_extrinsic = snapshot.imu_extrinsic;
  state->imu_intrinsics = snapshot.imu_intrinsics;
  state->gravity = snapshot.gravity;
  state->camera_time_shift_s = snapshot.camera_time_shift_s;
  state->camera_extrinsics = snapshot.camera_extrinsics;
  state->camera_time_shifts = snapshot.camera_time_shifts;
  state->imu_extrinsics = snapshot.imu_extrinsics;
  state->imu_intrinsics_by_imu = snapshot.imu_intrinsics_by_imu;
  state->imu_time_offsets_s = snapshot.imu_time_offsets_s;
  state->gyro_bias_controls_by_imu = snapshot.gyro_bias_controls_by_imu;
  state->accel_bias_controls_by_imu = snapshot.accel_bias_controls_by_imu;
}

} // namespace ceres_cam_imu
