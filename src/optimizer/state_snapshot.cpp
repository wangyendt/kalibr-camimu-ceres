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
  return snapshot;
}

bool isCompatibleStateSnapshot(const CalibrationStateSnapshot &snapshot,
                               const CalibrationState &state) {
  return snapshot.pose_controls.size() == state.pose_controls.size() &&
         snapshot.gyro_bias_controls.size() ==
             state.gyro_bias_controls.size() &&
         snapshot.accel_bias_controls.size() ==
             state.accel_bias_controls.size();
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
}

} // namespace ceres_cam_imu
