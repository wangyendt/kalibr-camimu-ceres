#pragma once

#include <vector>

#include "ceres_cam_imu/optimizer/calibration_problem.h"

namespace ceres_cam_imu {

struct CalibrationStateSnapshot {
  std::vector<PoseControlBlock> pose_controls;
  std::vector<BiasControlBlock> gyro_bias_controls;
  std::vector<BiasControlBlock> accel_bias_controls;
  CameraExtrinsicBlock T_c_b;
  ImuExtrinsicBlock imu_extrinsic;
  ImuIntrinsicBlocks imu_intrinsics;
  GravityBlock gravity;
  TimeShiftBlock camera_time_shift_s;
};

CalibrationStateSnapshot
snapshotCalibrationState(const CalibrationState &state);

bool isCompatibleStateSnapshot(const CalibrationStateSnapshot &snapshot,
                               const CalibrationState &state);

void restoreCalibrationState(const CalibrationStateSnapshot &snapshot,
                             CalibrationState *state);

} // namespace ceres_cam_imu
