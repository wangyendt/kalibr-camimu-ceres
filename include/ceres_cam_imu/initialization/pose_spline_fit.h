#pragma once

#include <vector>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"
#include "ceres_cam_imu/variables/extrinsics.h"
#include "ceres_cam_imu/variables/pose_control.h"

namespace ceres_cam_imu {

struct PoseSplineFitOptions {
  // Small diagonal damping for numerical robustness. Kalibr-style smoothness
  // is controlled separately by motion_regularization.
  double regularization = 1e-9;
  double motion_regularization = 0.0;
  int motion_regularization_order = 2;
  bool add_boundary_anchors = false;
  bool unwrap_rotation_vectors = true;
};

struct PoseSplineFitSummary {
  int used_observations = 0;
  int skipped_observations = 0;
  int boundary_anchor_observations = 0;
  int num_coefficients = 0;
  double rms_translation_m = 0.0;
  double rms_rotation_rad = 0.0;
};

PoseSplineFitSummary fitPoseSplineControlsFromCameraPoses(
    const std::vector<PoseObservation>& pose_observations,
    const CameraExtrinsicBlock& T_c_b, double camera_time_shift_s,
    const UniformBSpline& pose_spline, const PoseSplineFitOptions& options,
    std::vector<PoseControlBlock>* pose_controls);

}  // namespace ceres_cam_imu
