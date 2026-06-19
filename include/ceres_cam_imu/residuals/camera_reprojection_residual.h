#pragma once

#include <vector>

#include <ceres/cost_function.h>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"

namespace ceres_cam_imu {

ceres::CostFunction* createCameraReprojectionResidual(
    const CameraIntrinsics& intrinsics, const CornerMeasurement& corner,
    double observation_time_s, const SplineSegmentMeta6& pose_segment,
    double reprojection_sigma_px);

ceres::CostFunction* createCameraReprojectionTimeOffsetResidual(
    const CameraIntrinsics& intrinsics, const CornerMeasurement& corner,
    double observation_time_s,
    const std::vector<SplineSegmentMeta6>& pose_segments,
    int local_coeff_start, double buffer_start_s, double buffer_end_s,
    double reprojection_sigma_px);

}  // namespace ceres_cam_imu
