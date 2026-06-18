#pragma once

#include <ceres/cost_function.h>

#include "ceres_cam_imu/trajectory/uniform_bspline.h"

namespace ceres_cam_imu {

ceres::CostFunction* createPoseMotionPrior(
    const SplineSegmentMeta6& pose_segment, double translation_variance,
    double rotation_variance, int derivative_order);

}  // namespace ceres_cam_imu
