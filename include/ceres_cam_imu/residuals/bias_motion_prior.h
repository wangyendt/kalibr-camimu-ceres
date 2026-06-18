#pragma once

#include <ceres/cost_function.h>

#include "ceres_cam_imu/trajectory/uniform_bspline.h"

namespace ceres_cam_imu {

ceres::CostFunction* createBiasMotionPrior(
    const SplineSegmentMeta6& bias_segment, double random_walk_sigma);

}  // namespace ceres_cam_imu
