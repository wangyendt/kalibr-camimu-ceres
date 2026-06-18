#include "ceres_cam_imu/residuals/pose_motion_prior.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "ceres_cam_imu/residuals/spline_motion_prior.h"

namespace ceres_cam_imu {

ceres::CostFunction* createPoseMotionPrior(
    const SplineSegmentMeta6& pose_segment, const double translation_variance,
    const double rotation_variance, const int derivative_order) {
  const double clamped_translation_variance =
      std::max(1e-24, translation_variance);
  const double clamped_rotation_variance = std::max(1e-24, rotation_variance);
  const double sqrt_translation_info =
      1.0 / std::sqrt(clamped_translation_variance);
  const double sqrt_rotation_info =
      1.0 / std::sqrt(clamped_rotation_variance);
  return createEuclideanSplineMotionPrior<6>(
      pose_segment,
      {sqrt_translation_info, sqrt_translation_info, sqrt_translation_info,
       sqrt_rotation_info, sqrt_rotation_info, sqrt_rotation_info},
      derivative_order);
}

}  // namespace ceres_cam_imu
