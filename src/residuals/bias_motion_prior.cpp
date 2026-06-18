#include "ceres_cam_imu/residuals/bias_motion_prior.h"

#include <array>
#include <cmath>

#include "ceres_cam_imu/residuals/spline_motion_prior.h"

namespace ceres_cam_imu {

ceres::CostFunction* createBiasMotionPrior(
    const SplineSegmentMeta6& bias_segment, const double random_walk_sigma) {
  const double sigma = std::max(1e-12, random_walk_sigma);
  return createEuclideanSplineMotionPrior<3>(
      bias_segment, {1.0 / sigma, 1.0 / sigma, 1.0 / sigma}, 1);
}

}  // namespace ceres_cam_imu
