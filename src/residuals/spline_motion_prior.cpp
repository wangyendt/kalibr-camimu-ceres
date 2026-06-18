#include "ceres_cam_imu/residuals/spline_motion_prior.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace ceres_cam_imu {

SplineBasisMatrix6 segmentWeightDerivativeIntegral(
    const SplineSegmentMeta6& segment, const int derivative_order) {
  SplineBasisMatrix6 integral = SplineBasisMatrix6::Zero();
  const double dt_scale =
      std::pow(segment.dt_s, 1.0 - 2.0 * static_cast<double>(derivative_order));

  for (int a = 0; a < SplineSegmentMeta6::kOrder; ++a) {
    for (int b = 0; b < SplineSegmentMeta6::kOrder; ++b) {
      double value = 0.0;
      for (int i = derivative_order; i < SplineSegmentMeta6::kOrder; ++i) {
        const int i_power = i - derivative_order;
        const int i_multiplier = derivativeMultiplier(i, derivative_order);
        for (int j = derivative_order; j < SplineSegmentMeta6::kOrder; ++j) {
          const int j_power = j - derivative_order;
          const int j_multiplier = derivativeMultiplier(j, derivative_order);
          value += segment.basis(i, a) * segment.basis(j, b)
                 * static_cast<double>(i_multiplier * j_multiplier)
                 / static_cast<double>(i_power + j_power + 1);
        }
      }
      integral(a, b) = dt_scale * value;
    }
  }
  return 0.5 * (integral + integral.transpose());
}

SplineBasisMatrix6 sqrtSegmentWeightDerivativeIntegral(
    const SplineSegmentMeta6& segment, const int derivative_order) {
  const SplineBasisMatrix6 integral =
      segmentWeightDerivativeIntegral(segment, derivative_order);
  Eigen::SelfAdjointEigenSolver<SplineBasisMatrix6> eig(integral);
  SplineBasisMatrix6 sqrt_diag = SplineBasisMatrix6::Zero();
  const double max_eigenvalue = std::max(0.0, eig.eigenvalues().maxCoeff());
  const double threshold = std::max(1e-18, 1e-12 * max_eigenvalue);
  for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
    const double lambda = eig.eigenvalues()(i);
    if (lambda > threshold) {
      sqrt_diag(i, i) = std::sqrt(lambda);
    }
  }
  return sqrt_diag * eig.eigenvectors().transpose();
}

}  // namespace ceres_cam_imu
