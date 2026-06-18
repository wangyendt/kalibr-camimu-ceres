#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <ceres/cost_function.h>
#include <ceres/sized_cost_function.h>

#include "ceres_cam_imu/trajectory/uniform_bspline.h"

namespace ceres_cam_imu {

using SplineBasisMatrix6 =
    Eigen::Matrix<double, SplineSegmentMeta6::kOrder,
                  SplineSegmentMeta6::kOrder>;

SplineBasisMatrix6 segmentWeightDerivativeIntegral(
    const SplineSegmentMeta6& segment, int derivative_order);

SplineBasisMatrix6 sqrtSegmentWeightDerivativeIntegral(
    const SplineSegmentMeta6& segment, int derivative_order);

template <int Dimension>
class EuclideanSplineMotionPriorCost final
    : public ceres::SizedCostFunction<SplineSegmentMeta6::kOrder * Dimension,
                                      Dimension, Dimension, Dimension,
                                      Dimension, Dimension, Dimension> {
 public:
  EuclideanSplineMotionPriorCost(
      const SplineSegmentMeta6& segment,
      const std::array<double, Dimension>& sqrt_information_diag,
      const int derivative_order)
      : sqrt_basis_(sqrtSegmentWeightDerivativeIntegral(segment,
                                                        derivative_order)),
        sqrt_information_diag_(sqrt_information_diag) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    constexpr int kOrder = SplineSegmentMeta6::kOrder;
    constexpr int kResidualSize = kOrder * Dimension;
    for (int row = 0; row < kOrder; ++row) {
      for (int dim = 0; dim < Dimension; ++dim) {
        double value = 0.0;
        for (int control = 0; control < kOrder; ++control) {
          value += sqrt_basis_(row, control) * parameters[control][dim];
        }
        residuals[row * Dimension + dim] =
            sqrt_information_diag_[static_cast<std::size_t>(dim)] * value;
      }
    }

    if (jacobians) {
      for (int control = 0; control < kOrder; ++control) {
        if (!jacobians[control]) {
          continue;
        }
        double* J = jacobians[control];
        std::fill(J, J + kResidualSize * Dimension, 0.0);
        for (int row = 0; row < kOrder; ++row) {
          for (int dim = 0; dim < Dimension; ++dim) {
            J[(row * Dimension + dim) * Dimension + dim] =
                sqrt_information_diag_[static_cast<std::size_t>(dim)]
                * sqrt_basis_(row, control);
          }
        }
      }
    }
    return true;
  }

 private:
  SplineBasisMatrix6 sqrt_basis_ = SplineBasisMatrix6::Zero();
  std::array<double, Dimension> sqrt_information_diag_{};
};

template <int Dimension>
ceres::CostFunction* createEuclideanSplineMotionPrior(
    const SplineSegmentMeta6& segment,
    const std::array<double, Dimension>& sqrt_information_diag,
    const int derivative_order) {
  return new EuclideanSplineMotionPriorCost<Dimension>(
      segment, sqrt_information_diag, derivative_order);
}

}  // namespace ceres_cam_imu
