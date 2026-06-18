#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

int derivativeMultiplier(int power, int derivative_order);

struct SplineSegmentMeta6 {
  static constexpr int kOrder = 6;

  int coeff_start = 0;
  double segment_start_s = 0.0;
  double dt_s = 1.0;
  Eigen::Matrix<double, kOrder, kOrder> basis =
      Eigen::Matrix<double, kOrder, kOrder>::Identity();

  template <typename T>
  std::array<T, kOrder> weights(const T& timestamp_s,
                                int derivative_order) const {
    std::array<T, kOrder> u{};
    const T normalized_u = (timestamp_s - T(segment_start_s)) / T(dt_s);
    T power = T(1);
    const double inv_dt_power = derivativeScale(derivative_order);
    for (int i = 0; i < kOrder; ++i) {
      if (i >= derivative_order) {
        u[static_cast<std::size_t>(i)] =
            T(inv_dt_power * derivativeMultiplier(i, derivative_order)) * power;
        power *= normalized_u;
      }
    }

    std::array<T, kOrder> w{};
    for (int col = 0; col < kOrder; ++col) {
      T value = T(0);
      for (int row = 0; row < kOrder; ++row) {
        value += T(basis(row, col)) * u[static_cast<std::size_t>(row)];
      }
      w[static_cast<std::size_t>(col)] = value;
    }
    return w;
  }

 private:
  double derivativeScale(int derivative_order) const;
};

class UniformBSpline {
 public:
  UniformBSpline() = default;
  UniformBSpline(int dimension, int order, double t_min_s, double t_max_s,
                 int num_segments);

  int dimension() const { return dimension_; }
  int order() const { return order_; }
  int numSegments() const { return num_segments_; }
  int numCoefficients() const { return num_segments_ + order_ - 1; }
  double tMin() const { return t_min_s_; }
  double tMax() const { return t_max_s_; }
  double dt() const { return dt_s_; }

  bool valid() const { return dimension_ > 0 && order_ > 1 && num_segments_ > 0; }
  bool isValidTime(double timestamp_s) const;
  int segmentIndex(double timestamp_s) const;
  SplineSegmentMeta6 segmentMeta6(double timestamp_s) const;

  Eigen::VectorXd evaluate(const std::vector<Eigen::VectorXd>& coefficients,
                           double timestamp_s, int derivative_order) const;

  const Eigen::MatrixXd& basisMatrix(int segment_index) const;

 private:
  int dimension_ = 0;
  int order_ = 0;
  int num_segments_ = 0;
  double t_min_s_ = 0.0;
  double t_max_s_ = 0.0;
  double dt_s_ = 0.0;
  std::vector<double> knots_;
  std::vector<Eigen::MatrixXd> basis_matrices_;

  void initializeKnots();
  void initializeBasisMatrices();
  Eigen::MatrixXd basisMatrixRecursive(int k, int i) const;
  double d0(int k, int i, int j) const;
  double d1(int k, int i, int j) const;
};

UniformBSpline makeSplineForTimes(int dimension, int order, double first_time_s,
                                  double last_time_s, double knots_per_second,
                                  double padding_s);

}  // namespace ceres_cam_imu
