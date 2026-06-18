#include "ceres_cam_imu/trajectory/uniform_bspline.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ceres_cam_imu {

int derivativeMultiplier(const int power, const int derivative_order) {
  if (derivative_order == 0) {
    return 1;
  }
  int value = 1;
  for (int i = 0; i < derivative_order; ++i) {
    value *= (power - i);
  }
  return value;
}

double SplineSegmentMeta6::derivativeScale(const int derivative_order) const {
  double scale = 1.0;
  for (int i = 0; i < derivative_order; ++i) {
    scale /= dt_s;
  }
  return scale;
}

UniformBSpline::UniformBSpline(const int dimension, const int order,
                               const double t_min_s, const double t_max_s,
                               const int num_segments)
    : dimension_(dimension),
      order_(order),
      num_segments_(num_segments),
      t_min_s_(t_min_s),
      t_max_s_(t_max_s) {
  if (dimension_ <= 0) {
    throw std::invalid_argument("spline dimension must be positive");
  }
  if (order_ < 2) {
    throw std::invalid_argument("spline order must be >= 2");
  }
  if (num_segments_ <= 0) {
    throw std::invalid_argument("spline must have at least one segment");
  }
  if (!(t_max_s_ > t_min_s_)) {
    throw std::invalid_argument("spline time interval must be increasing");
  }
  dt_s_ = (t_max_s_ - t_min_s_) / static_cast<double>(num_segments_);
  initializeKnots();
  initializeBasisMatrices();
}

bool UniformBSpline::isValidTime(const double timestamp_s) const {
  return valid() && timestamp_s >= t_min_s_ && timestamp_s <= t_max_s_;
}

int UniformBSpline::segmentIndex(const double timestamp_s) const {
  if (!isValidTime(timestamp_s)) {
    throw std::out_of_range("timestamp outside spline interval");
  }
  if (timestamp_s == t_max_s_) {
    return num_segments_ - 1;
  }
  const double s = (timestamp_s - t_min_s_) / dt_s_;
  return std::clamp(static_cast<int>(std::floor(s)), 0, num_segments_ - 1);
}

SplineSegmentMeta6 UniformBSpline::segmentMeta6(const double timestamp_s) const {
  if (order_ != SplineSegmentMeta6::kOrder) {
    throw std::logic_error("segmentMeta6 requires an order-6 spline");
  }
  const int segment = segmentIndex(timestamp_s);
  SplineSegmentMeta6 meta;
  meta.coeff_start = segment;
  meta.segment_start_s = t_min_s_ + static_cast<double>(segment) * dt_s_;
  meta.dt_s = dt_s_;
  meta.basis = basis_matrices_.at(static_cast<std::size_t>(segment));
  return meta;
}

Eigen::VectorXd UniformBSpline::evaluate(
    const std::vector<Eigen::VectorXd>& coefficients, const double timestamp_s,
    const int derivative_order) const {
  if (static_cast<int>(coefficients.size()) != numCoefficients()) {
    throw std::invalid_argument("coefficient count does not match spline");
  }
  const int segment = segmentIndex(timestamp_s);
  const double u_value = (timestamp_s - (t_min_s_ + segment * dt_s_)) / dt_s_;
  Eigen::VectorXd u = Eigen::VectorXd::Zero(order_);
  double power = 1.0;
  double scale = 1.0;
  for (int i = 0; i < derivative_order; ++i) {
    scale /= dt_s_;
  }
  for (int i = 0; i < order_; ++i) {
    if (i >= derivative_order) {
      u(i) = scale * derivativeMultiplier(i, derivative_order) * power;
      power *= u_value;
    }
  }
  const Eigen::VectorXd weights = basis_matrices_[segment].transpose() * u;

  Eigen::VectorXd value = Eigen::VectorXd::Zero(dimension_);
  for (int i = 0; i < order_; ++i) {
    value += weights(i) * coefficients.at(static_cast<std::size_t>(segment + i));
  }
  return value;
}

const Eigen::MatrixXd& UniformBSpline::basisMatrix(const int segment_index) const {
  return basis_matrices_.at(static_cast<std::size_t>(segment_index));
}

void UniformBSpline::initializeKnots() {
  const int num_knots = numCoefficients() + order_;
  knots_.resize(static_cast<std::size_t>(num_knots));
  for (int i = 0; i < num_knots; ++i) {
    knots_[static_cast<std::size_t>(i)] =
        t_min_s_ + static_cast<double>(i - order_ + 1) * dt_s_;
  }
}

void UniformBSpline::initializeBasisMatrices() {
  basis_matrices_.resize(static_cast<std::size_t>(num_segments_));
  for (int segment = 0; segment < num_segments_; ++segment) {
    basis_matrices_[static_cast<std::size_t>(segment)] =
        basisMatrixRecursive(order_, segment + order_ - 1);
  }
}

Eigen::MatrixXd UniformBSpline::basisMatrixRecursive(const int k,
                                                     const int i) const {
  if (k == 1) {
    Eigen::MatrixXd m(1, 1);
    m(0, 0) = 1.0;
    return m;
  }

  const Eigen::MatrixXd previous = basisMatrixRecursive(k - 1, i);
  Eigen::MatrixXd m1 = Eigen::MatrixXd::Zero(previous.rows() + 1, previous.cols());
  Eigen::MatrixXd m2 = Eigen::MatrixXd::Zero(previous.rows() + 1, previous.cols());
  m1.topRightCorner(previous.rows(), previous.cols()) = previous;
  m2.bottomRightCorner(previous.rows(), previous.cols()) = previous;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(k - 1, k);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(k - 1, k);
  for (int row = 0; row < A.rows(); ++row) {
    const int j = i - k + 2 + row;
    const double d_0 = d0(k, i, j);
    const double d_1 = d1(k, i, j);
    A(row, row) = 1.0 - d_0;
    A(row, row + 1) = d_0;
    B(row, row) = -d_1;
    B(row, row + 1) = d_1;
  }
  return m1 * A + m2 * B;
}

double UniformBSpline::d0(const int k, const int i, const int j) const {
  const double denom = knots_.at(static_cast<std::size_t>(j + k - 1))
                     - knots_.at(static_cast<std::size_t>(j));
  if (denom <= 0.0) {
    return 0.0;
  }
  return (knots_.at(static_cast<std::size_t>(i))
        - knots_.at(static_cast<std::size_t>(j))) / denom;
}

double UniformBSpline::d1(const int k, const int i, const int j) const {
  const double denom = knots_.at(static_cast<std::size_t>(j + k - 1))
                     - knots_.at(static_cast<std::size_t>(j));
  if (denom <= 0.0) {
    return 0.0;
  }
  return (knots_.at(static_cast<std::size_t>(i + 1))
        - knots_.at(static_cast<std::size_t>(i))) / denom;
}

UniformBSpline makeSplineForTimes(const int dimension, const int order,
                                  const double first_time_s,
                                  const double last_time_s,
                                  const double knots_per_second,
                                  const double padding_s) {
  // Kalibr's cam-imu initializer treats timeoffset-padding as the maximum
  // calibration-time shift and pads the fitted pose-spline observations by
  // twice that value on both sides before computing the segment count.
  const double kalibr_side_padding_s = 2.0 * padding_s;
  const double t_min = first_time_s - kalibr_side_padding_s;
  const double t_max = last_time_s + kalibr_side_padding_s;
  const double seconds = std::max(1e-3, t_max - t_min);
  const int segments = std::max(1, static_cast<int>(std::round(seconds * knots_per_second)));
  return UniformBSpline(dimension, order, t_min, t_max, segments);
}

}  // namespace ceres_cam_imu
