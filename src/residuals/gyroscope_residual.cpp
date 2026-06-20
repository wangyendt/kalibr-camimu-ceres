#include "ceres_cam_imu/residuals/gyroscope_residual.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ceres/sized_cost_function.h>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/so3_jacobians.h"
#include "ceres_cam_imu/residuals/imu_model.h"
#include "ceres_cam_imu/variables/imu_intrinsics.h"

namespace ceres_cam_imu {
namespace {

template <typename Derived>
void writeMatrixRowMajor(const Eigen::MatrixBase<Derived> &matrix,
                         const int block_size, double *jacobian,
                         const int col_offset = 0) {
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      jacobian[row * block_size + col_offset + col] = matrix(row, col);
    }
  }
}

void writeLowerTriangularProductJacobian(const Vec3 &vector,
                                         const double scale,
                                         double *jacobian) {
  std::fill(jacobian, jacobian + 18, 0.0);
  jacobian[0 * 6 + 0] = scale * vector.x();
  jacobian[1 * 6 + 1] = scale * vector.x();
  jacobian[1 * 6 + 2] = scale * vector.y();
  jacobian[2 * 6 + 3] = scale * vector.x();
  jacobian[2 * 6 + 4] = scale * vector.y();
  jacobian[2 * 6 + 5] = scale * vector.z();
}

void writeFullMatrixProductJacobian(const Vec3 &vector, const double scale,
                                    double *jacobian) {
  std::fill(jacobian, jacobian + 27, 0.0);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      jacobian[row * 9 + row * 3 + col] = scale * vector(col);
    }
  }
}

const SplineSegmentMeta6 *
findBufferedSegment(const std::vector<SplineSegmentMeta6> &segments,
                    const double query_time_s, const double buffer_start_s,
                    const double buffer_end_s) {
  constexpr double kEpsilon = 1e-12;
  if (segments.empty()) {
    return nullptr;
  }
  if (query_time_s < buffer_start_s - kEpsilon ||
      query_time_s > buffer_end_s + kEpsilon) {
    return nullptr;
  }
  const double dt_s = segments.front().dt_s;
  int segment_index = static_cast<int>(std::floor(
      (query_time_s - segments.front().segment_start_s) / dt_s));
  if (segment_index == static_cast<int>(segments.size()) &&
      query_time_s <= segments.back().segment_start_s + dt_s + kEpsilon) {
    segment_index = static_cast<int>(segments.size()) - 1;
  }
  if (segment_index < 0 ||
      segment_index >= static_cast<int>(segments.size())) {
    return nullptr;
  }
  const SplineSegmentMeta6 &segment =
      segments[static_cast<std::size_t>(segment_index)];
  if (query_time_s < segment.segment_start_s - kEpsilon ||
      query_time_s > segment.segment_start_s + segment.dt_s + kEpsilon) {
    return nullptr;
  }
  return &segment;
}

class GyroscopeCost final
    : public ceres::SizedCostFunction<3, 6, 6, 6, 6, 6, 6, 6, 3, 3, 3, 3, 3,
                                      3> {
public:
  GyroscopeCost(ImuSample sample, ImuNoise noise,
                SplineSegmentMeta6 pose_segment,
                SplineSegmentMeta6 bias_segment)
      : sample(std::move(sample)), noise(std::move(noise)),
        pose_segment(std::move(pose_segment)),
        bias_segment(std::move(bias_segment)) {
    inv_sigma = 1.0 / std::max(1e-12, this->noise.gyroDiscreteSigma());
  }

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    const std::array<double, 6> pose_weights =
        pose_segment.weights(sample.timestamp_s, 0);
    const std::array<double, 6> pose_dot_weights =
        pose_segment.weights(sample.timestamp_s, 1);
    const std::array<double, 6> bias_weights =
        bias_segment.weights(sample.timestamp_s, 0);

    Vec6 curve = Vec6::Zero();
    Vec6 curve_dot = Vec6::Zero();
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec6> control(parameters[1 + i]);
      curve += pose_weights[static_cast<std::size_t>(i)] * control;
      curve_dot += pose_dot_weights[static_cast<std::size_t>(i)] * control;
    }

    Vec3 bias = Vec3::Zero();
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec3> control(parameters[7 + i]);
      bias += bias_weights[static_cast<std::size_t>(i)] * control;
    }

    const Vec3 r = curve.tail<3>();
    const Vec3 r_dot = curve_dot.tail<3>();
    const Mat3 J_left = leftJacobianSO3(r);
    const Vec3 omega_b = -J_left * r_dot;

    const Eigen::Map<const Vec3> r_i_b(parameters[0] + 3);
    const Mat3 R_i_b = rotationVectorToMatrix(r_i_b);
    const Vec3 predicted = R_i_b * omega_b + bias;
    const Vec3 residual = inv_sigma * (predicted - sample.gyro_rad_s);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (jacobians) {
      const Mat3 d_omega_d_r = -leftJacobianTimesVectorDerivative(r, r_dot);
      const Mat3 d_omega_d_r_dot = -J_left;

      if (jacobians[0]) {
        std::fill(jacobians[0], jacobians[0] + 18, 0.0);
        const Mat3 d_pred_d_r_i_b =
            inv_sigma * R_i_b * skew(omega_b) * leftJacobianSO3(r_i_b);
        for (int row = 0; row < 3; ++row) {
          for (int col = 0; col < 3; ++col) {
            jacobians[0][row * 6 + 3 + col] = d_pred_d_r_i_b(row, col);
          }
        }
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[1 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 18, 0.0);
        const Mat3 d_omega_d_control_rotation =
            d_omega_d_r * pose_weights[static_cast<std::size_t>(i)] +
            d_omega_d_r_dot * pose_dot_weights[static_cast<std::size_t>(i)];
        const Mat3 d_residual_d_control_rotation =
            inv_sigma * R_i_b * d_omega_d_control_rotation;
        for (int row = 0; row < 3; ++row) {
          for (int col = 0; col < 3; ++col) {
            J[row * 6 + 3 + col] = d_residual_d_control_rotation(row, col);
          }
        }
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[7 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 9, 0.0);
        const Mat3 d_residual_d_bias =
            inv_sigma * bias_weights[static_cast<std::size_t>(i)] *
            Mat3::Identity();
        writeMatrixRowMajor(d_residual_d_bias, 3, J);
      }
    }
    return true;
  }

private:
  ImuSample sample;
  ImuNoise noise;
  SplineSegmentMeta6 pose_segment;
  SplineSegmentMeta6 bias_segment;
  double inv_sigma = 1.0;
};

struct GyroscopeTimeOffsetEvaluation {
  const SplineSegmentMeta6 *pose_segment = nullptr;
  const SplineSegmentMeta6 *bias_segment = nullptr;
  int pose_active_offset = 0;
  int bias_active_offset = 0;
  std::array<double, 6> pose_weights = {};
  std::array<double, 6> pose_dot_weights = {};
  std::array<double, 6> bias_weights = {};
  Vec6 curve = Vec6::Zero();
  Vec6 curve_dot = Vec6::Zero();
  Vec3 bias = Vec3::Zero();
  Vec3 r = Vec3::Zero();
  Vec3 r_dot = Vec3::Zero();
  Mat3 J_left = Mat3::Identity();
  Vec3 omega_b = Vec3::Zero();
  Vec3 r_i_b = Vec3::Zero();
  Mat3 R_i_b = Mat3::Identity();
};

class GyroscopeTimeOffsetCost final : public ceres::CostFunction {
public:
  GyroscopeTimeOffsetCost(ImuSample sample, ImuNoise noise,
                          std::vector<SplineSegmentMeta6> pose_segments,
                          const int pose_local_coeff_start,
                          std::vector<SplineSegmentMeta6> bias_segments,
                          const int bias_local_coeff_start,
                          const double buffer_start_s,
                          const double buffer_end_s)
      : sample_(std::move(sample)), noise_(std::move(noise)),
        pose_segments_(std::move(pose_segments)),
        bias_segments_(std::move(bias_segments)),
        pose_local_coeff_start_(pose_local_coeff_start),
        bias_local_coeff_start_(bias_local_coeff_start),
        buffer_start_s_(buffer_start_s), buffer_end_s_(buffer_end_s) {
    if (pose_segments_.empty() || bias_segments_.empty()) {
      throw std::invalid_argument(
          "time-offset gyro residual requires spline segments");
    }
    if (!(buffer_end_s_ >= buffer_start_s_)) {
      throw std::invalid_argument("invalid time-offset gyro residual buffer");
    }
    pose_local_coeff_count_ =
        pose_segments_.back().coeff_start + SplineSegmentMeta6::kOrder -
        pose_local_coeff_start_;
    bias_local_coeff_count_ =
        bias_segments_.back().coeff_start + SplineSegmentMeta6::kOrder -
        bias_local_coeff_start_;
    if (pose_local_coeff_count_ < SplineSegmentMeta6::kOrder ||
        bias_local_coeff_count_ < SplineSegmentMeta6::kOrder) {
      throw std::invalid_argument(
          "invalid time-offset gyro residual coefficient span");
    }
    inv_sigma_ =
        1.0 / std::max(1e-12, this->noise_.gyroDiscreteSigma());

    set_num_residuals(3);
    mutable_parameter_block_sizes()->push_back(6);
    mutable_parameter_block_sizes()->push_back(1);
    for (int i = 0; i < pose_local_coeff_count_; ++i) {
      mutable_parameter_block_sizes()->push_back(6);
    }
    for (int i = 0; i < bias_local_coeff_count_; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    const double query_time_s = sample_.timestamp_s + parameters[1][0];
    GyroscopeTimeOffsetEvaluation evaluation;
    if (!evaluateAtTime(parameters, query_time_s, residuals, &evaluation)) {
      return false;
    }

    if (!jacobians) {
      return true;
    }

    const Mat3 d_omega_d_r =
        -leftJacobianTimesVectorDerivative(evaluation.r, evaluation.r_dot);
    const Mat3 d_omega_d_r_dot = -evaluation.J_left;

    if (jacobians[0]) {
      std::fill(jacobians[0], jacobians[0] + 18, 0.0);
      const Mat3 d_pred_d_r_i_b =
          inv_sigma_ * evaluation.R_i_b * skew(evaluation.omega_b) *
          leftJacobianSO3(evaluation.r_i_b);
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          jacobians[0][row * 6 + 3 + col] = d_pred_d_r_i_b(row, col);
        }
      }
    }

    if (jacobians[1]) {
      writeTimeOffsetJacobian(parameters, query_time_s, residuals,
                              jacobians[1]);
    }

    for (int control = 0; control < pose_local_coeff_count_; ++control) {
      double *J = jacobians[2 + control];
      if (J) {
        std::fill(J, J + 18, 0.0);
      }
    }
    for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
      double *J = jacobians[2 + evaluation.pose_active_offset + i];
      if (!J) {
        continue;
      }
      const Mat3 d_omega_d_control_rotation =
          d_omega_d_r * evaluation.pose_weights[static_cast<std::size_t>(i)] +
          d_omega_d_r_dot *
              evaluation.pose_dot_weights[static_cast<std::size_t>(i)];
      const Mat3 d_residual_d_control_rotation =
          inv_sigma_ * evaluation.R_i_b * d_omega_d_control_rotation;
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          J[row * 6 + 3 + col] = d_residual_d_control_rotation(row, col);
        }
      }
    }

    const int bias_block_offset = 2 + pose_local_coeff_count_;
    for (int control = 0; control < bias_local_coeff_count_; ++control) {
      double *J = jacobians[bias_block_offset + control];
      if (J) {
        std::fill(J, J + 9, 0.0);
      }
    }
    for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
      double *J =
          jacobians[bias_block_offset + evaluation.bias_active_offset + i];
      if (!J) {
        continue;
      }
      const Mat3 d_residual_d_bias =
          inv_sigma_ *
          evaluation.bias_weights[static_cast<std::size_t>(i)] *
          Mat3::Identity();
      writeMatrixRowMajor(d_residual_d_bias, 3, J);
    }

    return true;
  }

private:
  bool evaluateAtTime(double const *const *parameters,
                      const double query_time_s, double *residuals,
                      GyroscopeTimeOffsetEvaluation *evaluation) const {
    const SplineSegmentMeta6 *pose_segment = findBufferedSegment(
        pose_segments_, query_time_s, buffer_start_s_, buffer_end_s_);
    const SplineSegmentMeta6 *bias_segment = findBufferedSegment(
        bias_segments_, query_time_s, buffer_start_s_, buffer_end_s_);
    if (!pose_segment || !bias_segment) {
      return false;
    }
    const int pose_active_offset =
        pose_segment->coeff_start - pose_local_coeff_start_;
    const int bias_active_offset =
        bias_segment->coeff_start - bias_local_coeff_start_;
    if (pose_active_offset < 0 ||
        pose_active_offset + SplineSegmentMeta6::kOrder >
            pose_local_coeff_count_ ||
        bias_active_offset < 0 ||
        bias_active_offset + SplineSegmentMeta6::kOrder >
            bias_local_coeff_count_) {
      return false;
    }

    GyroscopeTimeOffsetEvaluation local;
    local.pose_segment = pose_segment;
    local.bias_segment = bias_segment;
    local.pose_active_offset = pose_active_offset;
    local.bias_active_offset = bias_active_offset;
    local.pose_weights = pose_segment->weights(query_time_s, 0);
    local.pose_dot_weights = pose_segment->weights(query_time_s, 1);
    local.bias_weights = bias_segment->weights(query_time_s, 0);

    for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
      const Eigen::Map<const Vec6> control(
          parameters[2 + pose_active_offset + i]);
      local.curve += local.pose_weights[static_cast<std::size_t>(i)] * control;
      local.curve_dot +=
          local.pose_dot_weights[static_cast<std::size_t>(i)] * control;
    }

    const int bias_block_offset = 2 + pose_local_coeff_count_;
    for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
      const Eigen::Map<const Vec3> control(
          parameters[bias_block_offset + bias_active_offset + i]);
      local.bias += local.bias_weights[static_cast<std::size_t>(i)] * control;
    }

    local.r = local.curve.tail<3>();
    local.r_dot = local.curve_dot.tail<3>();
    local.J_left = leftJacobianSO3(local.r);
    local.omega_b = -local.J_left * local.r_dot;
    local.r_i_b = Eigen::Map<const Vec3>(parameters[0] + 3);
    local.R_i_b = rotationVectorToMatrix(local.r_i_b);
    const Vec3 predicted = local.R_i_b * local.omega_b + local.bias;
    const Vec3 residual = inv_sigma_ * (predicted - sample_.gyro_rad_s);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();
    if (evaluation) {
      *evaluation = local;
    }
    return true;
  }

  void writeTimeOffsetJacobian(double const *const *parameters,
                               const double query_time_s,
                               const double *residuals,
                               double *jacobian) const {
    constexpr double kEps = 1e-6;
    double plus[3] = {};
    double minus[3] = {};
    const bool plus_ok =
        evaluateAtTime(parameters, query_time_s + kEps, plus, nullptr);
    const bool minus_ok =
        evaluateAtTime(parameters, query_time_s - kEps, minus, nullptr);
    if (plus_ok && minus_ok) {
      for (int row = 0; row < 3; ++row) {
        jacobian[row] = (plus[row] - minus[row]) / (2.0 * kEps);
      }
      return;
    }
    if (plus_ok) {
      for (int row = 0; row < 3; ++row) {
        jacobian[row] = (plus[row] - residuals[row]) / kEps;
      }
      return;
    }
    if (minus_ok) {
      for (int row = 0; row < 3; ++row) {
        jacobian[row] = (residuals[row] - minus[row]) / kEps;
      }
      return;
    }
    std::fill(jacobian, jacobian + 3, 0.0);
  }

  ImuSample sample_;
  ImuNoise noise_;
  std::vector<SplineSegmentMeta6> pose_segments_;
  std::vector<SplineSegmentMeta6> bias_segments_;
  int pose_local_coeff_start_ = 0;
  int bias_local_coeff_start_ = 0;
  int pose_local_coeff_count_ = 0;
  int bias_local_coeff_count_ = 0;
  double buffer_start_s_ = 0.0;
  double buffer_end_s_ = 0.0;
  double inv_sigma_ = 1.0;
};

class ScaleMisalignedGyroscopeCost final
    : public ceres::SizedCostFunction<3, 6, 3, 6, 6, 6, 6, 6, 6, 3, 3, 3, 3,
                                      3, 3, 3, 6, 9> {
public:
  ScaleMisalignedGyroscopeCost(ImuSample sample, ImuNoise noise,
                               SplineSegmentMeta6 pose_segment,
                               SplineSegmentMeta6 bias_segment)
      : sample(std::move(sample)), noise(std::move(noise)),
        pose_segment(std::move(pose_segment)),
        bias_segment(std::move(bias_segment)) {
    inv_sigma = 1.0 / std::max(1e-12, this->noise.gyroDiscreteSigma());
  }

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    Vec6 curve = Vec6::Zero();
    Vec6 curve_dot = Vec6::Zero();
    Vec6 curve_ddot = Vec6::Zero();
    const std::array<double, 6> pose_weights =
        pose_segment.weights(sample.timestamp_s, 0);
    const std::array<double, 6> pose_dot_weights =
        pose_segment.weights(sample.timestamp_s, 1);
    const std::array<double, 6> pose_ddot_weights =
        pose_segment.weights(sample.timestamp_s, 2);
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec6> control(parameters[2 + i]);
      curve += pose_weights[static_cast<std::size_t>(i)] * control;
      curve_dot += pose_dot_weights[static_cast<std::size_t>(i)] * control;
      curve_ddot += pose_ddot_weights[static_cast<std::size_t>(i)] * control;
    }

    Vec3 bias = Vec3::Zero();
    const std::array<double, 6> bias_weights =
        bias_segment.weights(sample.timestamp_s, 0);
    for (int i = 0; i < 6; ++i) {
      bias += bias_weights[static_cast<std::size_t>(i)] *
              Eigen::Map<const Vec3>(parameters[8 + i]);
    }

    const Vec3 r_w_b = curve.tail<3>();
    const Mat3 R_b_w = rotationVectorToMatrix(r_w_b).transpose();
    const Mat3 J_left = leftJacobianSO3(r_w_b);
    const Vec3 omega_b = -J_left * curve_dot.tail<3>();
    const Vec3 alpha_b = -J_left * curve_ddot.tail<3>();
    const Vec3 q_w = curve_ddot.head<3>() - Eigen::Map<const Vec3>(parameters[1]);
    const Vec3 h_b =
        R_b_w * q_w;
    const Vec3 r_b = Eigen::Map<const Vec3>(parameters[0]);
    const Vec3 a_b = h_b + commonLeverAcceleration(omega_b, alpha_b, r_b);

    const Eigen::Map<const Vec3> r_i_b(parameters[0] + 3);
    const Mat3 R_i_b =
        rotationVectorToMatrix(r_i_b);
    const Eigen::Map<const Vec3> r_gyro_i(parameters[14]);
    const Mat3 R_gyro_i =
        rotationVectorToMatrix(r_gyro_i);
    const Mat3 M_gyro = lowerTriangularMatrix(parameters[15]);
    const Mat3 A_gyro_accel = matrix3Block(parameters[16]);
    const Vec3 predicted = predictScaleMisalignedGyroscope(
        R_i_b, R_gyro_i, M_gyro, A_gyro_accel, omega_b, a_b, bias);
    const Vec3 residual = inv_sigma * (predicted - sample.gyro_rad_s);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (jacobians) {
      const Mat3 R_gyro_b = R_gyro_i * R_i_b;
      const Vec3 omega_g = R_gyro_b * omega_b;
      const Vec3 accel_g = R_gyro_b * a_b;
      const Mat3 d_omega_d_r =
          -leftJacobianTimesVectorDerivative(r_w_b, curve_dot.tail<3>());
      const Mat3 d_omega_d_r_dot = -J_left;
      const Mat3 d_alpha_d_r =
          -leftJacobianTimesVectorDerivative(r_w_b, curve_ddot.tail<3>());
      const Mat3 d_alpha_d_r_ddot = -J_left;
      const Mat3 d_h_d_r = rotationTransposeTimesVectorDerivative(r_w_b, q_w);
      const Mat3 d_lever_d_alpha = -skew(r_b);
      const Mat3 d_lever_d_omega =
          -skew(omega_b.cross(r_b)) - skew(omega_b) * skew(r_b);
      const Mat3 d_a_d_r = d_h_d_r + d_lever_d_alpha * d_alpha_d_r +
                           d_lever_d_omega * d_omega_d_r;
      const Mat3 d_a_d_r_dot = d_lever_d_omega * d_omega_d_r_dot;
      const Mat3 d_a_d_r_ddot = d_lever_d_alpha * d_alpha_d_r_ddot;

      if (jacobians[0]) {
        std::fill(jacobians[0], jacobians[0] + 18, 0.0);
        const Mat3 d_a_d_r_b = skew(alpha_b) + skew(omega_b) * skew(omega_b);
        const Mat3 d_residual_d_r_b =
            inv_sigma * A_gyro_accel * R_gyro_b * d_a_d_r_b;
        const Mat3 d_residual_d_r_i_b =
            inv_sigma *
            (M_gyro * R_gyro_i * R_i_b * skew(omega_b) *
                 leftJacobianSO3(r_i_b) +
             A_gyro_accel * R_gyro_i * R_i_b * skew(a_b) *
                 leftJacobianSO3(r_i_b));
        writeMatrixRowMajor(d_residual_d_r_b, 6, jacobians[0], 0);
        writeMatrixRowMajor(d_residual_d_r_i_b, 6, jacobians[0], 3);
      }

      if (jacobians[1]) {
        const Mat3 d_residual_d_gravity =
            -inv_sigma * A_gyro_accel * R_gyro_b * R_b_w;
        writeMatrixRowMajor(d_residual_d_gravity, 3, jacobians[1]);
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[2 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 18, 0.0);
        const Mat3 d_omega_d_control_rotation =
            d_omega_d_r * pose_weights[static_cast<std::size_t>(i)] +
            d_omega_d_r_dot * pose_dot_weights[static_cast<std::size_t>(i)];
        const Mat3 d_a_d_control_rotation =
            d_a_d_r * pose_weights[static_cast<std::size_t>(i)] +
            d_a_d_r_dot * pose_dot_weights[static_cast<std::size_t>(i)] +
            d_a_d_r_ddot * pose_ddot_weights[static_cast<std::size_t>(i)];
        const Mat3 d_a_d_control_translation =
            pose_ddot_weights[static_cast<std::size_t>(i)] * R_b_w;
        const Mat3 d_residual_d_translation =
            inv_sigma * A_gyro_accel * R_gyro_b *
            d_a_d_control_translation;
        const Mat3 d_residual_d_rotation =
            inv_sigma *
            (M_gyro * R_gyro_b * d_omega_d_control_rotation +
             A_gyro_accel * R_gyro_b * d_a_d_control_rotation);
        writeMatrixRowMajor(d_residual_d_translation, 6, J, 0);
        writeMatrixRowMajor(d_residual_d_rotation, 6, J, 3);
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[8 + i];
        if (!J) {
          continue;
        }
        const Mat3 d_residual_d_bias =
            inv_sigma * bias_weights[static_cast<std::size_t>(i)] *
            Mat3::Identity();
        writeMatrixRowMajor(d_residual_d_bias, 3, J);
      }

      if (jacobians[14]) {
        const Mat3 d_residual_d_r_gyro_i =
            inv_sigma *
            (M_gyro * R_gyro_i * skew(R_i_b * omega_b) *
                 leftJacobianSO3(r_gyro_i) +
             A_gyro_accel * R_gyro_i * skew(R_i_b * a_b) *
                 leftJacobianSO3(r_gyro_i));
        writeMatrixRowMajor(d_residual_d_r_gyro_i, 3, jacobians[14]);
      }

      if (jacobians[15]) {
        writeLowerTriangularProductJacobian(omega_g, inv_sigma,
                                            jacobians[15]);
      }

      if (jacobians[16]) {
        writeFullMatrixProductJacobian(accel_g, inv_sigma, jacobians[16]);
      }
    }
    return true;
  }

private:
  ImuSample sample;
  ImuNoise noise;
  SplineSegmentMeta6 pose_segment;
  SplineSegmentMeta6 bias_segment;
  double inv_sigma = 1.0;
};

} // namespace

ceres::CostFunction *
createGyroscopeResidual(const ImuSample &sample, const ImuNoise &noise,
                        const SplineSegmentMeta6 &pose_segment,
                        const SplineSegmentMeta6 &gyro_bias_segment) {
  return new GyroscopeCost(sample, noise, pose_segment, gyro_bias_segment);
}

ceres::CostFunction *createGyroscopeTimeOffsetResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const std::vector<SplineSegmentMeta6> &pose_segments,
    const int pose_local_coeff_start,
    const std::vector<SplineSegmentMeta6> &gyro_bias_segments,
    const int gyro_bias_local_coeff_start, const double buffer_start_s,
    const double buffer_end_s) {
  return new GyroscopeTimeOffsetCost(
      sample, noise, pose_segments, pose_local_coeff_start, gyro_bias_segments,
      gyro_bias_local_coeff_start, buffer_start_s, buffer_end_s);
}

ceres::CostFunction *createScaleMisalignedGyroscopeResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const SplineSegmentMeta6 &pose_segment,
    const SplineSegmentMeta6 &gyro_bias_segment) {
  return new ScaleMisalignedGyroscopeCost(sample, noise, pose_segment,
                                          gyro_bias_segment);
}

} // namespace ceres_cam_imu
