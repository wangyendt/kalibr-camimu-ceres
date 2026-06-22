#include "ceres_cam_imu/residuals/accelerometer_residual.h"

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

void writeMatrixRowMajor(const Mat3 &matrix, const int block_size,
                         double *jacobian, const int col_offset = 0) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
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

Mat3 leverAccelerationPointJacobian(const Vec3 &omega_b,
                                    const Vec3 &alpha_b) {
  return skew(alpha_b) + skew(omega_b) * skew(omega_b);
}

Mat3 leverAccelerationOmegaJacobian(const Vec3 &omega_b, const Vec3 &r_b) {
  return -skew(omega_b.cross(r_b)) - skew(omega_b) * skew(r_b);
}

Mat3 leverAccelerationAlphaJacobian(const Vec3 &r_b) { return -skew(r_b); }

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

class AccelerometerCost final
    : public ceres::SizedCostFunction<3, 6, 3, 6, 6, 6, 6, 6, 6, 3, 3, 3, 3, 3,
                                      3> {
public:
  AccelerometerCost(ImuSample sample, ImuNoise noise,
                    SplineSegmentMeta6 pose_segment,
                    SplineSegmentMeta6 bias_segment)
      : sample(std::move(sample)), noise(std::move(noise)),
        pose_segment(std::move(pose_segment)),
        bias_segment(std::move(bias_segment)) {
    inv_sigma = 1.0 / std::max(1e-12, this->noise.accelDiscreteSigma());
  }

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    const std::array<double, 6> pose_weights =
        pose_segment.weights(sample.timestamp_s, 0);
    const std::array<double, 6> pose_dot_weights =
        pose_segment.weights(sample.timestamp_s, 1);
    const std::array<double, 6> pose_ddot_weights =
        pose_segment.weights(sample.timestamp_s, 2);
    const std::array<double, 6> bias_weights =
        bias_segment.weights(sample.timestamp_s, 0);

    Vec6 curve = Vec6::Zero();
    Vec6 curve_dot = Vec6::Zero();
    Vec6 curve_ddot = Vec6::Zero();
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec6> control(parameters[2 + i]);
      curve += pose_weights[static_cast<std::size_t>(i)] * control;
      curve_dot += pose_dot_weights[static_cast<std::size_t>(i)] * control;
      curve_ddot += pose_ddot_weights[static_cast<std::size_t>(i)] * control;
    }

    Vec3 bias = Vec3::Zero();
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec3> control(parameters[8 + i]);
      bias += bias_weights[static_cast<std::size_t>(i)] * control;
    }

    const Vec3 r_w_b = curve.tail<3>();
    const Mat3 R_bw = rotationVectorToMatrix(r_w_b).transpose();
    const Vec3 a_w = curve_ddot.head<3>();
    const Eigen::Map<const Vec3> g_w(parameters[1]);
    const Vec3 q_w = a_w - g_w;
    const Vec3 h_b = R_bw * q_w;

    const Vec3 r_dot = curve_dot.tail<3>();
    const Vec3 r_ddot = curve_ddot.tail<3>();
    const Mat3 J_left = leftJacobianSO3(r_w_b);
    const Vec3 omega_b = -J_left * r_dot;
    const Vec3 alpha_b = -J_left * r_ddot;

    const Eigen::Map<const Vec3> r_b(parameters[0]);
    const Vec3 lever = alpha_b.cross(r_b) + omega_b.cross(omega_b.cross(r_b));
    const Vec3 body_specific_force = h_b + lever;

    const Eigen::Map<const Vec3> r_i_b(parameters[0] + 3);
    const Mat3 R_i_b = rotationVectorToMatrix(r_i_b);
    const Vec3 predicted = R_i_b * body_specific_force + bias;
    const Vec3 residual = inv_sigma * (predicted - sample.accel_m_s2);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (jacobians) {
      const Mat3 d_omega_d_r = -leftJacobianTimesVectorDerivative(r_w_b, r_dot);
      const Mat3 d_omega_d_r_dot = -J_left;
      const Mat3 d_alpha_d_r =
          -leftJacobianTimesVectorDerivative(r_w_b, r_ddot);
      const Mat3 d_alpha_d_r_ddot = -J_left;
      const Mat3 d_h_d_r = rotationTransposeTimesVectorDerivative(r_w_b, q_w);
      const Mat3 d_lever_d_alpha = -skew(r_b);
      const Mat3 d_lever_d_omega =
          -skew(omega_b.cross(r_b)) - skew(omega_b) * skew(r_b);
      const Mat3 d_body_d_r = d_h_d_r + d_lever_d_alpha * d_alpha_d_r +
                              d_lever_d_omega * d_omega_d_r;
      const Mat3 d_body_d_r_dot = d_lever_d_omega * d_omega_d_r_dot;
      const Mat3 d_body_d_r_ddot = d_lever_d_alpha * d_alpha_d_r_ddot;

      if (jacobians[0]) {
        std::fill(jacobians[0], jacobians[0] + 18, 0.0);
        const Mat3 d_body_d_r_b = skew(alpha_b) + skew(omega_b) * skew(omega_b);
        const Mat3 d_residual_d_r_b = inv_sigma * R_i_b * d_body_d_r_b;
        const Mat3 d_residual_d_r_i_b = inv_sigma * R_i_b *
                                        skew(body_specific_force) *
                                        leftJacobianSO3(r_i_b);
        writeMatrixRowMajor(d_residual_d_r_b, 6, jacobians[0], 0);
        writeMatrixRowMajor(d_residual_d_r_i_b, 6, jacobians[0], 3);
      }

      if (jacobians[1]) {
        const Mat3 d_residual_d_gravity = -inv_sigma * R_i_b * R_bw;
        writeMatrixRowMajor(d_residual_d_gravity, 3, jacobians[1]);
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[2 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 18, 0.0);
        const Mat3 d_body_d_translation_control =
            pose_ddot_weights[static_cast<std::size_t>(i)] * R_bw;
        const Mat3 d_body_d_rotation_control =
            pose_weights[static_cast<std::size_t>(i)] * d_body_d_r +
            pose_dot_weights[static_cast<std::size_t>(i)] * d_body_d_r_dot +
            pose_ddot_weights[static_cast<std::size_t>(i)] * d_body_d_r_ddot;
        writeMatrixRowMajor(inv_sigma * R_i_b * d_body_d_translation_control, 6,
                            J, 0);
        writeMatrixRowMajor(inv_sigma * R_i_b * d_body_d_rotation_control, 6, J,
                            3);
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

struct AccelerometerTimeOffsetEvaluation {
  const SplineSegmentMeta6 *pose_segment = nullptr;
  const SplineSegmentMeta6 *bias_segment = nullptr;
  int pose_active_offset = 0;
  int bias_active_offset = 0;
  std::array<double, 6> pose_weights = {};
  std::array<double, 6> pose_dot_weights = {};
  std::array<double, 6> pose_ddot_weights = {};
  std::array<double, 6> pose_dddot_weights = {};
  std::array<double, 6> bias_weights = {};
  std::array<double, 6> bias_dot_weights = {};
  Vec6 curve = Vec6::Zero();
  Vec6 curve_dot = Vec6::Zero();
  Vec6 curve_ddot = Vec6::Zero();
  Vec6 curve_dddot = Vec6::Zero();
  Vec3 bias = Vec3::Zero();
  Vec3 bias_dot = Vec3::Zero();
  Vec3 r_w_b = Vec3::Zero();
  Mat3 R_b_w = Mat3::Identity();
  Vec3 q_w = Vec3::Zero();
  Vec3 h_b = Vec3::Zero();
  Vec3 omega_b = Vec3::Zero();
  Vec3 alpha_b = Vec3::Zero();
  Vec3 r_b = Vec3::Zero();
  Vec3 r_i_b = Vec3::Zero();
  Mat3 R_i_b = Mat3::Identity();
  Vec3 body_specific_force = Vec3::Zero();
};

class AccelerometerTimeOffsetCost final : public ceres::CostFunction {
public:
  AccelerometerTimeOffsetCost(ImuSample sample, ImuNoise noise,
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
          "time-offset accel residual requires spline segments");
    }
    if (!(buffer_end_s_ >= buffer_start_s_)) {
      throw std::invalid_argument("invalid time-offset accel residual buffer");
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
          "invalid time-offset accel residual coefficient span");
    }
    inv_sigma_ =
        1.0 / std::max(1e-12, this->noise_.accelDiscreteSigma());

    set_num_residuals(3);
    mutable_parameter_block_sizes()->push_back(6);
    mutable_parameter_block_sizes()->push_back(3);
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
    const double query_time_s = sample_.timestamp_s + parameters[2][0];
    AccelerometerTimeOffsetEvaluation evaluation;
    if (!evaluateAtTime(parameters, query_time_s, residuals, &evaluation)) {
      return false;
    }

    if (!jacobians) {
      return true;
    }

    const Mat3 d_omega_d_r =
        -leftJacobianTimesVectorDerivative(evaluation.r_w_b,
                                           evaluation.curve_dot.tail<3>());
    const Mat3 d_omega_d_r_dot = -leftJacobianSO3(evaluation.r_w_b);
    const Mat3 d_alpha_d_r =
        -leftJacobianTimesVectorDerivative(evaluation.r_w_b,
                                           evaluation.curve_ddot.tail<3>());
    const Mat3 d_alpha_d_r_ddot = -leftJacobianSO3(evaluation.r_w_b);
    const Mat3 d_h_d_r =
        rotationTransposeTimesVectorDerivative(evaluation.r_w_b,
                                               evaluation.q_w);
    const Mat3 d_lever_d_alpha =
        leverAccelerationAlphaJacobian(evaluation.r_b);
    const Mat3 d_lever_d_omega =
        leverAccelerationOmegaJacobian(evaluation.omega_b, evaluation.r_b);
    const Mat3 d_body_d_r = d_h_d_r + d_lever_d_alpha * d_alpha_d_r +
                            d_lever_d_omega * d_omega_d_r;
    const Mat3 d_body_d_r_dot = d_lever_d_omega * d_omega_d_r_dot;
    const Mat3 d_body_d_r_ddot = d_lever_d_alpha * d_alpha_d_r_ddot;

    if (jacobians[0]) {
      std::fill(jacobians[0], jacobians[0] + 18, 0.0);
      const Mat3 d_body_d_r_b =
          leverAccelerationPointJacobian(evaluation.omega_b,
                                         evaluation.alpha_b);
      const Mat3 d_residual_d_r_b =
          inv_sigma_ * evaluation.R_i_b * d_body_d_r_b;
      const Mat3 d_residual_d_r_i_b =
          inv_sigma_ * evaluation.R_i_b *
          skew(evaluation.body_specific_force) *
          leftJacobianSO3(evaluation.r_i_b);
      writeMatrixRowMajor(d_residual_d_r_b, 6, jacobians[0], 0);
      writeMatrixRowMajor(d_residual_d_r_i_b, 6, jacobians[0], 3);
    }

    if (jacobians[1]) {
      const Mat3 d_residual_d_gravity =
          -inv_sigma_ * evaluation.R_i_b * evaluation.R_b_w;
      writeMatrixRowMajor(d_residual_d_gravity, 3, jacobians[1]);
    }

    if (jacobians[2]) {
      const Vec3 omega_dot_b =
          d_omega_d_r * evaluation.curve_dot.tail<3>() +
          d_omega_d_r_dot * evaluation.curve_ddot.tail<3>();
      const Vec3 alpha_dot_b =
          d_alpha_d_r * evaluation.curve_dot.tail<3>() +
          d_alpha_d_r_ddot * evaluation.curve_dddot.tail<3>();
      const Vec3 h_dot_b =
          d_h_d_r * evaluation.curve_dot.tail<3>() +
          evaluation.R_b_w * evaluation.curve_dddot.head<3>();
      const Vec3 body_dot_b =
          h_dot_b + d_lever_d_alpha * alpha_dot_b +
          d_lever_d_omega * omega_dot_b;
      const Vec3 J_time =
          inv_sigma_ * (evaluation.R_i_b * body_dot_b +
                        evaluation.bias_dot);
      jacobians[2][0] = J_time.x();
      jacobians[2][1] = J_time.y();
      jacobians[2][2] = J_time.z();
    }

    for (int control = 0; control < pose_local_coeff_count_; ++control) {
      double *J = jacobians[3 + control];
      if (J) {
        std::fill(J, J + 18, 0.0);
      }
    }
    for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
      double *J = jacobians[3 + evaluation.pose_active_offset + i];
      if (!J) {
        continue;
      }
      const Mat3 d_body_d_translation_control =
          evaluation.pose_ddot_weights[static_cast<std::size_t>(i)] *
          evaluation.R_b_w;
      const Mat3 d_body_d_rotation_control =
          evaluation.pose_weights[static_cast<std::size_t>(i)] * d_body_d_r +
          evaluation.pose_dot_weights[static_cast<std::size_t>(i)] *
              d_body_d_r_dot +
          evaluation.pose_ddot_weights[static_cast<std::size_t>(i)] *
              d_body_d_r_ddot;
      writeMatrixRowMajor(
          inv_sigma_ * evaluation.R_i_b * d_body_d_translation_control, 6, J,
          0);
      writeMatrixRowMajor(
          inv_sigma_ * evaluation.R_i_b * d_body_d_rotation_control, 6, J, 3);
    }

    const int bias_block_offset = 3 + pose_local_coeff_count_;
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
                      AccelerometerTimeOffsetEvaluation *evaluation) const {
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

    AccelerometerTimeOffsetEvaluation local;
    local.pose_segment = pose_segment;
    local.bias_segment = bias_segment;
    local.pose_active_offset = pose_active_offset;
    local.bias_active_offset = bias_active_offset;
    local.pose_weights = pose_segment->weights(query_time_s, 0);
    local.pose_dot_weights = pose_segment->weights(query_time_s, 1);
    local.pose_ddot_weights = pose_segment->weights(query_time_s, 2);
    local.pose_dddot_weights = pose_segment->weights(query_time_s, 3);
    local.bias_weights = bias_segment->weights(query_time_s, 0);
    local.bias_dot_weights = bias_segment->weights(query_time_s, 1);

    for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
      const Eigen::Map<const Vec6> control(
          parameters[3 + pose_active_offset + i]);
      local.curve += local.pose_weights[static_cast<std::size_t>(i)] * control;
      local.curve_dot +=
          local.pose_dot_weights[static_cast<std::size_t>(i)] * control;
      local.curve_ddot +=
          local.pose_ddot_weights[static_cast<std::size_t>(i)] * control;
      local.curve_dddot +=
          local.pose_dddot_weights[static_cast<std::size_t>(i)] * control;
    }

    const int bias_block_offset = 3 + pose_local_coeff_count_;
    for (int i = 0; i < SplineSegmentMeta6::kOrder; ++i) {
      const Eigen::Map<const Vec3> control(
          parameters[bias_block_offset + bias_active_offset + i]);
      local.bias += local.bias_weights[static_cast<std::size_t>(i)] * control;
      local.bias_dot +=
          local.bias_dot_weights[static_cast<std::size_t>(i)] * control;
    }

    local.r_w_b = local.curve.tail<3>();
    local.R_b_w = rotationVectorToMatrix(local.r_w_b).transpose();
    local.q_w =
        local.curve_ddot.head<3>() - Eigen::Map<const Vec3>(parameters[1]);
    local.h_b = local.R_b_w * local.q_w;
    const Mat3 J_left = leftJacobianSO3(local.r_w_b);
    local.omega_b = -J_left * local.curve_dot.tail<3>();
    local.alpha_b = -J_left * local.curve_ddot.tail<3>();
    local.r_b = Eigen::Map<const Vec3>(parameters[0]);
    const Vec3 lever = commonLeverAcceleration(local.omega_b, local.alpha_b,
                                               local.r_b);
    local.body_specific_force = local.h_b + lever;
    local.r_i_b = Eigen::Map<const Vec3>(parameters[0] + 3);
    local.R_i_b = rotationVectorToMatrix(local.r_i_b);
    const Vec3 predicted = local.R_i_b * local.body_specific_force + local.bias;
    const Vec3 residual = inv_sigma_ * (predicted - sample_.accel_m_s2);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();
    if (evaluation) {
      *evaluation = local;
    }
    return true;
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

struct AccelKinematicEvaluation {
  Vec6 curve = Vec6::Zero();
  Vec6 curve_dot = Vec6::Zero();
  Vec6 curve_ddot = Vec6::Zero();
  std::array<double, 6> pose_weights = {};
  std::array<double, 6> pose_dot_weights = {};
  std::array<double, 6> pose_ddot_weights = {};
  std::array<double, 6> bias_weights = {};
  Vec3 r_w_b = Vec3::Zero();
  Mat3 R_b_w = Mat3::Identity();
  Vec3 q_w = Vec3::Zero();
  Vec3 h_b = Vec3::Zero();
  Vec3 omega_b = Vec3::Zero();
  Vec3 alpha_b = Vec3::Zero();
  Vec3 bias = Vec3::Zero();
};

AccelKinematicEvaluation evaluateAccelKinematics(
    const ImuSample &sample, const SplineSegmentMeta6 &pose_segment,
    const SplineSegmentMeta6 &bias_segment, const double *gravity,
    const std::array<const double *, 6> &pose_controls,
    const std::array<const double *, 6> &bias_controls) {
  AccelKinematicEvaluation evaluation;
  evaluation.pose_weights =
      pose_segment.weights(sample.timestamp_s, 0);
  evaluation.pose_dot_weights =
      pose_segment.weights(sample.timestamp_s, 1);
  evaluation.pose_ddot_weights =
      pose_segment.weights(sample.timestamp_s, 2);
  for (int i = 0; i < 6; ++i) {
    const Eigen::Map<const Vec6> control(pose_controls[i]);
    evaluation.curve +=
        evaluation.pose_weights[static_cast<std::size_t>(i)] * control;
    evaluation.curve_dot +=
        evaluation.pose_dot_weights[static_cast<std::size_t>(i)] * control;
    evaluation.curve_ddot +=
        evaluation.pose_ddot_weights[static_cast<std::size_t>(i)] * control;
  }

  evaluation.r_w_b = evaluation.curve.tail<3>();
  evaluation.R_b_w = rotationVectorToMatrix(evaluation.r_w_b).transpose();
  evaluation.q_w =
      evaluation.curve_ddot.head<3>() - Eigen::Map<const Vec3>(gravity);
  evaluation.h_b = evaluation.R_b_w * evaluation.q_w;
  const Mat3 J_left = leftJacobianSO3(evaluation.r_w_b);
  evaluation.omega_b = -J_left * evaluation.curve_dot.tail<3>();
  evaluation.alpha_b = -J_left * evaluation.curve_ddot.tail<3>();

  evaluation.bias_weights =
      bias_segment.weights(sample.timestamp_s, 0);
  for (int i = 0; i < 6; ++i) {
    evaluation.bias += evaluation.bias_weights[static_cast<std::size_t>(i)] *
                       Eigen::Map<const Vec3>(bias_controls[i]);
  }
  return evaluation;
}

class ScaleMisalignedAccelerometerCost final
    : public ceres::SizedCostFunction<3, 6, 3, 6, 6, 6, 6, 6, 6, 3, 3, 3, 3,
                                      3, 3, 6> {
public:
  ScaleMisalignedAccelerometerCost(ImuSample sample, ImuNoise noise,
                                   SplineSegmentMeta6 pose_segment,
                                   SplineSegmentMeta6 bias_segment)
      : sample(std::move(sample)), noise(std::move(noise)),
        pose_segment(std::move(pose_segment)),
        bias_segment(std::move(bias_segment)) {
    inv_sigma = 1.0 / std::max(1e-12, this->noise.accelDiscreteSigma());
  }

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    const std::array<const double *, 6> pose_controls = {
        parameters[2], parameters[3], parameters[4],
        parameters[5], parameters[6], parameters[7]};
    const std::array<const double *, 6> bias_controls = {
        parameters[8],  parameters[9],  parameters[10],
        parameters[11], parameters[12], parameters[13]};
    const AccelKinematicEvaluation kinematics =
        evaluateAccelKinematics(sample, pose_segment, bias_segment, parameters[1],
                                pose_controls, bias_controls);
    const Vec3 r_b = Eigen::Map<const Vec3>(parameters[0]);
    const Vec3 lever =
        commonLeverAcceleration(kinematics.omega_b, kinematics.alpha_b, r_b);
    const Vec3 body_specific_force = kinematics.h_b + lever;
    const Eigen::Map<const Vec3> r_i_b(parameters[0] + 3);
    const Mat3 R_i_b =
        rotationVectorToMatrix(r_i_b);
    const Mat3 M_accel = lowerTriangularMatrix(parameters[14]);
    const Vec3 predicted = predictScaleMisalignedAccelerometer(
        R_i_b, M_accel, kinematics.h_b, lever, kinematics.bias);
    const Vec3 residual = inv_sigma * (predicted - sample.accel_m_s2);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (jacobians) {
      const Mat3 d_omega_d_r = -leftJacobianTimesVectorDerivative(
          kinematics.r_w_b, kinematics.curve_dot.tail<3>());
      const Mat3 d_omega_d_r_dot = -leftJacobianSO3(kinematics.r_w_b);
      const Mat3 d_alpha_d_r = -leftJacobianTimesVectorDerivative(
          kinematics.r_w_b, kinematics.curve_ddot.tail<3>());
      const Mat3 d_alpha_d_r_ddot = -leftJacobianSO3(kinematics.r_w_b);
      const Mat3 d_h_d_r =
          rotationTransposeTimesVectorDerivative(kinematics.r_w_b,
                                                 kinematics.q_w);
      const Mat3 d_lever_d_alpha = leverAccelerationAlphaJacobian(r_b);
      const Mat3 d_lever_d_omega =
          leverAccelerationOmegaJacobian(kinematics.omega_b, r_b);
      const Mat3 d_body_d_r = d_h_d_r + d_lever_d_alpha * d_alpha_d_r +
                              d_lever_d_omega * d_omega_d_r;
      const Mat3 d_body_d_r_dot = d_lever_d_omega * d_omega_d_r_dot;
      const Mat3 d_body_d_r_ddot = d_lever_d_alpha * d_alpha_d_r_ddot;

      if (jacobians[0]) {
        std::fill(jacobians[0], jacobians[0] + 18, 0.0);
        const Mat3 d_body_d_r_b = leverAccelerationPointJacobian(
            kinematics.omega_b, kinematics.alpha_b);
        const Mat3 d_residual_d_r_b =
            inv_sigma * M_accel * R_i_b * d_body_d_r_b;
        const Mat3 d_residual_d_r_i_b =
            inv_sigma * M_accel * R_i_b * skew(body_specific_force) *
            leftJacobianSO3(r_i_b);
        writeMatrixRowMajor(d_residual_d_r_b, 6, jacobians[0], 0);
        writeMatrixRowMajor(d_residual_d_r_i_b, 6, jacobians[0], 3);
      }

      if (jacobians[1]) {
        const Mat3 d_residual_d_gravity =
            -inv_sigma * M_accel * R_i_b * kinematics.R_b_w;
        writeMatrixRowMajor(d_residual_d_gravity, 3, jacobians[1]);
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[2 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 18, 0.0);
        const Mat3 d_body_d_translation =
            kinematics.pose_ddot_weights[static_cast<std::size_t>(i)] *
            kinematics.R_b_w;
        const Mat3 d_body_d_rotation =
            kinematics.pose_weights[static_cast<std::size_t>(i)] * d_body_d_r +
            kinematics.pose_dot_weights[static_cast<std::size_t>(i)] *
                d_body_d_r_dot +
            kinematics.pose_ddot_weights[static_cast<std::size_t>(i)] *
                d_body_d_r_ddot;
        writeMatrixRowMajor(inv_sigma * M_accel * R_i_b * d_body_d_translation,
                            6, J, 0);
        writeMatrixRowMajor(inv_sigma * M_accel * R_i_b * d_body_d_rotation, 6,
                            J, 3);
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[8 + i];
        if (!J) {
          continue;
        }
        const Mat3 d_residual_d_bias =
            inv_sigma * kinematics.bias_weights[static_cast<std::size_t>(i)] *
            Mat3::Identity();
        writeMatrixRowMajor(d_residual_d_bias, 3, J);
      }

      if (jacobians[14]) {
        writeLowerTriangularProductJacobian(R_i_b * body_specific_force,
                                            inv_sigma, jacobians[14]);
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

class SizeEffectAccelerometerCost final
    : public ceres::SizedCostFunction<3, 6, 3, 6, 6, 6, 6, 6, 6, 3, 3, 3, 3,
                                      3, 3, 6, 3, 3, 3> {
public:
  SizeEffectAccelerometerCost(ImuSample sample, ImuNoise noise,
                              SplineSegmentMeta6 pose_segment,
                              SplineSegmentMeta6 bias_segment)
      : sample(std::move(sample)), noise(std::move(noise)),
        pose_segment(std::move(pose_segment)),
        bias_segment(std::move(bias_segment)) {
    inv_sigma = 1.0 / std::max(1e-12, this->noise.accelDiscreteSigma());
  }

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    const std::array<const double *, 6> pose_controls = {
        parameters[2], parameters[3], parameters[4],
        parameters[5], parameters[6], parameters[7]};
    const std::array<const double *, 6> bias_controls = {
        parameters[8],  parameters[9],  parameters[10],
        parameters[11], parameters[12], parameters[13]};
    const AccelKinematicEvaluation kinematics =
        evaluateAccelKinematics(sample, pose_segment, bias_segment, parameters[1],
                                pose_controls, bias_controls);
    const Vec3 r_b = Eigen::Map<const Vec3>(parameters[0]);
    const Eigen::Map<const Vec3> r_i_b(parameters[0] + 3);
    const Mat3 R_i_b =
        rotationVectorToMatrix(r_i_b);
    const Mat3 M_accel = lowerTriangularMatrix(parameters[14]);
    const Vec3 rx_i = Eigen::Map<const Vec3>(parameters[15]);
    const Vec3 ry_i = Eigen::Map<const Vec3>(parameters[16]);
    const Vec3 rz_i = Eigen::Map<const Vec3>(parameters[17]);
    const Vec3 predicted = predictSizeEffectAccelerometer(
        R_i_b, M_accel, kinematics.h_b, r_b, rx_i, ry_i, rz_i,
        kinematics.omega_b, kinematics.alpha_b, kinematics.bias);
    const Vec3 residual = inv_sigma * (predicted - sample.accel_m_s2);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (jacobians) {
      const Mat3 R_b_i = R_i_b.transpose();
      const std::array<Vec3, 3> axis_offsets = {rx_i, ry_i, rz_i};
      std::array<Vec3, 3> axis_points_b = {};
      std::array<Vec3, 3> axis_levers_b = {};
      std::array<Vec3, 3> axis_levers_i = {};
      for (int axis = 0; axis < 3; ++axis) {
        axis_points_b[static_cast<std::size_t>(axis)] =
            r_b + R_b_i * axis_offsets[static_cast<std::size_t>(axis)];
        axis_levers_b[static_cast<std::size_t>(axis)] =
            commonLeverAcceleration(kinematics.omega_b, kinematics.alpha_b,
                                    axis_points_b[static_cast<std::size_t>(axis)]);
        axis_levers_i[static_cast<std::size_t>(axis)] =
            R_i_b * axis_levers_b[static_cast<std::size_t>(axis)];
      }

      Vec3 axis_specific = R_i_b * kinematics.h_b;
      axis_specific.x() += axis_levers_i[0].x();
      axis_specific.y() += axis_levers_i[1].y();
      axis_specific.z() += axis_levers_i[2].z();

      const Mat3 d_omega_d_r = -leftJacobianTimesVectorDerivative(
          kinematics.r_w_b, kinematics.curve_dot.tail<3>());
      const Mat3 d_omega_d_r_dot = -leftJacobianSO3(kinematics.r_w_b);
      const Mat3 d_alpha_d_r = -leftJacobianTimesVectorDerivative(
          kinematics.r_w_b, kinematics.curve_ddot.tail<3>());
      const Mat3 d_alpha_d_r_ddot = -leftJacobianSO3(kinematics.r_w_b);
      const Mat3 d_h_d_r =
          rotationTransposeTimesVectorDerivative(kinematics.r_w_b,
                                                 kinematics.q_w);

      auto selector = [](const int axis) {
        Mat3 matrix = Mat3::Zero();
        matrix(axis, axis) = 1.0;
        return matrix;
      };

      if (jacobians[0]) {
        std::fill(jacobians[0], jacobians[0] + 18, 0.0);
        Mat3 d_axis_d_r_b = Mat3::Zero();
        Mat3 d_axis_d_r_i_b = R_i_b * skew(kinematics.h_b) *
                              leftJacobianSO3(r_i_b);
        for (int axis = 0; axis < 3; ++axis) {
          const Mat3 I_axis = selector(axis);
          const Vec3 &p_b = axis_points_b[static_cast<std::size_t>(axis)];
          const Vec3 &lever_b = axis_levers_b[static_cast<std::size_t>(axis)];
          const Vec3 &offset_i = axis_offsets[static_cast<std::size_t>(axis)];
          const Mat3 d_lever_d_point = leverAccelerationPointJacobian(
              kinematics.omega_b, kinematics.alpha_b);
          d_axis_d_r_b += I_axis * R_i_b * d_lever_d_point;
          d_axis_d_r_i_b +=
              I_axis *
              (R_i_b * skew(lever_b) * leftJacobianSO3(r_i_b) +
               R_i_b * d_lever_d_point *
                   rotationTransposeTimesVectorDerivative(r_i_b, offset_i));
          (void)p_b;
        }
        writeMatrixRowMajor(inv_sigma * M_accel * d_axis_d_r_b, 6,
                            jacobians[0], 0);
        writeMatrixRowMajor(inv_sigma * M_accel * d_axis_d_r_i_b, 6,
                            jacobians[0], 3);
      }

      if (jacobians[1]) {
        const Mat3 d_residual_d_gravity =
            -inv_sigma * M_accel * R_i_b * kinematics.R_b_w;
        writeMatrixRowMajor(d_residual_d_gravity, 3, jacobians[1]);
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[2 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 18, 0.0);
        const Mat3 d_omega_d_control_rotation =
            d_omega_d_r * kinematics.pose_weights[static_cast<std::size_t>(i)] +
            d_omega_d_r_dot *
                kinematics.pose_dot_weights[static_cast<std::size_t>(i)];
        const Mat3 d_alpha_d_control_rotation =
            d_alpha_d_r * kinematics.pose_weights[static_cast<std::size_t>(i)] +
            d_alpha_d_r_ddot *
                kinematics.pose_ddot_weights[static_cast<std::size_t>(i)];
        Mat3 d_axis_d_rotation =
            R_i_b * d_h_d_r *
            kinematics.pose_weights[static_cast<std::size_t>(i)];
        for (int axis = 0; axis < 3; ++axis) {
          const Mat3 I_axis = selector(axis);
          const Vec3 &p_b = axis_points_b[static_cast<std::size_t>(axis)];
          const Mat3 d_lever_d_omega =
              leverAccelerationOmegaJacobian(kinematics.omega_b, p_b);
          const Mat3 d_lever_d_alpha = leverAccelerationAlphaJacobian(p_b);
          d_axis_d_rotation +=
              I_axis * R_i_b *
              (d_lever_d_omega * d_omega_d_control_rotation +
               d_lever_d_alpha * d_alpha_d_control_rotation);
        }
        const Mat3 d_axis_d_translation =
            R_i_b * kinematics.R_b_w *
            kinematics.pose_ddot_weights[static_cast<std::size_t>(i)];
        writeMatrixRowMajor(inv_sigma * M_accel * d_axis_d_translation, 6, J,
                            0);
        writeMatrixRowMajor(inv_sigma * M_accel * d_axis_d_rotation, 6, J, 3);
      }

      for (int i = 0; i < 6; ++i) {
        double *J = jacobians[8 + i];
        if (!J) {
          continue;
        }
        const Mat3 d_residual_d_bias =
            inv_sigma * kinematics.bias_weights[static_cast<std::size_t>(i)] *
            Mat3::Identity();
        writeMatrixRowMajor(d_residual_d_bias, 3, J);
      }

      if (jacobians[14]) {
        writeLowerTriangularProductJacobian(axis_specific, inv_sigma,
                                            jacobians[14]);
      }

      for (int axis = 0; axis < 3; ++axis) {
        double *J = jacobians[15 + axis];
        if (!J) {
          continue;
        }
        const Mat3 I_axis = selector(axis);
        const Mat3 d_lever_d_point = leverAccelerationPointJacobian(
            kinematics.omega_b, kinematics.alpha_b);
        const Mat3 d_residual_d_offset =
            inv_sigma * M_accel * I_axis * R_i_b * d_lever_d_point * R_b_i;
        writeMatrixRowMajor(d_residual_d_offset, 3, J);
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
createAccelerometerResidual(const ImuSample &sample, const ImuNoise &noise,
                            const SplineSegmentMeta6 &pose_segment,
                            const SplineSegmentMeta6 &accel_bias_segment) {
  return new AccelerometerCost(sample, noise, pose_segment, accel_bias_segment);
}

ceres::CostFunction *createAccelerometerTimeOffsetResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const std::vector<SplineSegmentMeta6> &pose_segments,
    const int pose_local_coeff_start,
    const std::vector<SplineSegmentMeta6> &accel_bias_segments,
    const int accel_bias_local_coeff_start, const double buffer_start_s,
    const double buffer_end_s) {
  return new AccelerometerTimeOffsetCost(
      sample, noise, pose_segments, pose_local_coeff_start,
      accel_bias_segments, accel_bias_local_coeff_start, buffer_start_s,
      buffer_end_s);
}

ceres::CostFunction *createScaleMisalignedAccelerometerResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const SplineSegmentMeta6 &pose_segment,
    const SplineSegmentMeta6 &accel_bias_segment) {
  return new ScaleMisalignedAccelerometerCost(sample, noise, pose_segment,
                                              accel_bias_segment);
}

ceres::CostFunction *createSizeEffectAccelerometerResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const SplineSegmentMeta6 &pose_segment,
    const SplineSegmentMeta6 &accel_bias_segment) {
  return new SizeEffectAccelerometerCost(sample, noise, pose_segment,
                                         accel_bias_segment);
}

} // namespace ceres_cam_imu
