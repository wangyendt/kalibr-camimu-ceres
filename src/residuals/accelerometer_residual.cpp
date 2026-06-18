#include "ceres_cam_imu/residuals/accelerometer_residual.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

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
