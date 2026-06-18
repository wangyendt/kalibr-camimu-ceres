#include "ceres_cam_imu/residuals/gyroscope_residual.h"

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

ceres::CostFunction *createScaleMisalignedGyroscopeResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const SplineSegmentMeta6 &pose_segment,
    const SplineSegmentMeta6 &gyro_bias_segment) {
  return new ScaleMisalignedGyroscopeCost(sample, noise, pose_segment,
                                          gyro_bias_segment);
}

} // namespace ceres_cam_imu
