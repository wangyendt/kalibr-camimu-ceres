#include "ceres_cam_imu/residuals/camera_reprojection_residual.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include <ceres/sized_cost_function.h>

#include "ceres_cam_imu/camera/camera_model.h"
#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/trajectory/spline_eval.h"

namespace ceres_cam_imu {
namespace {

using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat3x6 = Eigen::Matrix<double, 3, 6>;
using Mat2x6 = Eigen::Matrix<double, 2, 6>;

Eigen::Map<const Vec3> mapVec3(const double* data) {
  return Eigen::Map<const Vec3>(data);
}

Eigen::Matrix3d rightJacobianSO3Exp(const Vec3& phi) {
  const double theta2 = phi.squaredNorm();
  const Eigen::Matrix3d K = skew(phi);
  if (theta2 < 1e-12) {
    return Eigen::Matrix3d::Identity() - 0.5 * K + (1.0 / 6.0) * K * K;
  }

  const double theta = std::sqrt(theta2);
  const double a = (1.0 - std::cos(theta)) / theta2;
  const double b = (theta - std::sin(theta)) / (theta2 * theta);
  return Eigen::Matrix3d::Identity() - a * K + b * K * K;
}

Vec6 evalPose(const SplineSegmentMeta6& segment, const double timestamp_s,
              double const* const* controls, const int derivative_order) {
  const std::array<const double*, 6> active = {
      controls[0], controls[1], controls[2],
      controls[3], controls[4], controls[5]};
  return evalPoseCurve6(segment, timestamp_s, active, derivative_order);
}

class CameraReprojectionCost final
    : public ceres::SizedCostFunction<2, 6, 1, 6, 6, 6, 6, 6, 6> {
 public:
  CameraReprojectionCost(CameraIntrinsics intrinsics, CornerMeasurement corner,
                         const double observation_time_s,
                         SplineSegmentMeta6 pose_segment,
                         const double reprojection_sigma_px)
      : camera_(std::move(intrinsics)),
        corner_(std::move(corner)),
        timestamp_s_(observation_time_s),
        segment_(std::move(pose_segment)),
        inv_sigma_(1.0 / (std::max(1e-12, reprojection_sigma_px)
                          * std::sqrt(2.0))) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double* const T_c_b = parameters[0];
    const double camera_time_shift_s = parameters[1][0];
    double const* const* pose_controls = parameters + 2;

    const double query_time_s = timestamp_s_ + camera_time_shift_s;
    const Vec6 pose = evalPose(segment_, query_time_s, pose_controls, 0);
    const Vec6 pose_dot = evalPose(segment_, query_time_s, pose_controls, 1);

    const Vec3 p_w = corner_.target_point;
    const Vec3 t_w_b = pose.head<3>();
    const Vec3 r_w_b = pose.tail<3>();
    const Vec3 q_w = p_w - t_w_b;
    const Eigen::Matrix3d R_w_b = rotationVectorToMatrix(r_w_b);
    const Eigen::Matrix3d R_b_w = R_w_b.transpose();
    const Vec3 p_b = R_b_w * q_w;

    const Vec3 t_c_b = mapVec3(T_c_b);
    const Vec3 r_c_b = mapVec3(T_c_b + 3);
    const Eigen::Matrix3d R_c_b = rotationVectorToMatrix(r_c_b);
    const Vec3 p_c = R_c_b * p_b + t_c_b;

    Vec2 pixel;
    Mat23 d_pixel_d_point;
    if (!camera_.projectWithJacobian(p_c, &pixel, &d_pixel_d_point)) {
      return false;
    }

    residuals[0] = inv_sigma_ * (corner_.pixel.x() - pixel.x());
    residuals[1] = inv_sigma_ * (corner_.pixel.y() - pixel.y());

    if (!jacobians) {
      return true;
    }

    const Mat23 d_residual_d_p_c = -inv_sigma_ * d_pixel_d_point;

    const Eigen::Matrix3d d_p_c_d_r_c_b =
        R_c_b * skew(p_b) * rightJacobianSO3Exp(-r_c_b);
    Mat3x6 d_p_c_d_T_c_b = Mat3x6::Zero();
    d_p_c_d_T_c_b.block<3, 3>(0, 0).setIdentity();
    d_p_c_d_T_c_b.block<3, 3>(0, 3) = d_p_c_d_r_c_b;

    const Eigen::Matrix3d d_p_b_d_t_w_b = -R_b_w;
    const Eigen::Matrix3d d_p_b_d_r_w_b =
        -R_b_w * skew(q_w) * rightJacobianSO3Exp(r_w_b);
    Mat3x6 d_p_c_d_pose = Mat3x6::Zero();
    d_p_c_d_pose.block<3, 3>(0, 0) = R_c_b * d_p_b_d_t_w_b;
    d_p_c_d_pose.block<3, 3>(0, 3) = R_c_b * d_p_b_d_r_w_b;

    const Mat2x6 d_residual_d_pose = d_residual_d_p_c * d_p_c_d_pose;

    if (jacobians[0]) {
      const Mat2x6 J = d_residual_d_p_c * d_p_c_d_T_c_b;
      writeRowMajor(J, jacobians[0]);
    }
    if (jacobians[1]) {
      const Vec3 p_c_dot = d_p_c_d_pose * pose_dot;
      const Vec2 J = d_residual_d_p_c * p_c_dot;
      jacobians[1][0] = J.x();
      jacobians[1][1] = J.y();
    }

    const std::array<double, SplineSegmentMeta6::kOrder> weights =
        segment_.weights(query_time_s, 0);
    for (int control = 0; control < SplineSegmentMeta6::kOrder; ++control) {
      double* J_control = jacobians[2 + control];
      if (!J_control) {
        continue;
      }
      const Mat2x6 J = weights[static_cast<std::size_t>(control)]
                     * d_residual_d_pose;
      writeRowMajor(J, J_control);
    }

    return true;
  }

 private:
  template <typename Derived>
  static void writeRowMajor(const Eigen::MatrixBase<Derived>& matrix,
                            double* output) {
    for (int r = 0; r < matrix.rows(); ++r) {
      for (int c = 0; c < matrix.cols(); ++c) {
        output[r * matrix.cols() + c] = matrix(r, c);
      }
    }
  }

  CameraModel camera_;
  CornerMeasurement corner_;
  double timestamp_s_ = 0.0;
  SplineSegmentMeta6 segment_;
  double inv_sigma_ = 1.0;
};

}  // namespace

ceres::CostFunction* createCameraReprojectionResidual(
    const CameraIntrinsics& intrinsics, const CornerMeasurement& corner,
    const double observation_time_s, const SplineSegmentMeta6& pose_segment,
    const double reprojection_sigma_px) {
  return new CameraReprojectionCost(intrinsics, corner, observation_time_s,
                                    pose_segment, reprojection_sigma_px);
}

}  // namespace ceres_cam_imu
