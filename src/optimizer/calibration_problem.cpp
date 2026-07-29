#include "ceres_cam_imu/optimizer/calibration_problem.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ceres/manifold.h>
#include <ceres/sized_cost_function.h>
#include <ceres/sphere_manifold.h>

#include "ceres_cam_imu/core/se3.h"
#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/so3_jacobians.h"
#include "ceres_cam_imu/initialization/pose_spline_fit.h"
#include "ceres_cam_imu/optimizer/parameter_delta_tracker.h"
#include "ceres_cam_imu/optimizer/state_snapshot.h"
#include "ceres_cam_imu/residuals/accelerometer_residual.h"
#include "ceres_cam_imu/residuals/bias_motion_prior.h"
#include "ceres_cam_imu/residuals/camera_reprojection_residual.h"
#include "ceres_cam_imu/residuals/gyroscope_residual.h"
#include "ceres_cam_imu/residuals/pose_motion_prior.h"
#include "ceres_cam_imu/residuals/time_shift_prior.h"

namespace ceres_cam_imu {
namespace {

std::pair<double, double> timeSpan(const std::vector<ImageObservation> &images,
                                   const std::vector<ImuSample> &imu_samples,
                                   const double camera_time_shift_s,
                                   const double imu_time_offset_s) {
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  for (const ImageObservation &image : images) {
    first = std::min(first, image.timestamp_s + camera_time_shift_s);
    last = std::max(last, image.timestamp_s + camera_time_shift_s);
  }
  for (const ImuSample &sample : imu_samples) {
    first = std::min(first, sample.timestamp_s + imu_time_offset_s);
    last = std::max(last, sample.timestamp_s + imu_time_offset_s);
  }
  if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
    throw std::runtime_error(
        "cannot initialize splines from empty or degenerate time span");
  }
  return {first, last};
}

template <typename Block> double *dataPtr(Block &block) { return block.data(); }

double cameraTimeShiftPriorCenter(const CalibrationOptions &options,
                                  const std::size_t camera_index) {
  if (options.camera_time_shift_priors_s.empty()) {
    return options.time_shift_prior_s;
  }
  return options.camera_time_shift_priors_s.at(camera_index);
}

template <typename Derived>
void writeRowMajor(const Eigen::MatrixBase<Derived> &matrix,
                   const int block_size, double *output,
                   const int col_offset = 0) {
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      output[row * block_size + col_offset + col] = matrix(row, col);
    }
  }
}

Mat4 cameraExtrinsicToMatrix(const CameraExtrinsicBlock &pose) {
  Vec6 p;
  for (int i = 0; i < 6; ++i) {
    p(i) = pose.values[static_cast<std::size_t>(i)];
  }
  return ceres_cam_imu::pose6ToMatrix(p);
}

class CameraChainExtrinsicPriorCost final
    : public ceres::SizedCostFunction<6, 6, 6> {
public:
  CameraChainExtrinsicPriorCost(Mat4 T_ci_c0,
                                const double translation_sigma_m,
                                const double rotation_sigma_rad)
      : T_ci_c0_(std::move(T_ci_c0)),
        T_c0_ci_(T_ci_c0_.inverse()),
        inv_translation_sigma_m_(1.0 / std::max(1e-12, translation_sigma_m)),
        inv_rotation_sigma_rad_(1.0 / std::max(1e-12, rotation_sigma_rad)) {}

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    const Eigen::Map<const Vec3> t_c0_b(parameters[0]);
    const Eigen::Map<const Vec3> r_c0_b(parameters[0] + 3);
    const Eigen::Map<const Vec3> t_ci_b(parameters[1]);
    const Eigen::Map<const Vec3> r_ci_b(parameters[1] + 3);

    const Mat3 R_c0_b = rotationVectorToMatrix(r_c0_b);
    const Mat3 R_ci_b = rotationVectorToMatrix(r_ci_b);
    const Mat3 R_c0_ci = T_c0_ci_.block<3, 3>(0, 0);
    const Vec3 t_c0_ci = T_c0_ci_.block<3, 1>(0, 3);
    const Mat3 R_b_c0 = R_c0_b.transpose();
    const Vec3 t_ci_c0_est = t_ci_b - R_ci_b * R_b_c0 * t_c0_b;
    const Mat3 R_delta = R_c0_ci * R_ci_b * R_b_c0;
    const Vec3 translation = R_c0_ci * t_ci_c0_est + t_c0_ci;
    const Vec3 rotation = rotationMatrixToVector(R_delta);

    for (int i = 0; i < 3; ++i) {
      residuals[i] = inv_translation_sigma_m_ * translation(i);
      residuals[3 + i] = inv_rotation_sigma_rad_ * rotation(i);
    }

    if (!jacobians) {
      return true;
    }

    const Mat3 J_left_rotation_inv = leftJacobianSO3(rotation).inverse();
    const Mat3 J_left_minus_rotation_inv =
        leftJacobianSO3(-rotation).inverse();
    const Mat3 J_left_r_c0_b = leftJacobianSO3(r_c0_b);
    const Mat3 J_left_r_ci_b = leftJacobianSO3(r_ci_b);

    if (jacobians[0]) {
      std::fill(jacobians[0], jacobians[0] + 36, 0.0);
      const Mat3 d_translation_d_t_c0_b = -R_c0_ci * R_ci_b * R_b_c0;
      const Mat3 d_translation_d_r_c0_b =
          -R_c0_ci * R_ci_b *
          rotationTransposeTimesVectorDerivative(r_c0_b, t_c0_b);
      const Mat3 d_rotation_d_r_c0_b =
          -J_left_minus_rotation_inv * R_c0_ci * R_ci_b * J_left_r_c0_b;
      writeRowMajor(inv_translation_sigma_m_ * d_translation_d_t_c0_b, 6,
                    jacobians[0], 0);
      writeRowMajor(inv_translation_sigma_m_ * d_translation_d_r_c0_b, 6,
                    jacobians[0], 3);
      writeRowMajor(inv_rotation_sigma_rad_ * d_rotation_d_r_c0_b, 6,
                    jacobians[0] + 18, 3);
    }

    if (jacobians[1]) {
      std::fill(jacobians[1], jacobians[1] + 36, 0.0);
      const Mat3 d_translation_d_t_ci_b = R_c0_ci;
      const Vec3 t_b_c0_in_ci = R_b_c0 * t_c0_b;
      const Mat3 d_translation_d_r_ci_b =
          -R_c0_ci * R_ci_b * skew(t_b_c0_in_ci) * J_left_r_ci_b;
      const Mat3 d_rotation_d_r_ci_b =
          J_left_rotation_inv * R_c0_b * J_left_r_ci_b;
      writeRowMajor(inv_translation_sigma_m_ * d_translation_d_t_ci_b, 6,
                    jacobians[1], 0);
      writeRowMajor(inv_translation_sigma_m_ * d_translation_d_r_ci_b, 6,
                    jacobians[1], 3);
      writeRowMajor(inv_rotation_sigma_rad_ * d_rotation_d_r_ci_b, 6,
                    jacobians[1] + 18, 3);
    }

    return true;
  }

private:
  Mat4 T_ci_c0_ = Mat4::Identity();
  Mat4 T_c0_ci_ = Mat4::Identity();
  double inv_translation_sigma_m_ = 1.0;
  double inv_rotation_sigma_rad_ = 1.0;
};

ceres::CostFunction *createCameraChainExtrinsicPrior(
    const Mat4 &T_ci_c0, const double translation_sigma_m,
    const double rotation_sigma_rad) {
  return new CameraChainExtrinsicPriorCost(T_ci_c0, translation_sigma_m,
                                           rotation_sigma_rad);
}

class KalibrMEstimatorLoss final : public ceres::LossFunction {
public:
  KalibrMEstimatorLoss(const RobustLossType type, const double width)
      : type_(type), width_(width) {}

  void Evaluate(const double s, double rho[3]) const override {
    // Kalibr's aslam_backend treats M-estimators as an iteratively reweighted
    // least-squares factor: both residuals and Jacobians are scaled by
    // sqrt(weight). It does not use the derivative of the weight function in
    // the Hessian. We therefore force rho'' to zero, and pick rho as the
    // antiderivative of the weight so that rho' == weight holds exactly.
    // The resulting cost is NOT numerically equal to Kalibr's reported cost,
    // but the per-iteration linear system is identical, and Ceres' own
    // trust-region accounting stays self-consistent.
    //
    // Forcing rho'' to zero has no numerical effect for the loss types we
    // currently use: ceres::internal::Corrector short-circuits the Triggs
    // correction whenever rho[2] <= 0, and both Cauchy and Huber satisfy
    // rho'' <= 0 everywhere (Cauchy strictly negative; Huber is affine below
    // the threshold, where rho'' is exactly zero). On Ceres 2.1 that makes
    // this class numerically identical to ceres::CauchyLoss(sqrt(width)) /
    // ceres::HuberLoss(width), which was confirmed end-to-end -- see
    // docs/knowhow/20260619_Kalibr_IRLS robust kernel alignment.
    //
    // Scope of that guarantee: tests/test_math.cpp pins the analytic half of
    // it, asserting rho0/rho1 agree with the stock losses pointwise. The other
    // half is the Corrector short-circuit, which is a Ceres implementation
    // detail rather than a documented LossFunction contract, and the build
    // does not pin a Ceres version -- so the equivalence is verified, not
    // guaranteed across upgrades.
    //
    // This class is kept because it states the IRLS intent explicitly, and
    // because it is where the weight functions Ceres does not ship
    // (Geman-McClure, Blake-Zisserman) belong; for a convex rho the choice
    // would start to matter.
    const std::array<double, 3> value =
        kalibrMEstimatorRho(type_, width_, s);
    rho[0] = value[0];
    rho[1] = value[1];
    rho[2] = value[2];
  }

private:
  RobustLossType type_ = RobustLossType::kNone;
  double width_ = 1.0;
};

std::unique_ptr<ceres::LossFunction> makeLoss(const RobustLossType type,
                                              const double width) {
  if (width <= 0.0) {
    return nullptr;
  }
  if (type == RobustLossType::kNone) {
    return nullptr;
  }
  if (type == RobustLossType::kHuber || type == RobustLossType::kCauchy) {
    return std::unique_ptr<ceres::LossFunction>(
        new KalibrMEstimatorLoss(type, width));
  }
  return nullptr;
}

class Pose6Manifold final : public ceres::Manifold {
public:
  int AmbientSize() const override { return 6; }
  int TangentSize() const override { return 6; }

  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const override {
    for (int i = 0; i < 3; ++i) {
      x_plus_delta[i] = x[i] + delta[i];
    }
    const Eigen::Map<const Vec3> r(x + 3);
    const Eigen::Map<const Vec3> dr(delta + 3);
    const Mat3 R_plus = rotationVectorToMatrix(dr) * rotationVectorToMatrix(r);
    const Vec3 r_plus = rotationMatrixToVector(R_plus);
    for (int i = 0; i < 3; ++i) {
      x_plus_delta[i + 3] = r_plus(i);
    }
    return true;
  }

  bool PlusJacobian(const double *x, double *jacobian) const override {
    std::fill(jacobian, jacobian + 36, 0.0);
    for (int i = 0; i < 3; ++i) {
      jacobian[i * 6 + i] = 1.0;
    }
    const Eigen::Map<const Vec3> r(x + 3);
    const Mat3 rotation_jacobian = leftJacobianSO3(-r).inverse();
    writeRowMajor(rotation_jacobian, 6, jacobian + 18, 3);
    return true;
  }

  bool Minus(const double *y, const double *x,
             double *y_minus_x) const override {
    for (int i = 0; i < 3; ++i) {
      y_minus_x[i] = y[i] - x[i];
    }
    const Eigen::Map<const Vec3> r_x(x + 3);
    const Eigen::Map<const Vec3> r_y(y + 3);
    const Mat3 R_delta =
        rotationVectorToMatrix(r_y) * rotationVectorToMatrix(r_x).transpose();
    const Vec3 dr = rotationMatrixToVector(R_delta);
    for (int i = 0; i < 3; ++i) {
      y_minus_x[i + 3] = dr(i);
    }
    return true;
  }

  bool MinusJacobian(const double *x, double *jacobian) const override {
    std::fill(jacobian, jacobian + 36, 0.0);
    for (int i = 0; i < 3; ++i) {
      jacobian[i * 6 + i] = 1.0;
    }
    const Eigen::Map<const Vec3> r(x + 3);
    const Mat3 rotation_jacobian = rotationVectorToMatrix(r) * leftJacobianSO3(r);
    writeRowMajor(rotation_jacobian, 6, jacobian + 18, 3);
    return true;
  }
};

void setExtrinsicManifoldIfEnabled(const CalibrationOptions &options,
                                   double *parameter_block,
                                   ceres::Problem *problem) {
  if (options.use_extrinsic_manifold) {
    problem->SetManifold(parameter_block, new Pose6Manifold());
  }
}

void setPoseControlManifoldIfEnabled(const CalibrationOptions &options,
                                     double *parameter_block,
                                     ceres::Problem *problem) {
  if (options.use_pose_control_manifold) {
    problem->SetManifold(parameter_block, new Pose6Manifold());
  }
}

void markActiveSegment(const SplineSegmentMeta6 &meta,
                       std::vector<char> *active_segments) {
  if (!active_segments) {
    return;
  }
  if (meta.coeff_start >= 0 &&
      meta.coeff_start < static_cast<int>(active_segments->size())) {
    active_segments->at(static_cast<std::size_t>(meta.coeff_start)) = 1;
  }
}

struct PoseSegmentBuffer {
  std::vector<SplineSegmentMeta6> metas;
  int local_coeff_start = 0;
  int local_coeff_end = 0;
  double buffer_start_s = 0.0;
  double buffer_end_s = 0.0;
};

PoseSegmentBuffer makePoseSegmentBuffer(const UniformBSpline &spline,
                                        const double center_time_s,
                                        const double padding_s) {
  PoseSegmentBuffer buffer;
  buffer.buffer_start_s = std::max(spline.tMin(), center_time_s - padding_s);
  buffer.buffer_end_s = std::min(spline.tMax(), center_time_s + padding_s);
  const int first_segment = spline.segmentIndex(buffer.buffer_start_s);
  const int last_segment = spline.segmentIndex(buffer.buffer_end_s);
  buffer.metas.reserve(static_cast<std::size_t>(last_segment - first_segment + 1));
  for (int segment = first_segment; segment <= last_segment; ++segment) {
    const double segment_time =
        spline.tMin() + (static_cast<double>(segment) + 0.5) * spline.dt();
    buffer.metas.push_back(spline.segmentMeta6(segment_time));
  }
  buffer.local_coeff_start = buffer.metas.front().coeff_start;
  buffer.local_coeff_end =
      buffer.metas.back().coeff_start + SplineSegmentMeta6::kOrder;
  return buffer;
}

void addCameraReprojectionResidualBlock(
    const CameraIntrinsics &intrinsics, const CornerMeasurement &corner,
    const double observation_time_s, const CalibrationOptions &options,
    CalibrationState *state, CameraExtrinsicBlock *T_c_b,
    TimeShiftBlock *time_shift, const SplineSegmentMeta6 &pose_meta,
    std::vector<char> *active_pose_segments, ceres::Problem *problem,
    CalibrationBuildSummary *summary) {
  std::unique_ptr<ceres::LossFunction> loss =
      makeLoss(options.camera_loss_type, options.camera_loss_width);

  const double camera_time_offset_buffer_s =
      options.camera_time_offset_buffer_s >= 0.0
          ? options.camera_time_offset_buffer_s
          : options.time_padding_s;
  if (!options.fix_time_shift && camera_time_offset_buffer_s > 0.0) {
    const double query_time_s = observation_time_s + time_shift->value;
    const PoseSegmentBuffer buffer =
        makePoseSegmentBuffer(state->pose_spline, query_time_s,
                              camera_time_offset_buffer_s);
    for (const SplineSegmentMeta6 &meta : buffer.metas) {
      markActiveSegment(meta, active_pose_segments);
    }
    ceres::CostFunction *cost = createCameraReprojectionTimeOffsetResidual(
        intrinsics, corner, observation_time_s, buffer.metas,
        buffer.local_coeff_start, buffer.buffer_start_s, buffer.buffer_end_s,
        options.reprojection_sigma_px);

    std::vector<double *> parameter_blocks;
    parameter_blocks.reserve(
        static_cast<std::size_t>(2 + buffer.local_coeff_end -
                                 buffer.local_coeff_start));
    parameter_blocks.push_back(dataPtr(*T_c_b));
    parameter_blocks.push_back(dataPtr(*time_shift));
    for (int coeff = buffer.local_coeff_start; coeff < buffer.local_coeff_end;
         ++coeff) {
      parameter_blocks.push_back(dataPtr(state->pose_controls.at(coeff)));
    }
    problem->AddResidualBlock(cost, loss.release(), parameter_blocks);
  } else {
    markActiveSegment(pose_meta, active_pose_segments);
    ceres::CostFunction *cost = createCameraReprojectionResidual(
        intrinsics, corner, observation_time_s, pose_meta,
        options.reprojection_sigma_px);
    problem->AddResidualBlock(
        cost, loss.release(), dataPtr(*T_c_b), dataPtr(*time_shift),
        dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
        dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
        dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
        dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
        dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
        dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)));
  }
  ++summary->camera_residuals;
}

bool usesLocalPoseMotionScaling(const CalibrationOptions &options) {
  return options.add_pose_motion_local_scaling &&
         options.pose_motion_local_half_window_s > 0.0;
}

bool overlapsLocalPoseMotionWindow(const SplineSegmentMeta6 &meta,
                                   const CalibrationOptions &options) {
  if (!usesLocalPoseMotionScaling(options)) {
    return false;
  }
  const double window_begin = options.pose_motion_local_center_s -
                              options.pose_motion_local_half_window_s;
  const double window_end = options.pose_motion_local_center_s +
                            options.pose_motion_local_half_window_s;
  const double segment_end = meta.segment_start_s + meta.dt_s;
  return meta.segment_start_s <= window_end && segment_end >= window_begin;
}

double rotationDeltaDeg(const Mat4 &lhs, const Mat4 &rhs) {
  const Mat3 dR = lhs.block<3, 3>(0, 0) * rhs.block<3, 3>(0, 0).transpose();
  const double cos_angle = std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0);
  constexpr double kPi = 3.14159265358979323846;
  return std::acos(cos_angle) * 180.0 / kPi;
}

bool usesScaleMisalignment(const ImuCalibrationModel model) {
  return model == ImuCalibrationModel::kScaleMisalignment ||
         model == ImuCalibrationModel::kScaleMisalignmentSizeEffect;
}

bool usesSizeEffect(const ImuCalibrationModel model) {
  return model == ImuCalibrationModel::kScaleMisalignmentSizeEffect;
}

void addImuIntrinsicParameterBlocks(const CalibrationOptions &options,
                                    ImuIntrinsicBlocks *intrinsics,
                                    ceres::Problem *problem) {
  if (!usesScaleMisalignment(options.imu_model)) {
    return;
  }
  problem->AddParameterBlock(dataPtr(intrinsics->accel_M),
                             LowerTriangularMatrixBlock::kSize);
  problem->AddParameterBlock(dataPtr(intrinsics->gyro_M),
                             LowerTriangularMatrixBlock::kSize);
  problem->AddParameterBlock(
      dataPtr(intrinsics->gyro_accel_sensitivity),
      Matrix3Block::kSize);
  problem->AddParameterBlock(
      dataPtr(intrinsics->gyro_sensing_rotation),
      Vector3Block::kSize);
  if (options.fix_imu_intrinsics) {
    problem->SetParameterBlockConstant(dataPtr(intrinsics->accel_M));
    problem->SetParameterBlockConstant(dataPtr(intrinsics->gyro_M));
    problem->SetParameterBlockConstant(
        dataPtr(intrinsics->gyro_accel_sensitivity));
    problem->SetParameterBlockConstant(
        dataPtr(intrinsics->gyro_sensing_rotation));
  }
  if (!usesSizeEffect(options.imu_model)) {
    return;
  }
  problem->AddParameterBlock(dataPtr(intrinsics->accel_axis_rx_i),
                             Vector3Block::kSize);
  problem->AddParameterBlock(dataPtr(intrinsics->accel_axis_ry_i),
                             Vector3Block::kSize);
  problem->AddParameterBlock(dataPtr(intrinsics->accel_axis_rz_i),
                             Vector3Block::kSize);
  if (options.fix_imu_intrinsics || options.fix_accel_size_effect_rx) {
    problem->SetParameterBlockConstant(
        dataPtr(intrinsics->accel_axis_rx_i));
  }
  if (options.fix_imu_intrinsics) {
    problem->SetParameterBlockConstant(
        dataPtr(intrinsics->accel_axis_ry_i));
    problem->SetParameterBlockConstant(
        dataPtr(intrinsics->accel_axis_rz_i));
  }
}

void ensureMultiImuStateSize(CalibrationState *state,
                             const std::size_t imu_count) {
  if (!state) {
    return;
  }
  if (imu_count == 0) {
    return;
  }
  if (state->imu_extrinsics.size() < imu_count) {
    const std::size_t old_size = state->imu_extrinsics.size();
    state->imu_extrinsics.resize(imu_count);
    if (old_size == 0) {
      state->imu_extrinsics[0] = state->imu_extrinsic;
    }
  }
  if (state->imu_intrinsics_by_imu.size() < imu_count) {
    const std::size_t old_size = state->imu_intrinsics_by_imu.size();
    state->imu_intrinsics_by_imu.resize(imu_count);
    if (old_size == 0) {
      state->imu_intrinsics_by_imu[0] = state->imu_intrinsics;
    }
  }
  if (state->gyro_bias_controls_by_imu.size() < imu_count) {
    const std::size_t old_size = state->gyro_bias_controls_by_imu.size();
    state->gyro_bias_controls_by_imu.resize(imu_count);
    for (std::size_t i = old_size; i < imu_count; ++i) {
      state->gyro_bias_controls_by_imu[i] = state->gyro_bias_controls;
    }
    if (old_size == 0) {
      state->gyro_bias_controls_by_imu[0] = state->gyro_bias_controls;
    }
  }
  if (state->accel_bias_controls_by_imu.size() < imu_count) {
    const std::size_t old_size = state->accel_bias_controls_by_imu.size();
    state->accel_bias_controls_by_imu.resize(imu_count);
    for (std::size_t i = old_size; i < imu_count; ++i) {
      state->accel_bias_controls_by_imu[i] = state->accel_bias_controls;
    }
    if (old_size == 0) {
      state->accel_bias_controls_by_imu[0] = state->accel_bias_controls;
    }
  }
  if (state->imu_time_offsets_s.size() < imu_count) {
    state->imu_time_offsets_s.resize(imu_count, 0.0);
  }
}

ImuExtrinsicBlock &imuExtrinsicFor(CalibrationState *state,
                                   const std::size_t imu_index) {
  if (imu_index == 0) {
    return state->imu_extrinsic;
  }
  return state->imu_extrinsics.at(imu_index);
}

ImuIntrinsicBlocks &imuIntrinsicsFor(CalibrationState *state,
                                     const std::size_t imu_index) {
  if (imu_index == 0) {
    return state->imu_intrinsics;
  }
  return state->imu_intrinsics_by_imu.at(imu_index);
}

std::vector<BiasControlBlock> &gyroBiasControlsFor(
    CalibrationState *state, const std::size_t imu_index) {
  if (imu_index == 0) {
    return state->gyro_bias_controls;
  }
  return state->gyro_bias_controls_by_imu.at(imu_index);
}

std::vector<BiasControlBlock> &accelBiasControlsFor(
    CalibrationState *state, const std::size_t imu_index) {
  if (imu_index == 0) {
    return state->accel_bias_controls;
  }
  return state->accel_bias_controls_by_imu.at(imu_index);
}

double imuTimeOffsetFor(const CalibrationState &state,
                        const std::size_t imu_index) {
  if (imu_index < state.imu_time_offsets_s.size()) {
    return state.imu_time_offsets_s[imu_index];
  }
  return 0.0;
}

ImuSample shiftedImuSample(const ImuSample &sample,
                           const double time_offset_s) {
  ImuSample shifted = sample;
  shifted.timestamp_s += time_offset_s;
  return shifted;
}

bool shouldOptimizeImuTimeOffsetForIndex(const CalibrationOptions &options,
                                         const std::size_t imu_index) {
  return options.optimize_imu_time_offsets && imu_index > 0 &&
         options.imu_model == ImuCalibrationModel::kCalibrated;
}

bool isValidTimeWindow(const UniformBSpline &spline, const double center_time_s,
                       const double half_window_s) {
  return spline.isValidTime(center_time_s - half_window_s) &&
         spline.isValidTime(center_time_s + half_window_s);
}

void addImuParameterBlocksForIndex(const std::size_t imu_index,
                                   const CalibrationOptions &options,
                                   CalibrationState *state,
                                   ceres::Problem *problem) {
  ImuExtrinsicBlock &imu_extrinsic = imuExtrinsicFor(state, imu_index);
  problem->AddParameterBlock(dataPtr(imu_extrinsic), 6);
  setExtrinsicManifoldIfEnabled(options, dataPtr(imu_extrinsic), problem);
  if (imu_index == 0 && options.fix_reference_imu_extrinsic) {
    problem->SetParameterBlockConstant(dataPtr(imu_extrinsic));
  } else if (imu_index > 0 && options.fix_imu_extrinsics) {
    problem->SetParameterBlockConstant(dataPtr(imu_extrinsic));
  } else if (imu_index > 0 &&
             (options.imu_extrinsic_translation_bound_m > 0.0 ||
              options.imu_extrinsic_rotation_bound_rad > 0.0)) {
    double *extrinsic = dataPtr(imu_extrinsic);
    if (options.imu_extrinsic_translation_bound_m > 0.0) {
      for (int i = 0; i < 3; ++i) {
        const double center = extrinsic[i];
        problem->SetParameterLowerBound(
            extrinsic, i, center - options.imu_extrinsic_translation_bound_m);
        problem->SetParameterUpperBound(
            extrinsic, i, center + options.imu_extrinsic_translation_bound_m);
      }
    }
    if (options.imu_extrinsic_rotation_bound_rad > 0.0) {
      for (int i = 3; i < 6; ++i) {
        const double center = extrinsic[i];
        problem->SetParameterLowerBound(
            extrinsic, i, center - options.imu_extrinsic_rotation_bound_rad);
        problem->SetParameterUpperBound(
            extrinsic, i, center + options.imu_extrinsic_rotation_bound_rad);
      }
    }
  }

  ImuIntrinsicBlocks &imu_intrinsics = imuIntrinsicsFor(state, imu_index);
  addImuIntrinsicParameterBlocks(options, &imu_intrinsics, problem);

  for (BiasControlBlock &control : gyroBiasControlsFor(state, imu_index)) {
    problem->AddParameterBlock(dataPtr(control), 3);
    if (options.fix_bias_controls) {
      problem->SetParameterBlockConstant(dataPtr(control));
    }
  }
  for (BiasControlBlock &control : accelBiasControlsFor(state, imu_index)) {
    problem->AddParameterBlock(dataPtr(control), 3);
    if (options.fix_bias_controls) {
      problem->SetParameterBlockConstant(dataPtr(control));
    }
  }

  if (shouldOptimizeImuTimeOffsetForIndex(options, imu_index)) {
    if (options.imu_time_offset_bound_s <= 0.0) {
      throw std::invalid_argument(
          "IMU time offset optimization requires a positive bound");
    }
    double *time_offset = &state->imu_time_offsets_s.at(imu_index);
    const double initial_offset = *time_offset;
    problem->AddParameterBlock(time_offset, 1);
    problem->SetParameterLowerBound(
        time_offset, 0, initial_offset - options.imu_time_offset_bound_s);
    problem->SetParameterUpperBound(
        time_offset, 0, initial_offset + options.imu_time_offset_bound_s);
  }
}

void addImuResidualBlocksForIndex(
    const std::size_t imu_index, const ImuObservationDataset &imu,
    const CalibrationOptions &options, CalibrationState *state,
    ceres::Problem *problem, std::vector<char> *active_pose_segments,
    CalibrationBuildSummary *summary) {
  ImuExtrinsicBlock &imu_extrinsic = imuExtrinsicFor(state, imu_index);
  ImuIntrinsicBlocks &imu_intrinsics = imuIntrinsicsFor(state, imu_index);
  std::vector<BiasControlBlock> &gyro_bias_controls =
      gyroBiasControlsFor(state, imu_index);
  std::vector<BiasControlBlock> &accel_bias_controls =
      accelBiasControlsFor(state, imu_index);
  const double imu_time_offset_s = imuTimeOffsetFor(*state, imu_index);
  const bool optimize_imu_time_offset =
      shouldOptimizeImuTimeOffsetForIndex(options, imu_index);

  int added_imu = 0;
  const int stride = std::max(1, options.imu_stride);
  for (std::size_t i = 0; i < imu.samples.size();
       i += static_cast<std::size_t>(stride)) {
    if (options.max_imu_residuals > 0 &&
        added_imu >= options.max_imu_residuals) {
      break;
    }
    const ImuSample sample = shiftedImuSample(imu.samples[i],
                                              imu_time_offset_s);
    if (optimize_imu_time_offset) {
      const double offset_bound_s = options.imu_time_offset_bound_s;
      const double query_time_s = sample.timestamp_s;
      if (!isValidTimeWindow(state->pose_spline, query_time_s,
                             offset_bound_s) ||
          !isValidTimeWindow(state->gyro_bias_spline, query_time_s,
                             offset_bound_s) ||
          !isValidTimeWindow(state->accel_bias_spline, query_time_s,
                             offset_bound_s)) {
        ++summary->skipped_imu_samples;
        continue;
      }

      const PoseSegmentBuffer pose_buffer =
          makePoseSegmentBuffer(state->pose_spline, query_time_s,
                                offset_bound_s);
      const PoseSegmentBuffer gyro_bias_buffer =
          makePoseSegmentBuffer(state->gyro_bias_spline, query_time_s,
                                offset_bound_s);
      const PoseSegmentBuffer accel_bias_buffer =
          makePoseSegmentBuffer(state->accel_bias_spline, query_time_s,
                                offset_bound_s);
      for (const SplineSegmentMeta6 &meta : pose_buffer.metas) {
        markActiveSegment(meta, active_pose_segments);
      }

      std::unique_ptr<ceres::LossFunction> gyro_loss =
          makeLoss(options.gyro_loss_type, options.gyro_loss_width);
      ceres::CostFunction *gyro_cost = createGyroscopeTimeOffsetResidual(
          imu.samples[i], imu.noise, pose_buffer.metas,
          pose_buffer.local_coeff_start, gyro_bias_buffer.metas,
          gyro_bias_buffer.local_coeff_start,
          std::max(pose_buffer.buffer_start_s,
                   gyro_bias_buffer.buffer_start_s),
          std::min(pose_buffer.buffer_end_s, gyro_bias_buffer.buffer_end_s));
      std::vector<double *> gyro_parameter_blocks;
      gyro_parameter_blocks.reserve(
          static_cast<std::size_t>(2 + pose_buffer.local_coeff_end -
                                   pose_buffer.local_coeff_start +
                                   gyro_bias_buffer.local_coeff_end -
                                   gyro_bias_buffer.local_coeff_start));
      gyro_parameter_blocks.push_back(dataPtr(imu_extrinsic));
      gyro_parameter_blocks.push_back(&state->imu_time_offsets_s.at(imu_index));
      for (int coeff = pose_buffer.local_coeff_start;
           coeff < pose_buffer.local_coeff_end; ++coeff) {
        gyro_parameter_blocks.push_back(dataPtr(state->pose_controls.at(coeff)));
      }
      for (int coeff = gyro_bias_buffer.local_coeff_start;
           coeff < gyro_bias_buffer.local_coeff_end; ++coeff) {
        gyro_parameter_blocks.push_back(dataPtr(gyro_bias_controls.at(coeff)));
      }
      problem->AddResidualBlock(gyro_cost, gyro_loss.release(),
                                gyro_parameter_blocks);
      ++summary->gyro_residuals;

      std::unique_ptr<ceres::LossFunction> accel_loss =
          makeLoss(options.accel_loss_type, options.accel_loss_width);
      ceres::CostFunction *accel_cost = createAccelerometerTimeOffsetResidual(
          imu.samples[i], imu.noise, pose_buffer.metas,
          pose_buffer.local_coeff_start, accel_bias_buffer.metas,
          accel_bias_buffer.local_coeff_start,
          std::max(pose_buffer.buffer_start_s,
                   accel_bias_buffer.buffer_start_s),
          std::min(pose_buffer.buffer_end_s,
                   accel_bias_buffer.buffer_end_s));
      std::vector<double *> accel_parameter_blocks;
      accel_parameter_blocks.reserve(
          static_cast<std::size_t>(3 + pose_buffer.local_coeff_end -
                                   pose_buffer.local_coeff_start +
                                   accel_bias_buffer.local_coeff_end -
                                   accel_bias_buffer.local_coeff_start));
      accel_parameter_blocks.push_back(dataPtr(imu_extrinsic));
      accel_parameter_blocks.push_back(dataPtr(state->gravity));
      accel_parameter_blocks.push_back(&state->imu_time_offsets_s.at(imu_index));
      for (int coeff = pose_buffer.local_coeff_start;
           coeff < pose_buffer.local_coeff_end; ++coeff) {
        accel_parameter_blocks.push_back(
            dataPtr(state->pose_controls.at(coeff)));
      }
      for (int coeff = accel_bias_buffer.local_coeff_start;
           coeff < accel_bias_buffer.local_coeff_end; ++coeff) {
        accel_parameter_blocks.push_back(dataPtr(accel_bias_controls.at(coeff)));
      }
      problem->AddResidualBlock(accel_cost, accel_loss.release(),
                                accel_parameter_blocks);
      ++summary->accel_residuals;
      ++added_imu;
      continue;
    }
    if (!state->pose_spline.isValidTime(sample.timestamp_s) ||
        !state->gyro_bias_spline.isValidTime(sample.timestamp_s) ||
        !state->accel_bias_spline.isValidTime(sample.timestamp_s)) {
      ++summary->skipped_imu_samples;
      continue;
    }
    const SplineSegmentMeta6 pose_meta =
        state->pose_spline.segmentMeta6(sample.timestamp_s);
    const SplineSegmentMeta6 gyro_bias_meta =
        state->gyro_bias_spline.segmentMeta6(sample.timestamp_s);
    const SplineSegmentMeta6 accel_bias_meta =
        state->accel_bias_spline.segmentMeta6(sample.timestamp_s);
    markActiveSegment(pose_meta, active_pose_segments);

    std::unique_ptr<ceres::LossFunction> gyro_loss =
        makeLoss(options.gyro_loss_type, options.gyro_loss_width);
    if (usesScaleMisalignment(options.imu_model)) {
      problem->AddResidualBlock(
          createScaleMisalignedGyroscopeResidual(sample, imu.noise, pose_meta,
                                                 gyro_bias_meta),
          gyro_loss.release(), dataPtr(imu_extrinsic),
          dataPtr(state->gravity),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 0)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 1)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 2)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 3)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 4)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 5)),
          dataPtr(imu_intrinsics.gyro_sensing_rotation),
          dataPtr(imu_intrinsics.gyro_M),
          dataPtr(imu_intrinsics.gyro_accel_sensitivity));
    } else {
      problem->AddResidualBlock(
          createGyroscopeResidual(sample, imu.noise, pose_meta,
                                  gyro_bias_meta),
          gyro_loss.release(), dataPtr(imu_extrinsic),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 0)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 1)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 2)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 3)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 4)),
          dataPtr(gyro_bias_controls.at(gyro_bias_meta.coeff_start + 5)));
    }
    ++summary->gyro_residuals;

    std::unique_ptr<ceres::LossFunction> accel_loss =
        makeLoss(options.accel_loss_type, options.accel_loss_width);
    if (usesSizeEffect(options.imu_model)) {
      problem->AddResidualBlock(
          createSizeEffectAccelerometerResidual(sample, imu.noise, pose_meta,
                                                accel_bias_meta),
          accel_loss.release(), dataPtr(imu_extrinsic),
          dataPtr(state->gravity),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 0)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 1)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 2)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 3)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 4)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 5)),
          dataPtr(imu_intrinsics.accel_M),
          dataPtr(imu_intrinsics.accel_axis_rx_i),
          dataPtr(imu_intrinsics.accel_axis_ry_i),
          dataPtr(imu_intrinsics.accel_axis_rz_i));
    } else if (usesScaleMisalignment(options.imu_model)) {
      problem->AddResidualBlock(
          createScaleMisalignedAccelerometerResidual(sample, imu.noise,
                                                     pose_meta,
                                                     accel_bias_meta),
          accel_loss.release(), dataPtr(imu_extrinsic),
          dataPtr(state->gravity),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 0)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 1)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 2)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 3)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 4)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 5)),
          dataPtr(imu_intrinsics.accel_M));
    } else {
      problem->AddResidualBlock(
          createAccelerometerResidual(sample, imu.noise, pose_meta,
                                      accel_bias_meta),
          accel_loss.release(), dataPtr(imu_extrinsic),
          dataPtr(state->gravity),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 0)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 1)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 2)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 3)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 4)),
          dataPtr(accel_bias_controls.at(accel_bias_meta.coeff_start + 5)));
    }
    ++summary->accel_residuals;
    ++added_imu;
  }
}

void addBiasMotionPriorsForIndex(const std::size_t imu_index,
                                 const ImuNoise &imu_noise,
                                 CalibrationState *state,
                                 ceres::Problem *problem,
                                 CalibrationBuildSummary *summary) {
  std::vector<BiasControlBlock> &gyro_bias_controls =
      gyroBiasControlsFor(state, imu_index);
  std::vector<BiasControlBlock> &accel_bias_controls =
      accelBiasControlsFor(state, imu_index);

  for (int segment = 0; segment < state->gyro_bias_spline.numSegments();
       ++segment) {
    const SplineSegmentMeta6 meta = state->gyro_bias_spline.segmentMeta6(
        state->gyro_bias_spline.tMin() +
        static_cast<double>(segment) * state->gyro_bias_spline.dt());
    problem->AddResidualBlock(
        createBiasMotionPrior(meta, imu_noise.gyroscope_random_walk), nullptr,
        dataPtr(gyro_bias_controls.at(meta.coeff_start + 0)),
        dataPtr(gyro_bias_controls.at(meta.coeff_start + 1)),
        dataPtr(gyro_bias_controls.at(meta.coeff_start + 2)),
        dataPtr(gyro_bias_controls.at(meta.coeff_start + 3)),
        dataPtr(gyro_bias_controls.at(meta.coeff_start + 4)),
        dataPtr(gyro_bias_controls.at(meta.coeff_start + 5)));
    ++summary->gyro_bias_priors;
  }

  for (int segment = 0; segment < state->accel_bias_spline.numSegments();
       ++segment) {
    const SplineSegmentMeta6 meta = state->accel_bias_spline.segmentMeta6(
        state->accel_bias_spline.tMin() +
        static_cast<double>(segment) * state->accel_bias_spline.dt());
    problem->AddResidualBlock(
        createBiasMotionPrior(meta, imu_noise.accelerometer_random_walk),
        nullptr, dataPtr(accel_bias_controls.at(meta.coeff_start + 0)),
        dataPtr(accel_bias_controls.at(meta.coeff_start + 1)),
        dataPtr(accel_bias_controls.at(meta.coeff_start + 2)),
        dataPtr(accel_bias_controls.at(meta.coeff_start + 3)),
        dataPtr(accel_bias_controls.at(meta.coeff_start + 4)),
        dataPtr(accel_bias_controls.at(meta.coeff_start + 5)));
    ++summary->accel_bias_priors;
  }
}

class StateTraceCallback final : public ceres::IterationCallback {
public:
  StateTraceCallback(const CalibrationOptions &options,
                     const CalibrationState *state,
                     const ceres::Problem *problem)
      : options_(options), state_(state) {
    if (problem) {
      parameter_delta_tracker_.reset(*problem);
    }
  }

  ceres::CallbackReturnType
  operator()(const ceres::IterationSummary &summary) override {
    if (!state_) {
      return ceres::SOLVER_CONTINUE;
    }
    const std::streamsize old_precision = std::cout.precision();
    const Mat4 T_c_b = pose6ToMatrix(state_->T_c_b);
    const Vec3 translation = T_c_b.block<3, 1>(0, 3);
    const Vec3 gravity(state_->gravity.values[0], state_->gravity.values[1],
                       state_->gravity.values[2]);
    double parameter_delta = -1.0;
    if (summary.step_is_successful) {
      parameter_delta = parameter_delta_tracker_.updateAndReturnMaxDelta();
    }
    std::cout << std::setprecision(17) << "iteration_state";
    if (!options_.trace_label.empty()) {
      std::cout << " label=" << options_.trace_label;
    }
    std::cout << " iter=" << summary.iteration
              << " step_success=" << (summary.step_is_successful ? 1 : 0)
              << " cost=" << summary.cost
              << " cost_change=" << summary.cost_change
              << " step_norm=" << summary.step_norm
              << " parameter_delta=" << parameter_delta
              << " tr_radius=" << summary.trust_region_radius
              << " translation_m=" << translation.x() << " " << translation.y()
              << " " << translation.z()
              << " time_shift_s=" << state_->camera_time_shift_s.value
              << " gravity_m_s2=" << gravity.x() << " " << gravity.y() << " "
              << gravity.z();
    if (options_.trace_has_reference_state) {
      std::cout << " reference_rotation_deg="
                << rotationDeltaDeg(T_c_b, options_.trace_reference_T_c_b)
                << " reference_translation_m="
                << (translation -
                    options_.trace_reference_T_c_b.block<3, 1>(0, 3))
                       .norm()
                << " reference_time_shift_s="
                << (state_->camera_time_shift_s.value -
                    options_.trace_reference_time_shift_s)
                << " reference_gravity_norm="
                << (gravity - options_.trace_reference_gravity).norm();
    }
    std::cout << "\n";
    std::cout.precision(old_precision);
    return ceres::SOLVER_CONTINUE;
  }

private:
  const CalibrationOptions &options_;
  const CalibrationState *state_ = nullptr;
  ParameterDeltaTracker parameter_delta_tracker_;
};

class AbsoluteStopCallback final : public ceres::IterationCallback {
public:
  AbsoluteStopCallback(const CalibrationOptions &options,
                       const ceres::Problem *problem)
      : cost_tolerance_(options.solver_absolute_cost_change_tolerance),
        step_tolerance_(options.solver_absolute_step_tolerance),
        parameter_tolerance_(options.solver_absolute_parameter_tolerance),
        label_(options.trace_label) {
    if (problem && parameter_tolerance_ >= 0.0) {
      parameter_delta_tracker_.reset(*problem);
    }
  }

  bool enabled() const {
    return cost_tolerance_ >= 0.0 || step_tolerance_ >= 0.0 ||
           parameter_tolerance_ >= 0.0;
  }

  ceres::CallbackReturnType
  operator()(const ceres::IterationSummary &summary) override {
    if (!summary.step_is_successful) {
      return ceres::SOLVER_CONTINUE;
    }
    const double parameter_delta =
        parameter_tolerance_ >= 0.0
            ? parameter_delta_tracker_.updateAndReturnMaxDelta()
            : -1.0;
    if (summary.iteration == 0) {
      return ceres::SOLVER_CONTINUE;
    }
    const bool cost_trigger = cost_tolerance_ >= 0.0 &&
                              std::abs(summary.cost_change) <= cost_tolerance_;
    const bool step_trigger =
        step_tolerance_ >= 0.0 && summary.step_norm <= step_tolerance_;
    const bool parameter_trigger =
        parameter_tolerance_ >= 0.0 && parameter_delta <= parameter_tolerance_;
    if (!cost_trigger && !step_trigger && !parameter_trigger) {
      return ceres::SOLVER_CONTINUE;
    }
    const std::streamsize old_precision = std::cout.precision();
    std::cout << std::setprecision(17) << "absolute_stop";
    if (!label_.empty()) {
      std::cout << " label=" << label_;
    }
    std::cout << " iter=" << summary.iteration
              << " cost_change=" << summary.cost_change
              << " step_norm=" << summary.step_norm
              << " parameter_delta=" << parameter_delta
              << " cost_tolerance=" << cost_tolerance_
              << " step_tolerance=" << step_tolerance_
              << " parameter_tolerance=" << parameter_tolerance_
              << " cost_trigger=" << (cost_trigger ? 1 : 0)
              << " step_trigger=" << (step_trigger ? 1 : 0)
              << " parameter_trigger=" << (parameter_trigger ? 1 : 0) << "\n";
    std::cout.precision(old_precision);
    return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
  }

private:
  double cost_tolerance_ = -1.0;
  double step_tolerance_ = -1.0;
  double parameter_tolerance_ = -1.0;
  std::string label_;
  ParameterDeltaTracker parameter_delta_tracker_;
};

class BestStateCallback final : public ceres::IterationCallback {
public:
  BestStateCallback(const CalibrationOptions &options,
                    const CalibrationState *state)
      : enabled_(options.solver_restore_best_state && state != nullptr),
        state_(state), label_(options.trace_label) {
    if (enabled_) {
      best_snapshot_ = snapshotCalibrationState(*state_);
    }
  }

  bool enabled() const { return enabled_; }

  ceres::CallbackReturnType
  operator()(const ceres::IterationSummary &summary) override {
    if (!enabled_ || !state_) {
      return ceres::SOLVER_CONTINUE;
    }
    if (summary.iteration != 0 && !summary.step_is_successful) {
      return ceres::SOLVER_CONTINUE;
    }
    if (!std::isfinite(summary.cost)) {
      return ceres::SOLVER_CONTINUE;
    }
    if (!has_best_ || summary.cost < best_cost_) {
      best_cost_ = summary.cost;
      best_iteration_ = summary.iteration;
      best_snapshot_ = snapshotCalibrationState(*state_);
      has_best_ = true;
    }
    return ceres::SOLVER_CONTINUE;
  }

  bool restoreIfBetter(CalibrationState *state,
                       ceres::Solver::Summary *summary) const {
    if (!enabled_ || !state || !summary || !has_best_) {
      return false;
    }
    if (!std::isfinite(summary->final_cost)) {
      return false;
    }
    const double tolerance =
        1e-12 * std::max(1.0, std::abs(summary->final_cost));
    if (best_cost_ >= summary->final_cost - tolerance) {
      return false;
    }
    restoreCalibrationState(best_snapshot_, state);
    const std::streamsize old_precision = std::cout.precision();
    std::cout << std::setprecision(17) << "best_state_restore";
    if (!label_.empty()) {
      std::cout << " label=" << label_;
    }
    std::cout << " iter=" << best_iteration_ << " best_cost=" << best_cost_
              << " final_cost_before_restore=" << summary->final_cost << "\n";
    std::cout.precision(old_precision);
    summary->final_cost = best_cost_;
    return true;
  }

private:
  bool enabled_ = false;
  const CalibrationState *state_ = nullptr;
  std::string label_;
  bool has_best_ = false;
  int best_iteration_ = -1;
  double best_cost_ = std::numeric_limits<double>::infinity();
  CalibrationStateSnapshot best_snapshot_;
};

void fillProblemSizeSummary(const ceres::Problem &problem,
                            CalibrationBuildSummary *summary) {
  if (!summary) {
    return;
  }
  summary->residual_blocks = problem.NumResidualBlocks();
  summary->scalar_residuals = problem.NumResiduals();
  summary->parameter_blocks = problem.NumParameterBlocks();
  summary->ambient_parameters = problem.NumParameters();
  summary->active_parameter_blocks = 0;
  summary->tangent_parameters = 0;

  std::vector<double *> parameter_blocks;
  problem.GetParameterBlocks(&parameter_blocks);
  for (double *parameter_block : parameter_blocks) {
    const int tangent_size =
        problem.IsParameterBlockConstant(parameter_block)
            ? 0
            : problem.ParameterBlockTangentSize(parameter_block);
    summary->tangent_parameters += tangent_size;
    if (tangent_size > 0) {
      ++summary->active_parameter_blocks;
    }
  }
}

} // namespace

double kalibrMEstimatorWeight(const RobustLossType type, const double width,
                              const double squared_residual_norm) {
  if (width <= 0.0 || type == RobustLossType::kNone) {
    return 1.0;
  }
  if (type == RobustLossType::kCauchy) {
    return 1.0 / (1.0 + squared_residual_norm / width);
  }
  if (type == RobustLossType::kHuber) {
    const double threshold_s = width * width;
    return squared_residual_norm < threshold_s
               ? 1.0
               : width / std::sqrt(std::max(squared_residual_norm, 1e-300));
  }
  return 1.0;
}

std::array<double, 3> kalibrMEstimatorRho(
    const RobustLossType type, const double width,
    const double squared_residual_norm) {
  const double s = squared_residual_norm;
  const double weight = kalibrMEstimatorWeight(type, width, s);
  // rho is the antiderivative of the weight, rho(s) = \int_0^s w(u) du, so
  // that rho'(s) == w(s) exactly. rho'' is forced to zero to reproduce
  // Kalibr's IRLS linearization (no Triggs correction).
  double rho0 = s;
  if (width > 0.0 && type == RobustLossType::kCauchy) {
    // w(u) = 1 / (1 + u / c2)  ->  rho(s) = c2 * log(1 + s / c2)
    rho0 = width * std::log1p(s / width);
  } else if (width > 0.0 && type == RobustLossType::kHuber) {
    // w(u) = min(1, k / sqrt(u))  ->  rho(s) = s        (s <= k^2)
    //                                       = 2k sqrt(s) - k^2  (s > k^2)
    const double threshold_s = width * width;
    rho0 = s < threshold_s ? s : 2.0 * width * std::sqrt(s) - threshold_s;
  }
  return {rho0, weight, 0.0};
}

namespace {

double initialCameraTimeShiftFor(const CalibrationOptions &options,
                                 const std::size_t camera_index,
                                 const std::size_t camera_count) {
  if (options.initial_camera_time_shifts_s.empty()) {
    if (!std::isfinite(options.initial_camera_time_shift_s)) {
      throw std::invalid_argument("initial camera time shift must be finite");
    }
    return options.initial_camera_time_shift_s;
  }
  if (options.initial_camera_time_shifts_s.size() != camera_count) {
    throw std::invalid_argument(
        "initial camera time shifts must match camera count");
  }
  const double shift_s = options.initial_camera_time_shifts_s.at(camera_index);
  if (!std::isfinite(shift_s)) {
    throw std::invalid_argument("initial camera time shift must be finite");
  }
  return shift_s;
}

double initialImuTimeOffsetFor(const CalibrationOptions &options,
                               const std::size_t imu_index,
                               const std::size_t imu_count) {
  if (options.initial_imu_time_offsets_s.empty()) {
    return 0.0;
  }
  if (options.initial_imu_time_offsets_s.size() != imu_count) {
    throw std::invalid_argument(
        "initial IMU time offsets must match IMU count");
  }
  const double offset_s = options.initial_imu_time_offsets_s.at(imu_index);
  if (!std::isfinite(offset_s)) {
    throw std::invalid_argument("initial IMU time offset must be finite");
  }
  if (imu_index == 0 && offset_s != 0.0) {
    throw std::invalid_argument(
        "reference IMU time offset must be zero in the shared time domain");
  }
  return offset_s;
}

void extendTimeSpan(const double timestamp_s, double *first, double *last) {
  if (!std::isfinite(timestamp_s)) {
    throw std::invalid_argument("aligned sensor timestamp must be finite");
  }
  *first = std::min(*first, timestamp_s);
  *last = std::max(*last, timestamp_s);
}

std::pair<double, double>
alignedTimeSpan(const std::vector<CameraObservationDataset> &cameras,
                const std::vector<ImuObservationDataset> &imus,
                const CalibrationOptions &options) {
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  for (std::size_t camera_index = 0; camera_index < cameras.size();
       ++camera_index) {
    const double shift_s =
        initialCameraTimeShiftFor(options, camera_index, cameras.size());
    for (const ImageObservation &image : cameras[camera_index].images) {
      extendTimeSpan(image.timestamp_s + shift_s, &first, &last);
    }
  }
  for (std::size_t imu_index = 0; imu_index < imus.size(); ++imu_index) {
    const double offset_s =
        initialImuTimeOffsetFor(options, imu_index, imus.size());
    for (const ImuSample &sample : imus[imu_index].samples) {
      extendTimeSpan(sample.timestamp_s + offset_s, &first, &last);
    }
  }
  if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
    throw std::runtime_error(
        "cannot initialize splines from empty or degenerate aligned time span");
  }
  return {first, last};
}

std::pair<double, double>
alignedTimeSpan(const std::vector<CameraObservationDataset> &cameras,
                           const std::vector<ImuSample> &imu_samples,
                           const CalibrationOptions &options) {
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  for (std::size_t camera_index = 0; camera_index < cameras.size();
       ++camera_index) {
    const double shift_s =
        initialCameraTimeShiftFor(options, camera_index, cameras.size());
    for (const ImageObservation &image : cameras[camera_index].images) {
      extendTimeSpan(image.timestamp_s + shift_s, &first, &last);
    }
  }
  const double imu_offset_s = initialImuTimeOffsetFor(options, 0, 1);
  for (const ImuSample &sample : imu_samples) {
    extendTimeSpan(sample.timestamp_s + imu_offset_s, &first, &last);
  }
  if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
    throw std::runtime_error(
        "cannot initialize splines from empty or degenerate aligned time span");
  }
  return {first, last};
}

CalibrationState
initializeCalibrationStateForTimeSpan(const double first, const double last,
                                      const CalibrationOptions &options) {
  if (options.spline_order != 6) {
    throw std::runtime_error(
        "first Ceres implementation currently requires order-6 splines");
  }

  CalibrationState state;
  state.pose_spline =
      makeSplineForTimes(6, options.spline_order, first, last,
                         options.pose_knots_per_second, options.time_padding_s);
  state.gyro_bias_spline =
      makeSplineForTimes(3, options.spline_order, first, last,
                         options.bias_knots_per_second, options.time_padding_s);
  state.accel_bias_spline =
      makeSplineForTimes(3, options.spline_order, first, last,
                         options.bias_knots_per_second, options.time_padding_s);

  state.pose_controls.resize(
      static_cast<std::size_t>(state.pose_spline.numCoefficients()));
  for (PoseControlBlock &control : state.pose_controls) {
    control.values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  state.gyro_bias_controls.resize(
      static_cast<std::size_t>(state.gyro_bias_spline.numCoefficients()));
  for (BiasControlBlock &control : state.gyro_bias_controls) {
    control.values = {options.initial_gyro_bias_rad_s.x(),
                      options.initial_gyro_bias_rad_s.y(),
                      options.initial_gyro_bias_rad_s.z()};
  }

  state.accel_bias_controls.resize(
      static_cast<std::size_t>(state.accel_bias_spline.numCoefficients()));
  for (BiasControlBlock &control : state.accel_bias_controls) {
    control.values = {options.initial_accel_bias_m_s2.x(),
                      options.initial_accel_bias_m_s2.y(),
                      options.initial_accel_bias_m_s2.z()};
  }

  return state;
}

} // namespace

CalibrationState
initializeCalibrationState(const std::vector<ImageObservation> &images,
                           const std::vector<ImuSample> &imu_samples,
                           const CalibrationOptions &options) {
  const double camera_time_shift_s = initialCameraTimeShiftFor(options, 0, 1);
  const double imu_time_offset_s = initialImuTimeOffsetFor(options, 0, 1);
  const auto [first, last] =
      timeSpan(images, imu_samples, camera_time_shift_s, imu_time_offset_s);
  CalibrationState state =
      initializeCalibrationStateForTimeSpan(first, last, options);
  state.camera_time_shift_s.value = camera_time_shift_s;
  if (!options.initial_imu_time_offsets_s.empty()) {
    state.imu_time_offsets_s = {imu_time_offset_s};
  }
  return state;
}

CalibrationState initializeCalibrationState(
    const std::vector<CameraObservationDataset> &cameras,
    const std::vector<ImuSample> &imu_samples,
    const CalibrationOptions &options) {
  if (cameras.empty()) {
    throw std::runtime_error("at least one camera dataset is required");
  }
  const auto [first, last] = alignedTimeSpan(cameras, imu_samples, options);
  CalibrationState state =
      initializeCalibrationStateForTimeSpan(first, last, options);
  state.camera_extrinsics.resize(cameras.size());
  state.camera_time_shifts.resize(cameras.size());
  state.camera_time_shift_s.value =
      initialCameraTimeShiftFor(options, 0, cameras.size());
  state.camera_extrinsics[0] = state.T_c_b;
  state.camera_time_shifts[0] = state.camera_time_shift_s;
  for (std::size_t i = 1; i < cameras.size(); ++i) {
    state.camera_time_shifts[i].value =
        initialCameraTimeShiftFor(options, i, cameras.size());
  }
  return state;
}

CalibrationState initializeCalibrationState(
    const std::vector<CameraObservationDataset> &cameras,
    const std::vector<ImuObservationDataset> &imus,
    const CalibrationOptions &options) {
  if (cameras.empty()) {
    throw std::runtime_error("at least one camera dataset is required");
  }
  if (imus.empty()) {
    throw std::runtime_error("at least one IMU dataset is required");
  }
  const auto [first, last] = alignedTimeSpan(cameras, imus, options);
  CalibrationState state =
      initializeCalibrationStateForTimeSpan(first, last, options);
  state.camera_extrinsics.resize(cameras.size());
  state.camera_time_shifts.resize(cameras.size());
  state.camera_time_shift_s.value =
      initialCameraTimeShiftFor(options, 0, cameras.size());
  state.camera_extrinsics[0] = state.T_c_b;
  state.camera_time_shifts[0] = state.camera_time_shift_s;
  for (std::size_t i = 1; i < cameras.size(); ++i) {
    state.camera_time_shifts[i].value =
        initialCameraTimeShiftFor(options, i, cameras.size());
  }
  ensureMultiImuStateSize(&state, imus.size());
  for (std::size_t imu_index = 0; imu_index < imus.size(); ++imu_index) {
    state.imu_time_offsets_s[imu_index] =
        initialImuTimeOffsetFor(options, imu_index, imus.size());
  }
  return state;
}

CalibrationBuildSummary
buildCalibrationProblem(const CameraIntrinsics &intrinsics,
                        const ImuNoise &imu_noise,
                        const std::vector<ImageObservation> &images,
                        const std::vector<ImuSample> &imu_samples,
                        const CalibrationOptions &options,
                        CalibrationState *state, ceres::Problem *problem) {
  if (!state || !problem) {
    throw std::invalid_argument("state and problem must be non-null");
  }

  CalibrationBuildSummary summary;
  std::vector<char> active_pose_segments(
      static_cast<std::size_t>(state->pose_spline.numSegments()), 0);

  problem->AddParameterBlock(dataPtr(state->T_c_b), 6);
  setExtrinsicManifoldIfEnabled(options, dataPtr(state->T_c_b), problem);
  problem->AddParameterBlock(dataPtr(state->camera_time_shift_s), 1);
  problem->AddParameterBlock(dataPtr(state->imu_extrinsic), 6);
  setExtrinsicManifoldIfEnabled(options, dataPtr(state->imu_extrinsic),
                                problem);
  problem->AddParameterBlock(dataPtr(state->gravity), 3);
  addImuIntrinsicParameterBlocks(options, &state->imu_intrinsics, problem);
  if (!options.estimate_gravity_length) {
    const Vec3 gravity(state->gravity.values[0], state->gravity.values[1],
                       state->gravity.values[2]);
    if (gravity.norm() <= 0.0) {
      throw std::runtime_error(
          "gravity direction manifold requires non-zero gravity");
    }
    problem->SetManifold(dataPtr(state->gravity),
                         new ceres::SphereManifold<3>());
  }
  if (options.fix_reference_imu_extrinsic) {
    problem->SetParameterBlockConstant(dataPtr(state->imu_extrinsic));
  }
  if (options.fix_camera_extrinsic) {
    problem->SetParameterBlockConstant(dataPtr(state->T_c_b));
  }
  if (options.fix_time_shift) {
    problem->SetParameterBlockConstant(dataPtr(state->camera_time_shift_s));
  }
  if (options.fix_gravity) {
    problem->SetParameterBlockConstant(dataPtr(state->gravity));
  }
  // ParameterBlockTangentSize reports the manifold's tangent size whether or
  // not the block is constant, so a fixed gravity would still be counted as 2
  // (or 3) free directions and inflate kalibr_style_error_terms. Report the
  // number of directions actually being solved for, matching the convention
  // fillProblemSizeSummary uses for tangent_parameters.
  summary.gravity_tangent_size =
      problem->IsParameterBlockConstant(dataPtr(state->gravity))
          ? 0
          : problem->ParameterBlockTangentSize(dataPtr(state->gravity));
  if (options.add_time_shift_prior && options.time_shift_prior_sigma_s > 0.0) {
    problem->AddResidualBlock(
        createTimeShiftPrior(cameraTimeShiftPriorCenter(options, 0),
                             options.time_shift_prior_sigma_s),
        nullptr, dataPtr(state->camera_time_shift_s));
    ++summary.time_shift_priors;
  }

  for (PoseControlBlock &control : state->pose_controls) {
    problem->AddParameterBlock(dataPtr(control), 6);
    setPoseControlManifoldIfEnabled(options, dataPtr(control), problem);
    if (options.fix_pose_controls) {
      problem->SetParameterBlockConstant(dataPtr(control));
    }
  }
  for (BiasControlBlock &control : state->gyro_bias_controls) {
    problem->AddParameterBlock(dataPtr(control), 3);
    if (options.fix_bias_controls) {
      problem->SetParameterBlockConstant(dataPtr(control));
    }
  }
  for (BiasControlBlock &control : state->accel_bias_controls) {
    problem->AddParameterBlock(dataPtr(control), 3);
    if (options.fix_bias_controls) {
      problem->SetParameterBlockConstant(dataPtr(control));
    }
  }

  int frame_count = 0;
  for (const ImageObservation &image : images) {
    if (options.max_frames > 0 && frame_count >= options.max_frames) {
      break;
    }
    ++frame_count;
    const double query_time =
        image.timestamp_s + state->camera_time_shift_s.value;
    if (!state->pose_spline.isValidTime(query_time)) {
      ++summary.skipped_camera_frames;
      continue;
    }
    const SplineSegmentMeta6 pose_meta =
        state->pose_spline.segmentMeta6(query_time);
    for (const CornerMeasurement &corner : image.corners) {
      addCameraReprojectionResidualBlock(
          intrinsics, corner, image.timestamp_s, options, state, &state->T_c_b,
          &state->camera_time_shift_s, pose_meta, &active_pose_segments, problem,
          &summary);
    }
  }

  int added_imu = 0;
  const int stride = std::max(1, options.imu_stride);
  for (std::size_t i = 0; i < imu_samples.size();
       i += static_cast<std::size_t>(stride)) {
    if (options.max_imu_residuals > 0 &&
        added_imu >= options.max_imu_residuals) {
      break;
    }
    const ImuSample &sample = imu_samples[i];
    if (!state->pose_spline.isValidTime(sample.timestamp_s) ||
        !state->gyro_bias_spline.isValidTime(sample.timestamp_s) ||
        !state->accel_bias_spline.isValidTime(sample.timestamp_s)) {
      ++summary.skipped_imu_samples;
      continue;
    }
    const SplineSegmentMeta6 pose_meta =
        state->pose_spline.segmentMeta6(sample.timestamp_s);
    const SplineSegmentMeta6 gyro_bias_meta =
        state->gyro_bias_spline.segmentMeta6(sample.timestamp_s);
    const SplineSegmentMeta6 accel_bias_meta =
        state->accel_bias_spline.segmentMeta6(sample.timestamp_s);
    markActiveSegment(pose_meta, &active_pose_segments);

    std::unique_ptr<ceres::LossFunction> gyro_loss =
        makeLoss(options.gyro_loss_type, options.gyro_loss_width);
    if (usesScaleMisalignment(options.imu_model)) {
      problem->AddResidualBlock(
          createScaleMisalignedGyroscopeResidual(sample, imu_noise, pose_meta,
                                                 gyro_bias_meta),
          gyro_loss.release(), dataPtr(state->imu_extrinsic),
          dataPtr(state->gravity),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 0)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 1)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 2)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 3)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 4)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 5)),
          dataPtr(state->imu_intrinsics.gyro_sensing_rotation),
          dataPtr(state->imu_intrinsics.gyro_M),
          dataPtr(state->imu_intrinsics.gyro_accel_sensitivity));
    } else {
      problem->AddResidualBlock(
          createGyroscopeResidual(sample, imu_noise, pose_meta, gyro_bias_meta),
          gyro_loss.release(), dataPtr(state->imu_extrinsic),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 0)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 1)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 2)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 3)),
          dataPtr(state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 4)),
          dataPtr(
              state->gyro_bias_controls.at(gyro_bias_meta.coeff_start + 5)));
    }
    ++summary.gyro_residuals;

    std::unique_ptr<ceres::LossFunction> accel_loss =
        makeLoss(options.accel_loss_type, options.accel_loss_width);
    if (usesSizeEffect(options.imu_model)) {
      problem->AddResidualBlock(
          createSizeEffectAccelerometerResidual(sample, imu_noise, pose_meta,
                                                accel_bias_meta),
          accel_loss.release(), dataPtr(state->imu_extrinsic),
          dataPtr(state->gravity),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 0)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 1)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 2)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 3)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 4)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 5)),
          dataPtr(state->imu_intrinsics.accel_M),
          dataPtr(state->imu_intrinsics.accel_axis_rx_i),
          dataPtr(state->imu_intrinsics.accel_axis_ry_i),
          dataPtr(state->imu_intrinsics.accel_axis_rz_i));
    } else if (usesScaleMisalignment(options.imu_model)) {
      problem->AddResidualBlock(
          createScaleMisalignedAccelerometerResidual(
              sample, imu_noise, pose_meta, accel_bias_meta),
          accel_loss.release(), dataPtr(state->imu_extrinsic),
          dataPtr(state->gravity),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 0)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 1)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 2)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 3)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 4)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 5)),
          dataPtr(state->imu_intrinsics.accel_M));
    } else {
      problem->AddResidualBlock(
          createAccelerometerResidual(sample, imu_noise, pose_meta,
                                      accel_bias_meta),
          accel_loss.release(), dataPtr(state->imu_extrinsic),
          dataPtr(state->gravity),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 0)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 1)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 2)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 3)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 4)),
          dataPtr(
              state->accel_bias_controls.at(accel_bias_meta.coeff_start + 5)));
    }
    ++summary.accel_residuals;
    ++added_imu;
  }

  if (options.add_bias_motion_prior) {
    for (int segment = 0; segment < state->gyro_bias_spline.numSegments();
         ++segment) {
      const SplineSegmentMeta6 meta = state->gyro_bias_spline.segmentMeta6(
          state->gyro_bias_spline.tMin() +
          static_cast<double>(segment) * state->gyro_bias_spline.dt());
      problem->AddResidualBlock(
          createBiasMotionPrior(meta, imu_noise.gyroscope_random_walk), nullptr,
          dataPtr(state->gyro_bias_controls.at(meta.coeff_start + 0)),
          dataPtr(state->gyro_bias_controls.at(meta.coeff_start + 1)),
          dataPtr(state->gyro_bias_controls.at(meta.coeff_start + 2)),
          dataPtr(state->gyro_bias_controls.at(meta.coeff_start + 3)),
          dataPtr(state->gyro_bias_controls.at(meta.coeff_start + 4)),
          dataPtr(state->gyro_bias_controls.at(meta.coeff_start + 5)));
      ++summary.gyro_bias_priors;
    }

    for (int segment = 0; segment < state->accel_bias_spline.numSegments();
         ++segment) {
      const SplineSegmentMeta6 meta = state->accel_bias_spline.segmentMeta6(
          state->accel_bias_spline.tMin() +
          static_cast<double>(segment) * state->accel_bias_spline.dt());
      problem->AddResidualBlock(
          createBiasMotionPrior(meta, imu_noise.accelerometer_random_walk),
          nullptr, dataPtr(state->accel_bias_controls.at(meta.coeff_start + 0)),
          dataPtr(state->accel_bias_controls.at(meta.coeff_start + 1)),
          dataPtr(state->accel_bias_controls.at(meta.coeff_start + 2)),
          dataPtr(state->accel_bias_controls.at(meta.coeff_start + 3)),
          dataPtr(state->accel_bias_controls.at(meta.coeff_start + 4)),
          dataPtr(state->accel_bias_controls.at(meta.coeff_start + 5)));
      ++summary.accel_bias_priors;
    }
  }

  if (options.add_pose_motion_prior) {
    for (int segment = 0; segment < state->pose_spline.numSegments();
         ++segment) {
      if (!options.pose_motion_all_segments &&
          !active_pose_segments.at(static_cast<std::size_t>(segment))) {
        continue;
      }
      const SplineSegmentMeta6 meta = state->pose_spline.segmentMeta6(
          state->pose_spline.tMin() +
          static_cast<double>(segment) * state->pose_spline.dt());
      double translation_variance = options.pose_motion_translation_variance;
      double rotation_variance = options.pose_motion_rotation_variance;
      if (overlapsLocalPoseMotionWindow(meta, options)) {
        translation_variance *=
            options.pose_motion_local_translation_variance_scale;
        rotation_variance *= options.pose_motion_local_rotation_variance_scale;
        ++summary.local_pose_motion_priors;
      }
      problem->AddResidualBlock(
          createPoseMotionPrior(meta, translation_variance, rotation_variance,
                                options.pose_motion_derivative_order),
          nullptr, dataPtr(state->pose_controls.at(meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(meta.coeff_start + 5)));
      ++summary.pose_motion_priors;
    }
  }

  fillProblemSizeSummary(*problem, &summary);
  summary.kalibr_style_error_terms =
      summary.camera_residuals + summary.gyro_residuals +
      summary.accel_residuals + summary.gravity_tangent_size;

  return summary;
}

CalibrationBuildSummary
buildCalibrationProblem(const std::vector<CameraObservationDataset> &cameras,
                        const ImuNoise &imu_noise,
                        const std::vector<ImuSample> &imu_samples,
                        const CalibrationOptions &options,
                        CalibrationState *state, ceres::Problem *problem) {
  if (cameras.empty()) {
    throw std::invalid_argument("at least one camera dataset is required");
  }
  if (!state || !problem) {
    throw std::invalid_argument("state and problem must be non-null");
  }
  if (!options.camera_time_shift_priors_s.empty() &&
      options.camera_time_shift_priors_s.size() != cameras.size()) {
    throw std::invalid_argument(
        "per-camera time-shift prior centers must match camera count");
  }

  CalibrationBuildSummary summary =
      buildCalibrationProblem(cameras.front().intrinsics, imu_noise,
                              cameras.front().images, imu_samples, options,
                              state, problem);

  if (cameras.size() == 1) {
    return summary;
  }
  if (state->camera_extrinsics.size() < cameras.size()) {
    state->camera_extrinsics.resize(cameras.size());
  }
  if (state->camera_time_shifts.size() < cameras.size()) {
    state->camera_time_shifts.resize(cameras.size());
  }
  state->camera_extrinsics[0] = state->T_c_b;
  state->camera_time_shifts[0] = state->camera_time_shift_s;

  for (std::size_t camera_index = 1; camera_index < cameras.size();
       ++camera_index) {
    CameraExtrinsicBlock &T_c_b = state->camera_extrinsics[camera_index];
    TimeShiftBlock &time_shift = state->camera_time_shifts[camera_index];
    problem->AddParameterBlock(dataPtr(T_c_b), 6);
    setExtrinsicManifoldIfEnabled(options, dataPtr(T_c_b), problem);
    problem->AddParameterBlock(dataPtr(time_shift), 1);
    if (options.fix_camera_extrinsic) {
      problem->SetParameterBlockConstant(dataPtr(T_c_b));
    }
    if (options.fix_time_shift) {
      problem->SetParameterBlockConstant(dataPtr(time_shift));
    }
    if (options.fix_camera_chain_extrinsics) {
      if (options.camera_chain_T_ci_c0_prior.size() <= camera_index) {
        throw std::runtime_error(
            "--fix-camera-chain-extrinsics requires a camera-chain prior for "
            "each non-reference camera");
      }
      problem->AddResidualBlock(
          createCameraChainExtrinsicPrior(
              options.camera_chain_T_ci_c0_prior[camera_index],
              options.camera_chain_translation_sigma_m,
              options.camera_chain_rotation_sigma_rad),
          nullptr, dataPtr(state->T_c_b), dataPtr(T_c_b));
      ++summary.camera_chain_priors;
    }
    if (options.add_time_shift_prior &&
        options.time_shift_prior_sigma_s > 0.0) {
      problem->AddResidualBlock(
          createTimeShiftPrior(cameraTimeShiftPriorCenter(options,
                                                           camera_index),
                               options.time_shift_prior_sigma_s),
          nullptr, dataPtr(time_shift));
      ++summary.time_shift_priors;
    }

    int frame_count = 0;
    const CameraObservationDataset &camera = cameras[camera_index];
    for (const ImageObservation &image : camera.images) {
      if (options.max_frames > 0 && frame_count >= options.max_frames) {
        break;
      }
      ++frame_count;
      const double query_time = image.timestamp_s + time_shift.value;
      if (!state->pose_spline.isValidTime(query_time)) {
        ++summary.skipped_camera_frames;
        continue;
      }
      const SplineSegmentMeta6 pose_meta =
          state->pose_spline.segmentMeta6(query_time);
      for (const CornerMeasurement &corner : image.corners) {
        addCameraReprojectionResidualBlock(
            camera.intrinsics, corner, image.timestamp_s, options, state,
            &T_c_b, &time_shift, pose_meta, nullptr, problem, &summary);
      }
    }
  }

  fillProblemSizeSummary(*problem, &summary);
  summary.kalibr_style_error_terms =
      summary.camera_residuals + summary.gyro_residuals +
      summary.accel_residuals + summary.gravity_tangent_size;
  return summary;
}

CalibrationBuildSummary
buildCalibrationProblem(const std::vector<CameraObservationDataset> &cameras,
                        const std::vector<ImuObservationDataset> &imus,
                        const CalibrationOptions &options,
                        CalibrationState *state, ceres::Problem *problem) {
  if (imus.empty()) {
    throw std::invalid_argument("at least one IMU dataset is required");
  }
  if (!state || !problem) {
    throw std::invalid_argument("state and problem must be non-null");
  }
  ensureMultiImuStateSize(state, imus.size());

  CalibrationBuildSummary summary =
      buildCalibrationProblem(cameras, imus.front().noise,
                              imus.front().samples, options, state, problem);

  if (imus.size() == 1) {
    return summary;
  }

  for (std::size_t imu_index = 1; imu_index < imus.size(); ++imu_index) {
    addImuParameterBlocksForIndex(imu_index, options, state, problem);
    addImuResidualBlocksForIndex(imu_index, imus[imu_index], options, state,
                                 problem, nullptr, &summary);
    if (options.add_bias_motion_prior) {
      addBiasMotionPriorsForIndex(imu_index, imus[imu_index].noise, state,
                                  problem, &summary);
    }
  }

  fillProblemSizeSummary(*problem, &summary);
  summary.kalibr_style_error_terms =
      summary.camera_residuals + summary.gyro_residuals +
      summary.accel_residuals + summary.gravity_tangent_size;
  return summary;
}

ceres::Solver::Summary
solveCalibrationProblem(const CalibrationOptions &options,
                        ceres::Problem *problem) {
  return solveCalibrationProblem(options, static_cast<CalibrationState *>(nullptr),
                                 problem);
}

ceres::Solver::Summary
solveCalibrationProblem(const CalibrationOptions &options,
                        CalibrationState *state, ceres::Problem *problem) {
  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = options.max_iterations;
  solver_options.function_tolerance = options.solver_function_tolerance;
  solver_options.gradient_tolerance = options.solver_gradient_tolerance;
  solver_options.parameter_tolerance = options.solver_parameter_tolerance;
  solver_options.initial_trust_region_radius =
      options.solver_initial_trust_region_radius;
  solver_options.max_trust_region_radius =
      options.solver_max_trust_region_radius;
  solver_options.min_trust_region_radius =
      options.solver_min_trust_region_radius;
  solver_options.min_relative_decrease = options.solver_min_relative_decrease;
  solver_options.num_threads = options.solver_num_threads;
  solver_options.use_nonmonotonic_steps = options.solver_use_nonmonotonic_steps;
  solver_options.max_consecutive_nonmonotonic_steps =
      options.solver_max_consecutive_nonmonotonic_steps;
  solver_options.linear_solver_type = options.solver_linear_solver_type;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.minimizer_progress_to_stdout = true;

  // Diagnostic: set CERES_CHECK_GRADIENTS=1 to compare every analytic Jacobian
  // against a finite-difference reference and report mismatching residual blocks.
  if (const char *check_env = std::getenv("CERES_CHECK_GRADIENTS")) {
    if (check_env[0] != '\0' && check_env[0] != '0') {
      solver_options.check_gradients = true;
      solver_options.gradient_check_relative_precision = 1e-3;
    }
  }

  StateTraceCallback trace_callback(options, state, problem);
  if (options.trace_iteration_state ||
      options.solver_absolute_parameter_tolerance >= 0.0 ||
      options.solver_restore_best_state) {
    solver_options.update_state_every_iteration = true;
  }
  if (options.trace_iteration_state) {
    solver_options.callbacks.push_back(&trace_callback);
  }
  BestStateCallback best_state_callback(options, state);
  if (best_state_callback.enabled()) {
    solver_options.callbacks.push_back(&best_state_callback);
  }
  AbsoluteStopCallback absolute_stop_callback(options, problem);
  if (absolute_stop_callback.enabled()) {
    solver_options.callbacks.push_back(&absolute_stop_callback);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, problem, &summary);
  best_state_callback.restoreIfBetter(state, &summary);
  return summary;
}

PoseInitializationSummary initializePoseControlsFromCameraPoses(
    const std::vector<PoseObservation> &pose_observations,
    const CameraExtrinsicBlock &T_c_b, CalibrationState *state) {
  CalibrationOptions options;
  options.pose_fit_diagonal_regularization = 0.0;
  options.pose_fit_motion_regularization = 0.0;
  options.pose_fit_add_boundary_anchors = false;
  return initializePoseControlsFromCameraPoses(pose_observations, T_c_b,
                                               options, state);
}

PoseInitializationSummary initializePoseControlsFromCameraPoses(
    const std::vector<PoseObservation> &pose_observations,
    const CameraExtrinsicBlock &T_c_b, const CalibrationOptions &options,
    CalibrationState *state) {
  PoseInitializationSummary summary;
  if (!state || pose_observations.empty() || state->pose_controls.empty()) {
    return summary;
  }

  PoseSplineFitOptions fit_options;
  fit_options.regularization = options.pose_fit_diagonal_regularization;
  fit_options.motion_regularization = options.pose_fit_motion_regularization;
  fit_options.motion_regularization_order = 2;
  fit_options.add_boundary_anchors = options.pose_fit_add_boundary_anchors;
  fit_options.boundary_anchor_padding_s = options.time_padding_s;
  fit_options.unwrap_rotation_vectors = true;
  const PoseSplineFitSummary fit_summary = fitPoseSplineControlsFromCameraPoses(
      pose_observations, T_c_b, state->camera_time_shift_s.value,
      state->pose_spline, fit_options, &state->pose_controls);
  summary.used_observations = fit_summary.used_observations;
  summary.skipped_observations = fit_summary.skipped_observations;
  summary.boundary_anchor_observations =
      fit_summary.boundary_anchor_observations;
  summary.num_coefficients = fit_summary.num_coefficients;
  summary.rms_translation_m = fit_summary.rms_translation_m;
  summary.rms_rotation_rad = fit_summary.rms_rotation_rad;
  return summary;
}

Vec6 matrixToPose6(const Mat4 &T) {
  Vec6 pose;
  pose.head<3>() = T.block<3, 1>(0, 3);
  pose.tail<3>() = rotationMatrixToVector(T.block<3, 3>(0, 0));
  return pose;
}

Mat4 pose6ToMatrix(const CameraExtrinsicBlock &pose) {
  return cameraExtrinsicToMatrix(pose);
}

} // namespace ceres_cam_imu
