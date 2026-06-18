#include "ceres_cam_imu/optimizer/calibration_problem.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <ceres/sphere_manifold.h>

#include "ceres_cam_imu/core/se3.h"
#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/initialization/pose_spline_fit.h"
#include "ceres_cam_imu/optimizer/parameter_delta_tracker.h"
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
                                   const double camera_time_shift_s) {
  double first = std::numeric_limits<double>::infinity();
  double last = -std::numeric_limits<double>::infinity();
  for (const ImageObservation &image : images) {
    first = std::min(first, image.timestamp_s + camera_time_shift_s);
    last = std::max(last, image.timestamp_s + camera_time_shift_s);
  }
  for (const ImuSample &sample : imu_samples) {
    first = std::min(first, sample.timestamp_s);
    last = std::max(last, sample.timestamp_s);
  }
  if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
    throw std::runtime_error(
        "cannot initialize splines from empty or degenerate time span");
  }
  return {first, last};
}

template <typename Block> double *dataPtr(Block &block) { return block.data(); }

Mat4 cameraExtrinsicToMatrix(const CameraExtrinsicBlock &pose) {
  Vec6 p;
  for (int i = 0; i < 6; ++i) {
    p(i) = pose.values[static_cast<std::size_t>(i)];
  }
  return ceres_cam_imu::pose6ToMatrix(p);
}

std::unique_ptr<ceres::LossFunction> makeLoss(const RobustLossType type,
                                              const double width) {
  if (width <= 0.0) {
    return nullptr;
  }
  if (type == RobustLossType::kNone) {
    return nullptr;
  }
  if (type == RobustLossType::kHuber) {
    return std::unique_ptr<ceres::LossFunction>(new ceres::HuberLoss(width));
  }
  if (type == RobustLossType::kCauchy) {
    // Kalibr's CauchyMEstimator stores the squared-error denominator
    // directly: w(s) = 1 / (1 + s / sigma2). Ceres CauchyLoss uses a
    // scale `a` with rho'(s) = 1 / (1 + s / a^2), so the matching scale
    // is sqrt(sigma2).
    return std::unique_ptr<ceres::LossFunction>(
        new ceres::CauchyLoss(std::sqrt(width)));
  }
  return nullptr;
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
                                    CalibrationState *state,
                                    ceres::Problem *problem) {
  if (!usesScaleMisalignment(options.imu_model)) {
    return;
  }
  problem->AddParameterBlock(dataPtr(state->imu_intrinsics.accel_M),
                             LowerTriangularMatrixBlock::kSize);
  problem->AddParameterBlock(dataPtr(state->imu_intrinsics.gyro_M),
                             LowerTriangularMatrixBlock::kSize);
  problem->AddParameterBlock(
      dataPtr(state->imu_intrinsics.gyro_accel_sensitivity),
      Matrix3Block::kSize);
  problem->AddParameterBlock(
      dataPtr(state->imu_intrinsics.gyro_sensing_rotation),
      Vector3Block::kSize);
  if (options.fix_imu_intrinsics) {
    problem->SetParameterBlockConstant(dataPtr(state->imu_intrinsics.accel_M));
    problem->SetParameterBlockConstant(dataPtr(state->imu_intrinsics.gyro_M));
    problem->SetParameterBlockConstant(
        dataPtr(state->imu_intrinsics.gyro_accel_sensitivity));
    problem->SetParameterBlockConstant(
        dataPtr(state->imu_intrinsics.gyro_sensing_rotation));
  }
  if (!usesSizeEffect(options.imu_model)) {
    return;
  }
  problem->AddParameterBlock(dataPtr(state->imu_intrinsics.accel_axis_rx_i),
                             Vector3Block::kSize);
  problem->AddParameterBlock(dataPtr(state->imu_intrinsics.accel_axis_ry_i),
                             Vector3Block::kSize);
  problem->AddParameterBlock(dataPtr(state->imu_intrinsics.accel_axis_rz_i),
                             Vector3Block::kSize);
  if (options.fix_imu_intrinsics || options.fix_accel_size_effect_rx) {
    problem->SetParameterBlockConstant(
        dataPtr(state->imu_intrinsics.accel_axis_rx_i));
  }
  if (options.fix_imu_intrinsics) {
    problem->SetParameterBlockConstant(
        dataPtr(state->imu_intrinsics.accel_axis_ry_i));
    problem->SetParameterBlockConstant(
        dataPtr(state->imu_intrinsics.accel_axis_rz_i));
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

CalibrationState
initializeCalibrationState(const std::vector<ImageObservation> &images,
                           const std::vector<ImuSample> &imu_samples,
                           const CalibrationOptions &options) {
  if (options.spline_order != 6) {
    throw std::runtime_error(
        "first Ceres implementation currently requires order-6 splines");
  }

  const auto [first, last] =
      timeSpan(images, imu_samples, options.initial_camera_time_shift_s);
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

  state.camera_time_shift_s.value = options.initial_camera_time_shift_s;

  return state;
}

CalibrationState initializeCalibrationState(
    const std::vector<CameraObservationDataset> &cameras,
    const std::vector<ImuSample> &imu_samples,
    const CalibrationOptions &options) {
  if (cameras.empty()) {
    throw std::runtime_error("at least one camera dataset is required");
  }
  std::vector<ImageObservation> all_images;
  for (const CameraObservationDataset &camera : cameras) {
    all_images.insert(all_images.end(), camera.images.begin(),
                      camera.images.end());
  }
  CalibrationState state =
      initializeCalibrationState(all_images, imu_samples, options);
  state.camera_extrinsics.resize(cameras.size());
  state.camera_time_shifts.resize(cameras.size());
  state.camera_extrinsics[0] = state.T_c_b;
  state.camera_time_shifts[0] = state.camera_time_shift_s;
  for (std::size_t i = 1; i < cameras.size(); ++i) {
    state.camera_time_shifts[i].value = options.initial_camera_time_shift_s;
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
  problem->AddParameterBlock(dataPtr(state->camera_time_shift_s), 1);
  problem->AddParameterBlock(dataPtr(state->imu_extrinsic), 6);
  problem->AddParameterBlock(dataPtr(state->gravity), 3);
  addImuIntrinsicParameterBlocks(options, state, problem);
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
  summary.gravity_tangent_size =
      problem->ParameterBlockTangentSize(dataPtr(state->gravity));
  if (options.add_time_shift_prior && options.time_shift_prior_sigma_s > 0.0) {
    problem->AddResidualBlock(
        createTimeShiftPrior(options.time_shift_prior_s,
                             options.time_shift_prior_sigma_s),
        nullptr, dataPtr(state->camera_time_shift_s));
    ++summary.time_shift_priors;
  }

  for (PoseControlBlock &control : state->pose_controls) {
    problem->AddParameterBlock(dataPtr(control), 6);
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
    markActiveSegment(pose_meta, &active_pose_segments);
    for (const CornerMeasurement &corner : image.corners) {
      ceres::CostFunction *cost = createCameraReprojectionResidual(
          intrinsics, corner, image.timestamp_s, pose_meta,
          options.reprojection_sigma_px);
      std::unique_ptr<ceres::LossFunction> loss =
          makeLoss(options.camera_loss_type, options.camera_loss_width);
      problem->AddResidualBlock(
          cost, loss.release(), dataPtr(state->T_c_b),
          dataPtr(state->camera_time_shift_s),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
          dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)));
      ++summary.camera_residuals;
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
    problem->AddParameterBlock(dataPtr(time_shift), 1);
    if (options.fix_camera_extrinsic) {
      problem->SetParameterBlockConstant(dataPtr(T_c_b));
    }
    if (options.fix_time_shift) {
      problem->SetParameterBlockConstant(dataPtr(time_shift));
    }
    if (options.add_time_shift_prior &&
        options.time_shift_prior_sigma_s > 0.0) {
      problem->AddResidualBlock(
          createTimeShiftPrior(options.time_shift_prior_s,
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
        ceres::CostFunction *cost = createCameraReprojectionResidual(
            camera.intrinsics, corner, image.timestamp_s, pose_meta,
            options.reprojection_sigma_px);
        std::unique_ptr<ceres::LossFunction> loss =
            makeLoss(options.camera_loss_type, options.camera_loss_width);
        problem->AddResidualBlock(
            cost, loss.release(), dataPtr(T_c_b), dataPtr(time_shift),
            dataPtr(state->pose_controls.at(pose_meta.coeff_start + 0)),
            dataPtr(state->pose_controls.at(pose_meta.coeff_start + 1)),
            dataPtr(state->pose_controls.at(pose_meta.coeff_start + 2)),
            dataPtr(state->pose_controls.at(pose_meta.coeff_start + 3)),
            dataPtr(state->pose_controls.at(pose_meta.coeff_start + 4)),
            dataPtr(state->pose_controls.at(pose_meta.coeff_start + 5)));
        ++summary.camera_residuals;
      }
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
  return solveCalibrationProblem(options, nullptr, problem);
}

ceres::Solver::Summary
solveCalibrationProblem(const CalibrationOptions &options,
                        const CalibrationState *state,
                        ceres::Problem *problem) {
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

  StateTraceCallback trace_callback(options, state, problem);
  if (options.trace_iteration_state ||
      options.solver_absolute_parameter_tolerance >= 0.0) {
    solver_options.update_state_every_iteration = true;
  }
  if (options.trace_iteration_state) {
    solver_options.callbacks.push_back(&trace_callback);
  }
  AbsoluteStopCallback absolute_stop_callback(options, problem);
  if (absolute_stop_callback.enabled()) {
    solver_options.callbacks.push_back(&absolute_stop_callback);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, problem, &summary);
  return summary;
}

PoseInitializationSummary initializePoseControlsFromCameraPoses(
    const std::vector<PoseObservation> &pose_observations,
    const CameraExtrinsicBlock &T_c_b, CalibrationState *state) {
  CalibrationOptions options;
  options.pose_fit_diagonal_regularization = 1e-9;
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
