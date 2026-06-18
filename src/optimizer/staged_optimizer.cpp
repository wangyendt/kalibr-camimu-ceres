#include "ceres_cam_imu/optimizer/staged_optimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "ceres_cam_imu/optimizer/state_snapshot.h"

namespace ceres_cam_imu {

namespace {

bool isSolverFailure(const ceres::Solver::Summary &summary) {
  return summary.termination_type == ceres::FAILURE ||
         summary.termination_type == ceres::USER_FAILURE;
}

bool hasFiniteStageCosts(const ceres::Solver::Summary &summary) {
  return std::isfinite(summary.initial_cost) &&
         std::isfinite(summary.final_cost);
}

bool hasStageCostIncrease(const ceres::Solver::Summary &summary) {
  const double tolerance =
      1e-12 * std::max(1.0, std::abs(summary.initial_cost));
  return summary.final_cost > summary.initial_cost + tolerance;
}

CalibrationOptions
stageOptions(const CalibrationOptions &base_options, const int max_iterations,
             const bool fix_pose_controls, const bool fix_bias_controls,
             const bool fix_camera_extrinsic, const bool fix_time_shift,
             const bool fix_gravity) {
  CalibrationOptions options = base_options;
  options.fix_pose_controls =
      fix_pose_controls || base_options.fix_pose_controls;
  options.fix_bias_controls =
      fix_bias_controls || base_options.fix_bias_controls;
  options.fix_camera_extrinsic =
      fix_camera_extrinsic || base_options.fix_camera_extrinsic;
  options.fix_time_shift = fix_time_shift || base_options.fix_time_shift;
  options.fix_gravity = fix_gravity || base_options.fix_gravity;
  options.max_iterations = std::max(0, max_iterations);
  return options;
}

int stageIteration(const CalibrationOptions &base_options,
                   const std::vector<int> &stage_iterations,
                   const std::size_t stage_index,
                   const std::size_t expected_count) {
  if (stage_iterations.empty()) {
    return base_options.max_iterations;
  }
  if (stage_iterations.size() != expected_count) {
    throw std::invalid_argument(
        "stage iteration cap count must match the number of stages");
  }
  return stage_iterations.at(stage_index);
}

bool stageMaskContains(const std::string &mask, const char variable) {
  return mask.find(variable) != std::string::npos;
}

void validateStageFreeMask(const std::string &mask) {
  if (mask.empty()) {
    throw std::invalid_argument("stage free-variable mask must be non-empty");
  }
  if (mask == "none" || mask == "-") {
    return;
  }
  for (const char ch : mask) {
    if (ch != 'p' && ch != 'b' && ch != 'e' && ch != 't' && ch != 'g') {
      throw std::invalid_argument("unknown stage free-variable mask character");
    }
  }
}

void validateVarianceSchedule(const std::vector<double> &variances,
                              const std::size_t stage_count, const char *name) {
  if (variances.empty()) {
    return;
  }
  if (variances.size() != stage_count) {
    throw std::invalid_argument(std::string(name) +
                                " count must match the number of stages");
  }
  for (const double variance : variances) {
    if (!(variance > 0.0)) {
      throw std::invalid_argument(std::string(name) +
                                  " values must be positive");
    }
  }
}

void validateNonNegativeSchedule(const std::vector<double> &values,
                                 const std::size_t stage_count,
                                 const char *name) {
  if (values.empty()) {
    return;
  }
  if (values.size() != stage_count) {
    throw std::invalid_argument(std::string(name) +
                                " count must match the number of stages");
  }
  for (const double value : values) {
    if (value < 0.0) {
      throw std::invalid_argument(std::string(name) +
                                  " values must be non-negative");
    }
  }
}

void validatePositiveSchedule(const std::vector<double> &values,
                              const std::size_t stage_count, const char *name) {
  if (values.empty()) {
    return;
  }
  if (values.size() != stage_count) {
    throw std::invalid_argument(std::string(name) +
                                " count must match the number of stages");
  }
  for (const double value : values) {
    if (!(value > 0.0)) {
      throw std::invalid_argument(std::string(name) +
                                  " values must be positive");
    }
  }
}

void validateDisabledOrNonNegativeSchedule(const std::vector<double> &values,
                                           const std::size_t stage_count,
                                           const char *name) {
  if (values.empty()) {
    return;
  }
  if (values.size() != stage_count) {
    throw std::invalid_argument(std::string(name) +
                                " count must match the number of stages");
  }
  for (const double value : values) {
    if (value < -1.0) {
      throw std::invalid_argument(std::string(name) +
                                  " values must be -1 or non-negative");
    }
  }
}

void validateDerivativeOrderSchedule(const std::vector<int> &orders,
                                     const std::size_t stage_count) {
  if (orders.empty()) {
    return;
  }
  if (orders.size() != stage_count) {
    throw std::invalid_argument(
        "stage pose-motion order count must match the number of stages");
  }
  for (const int order : orders) {
    if (order <= 0) {
      throw std::invalid_argument(
          "stage pose-motion order values must be positive");
    }
  }
}

void validateTrustRegionBounds(const std::vector<CalibrationStage> &stages) {
  for (const CalibrationStage &stage : stages) {
    if (stage.options.solver_min_trust_region_radius >
        stage.options.solver_max_trust_region_radius) {
      throw std::invalid_argument(
          "stage solver min trust region radius must not exceed max radius");
    }
  }
}

} // namespace

const char *calibrationStageStateDecisionName(
    const CalibrationStageStateDecision decision) {
  switch (decision) {
  case CalibrationStageStateDecision::kAccepted:
    return "accepted";
  case CalibrationStageStateDecision::kRestoredSolverFailure:
    return "restored_solver_failure";
  case CalibrationStageStateDecision::kRestoredCostIncrease:
    return "restored_cost_increase";
  case CalibrationStageStateDecision::kRestoredNonFiniteCost:
    return "restored_non_finite_cost";
  }
  return "unknown";
}

CalibrationStageStateDecision
decideCalibrationStageStateUpdate(const ceres::Solver::Summary &summary) {
  if (isSolverFailure(summary)) {
    return CalibrationStageStateDecision::kRestoredSolverFailure;
  }
  if (!hasFiniteStageCosts(summary)) {
    return CalibrationStageStateDecision::kRestoredNonFiniteCost;
  }
  if (hasStageCostIncrease(summary)) {
    return CalibrationStageStateDecision::kRestoredCostIncrease;
  }
  return CalibrationStageStateDecision::kAccepted;
}

std::vector<CalibrationStage>
makeConservativeCalibrationStages(const CalibrationOptions &base_options) {
  return makeConservativeCalibrationStages(base_options, {});
}

std::vector<CalibrationStage>
makeConservativeCalibrationStages(const CalibrationOptions &base_options,
                                  const std::vector<int> &stage_iterations) {
  return {
      {"extrinsic_from_fixed_motion",
       stageOptions(base_options,
                    stageIteration(base_options, stage_iterations, 0, 4), true,
                    true, false, true, true)},
      {"time_bias_from_fixed_pose_extrinsic",
       stageOptions(base_options,
                    stageIteration(base_options, stage_iterations, 1, 4), true,
                    false, true, false, true)},
      {"pose_time_bias_from_fixed_extrinsic",
       stageOptions(base_options,
                    stageIteration(base_options, stage_iterations, 2, 4), false,
                    false, true, false, true)},
      {"extrinsic_time_bias_from_refined_pose",
       stageOptions(base_options,
                    stageIteration(base_options, stage_iterations, 3, 4), true,
                    false, false, false, true)},
  };
}

std::vector<CalibrationStage> makeCalibrationStagesFromFreeMasks(
    const CalibrationOptions &base_options,
    const std::vector<int> &stage_iterations,
    const std::vector<std::string> &stage_free_masks) {
  if (stage_free_masks.empty()) {
    throw std::invalid_argument(
        "at least one stage free-variable mask is required");
  }
  std::vector<CalibrationStage> stages;
  stages.reserve(stage_free_masks.size());
  for (std::size_t i = 0; i < stage_free_masks.size(); ++i) {
    const std::string &mask = stage_free_masks.at(i);
    validateStageFreeMask(mask);
    const bool free_pose = stageMaskContains(mask, 'p');
    const bool free_bias = stageMaskContains(mask, 'b');
    const bool free_extrinsic = stageMaskContains(mask, 'e');
    const bool free_time = stageMaskContains(mask, 't');
    const bool free_gravity = stageMaskContains(mask, 'g');
    CalibrationStage stage;
    stage.name = "custom_" + std::to_string(i) + "_free_" + mask;
    stage.options = stageOptions(base_options,
                                 stageIteration(base_options, stage_iterations,
                                                i, stage_free_masks.size()),
                                 !free_pose, !free_bias, !free_extrinsic,
                                 !free_time, !free_gravity);
    stages.push_back(stage);
  }
  return stages;
}

void applyStagePoseMotionVariances(
    const std::vector<double> &translation_variances,
    const std::vector<double> &rotation_variances,
    std::vector<CalibrationStage> *stages) {
  if (!stages) {
    throw std::invalid_argument("stages must be non-null");
  }
  validateVarianceSchedule(translation_variances, stages->size(),
                           "stage translation variance");
  validateVarianceSchedule(rotation_variances, stages->size(),
                           "stage rotation variance");
  if (translation_variances.empty() && rotation_variances.empty()) {
    return;
  }
  for (std::size_t i = 0; i < stages->size(); ++i) {
    CalibrationOptions &options = stages->at(i).options;
    options.add_pose_motion_prior = true;
    if (!translation_variances.empty()) {
      options.pose_motion_translation_variance = translation_variances.at(i);
    }
    if (!rotation_variances.empty()) {
      options.pose_motion_rotation_variance = rotation_variances.at(i);
    }
  }
}

void applyStagePoseMotionOrders(const std::vector<int> &derivative_orders,
                                std::vector<CalibrationStage> *stages) {
  if (!stages) {
    throw std::invalid_argument("stages must be non-null");
  }
  validateDerivativeOrderSchedule(derivative_orders, stages->size());
  if (derivative_orders.empty()) {
    return;
  }
  for (std::size_t i = 0; i < stages->size(); ++i) {
    CalibrationOptions &options = stages->at(i).options;
    if (derivative_orders.at(i) >= options.spline_order) {
      throw std::invalid_argument(
          "stage pose-motion order must be less than the spline order");
    }
    options.add_pose_motion_prior = true;
    options.pose_motion_derivative_order = derivative_orders.at(i);
  }
}

void applyStageTimeShiftPriorSigmas(
    const std::vector<double> &time_shift_prior_sigmas,
    std::vector<CalibrationStage> *stages) {
  if (!stages) {
    throw std::invalid_argument("stages must be non-null");
  }
  validateNonNegativeSchedule(time_shift_prior_sigmas, stages->size(),
                              "stage time-shift prior sigma");
  if (time_shift_prior_sigmas.empty()) {
    return;
  }
  for (std::size_t i = 0; i < stages->size(); ++i) {
    CalibrationOptions &options = stages->at(i).options;
    options.time_shift_prior_sigma_s = time_shift_prior_sigmas.at(i);
    options.add_time_shift_prior = options.time_shift_prior_sigma_s > 0.0;
  }
}

void applyStageSolverOptions(
    const std::vector<double> &initial_trust_region_radii,
    const std::vector<double> &max_trust_region_radii,
    const std::vector<double> &min_trust_region_radii,
    const std::vector<double> &min_relative_decreases,
    const std::vector<double> &absolute_cost_change_tolerances,
    const std::vector<double> &absolute_step_tolerances,
    const std::vector<double> &absolute_parameter_tolerances,
    std::vector<CalibrationStage> *stages) {
  if (!stages) {
    throw std::invalid_argument("stages must be non-null");
  }
  validatePositiveSchedule(initial_trust_region_radii, stages->size(),
                           "stage solver initial trust region radius");
  validatePositiveSchedule(max_trust_region_radii, stages->size(),
                           "stage solver max trust region radius");
  validatePositiveSchedule(min_trust_region_radii, stages->size(),
                           "stage solver min trust region radius");
  validateNonNegativeSchedule(min_relative_decreases, stages->size(),
                              "stage solver min relative decrease");
  validateDisabledOrNonNegativeSchedule(
      absolute_cost_change_tolerances, stages->size(),
      "stage solver absolute cost-change tolerance");
  validateDisabledOrNonNegativeSchedule(absolute_step_tolerances,
                                        stages->size(),
                                        "stage solver absolute step tolerance");
  validateDisabledOrNonNegativeSchedule(
      absolute_parameter_tolerances, stages->size(),
      "stage solver absolute parameter tolerance");
  if (initial_trust_region_radii.empty() && max_trust_region_radii.empty() &&
      min_trust_region_radii.empty() && min_relative_decreases.empty() &&
      absolute_cost_change_tolerances.empty() &&
      absolute_step_tolerances.empty() &&
      absolute_parameter_tolerances.empty()) {
    return;
  }
  for (std::size_t i = 0; i < stages->size(); ++i) {
    CalibrationOptions &options = stages->at(i).options;
    if (!initial_trust_region_radii.empty()) {
      options.solver_initial_trust_region_radius =
          initial_trust_region_radii.at(i);
    }
    if (!max_trust_region_radii.empty()) {
      options.solver_max_trust_region_radius = max_trust_region_radii.at(i);
    }
    if (!min_trust_region_radii.empty()) {
      options.solver_min_trust_region_radius = min_trust_region_radii.at(i);
    }
    if (!min_relative_decreases.empty()) {
      options.solver_min_relative_decrease = min_relative_decreases.at(i);
    }
    if (!absolute_cost_change_tolerances.empty()) {
      options.solver_absolute_cost_change_tolerance =
          absolute_cost_change_tolerances.at(i);
    }
    if (!absolute_step_tolerances.empty()) {
      options.solver_absolute_step_tolerance = absolute_step_tolerances.at(i);
    }
    if (!absolute_parameter_tolerances.empty()) {
      options.solver_absolute_parameter_tolerance =
          absolute_parameter_tolerances.at(i);
    }
  }
  validateTrustRegionBounds(*stages);
}

CalibrationStageResult
solveCalibrationStage(const CameraIntrinsics &intrinsics,
                      const ImuNoise &imu_noise,
                      const std::vector<ImageObservation> &images,
                      const std::vector<ImuSample> &imu_samples,
                      const CalibrationStage &stage, CalibrationState *state) {
  if (!state) {
    throw std::invalid_argument("state must be non-null");
  }
  ceres::Problem problem;
  CalibrationStageResult result;
  result.name = stage.name;
  const CalibrationStateSnapshot state_before_stage =
      snapshotCalibrationState(*state);
  CalibrationOptions options = stage.options;
  if ((options.trace_iteration_state ||
       options.solver_absolute_cost_change_tolerance >= 0.0 ||
       options.solver_absolute_step_tolerance >= 0.0 ||
       options.solver_absolute_parameter_tolerance >= 0.0) &&
      options.trace_label.empty()) {
    options.trace_label = stage.name;
  }
  result.build = buildCalibrationProblem(intrinsics, imu_noise, images,
                                         imu_samples, options, state, &problem);
  result.solver = solveCalibrationProblem(options, state, &problem);
  result.state_decision = decideCalibrationStageStateUpdate(result.solver);
  result.state_restored =
      result.state_decision != CalibrationStageStateDecision::kAccepted;
  result.state_cost_change =
      hasFiniteStageCosts(result.solver)
          ? result.solver.final_cost - result.solver.initial_cost
          : std::numeric_limits<double>::quiet_NaN();
  if (result.state_restored) {
    restoreCalibrationState(state_before_stage, state);
  }
  return result;
}

std::vector<CalibrationStageResult> solveCalibrationStages(
    const CameraIntrinsics &intrinsics, const ImuNoise &imu_noise,
    const std::vector<ImageObservation> &images,
    const std::vector<ImuSample> &imu_samples,
    const std::vector<CalibrationStage> &stages, CalibrationState *state) {
  std::vector<CalibrationStageResult> results;
  results.reserve(stages.size());
  for (const CalibrationStage &stage : stages) {
    results.push_back(solveCalibrationStage(intrinsics, imu_noise, images,
                                            imu_samples, stage, state));
  }
  return results;
}

} // namespace ceres_cam_imu
