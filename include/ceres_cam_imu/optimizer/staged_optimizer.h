#pragma once

#include <string>
#include <vector>

#include <ceres/ceres.h>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/optimizer/calibration_problem.h"

namespace ceres_cam_imu {

struct CalibrationStage {
  std::string name;
  CalibrationOptions options;
};

enum class CalibrationStageStateDecision {
  kAccepted,
  kRestoredSolverFailure,
  kRestoredCostIncrease,
  kRestoredNonFiniteCost,
};

struct CalibrationStageResult {
  std::string name;
  CalibrationBuildSummary build;
  ceres::Solver::Summary solver;
  CalibrationStageStateDecision state_decision =
      CalibrationStageStateDecision::kAccepted;
  bool state_restored = false;
  double state_cost_change = 0.0;
};

const char *
calibrationStageStateDecisionName(CalibrationStageStateDecision decision);

CalibrationStageStateDecision
decideCalibrationStageStateUpdate(const ceres::Solver::Summary &summary);

std::vector<CalibrationStage>
makeConservativeCalibrationStages(const CalibrationOptions &base_options);

std::vector<CalibrationStage>
makeConservativeCalibrationStages(const CalibrationOptions &base_options,
                                  const std::vector<int> &stage_iterations);

std::vector<CalibrationStage> makeCalibrationStagesFromFreeMasks(
    const CalibrationOptions &base_options,
    const std::vector<int> &stage_iterations,
    const std::vector<std::string> &stage_free_masks);

void applyStagePoseMotionVariances(
    const std::vector<double> &translation_variances,
    const std::vector<double> &rotation_variances,
    std::vector<CalibrationStage> *stages);

void applyStagePoseMotionOrders(const std::vector<int> &derivative_orders,
                                std::vector<CalibrationStage> *stages);

void applyStageTimeShiftPriorSigmas(
    const std::vector<double> &time_shift_prior_sigmas,
    std::vector<CalibrationStage> *stages);

void applyStageSolverOptions(
    const std::vector<double> &initial_trust_region_radii,
    const std::vector<double> &max_trust_region_radii,
    const std::vector<double> &min_trust_region_radii,
    const std::vector<double> &min_relative_decreases,
    const std::vector<double> &absolute_cost_change_tolerances,
    const std::vector<double> &absolute_step_tolerances,
    const std::vector<double> &absolute_parameter_tolerances,
    std::vector<CalibrationStage> *stages);

CalibrationStageResult
solveCalibrationStage(const CameraIntrinsics &intrinsics,
                      const ImuNoise &imu_noise,
                      const std::vector<ImageObservation> &images,
                      const std::vector<ImuSample> &imu_samples,
                      const CalibrationStage &stage, CalibrationState *state);

std::vector<CalibrationStageResult> solveCalibrationStages(
    const CameraIntrinsics &intrinsics, const ImuNoise &imu_noise,
    const std::vector<ImageObservation> &images,
    const std::vector<ImuSample> &imu_samples,
    const std::vector<CalibrationStage> &stages, CalibrationState *state);

} // namespace ceres_cam_imu
