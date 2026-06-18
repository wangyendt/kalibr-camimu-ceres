#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ceres/ceres.h>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/initialization/orientation_gravity_initializer.h"
#include "ceres_cam_imu/initialization/time_shift_initializer.h"
#include "ceres_cam_imu/io/calibration_result_reader.h"
#include "ceres_cam_imu/io/calibration_result_writer.h"
#include "ceres_cam_imu/io/config_reader.h"
#include "ceres_cam_imu/io/corner_csv_reader.h"
#include "ceres_cam_imu/io/imu_csv_reader.h"
#include "ceres_cam_imu/io/kalibr_result_parser.h"
#include "ceres_cam_imu/io/pose_csv_reader.h"
#include "ceres_cam_imu/optimizer/calibration_problem.h"
#include "ceres_cam_imu/optimizer/residual_statistics.h"
#include "ceres_cam_imu/optimizer/staged_optimizer.h"
#include "ceres_cam_imu/processing/dataset_processing.h"
#include "ceres_cam_imu/trajectory/spline_eval.h"
#include "ceres_cam_imu/variables/imu_intrinsics.h"

namespace {

std::string argValue(int argc, char **argv, const std::string &name,
                     const std::string &default_value = "") {
  const std::string prefix = name + "=";
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      return argv[i + 1];
    }
    const std::string arg = argv[i];
    if (arg.rfind(prefix, 0) == 0) {
      return arg.substr(prefix.size());
    }
  }
  if (argc > 1) {
    const std::string arg = argv[argc - 1];
    if (arg.rfind(prefix, 0) == 0) {
      return arg.substr(prefix.size());
    }
  }
  return default_value;
}

std::vector<std::string> argValues(int argc, char **argv,
                                   const std::string &name) {
  std::vector<std::string> values;
  const std::string prefix = name + "=";
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      values.emplace_back(argv[i + 1]);
      continue;
    }
    const std::string arg = argv[i];
    if (arg.rfind(prefix, 0) == 0) {
      values.emplace_back(arg.substr(prefix.size()));
    }
  }
  if (argc > 1) {
    const std::string arg = argv[argc - 1];
    if (arg.rfind(prefix, 0) == 0) {
      values.emplace_back(arg.substr(prefix.size()));
    }
  }
  return values;
}

bool hasFlag(int argc, char **argv, const std::string &name) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == name) {
      return true;
    }
  }
  return false;
}

int intArg(int argc, char **argv, const std::string &name, int default_value) {
  const std::string value = argValue(argc, argv, name);
  return value.empty() ? default_value : std::stoi(value);
}

double doubleArg(int argc, char **argv, const std::string &name,
                 double default_value) {
  const std::string value = argValue(argc, argv, name);
  return value.empty() ? default_value : std::stod(value);
}

ceres_cam_imu::RobustLossType parseLossType(const std::string &value) {
  if (value.empty() || value == "cauchy") {
    return ceres_cam_imu::RobustLossType::kCauchy;
  }
  if (value == "huber") {
    return ceres_cam_imu::RobustLossType::kHuber;
  }
  if (value == "none") {
    return ceres_cam_imu::RobustLossType::kNone;
  }
  throw std::invalid_argument("unknown robust loss type: " + value);
}

ceres::LinearSolverType
parseLinearSolverType(const std::string &value,
                      ceres::LinearSolverType fallback) {
  if (value.empty()) {
    return fallback;
  }
  std::string normalized;
  normalized.reserve(value.size());
  for (const char ch : value) {
    normalized.push_back(ch == '-' ? '_'
                                   : static_cast<char>(std::toupper(
                                         static_cast<unsigned char>(ch))));
  }
  ceres::LinearSolverType type = fallback;
  if (!ceres::StringToLinearSolverType(normalized, &type)) {
    throw std::invalid_argument("unknown Ceres linear solver type: " + value);
  }
  return type;
}

void appendDoubleList(const std::string &text, std::vector<double> *values) {
  if (!values) {
    return;
  }
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!token.empty()) {
      values->push_back(std::stod(token));
    }
  }
}

std::vector<int> parseIntList(const std::string &text) {
  std::vector<int> values;
  if (text.empty()) {
    return values;
  }
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!token.empty()) {
      values.push_back(std::stoi(token));
    }
  }
  return values;
}

std::vector<double> parseDoubleList(const std::string &text) {
  std::vector<double> values;
  if (text.empty()) {
    return values;
  }
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!token.empty()) {
      values.push_back(std::stod(token));
    }
  }
  return values;
}

std::string trimAscii(const std::string &value) {
  std::size_t begin = 0;
  while (begin < value.size() &&
         std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

std::vector<std::string> parseStringList(const std::string &text) {
  std::vector<std::string> values;
  if (text.empty()) {
    return values;
  }
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    token = trimAscii(token);
    if (!token.empty()) {
      values.push_back(token);
    }
  }
  return values;
}

void usage() {
  std::cout
      << "calibrate_cam_imu --cam camchain.yaml --imu imu.yaml --target "
         "aprilgrid.yaml "
         "--imu-data data.csv --corners corners.csv [--kalibr-result "
         "result.txt] "
         "[--corner-poses poses.csv] [--init-from-kalibr] [--init-from-camchain] "
         "[--init-from-result result.yaml] "
         "[--corner-defaults] "
         "[--dry-run] [--max-frames N] [--imu-stride N] "
         "[--max-imu-residuals N] [--imu-trim-edge-count N] "
         "[--max-iterations N] [--pose-kps K] [--bias-kps K] "
         "[--solver-function-tolerance V] [--solver-gradient-tolerance V] "
         "[--solver-parameter-tolerance V] "
         "[--solver-initial-trust-region-radius V] "
         "[--solver-max-trust-region-radius V] "
         "[--solver-min-trust-region-radius V] "
         "[--solver-min-relative-decrease V] "
         "[--solver-absolute-cost-change-tolerance V] "
         "[--solver-absolute-step-tolerance V] "
         "[--solver-absolute-parameter-tolerance V] "
         "[--solver-linear-solver TYPE] [--solver-num-threads N] "
         "[--solver-use-nonmonotonic-steps] "
         "[--solver-max-consecutive-nonmonotonic-steps N] "
         "[--trace-iteration-state] "
         "[--pose-fit-diagonal-lambda L] [--pose-fit-motion-lambda L] "
         "[--pose-fit-boundary-anchors] "
         "[--stage-iterations N0,N1,N2,N3] [--stage-free MASK[,MASK...]] "
         "[--stop-on-stage-failure] "
         "[--time-padding S|--timeoffset-padding S] "
         "[--fix-poses] [--fix-biases] [--fix-camera-extrinsic] "
         "[--fix-time-shift] [--fix-gravity] [--estimate-gravity-length] "
         "[--imu-model calibrated|scale-misalignment|"
         "scale-misalignment-size-effect] [--fix-imu-intrinsics] "
         "[--estimate-time-shift-prior] "
         "[--time-shift-pose-kps K] [--time-shift-fit-lambda L] "
         "[--estimate-orientation-gravity-prior] "
         "[--orientation-prior-pose-kps K] [--orientation-prior-fit-lambda L] "
         "[--no-orientation-prior-boundary-anchors] "
         "[--no-orientation-prior-ceres-refine] "
         "[--time-shift-prior-sigma S] [--pose-motion-prior] "
         "[--pose-motion-all-segments] "
         "[--camera-loss cauchy|huber|none] [--camera-loss-width W] "
         "[--gyro-loss cauchy|huber|none] [--gyro-loss-width W] "
         "[--accel-loss cauchy|huber|none] [--accel-loss-width W] "
         "[--pose-motion-order N] [--pose-motion-translation-variance V] "
         "[--pose-motion-rotation-variance V] "
         "[--stage-pose-translation-variances V0,V1,...] "
         "[--stage-pose-rotation-variances V0,V1,...] "
         "[--stage-pose-motion-orders N0,N1,...] "
         "[--stage-time-shift-prior-sigmas S0,S1,...] "
         "[--stage-solver-initial-trust-region-radii R0,R1,...] "
         "[--stage-solver-max-trust-region-radii R0,R1,...] "
         "[--stage-solver-min-trust-region-radii R0,R1,...] "
         "[--stage-solver-min-relative-decreases D0,D1,...] "
         "[--stage-solver-absolute-cost-change-tolerances J0,J1,...] "
         "[--stage-solver-absolute-step-tolerances X0,X1,...] "
         "[--stage-solver-absolute-parameter-tolerances X0,X1,...] "
         "[--pose-motion-local-center S] [--pose-motion-local-half-window S] "
         "[--pose-motion-local-translation-scale F] "
         "[--pose-motion-local-rotation-scale F] [--top-residuals N] "
         "[--inspect-time S] [--inspect-times S[,S...]] "
         "[--inspect-window S] [--output-result result.yaml] "
         "[--export-spline-controls] [--export-imu-diagnostics imu.csv] "
         "[--staged]\n";
  std::cout << "  --time-padding / --timeoffset-padding pads splines by 2*S "
               "on each side.\n";
  std::cout
      << "  --corner-defaults sets the standard corner-file defaults before "
         "parsing explicit overrides: pose/bias kps 100/50, max-iter 30, "
         "time padding 0.04, IMU edge trim 1000, Cauchy width 10, and "
         "production stopping: max-iter, absolute cost change 5e-2, "
         "max parameter delta 1e-2. "
         "--kalibr-corner-defaults is accepted as a deprecated alias.\n";
  std::cout
      << "  --pose-fit-motion-lambda adds Kalibr-style derivative-integral "
         "regularization to camera-pose spline initialization; "
         "--pose-fit-boundary-anchors duplicates the first/last camera pose at "
         "the padded spline boundaries.\n";
  std::cout
      << "  --estimate-time-shift-prior uses the Kalibr-style gyro-norm "
         "cross-correlation initializer: time-shift pose kps defaults to 100 "
         "and time-shift fit lambda defaults to 1e-4.\n";
  std::cout
      << "  --init-from-camchain reads T_cam_imu and timeshift_cam_imu from "
         "the --cam YAML when those keys are present.\n";
  std::cout
      << "  --estimate-orientation-gravity-prior estimates camera-IMU "
         "rotation, gyro bias, and gravity from camera-pose angular velocity "
         "and IMU samples before building the main problem.\n";
  std::cout << "  --solver-linear-solver accepts Ceres names such as "
               "SPARSE_NORMAL_CHOLESKY, DENSE_QR, DENSE_NORMAL_CHOLESKY, "
               "CGNR, SPARSE_SCHUR, or ITERATIVE_SCHUR.\n";
  std::cout
      << "  --no-orientation-prior-boundary-anchors disables the duplicated "
         "first/last pose anchors used by Kalibr initPoseSplineFromCamera().\n";
  std::cout
      << "  --no-orientation-prior-ceres-refine keeps only the closed-form "
         "Wahba rotation/bias initializer and skips the Kalibr-style small "
         "rotation+bias refinement problem.\n";
  std::cout
      << "  --stage-free overrides the conservative staged preset. Each mask "
         "lists "
         "free variables: p=pose, b=bias, e=extrinsic, t=time, g=gravity; "
         "use '-' or 'none' for an evaluation-only stage.\n";
  std::cout
      << "  gravity uses a Kalibr-style fixed-norm direction manifold by "
         "default; "
         "--estimate-gravity-length switches it to a 3D Euclidean vector.\n";
  std::cout
      << "  --imu-model selects the IMU residual family. calibrated is the "
         "default Kalibr IccImu model; scale-misalignment adds "
         "lower-triangular "
         "accelerometer/gyro M matrices, gyro sensing rotation, and gyro "
         "g-sensitivity; scale-misalignment-size-effect also adds three "
         "accelerometer sensing-axis offsets. --fix-imu-intrinsics keeps those "
         "extra parameter blocks constant.\n";
  std::cout << "  --pose-motion-all-segments applies pose motion "
               "regularization to the "
               "full pose spline, matching Kalibr's BSplineMotionError scope. "
               "Without "
               "it, only data-touched pose segments receive this prior.\n";
  std::cout
      << "  --stage-time-shift-prior-sigmas overrides the time-shift prior "
         "strength per stage; use 0 to disable the prior in a stage.\n";
  std::cout
      << "  --stage-pose-motion-orders overrides pose motion derivative order "
         "per stage; values must be in [1, spline_order).\n";
  std::cout
      << "  --stage-solver-* overrides selected Ceres trust-region controls "
         "per stage while leaving unspecified solver options at their global "
         "values. For absolute tolerances, use -1 to disable a stage.\n";
  std::cout << "  --solver-absolute-cost-change-tolerance, "
               "--solver-absolute-step-tolerance, and "
               "--solver-absolute-parameter-tolerance add optional absolute "
               "stopping callbacks; the parameter tolerance scans active "
               "parameter blocks and is the closest Ceres-side analogue of "
               "Kalibr deltaX. Negative values disable them.\n";
  std::cout
      << "  --trace-iteration-state prints accepted Ceres iteration states; "
         "when --kalibr-result is present it also prints per-iteration deltas "
         "to the Kalibr result.\n";
}

void printBuildSummary(const std::string &prefix,
                       const ceres_cam_imu::CalibrationBuildSummary &build) {
  std::cout << prefix << "camera=" << build.camera_residuals
            << " gyro=" << build.gyro_residuals
            << " accel=" << build.accel_residuals
            << " gyro_priors=" << build.gyro_bias_priors
            << " accel_priors=" << build.accel_bias_priors
            << " pose_priors=" << build.pose_motion_priors
            << " local_pose_priors=" << build.local_pose_motion_priors
            << " time_priors=" << build.time_shift_priors
            << " gravity_tangent=" << build.gravity_tangent_size
            << " residual_blocks=" << build.residual_blocks
            << " scalar_residuals=" << build.scalar_residuals
            << " parameter_blocks=" << build.parameter_blocks
            << " active_parameter_blocks=" << build.active_parameter_blocks
            << " ambient_params=" << build.ambient_parameters
            << " tangent_params=" << build.tangent_parameters
            << " kalibr_style_error_terms=" << build.kalibr_style_error_terms
            << " skipped_camera_frames=" << build.skipped_camera_frames
            << " skipped_imu_samples=" << build.skipped_imu_samples << "\n";
}

void setLowerTriangularBlock(
    const ceres_cam_imu::Mat3 &matrix,
    ceres_cam_imu::LowerTriangularMatrixBlock *block) {
  if (!block) {
    return;
  }
  block->values[0] = matrix(0, 0);
  block->values[1] = matrix(1, 0);
  block->values[2] = matrix(1, 1);
  block->values[3] = matrix(2, 0);
  block->values[4] = matrix(2, 1);
  block->values[5] = matrix(2, 2);
}

void setMatrix3Block(const ceres_cam_imu::Mat3 &matrix,
                     ceres_cam_imu::Matrix3Block *block) {
  if (!block) {
    return;
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      block->values[static_cast<std::size_t>(r * 3 + c)] = matrix(r, c);
    }
  }
}

void setVector3Block(const ceres_cam_imu::Vec3 &value,
                     ceres_cam_imu::Vector3Block *block) {
  if (!block) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    block->values[static_cast<std::size_t>(i)] = value(i);
  }
}

void initializeImuIntrinsicsFromKalibr(
    const ceres_cam_imu::KalibrResult &kalibr,
    ceres_cam_imu::CalibrationState *state) {
  if (!state) {
    return;
  }
  int initialized_blocks = 0;
  if (kalibr.has_accel_M) {
    setLowerTriangularBlock(kalibr.accel_M, &state->imu_intrinsics.accel_M);
    ++initialized_blocks;
  }
  if (kalibr.has_gyro_M) {
    setLowerTriangularBlock(kalibr.gyro_M, &state->imu_intrinsics.gyro_M);
    ++initialized_blocks;
  }
  if (kalibr.has_gyro_accel_sensitivity) {
    setMatrix3Block(kalibr.gyro_accel_sensitivity,
                    &state->imu_intrinsics.gyro_accel_sensitivity);
    ++initialized_blocks;
  }
  if (kalibr.has_gyro_sensing_rotation) {
    setVector3Block(
        ceres_cam_imu::rotationMatrixToVector(kalibr.gyro_sensing_rotation),
        &state->imu_intrinsics.gyro_sensing_rotation);
    ++initialized_blocks;
  }
  if (kalibr.has_accel_axis_rx_i) {
    setVector3Block(kalibr.accel_axis_rx_i,
                    &state->imu_intrinsics.accel_axis_rx_i);
    ++initialized_blocks;
  }
  if (kalibr.has_accel_axis_ry_i) {
    setVector3Block(kalibr.accel_axis_ry_i,
                    &state->imu_intrinsics.accel_axis_ry_i);
    ++initialized_blocks;
  }
  if (kalibr.has_accel_axis_rz_i) {
    setVector3Block(kalibr.accel_axis_rz_i,
                    &state->imu_intrinsics.accel_axis_rz_i);
    ++initialized_blocks;
  }
  if (initialized_blocks > 0) {
    std::cout << "initialized IMU intrinsics from Kalibr: blocks="
              << initialized_blocks
              << " accel_M=" << (kalibr.has_accel_M ? 1 : 0)
              << " gyro_M=" << (kalibr.has_gyro_M ? 1 : 0)
              << " gyro_A="
              << (kalibr.has_gyro_accel_sensitivity ? 1 : 0)
              << " gyro_C=" << (kalibr.has_gyro_sensing_rotation ? 1 : 0)
              << " accel_size_effect="
              << (kalibr.has_accel_axis_rx_i && kalibr.has_accel_axis_ry_i &&
                          kalibr.has_accel_axis_rz_i
                      ? 1
                      : 0)
              << "\n";
  }
}

void initializeImuIntrinsicsFromResult(
    const ceres_cam_imu::CalibrationResultFile &result,
    ceres_cam_imu::CalibrationState *state) {
  if (!state) {
    return;
  }
  int initialized_blocks = 0;
  if (result.has_accel_M) {
    setLowerTriangularBlock(result.accel_M, &state->imu_intrinsics.accel_M);
    ++initialized_blocks;
  }
  if (result.has_gyro_M) {
    setLowerTriangularBlock(result.gyro_M, &state->imu_intrinsics.gyro_M);
    ++initialized_blocks;
  }
  if (result.has_gyro_accel_sensitivity) {
    setMatrix3Block(result.gyro_accel_sensitivity,
                    &state->imu_intrinsics.gyro_accel_sensitivity);
    ++initialized_blocks;
  }
  if (result.has_gyro_sensing_rotation) {
    setVector3Block(
        ceres_cam_imu::rotationMatrixToVector(result.gyro_sensing_rotation),
        &state->imu_intrinsics.gyro_sensing_rotation);
    ++initialized_blocks;
  }
  if (result.has_accel_axis_rx_i) {
    setVector3Block(result.accel_axis_rx_i,
                    &state->imu_intrinsics.accel_axis_rx_i);
    ++initialized_blocks;
  }
  if (result.has_accel_axis_ry_i) {
    setVector3Block(result.accel_axis_ry_i,
                    &state->imu_intrinsics.accel_axis_ry_i);
    ++initialized_blocks;
  }
  if (result.has_accel_axis_rz_i) {
    setVector3Block(result.accel_axis_rz_i,
                    &state->imu_intrinsics.accel_axis_rz_i);
    ++initialized_blocks;
  }
  if (initialized_blocks > 0) {
    std::cout << "initialized IMU intrinsics from Ceres result: blocks="
              << initialized_blocks
              << " accel_M=" << (result.has_accel_M ? 1 : 0)
              << " gyro_M=" << (result.has_gyro_M ? 1 : 0)
              << " gyro_A="
              << (result.has_gyro_accel_sensitivity ? 1 : 0)
              << " gyro_C=" << (result.has_gyro_sensing_rotation ? 1 : 0)
              << " accel_size_effect="
              << (result.has_accel_axis_rx_i && result.has_accel_axis_ry_i &&
                          result.has_accel_axis_rz_i
                      ? 1
                      : 0)
              << "\n";
  }
}

bool printFinalState(const ceres_cam_imu::CalibrationState &state,
                     const bool have_kalibr_result,
                     const ceres_cam_imu::KalibrResult &kalibr) {
  const ceres_cam_imu::Mat4 final_T_c_b =
      ceres_cam_imu::pose6ToMatrix(state.T_c_b);
  std::cout << "T_c_b:\n" << final_T_c_b << "\n";
  std::cout << "time_shift_s: " << state.camera_time_shift_s.value << "\n";
  std::cout << "gravity: " << state.gravity.values[0] << " "
            << state.gravity.values[1] << " " << state.gravity.values[2]
            << "\n";

  if (have_kalibr_result) {
    const ceres_cam_imu::Mat3 dR = final_T_c_b.block<3, 3>(0, 0) *
                                   kalibr.T_ci.block<3, 3>(0, 0).transpose();
    const double cos_angle = std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0);
    constexpr double kPi = 3.14159265358979323846;
    const double angle_deg = std::acos(cos_angle) * 180.0 / kPi;
    const double trans_delta_m =
        (final_T_c_b.block<3, 1>(0, 3) - kalibr.T_ci.block<3, 1>(0, 3)).norm();
    const ceres_cam_imu::Vec3 gravity(state.gravity.values[0],
                                      state.gravity.values[1],
                                      state.gravity.values[2]);
    std::cout << "kalibr_delta: rotation_deg=" << angle_deg
              << " translation_m=" << trans_delta_m << " time_shift_s="
              << (state.camera_time_shift_s.value -
                  kalibr.timeshift_cam_to_imu_s)
              << " gravity_norm=" << (gravity - kalibr.gravity).norm() << "\n";
  }
  return true;
}

void printOneResidualStat(const std::string &name,
                          const ceres_cam_imu::ResidualMagnitudeStats &stats) {
  std::cout << "residual_stats " << name << ": count=" << stats.count
            << " mean=" << stats.mean << " median=" << stats.median
            << " std=" << stats.stddev << " rms=" << stats.rms
            << " max=" << stats.max << "\n";
}

void printResidualStatistics(
    const ceres_cam_imu::CalibrationResidualStatistics &stats) {
  printOneResidualStat("reprojection_px", stats.reprojection_px);
  printOneResidualStat("reprojection_normalized",
                       stats.reprojection_normalized);
  printOneResidualStat("gyro_rad_s", stats.gyro_rad_s);
  printOneResidualStat("gyro_normalized", stats.gyro_normalized);
  printOneResidualStat("accel_m_s2", stats.accel_m_s2);
  printOneResidualStat("accel_normalized", stats.accel_normalized);
  std::cout << "residual_stats skipped: camera_frames="
            << stats.skipped_camera_frames
            << " camera_projections=" << stats.skipped_camera_projections
            << " imu_samples=" << stats.skipped_imu_samples << "\n";
  int rank = 1;
  for (const ceres_cam_imu::ImuResidualOutlier &outlier :
       stats.top_accel_outliers) {
    std::cout << "residual_outlier accel rank=" << rank
              << " sample_index=" << outlier.sample_index
              << " timestamp_s=" << outlier.timestamp_s
              << " accel_m_s2=" << outlier.accel_error_m_s2
              << " accel_normalized=" << outlier.accel_normalized
              << " gyro_rad_s=" << outlier.gyro_error_rad_s
              << " gyro_normalized=" << outlier.gyro_normalized
              << " measured_accel_norm=" << outlier.measured_accel_norm
              << " predicted_accel_norm=" << outlier.predicted_accel_norm
              << " pose_accel_world_norm=" << outlier.pose_accel_world_norm
              << " gravity_corrected_body_accel_norm="
              << outlier.gravity_corrected_body_accel_norm
              << " angular_accel_lever_norm="
              << outlier.angular_accel_lever_norm
              << " centripetal_lever_norm=" << outlier.centripetal_lever_norm
              << " omega_body_norm=" << outlier.omega_body_norm
              << " alpha_body_norm=" << outlier.alpha_body_norm << "\n";
    ++rank;
  }
}

ceres_cam_imu::CalibrationResidualStatistics printFinalResidualStatistics(
    const ceres_cam_imu::CameraIntrinsics &intrinsics,
    const ceres_cam_imu::ImuNoise &imu_noise,
    const std::vector<ceres_cam_imu::ImageObservation> &images,
    const std::vector<ceres_cam_imu::ImuSample> &imu_samples,
    const ceres_cam_imu::CalibrationOptions &options,
    const ceres_cam_imu::CalibrationState &state) {
  const ceres_cam_imu::CalibrationResidualStatistics stats =
      ceres_cam_imu::evaluateCalibrationResidualStatistics(
          intrinsics, imu_noise, images, imu_samples, options, state);
  printResidualStatistics(stats);
  return stats;
}

ceres_cam_imu::CalibrationResidualStatistics printFinalResidualStatistics(
    const std::vector<ceres_cam_imu::CameraObservationDataset> &cameras,
    const ceres_cam_imu::ImuNoise &imu_noise,
    const std::vector<ceres_cam_imu::ImuSample> &imu_samples,
    const ceres_cam_imu::CalibrationOptions &options,
    const ceres_cam_imu::CalibrationState &state) {
  const ceres_cam_imu::CalibrationResidualStatistics stats =
      ceres_cam_imu::evaluateCalibrationResidualStatistics(
          cameras, imu_noise, imu_samples, options, state);
  printResidualStatistics(stats);
  return stats;
}

ceres_cam_imu::Vec6
poseControlVec(const ceres_cam_imu::PoseControlBlock &control) {
  return Eigen::Map<const ceres_cam_imu::Vec6>(control.data());
}

void printNearbyCameraPoses(
    const std::vector<ceres_cam_imu::PoseObservation> &poses,
    const double inspect_time_s, const double camera_time_s,
    const double time_shift_s) {
  if (poses.empty()) {
    std::cout << "time_inspect camera_pose_window: no_pose_csv\n";
    return;
  }
  const auto lower = std::lower_bound(
      poses.begin(), poses.end(), camera_time_s,
      [](const ceres_cam_imu::PoseObservation &pose, const double timestamp_s) {
        return pose.timestamp_s < timestamp_s;
      });
  const int center = static_cast<int>(lower - poses.begin());
  const int first = std::max(0, center - 3);
  const int last = std::min(static_cast<int>(poses.size()), center + 4);
  for (int i = first; i < last; ++i) {
    const ceres_cam_imu::PoseObservation &pose =
        poses[static_cast<std::size_t>(i)];
    const double query_time_s = pose.timestamp_s + time_shift_s;
    std::cout << "time_inspect camera_pose index=" << i
              << " camera_timestamp_s=" << pose.timestamp_s
              << " query_timestamp_s=" << query_time_s
              << " delta_camera_s=" << (pose.timestamp_s - camera_time_s)
              << " delta_query_s=" << (query_time_s - inspect_time_s) << "\n";
  }
}

void printNearbyImuSamples(
    const std::vector<ceres_cam_imu::ImuSample> &imu_samples,
    const double inspect_time_s, const double window_s) {
  const double first_time = inspect_time_s - window_s;
  const double last_time = inspect_time_s + window_s;
  int printed = 0;
  for (std::size_t i = 0; i < imu_samples.size(); ++i) {
    const ceres_cam_imu::ImuSample &sample = imu_samples[i];
    if (sample.timestamp_s < first_time) {
      continue;
    }
    if (sample.timestamp_s > last_time) {
      break;
    }
    std::cout << "time_inspect imu_sample index=" << i
              << " timestamp_s=" << sample.timestamp_s
              << " delta_s=" << (sample.timestamp_s - inspect_time_s)
              << " gyro_norm=" << sample.gyro_rad_s.norm()
              << " accel_norm=" << sample.accel_m_s2.norm() << "\n";
    ++printed;
  }
  std::cout << "time_inspect imu_window count=" << printed
            << " half_width_s=" << window_s << "\n";
}

void printLocalTimeDiagnostics(
    const double inspect_time_s, const double window_s,
    const std::vector<ceres_cam_imu::PoseObservation> &poses,
    const std::vector<ceres_cam_imu::ImuSample> &imu_samples,
    const ceres_cam_imu::CalibrationState &state) {
  const double camera_time_s = inspect_time_s - state.camera_time_shift_s.value;
  std::cout << "time_inspect begin timestamp_s=" << inspect_time_s
            << " camera_timestamp_s=" << camera_time_s
            << " time_shift_s=" << state.camera_time_shift_s.value
            << " window_s=" << window_s << "\n";
  if (!state.pose_spline.isValidTime(inspect_time_s)) {
    std::cout << "time_inspect pose_spline: invalid_time"
              << " t_min=" << state.pose_spline.tMin()
              << " t_max=" << state.pose_spline.tMax() << "\n";
    printNearbyCameraPoses(poses, inspect_time_s, camera_time_s,
                           state.camera_time_shift_s.value);
    printNearbyImuSamples(imu_samples, inspect_time_s, window_s);
    return;
  }

  const ceres_cam_imu::SplineSegmentMeta6 meta =
      state.pose_spline.segmentMeta6(inspect_time_s);
  const std::array<double, 6> weights0 = meta.weights(inspect_time_s, 0);
  const std::array<double, 6> weights1 = meta.weights(inspect_time_s, 1);
  const std::array<double, 6> weights2 = meta.weights(inspect_time_s, 2);
  std::array<const double *, 6> active{};
  for (int i = 0; i < 6; ++i) {
    active[static_cast<std::size_t>(i)] =
        state.pose_controls.at(static_cast<std::size_t>(meta.coeff_start + i))
            .data();
  }
  const ceres_cam_imu::Vec6 pose =
      ceres_cam_imu::evalPoseCurve6(meta, inspect_time_s, active, 0);
  const ceres_cam_imu::Vec6 pose_dot =
      ceres_cam_imu::evalPoseCurve6(meta, inspect_time_s, active, 1);
  const ceres_cam_imu::Vec6 pose_ddot =
      ceres_cam_imu::evalPoseCurve6(meta, inspect_time_s, active, 2);

  std::cout << "time_inspect pose_spline coeff_start=" << meta.coeff_start
            << " segment_start_s=" << meta.segment_start_s
            << " segment_end_s=" << (meta.segment_start_s + meta.dt_s)
            << " dt_s=" << meta.dt_s
            << " u=" << ((inspect_time_s - meta.segment_start_s) / meta.dt_s)
            << " translation_norm=" << pose.head<3>().norm()
            << " rotation_norm=" << pose.tail<3>().norm()
            << " velocity_norm=" << pose_dot.head<3>().norm()
            << " rotation_rate_param_norm=" << pose_dot.tail<3>().norm()
            << " accel_norm=" << pose_ddot.head<3>().norm()
            << " rotation_accel_param_norm=" << pose_ddot.tail<3>().norm()
            << "\n";

  for (int i = 0; i < 6; ++i) {
    const int coeff_index = meta.coeff_start + i;
    const ceres_cam_imu::Vec6 control = poseControlVec(
        state.pose_controls.at(static_cast<std::size_t>(coeff_index)));
    std::cout << "time_inspect pose_control local=" << i
              << " coeff_index=" << coeff_index
              << " w0=" << weights0[static_cast<std::size_t>(i)]
              << " w1=" << weights1[static_cast<std::size_t>(i)]
              << " w2=" << weights2[static_cast<std::size_t>(i)]
              << " t_norm=" << control.head<3>().norm()
              << " r_norm=" << control.tail<3>().norm() << " tx=" << control(0)
              << " ty=" << control(1) << " tz=" << control(2)
              << " rx=" << control(3) << " ry=" << control(4)
              << " rz=" << control(5) << "\n";
  }

  printNearbyCameraPoses(poses, inspect_time_s, camera_time_s,
                         state.camera_time_shift_s.value);
  printNearbyImuSamples(imu_samples, inspect_time_s, window_s);
  std::cout << "time_inspect end\n";
}

} // namespace

int main(int argc, char **argv) {
  if (hasFlag(argc, argv, "--help")) {
    usage();
    return 0;
  }

  const std::string cam_yaml = argValue(argc, argv, "--cam");
  const std::string imu_yaml = argValue(argc, argv, "--imu");
  const std::string target_yaml = argValue(argc, argv, "--target");
  const std::string imu_data = argValue(argc, argv, "--imu-data");
  const std::string corners_csv = argValue(argc, argv, "--corners");
  if (cam_yaml.empty() || imu_yaml.empty() || target_yaml.empty() ||
      imu_data.empty() || corners_csv.empty()) {
    usage();
    return 2;
  }

  const bool corner_defaults = hasFlag(argc, argv, "--corner-defaults") ||
                               hasFlag(argc, argv, "--kalibr-corner-defaults");
  ceres_cam_imu::CalibrationOptions options;
  if (corner_defaults) {
    options.pose_knots_per_second = 100.0;
    options.bias_knots_per_second = 50.0;
    options.time_padding_s = 0.04;
    options.max_iterations = 30;
    options.solver_absolute_cost_change_tolerance = 5e-2;
    options.solver_absolute_parameter_tolerance = 1e-2;
    options.camera_loss_type = ceres_cam_imu::RobustLossType::kCauchy;
    options.gyro_loss_type = ceres_cam_imu::RobustLossType::kCauchy;
    options.accel_loss_type = ceres_cam_imu::RobustLossType::kCauchy;
    options.camera_loss_width = 10.0;
    options.gyro_loss_width = 10.0;
    options.accel_loss_width = 10.0;
  }
  options.max_frames = intArg(argc, argv, "--max-frames", 0);
  options.imu_stride = intArg(argc, argv, "--imu-stride", 1);
  options.max_imu_residuals = intArg(argc, argv, "--max-imu-residuals", 0);
  options.max_iterations =
      intArg(argc, argv, "--max-iterations", options.max_iterations);
  if (options.max_iterations < 0) {
    std::cerr << "--max-iterations must be non-negative\n";
    return 2;
  }
  options.solver_function_tolerance =
      doubleArg(argc, argv, "--solver-function-tolerance",
                options.solver_function_tolerance);
  options.solver_gradient_tolerance =
      doubleArg(argc, argv, "--solver-gradient-tolerance",
                options.solver_gradient_tolerance);
  options.solver_parameter_tolerance =
      doubleArg(argc, argv, "--solver-parameter-tolerance",
                options.solver_parameter_tolerance);
  options.solver_initial_trust_region_radius =
      doubleArg(argc, argv, "--solver-initial-trust-region-radius",
                options.solver_initial_trust_region_radius);
  options.solver_max_trust_region_radius =
      doubleArg(argc, argv, "--solver-max-trust-region-radius",
                options.solver_max_trust_region_radius);
  options.solver_min_trust_region_radius =
      doubleArg(argc, argv, "--solver-min-trust-region-radius",
                options.solver_min_trust_region_radius);
  options.solver_min_relative_decrease =
      doubleArg(argc, argv, "--solver-min-relative-decrease",
                options.solver_min_relative_decrease);
  options.solver_absolute_cost_change_tolerance =
      doubleArg(argc, argv, "--solver-absolute-cost-change-tolerance",
                options.solver_absolute_cost_change_tolerance);
  options.solver_absolute_step_tolerance =
      doubleArg(argc, argv, "--solver-absolute-step-tolerance",
                options.solver_absolute_step_tolerance);
  options.solver_absolute_parameter_tolerance =
      doubleArg(argc, argv, "--solver-absolute-parameter-tolerance",
                options.solver_absolute_parameter_tolerance);
  options.solver_num_threads =
      intArg(argc, argv, "--solver-num-threads", options.solver_num_threads);
  options.solver_max_consecutive_nonmonotonic_steps =
      intArg(argc, argv, "--solver-max-consecutive-nonmonotonic-steps",
             options.solver_max_consecutive_nonmonotonic_steps);
  options.solver_use_nonmonotonic_steps =
      hasFlag(argc, argv, "--solver-use-nonmonotonic-steps");
  options.solver_linear_solver_type =
      parseLinearSolverType(argValue(argc, argv, "--solver-linear-solver"),
                            options.solver_linear_solver_type);
  options.trace_iteration_state =
      hasFlag(argc, argv, "--trace-iteration-state");
  if (options.solver_function_tolerance < 0.0 ||
      options.solver_gradient_tolerance < 0.0 ||
      options.solver_parameter_tolerance < 0.0 ||
      !(options.solver_initial_trust_region_radius > 0.0) ||
      !(options.solver_max_trust_region_radius > 0.0) ||
      !(options.solver_min_trust_region_radius > 0.0) ||
      options.solver_min_relative_decrease < 0.0 ||
      options.solver_absolute_cost_change_tolerance < -1.0 ||
      options.solver_absolute_step_tolerance < -1.0 ||
      options.solver_absolute_parameter_tolerance < -1.0 ||
      options.solver_num_threads <= 0 ||
      options.solver_max_consecutive_nonmonotonic_steps < 0 ||
      options.solver_min_trust_region_radius >
          options.solver_max_trust_region_radius) {
    std::cerr << "solver tolerances/decrease must be non-negative, optional "
                 "absolute tolerances must be -1 or non-negative, trust "
                 "region radii and thread count must be positive, and min "
                 "trust region radius must not exceed max radius\n";
    return 2;
  }
  std::cout
      << "solver options: linear_solver="
      << ceres::LinearSolverTypeToString(options.solver_linear_solver_type)
      << " num_threads=" << options.solver_num_threads
      << " initial_trust_region_radius="
      << options.solver_initial_trust_region_radius
      << " max_trust_region_radius=" << options.solver_max_trust_region_radius
      << " min_trust_region_radius=" << options.solver_min_trust_region_radius
      << " min_relative_decrease=" << options.solver_min_relative_decrease
      << " absolute_cost_change_tolerance="
      << options.solver_absolute_cost_change_tolerance
      << " absolute_step_tolerance=" << options.solver_absolute_step_tolerance
      << " absolute_parameter_tolerance="
      << options.solver_absolute_parameter_tolerance
      << " use_nonmonotonic_steps=" << options.solver_use_nonmonotonic_steps
      << " max_consecutive_nonmonotonic_steps="
      << options.solver_max_consecutive_nonmonotonic_steps << "\n";
  options.pose_knots_per_second =
      doubleArg(argc, argv, "--pose-kps", options.pose_knots_per_second);
  options.bias_knots_per_second =
      doubleArg(argc, argv, "--bias-kps", options.bias_knots_per_second);
  const std::string time_padding_arg = argValue(argc, argv, "--time-padding");
  const std::string kalibr_time_padding_arg =
      argValue(argc, argv, "--timeoffset-padding");
  if (!time_padding_arg.empty() && !kalibr_time_padding_arg.empty() &&
      time_padding_arg != kalibr_time_padding_arg) {
    std::cerr << "--time-padding and --timeoffset-padding disagree\n";
    return 2;
  }
  const std::string selected_time_padding =
      !time_padding_arg.empty() ? time_padding_arg : kalibr_time_padding_arg;
  if (!selected_time_padding.empty()) {
    options.time_padding_s = std::stod(selected_time_padding);
  }
  options.pose_fit_diagonal_regularization =
      doubleArg(argc, argv, "--pose-fit-diagonal-lambda",
                options.pose_fit_diagonal_regularization);
  options.pose_fit_motion_regularization =
      doubleArg(argc, argv, "--pose-fit-motion-lambda",
                options.pose_fit_motion_regularization);
  options.pose_fit_add_boundary_anchors =
      hasFlag(argc, argv, "--pose-fit-boundary-anchors");
  if (options.pose_fit_diagonal_regularization < 0.0 ||
      options.pose_fit_motion_regularization < 0.0) {
    std::cerr << "pose fit regularization values must be non-negative\n";
    return 2;
  }
  options.fix_pose_controls = hasFlag(argc, argv, "--fix-poses");
  options.fix_bias_controls = hasFlag(argc, argv, "--fix-biases");
  options.fix_camera_extrinsic = hasFlag(argc, argv, "--fix-camera-extrinsic");
  options.fix_time_shift = hasFlag(argc, argv, "--fix-time-shift");
  options.fix_gravity = hasFlag(argc, argv, "--fix-gravity");
  options.imu_model = ceres_cam_imu::parseImuCalibrationModel(
      argValue(argc, argv, "--imu-model", "calibrated"));
  options.fix_imu_intrinsics = hasFlag(argc, argv, "--fix-imu-intrinsics");
  options.estimate_gravity_length =
      hasFlag(argc, argv, "--estimate-gravity-length");
  options.camera_loss_type =
      parseLossType(argValue(argc, argv, "--camera-loss", "cauchy"));
  options.gyro_loss_type =
      parseLossType(argValue(argc, argv, "--gyro-loss", "cauchy"));
  options.accel_loss_type =
      parseLossType(argValue(argc, argv, "--accel-loss", "cauchy"));
  options.camera_loss_width =
      doubleArg(argc, argv, "--camera-loss-width", options.camera_loss_width);
  options.gyro_loss_width =
      doubleArg(argc, argv, "--gyro-loss-width", options.gyro_loss_width);
  options.accel_loss_width =
      doubleArg(argc, argv, "--accel-loss-width", options.accel_loss_width);
  if (options.camera_loss_width < 0.0 || options.gyro_loss_width < 0.0 ||
      options.accel_loss_width < 0.0) {
    std::cerr << "loss widths must be non-negative\n";
    return 2;
  }
  options.time_shift_prior_sigma_s =
      doubleArg(argc, argv, "--time-shift-prior-sigma", 0.0);
  options.add_time_shift_prior = options.time_shift_prior_sigma_s > 0.0;
  options.add_pose_motion_prior = hasFlag(argc, argv, "--pose-motion-prior");
  options.pose_motion_all_segments =
      hasFlag(argc, argv, "--pose-motion-all-segments");
  options.pose_motion_derivative_order = intArg(
      argc, argv, "--pose-motion-order", options.pose_motion_derivative_order);
  if (options.pose_motion_derivative_order <= 0 ||
      options.pose_motion_derivative_order >= options.spline_order) {
    std::cerr << "--pose-motion-order must be in [1, spline_order)\n";
    return 2;
  }
  options.pose_motion_translation_variance =
      doubleArg(argc, argv, "--pose-motion-translation-variance",
                options.pose_motion_translation_variance);
  options.pose_motion_rotation_variance =
      doubleArg(argc, argv, "--pose-motion-rotation-variance",
                options.pose_motion_rotation_variance);
  const std::string local_pose_center_arg =
      argValue(argc, argv, "--pose-motion-local-center");
  if (!local_pose_center_arg.empty()) {
    options.pose_motion_local_center_s = std::stod(local_pose_center_arg);
  }
  options.pose_motion_local_half_window_s =
      doubleArg(argc, argv, "--pose-motion-local-half-window", 0.0);
  options.pose_motion_local_translation_variance_scale =
      doubleArg(argc, argv, "--pose-motion-local-translation-scale", 1.0);
  options.pose_motion_local_rotation_variance_scale =
      doubleArg(argc, argv, "--pose-motion-local-rotation-scale", 1.0);
  options.add_pose_motion_local_scaling =
      !local_pose_center_arg.empty() &&
      options.pose_motion_local_half_window_s > 0.0;
  options.add_pose_motion_prior =
      options.add_pose_motion_prior || options.add_pose_motion_local_scaling;
  options.add_pose_motion_prior =
      options.add_pose_motion_prior || options.pose_motion_all_segments;
  if (options.pose_motion_local_translation_variance_scale <= 0.0 ||
      options.pose_motion_local_rotation_variance_scale <= 0.0) {
    std::cerr << "pose-motion local scales must be positive\n";
    return 2;
  }
  options.top_residuals =
      std::max(0, intArg(argc, argv, "--top-residuals", options.top_residuals));
  std::cout << "imu model: model="
            << ceres_cam_imu::imuCalibrationModelName(options.imu_model)
            << " fix_imu_intrinsics=" << options.fix_imu_intrinsics
            << " fix_accel_size_effect_rx=" << options.fix_accel_size_effect_rx
            << "\n";
  std::vector<double> inspect_times_s;
  for (const std::string &value : argValues(argc, argv, "--inspect-time")) {
    appendDoubleList(value, &inspect_times_s);
  }
  const std::string inspect_times_arg = argValue(argc, argv, "--inspect-times");
  if (!inspect_times_arg.empty()) {
    appendDoubleList(inspect_times_arg, &inspect_times_s);
  }
  const double inspect_window_s =
      doubleArg(argc, argv, "--inspect-window", 0.02);
  const std::string output_result_path =
      argValue(argc, argv, "--output-result");
  const bool export_spline_controls =
      hasFlag(argc, argv, "--export-spline-controls");
  const std::string imu_diagnostics_path =
      argValue(argc, argv, "--export-imu-diagnostics");
  const int imu_trim_edge_count =
      std::max(0, intArg(argc, argv, "--imu-trim-edge-count",
                         corner_defaults ? 1000 : 0));
  const bool staged = hasFlag(argc, argv, "--staged");
  const bool stop_on_stage_failure =
      hasFlag(argc, argv, "--stop-on-stage-failure");
  const std::vector<int> stage_iterations =
      parseIntList(argValue(argc, argv, "--stage-iterations"));
  const std::vector<std::string> stage_free_masks =
      parseStringList(argValue(argc, argv, "--stage-free"));
  const std::vector<double> stage_pose_translation_variances = parseDoubleList(
      argValue(argc, argv, "--stage-pose-translation-variances"));
  const std::vector<double> stage_pose_rotation_variances =
      parseDoubleList(argValue(argc, argv, "--stage-pose-rotation-variances"));
  const std::vector<int> stage_pose_motion_orders =
      parseIntList(argValue(argc, argv, "--stage-pose-motion-orders"));
  const std::vector<double> stage_time_shift_prior_sigmas =
      parseDoubleList(argValue(argc, argv, "--stage-time-shift-prior-sigmas"));
  const std::vector<double> stage_solver_initial_trust_region_radii =
      parseDoubleList(
          argValue(argc, argv, "--stage-solver-initial-trust-region-radii"));
  const std::vector<double> stage_solver_max_trust_region_radii =
      parseDoubleList(
          argValue(argc, argv, "--stage-solver-max-trust-region-radii"));
  const std::vector<double> stage_solver_min_trust_region_radii =
      parseDoubleList(
          argValue(argc, argv, "--stage-solver-min-trust-region-radii"));
  const std::vector<double> stage_solver_min_relative_decreases =
      parseDoubleList(
          argValue(argc, argv, "--stage-solver-min-relative-decreases"));
  const std::vector<double> stage_solver_absolute_cost_change_tolerances =
      parseDoubleList(argValue(
          argc, argv, "--stage-solver-absolute-cost-change-tolerances"));
  const std::vector<double> stage_solver_absolute_step_tolerances =
      parseDoubleList(
          argValue(argc, argv, "--stage-solver-absolute-step-tolerances"));
  const std::vector<double> stage_solver_absolute_parameter_tolerances =
      parseDoubleList(
          argValue(argc, argv, "--stage-solver-absolute-parameter-tolerances"));
  if (!stage_iterations.empty() && !staged) {
    std::cerr << "--stage-iterations requires --staged\n";
    return 2;
  }
  if (!stage_free_masks.empty() && !staged) {
    std::cerr << "--stage-free requires --staged\n";
    return 2;
  }
  if ((!stage_pose_translation_variances.empty() ||
       !stage_pose_rotation_variances.empty()) &&
      !staged) {
    std::cerr << "--stage-pose-*-variances require --staged\n";
    return 2;
  }
  if (!stage_pose_motion_orders.empty() && !staged) {
    std::cerr << "--stage-pose-motion-orders requires --staged\n";
    return 2;
  }
  if (!stage_time_shift_prior_sigmas.empty() && !staged) {
    std::cerr << "--stage-time-shift-prior-sigmas requires --staged\n";
    return 2;
  }
  if ((!stage_solver_initial_trust_region_radii.empty() ||
       !stage_solver_max_trust_region_radii.empty() ||
       !stage_solver_min_trust_region_radii.empty() ||
       !stage_solver_min_relative_decreases.empty() ||
       !stage_solver_absolute_cost_change_tolerances.empty() ||
       !stage_solver_absolute_step_tolerances.empty() ||
       !stage_solver_absolute_parameter_tolerances.empty()) &&
      !staged) {
    std::cerr << "--stage-solver-* options require --staged\n";
    return 2;
  }
  const std::size_t stage_count =
      stage_free_masks.empty() ? 4 : stage_free_masks.size();
  if (!stage_iterations.empty() && stage_iterations.size() != stage_count) {
    std::cerr << "--stage-iterations expects " << stage_count
              << " comma-separated values\n";
    return 2;
  }
  if (!stage_pose_translation_variances.empty() &&
      stage_pose_translation_variances.size() != stage_count) {
    std::cerr << "--stage-pose-translation-variances expects " << stage_count
              << " comma-separated values\n";
    return 2;
  }
  if (!stage_pose_rotation_variances.empty() &&
      stage_pose_rotation_variances.size() != stage_count) {
    std::cerr << "--stage-pose-rotation-variances expects " << stage_count
              << " comma-separated values\n";
    return 2;
  }
  if (!stage_pose_motion_orders.empty() &&
      stage_pose_motion_orders.size() != stage_count) {
    std::cerr << "--stage-pose-motion-orders expects " << stage_count
              << " comma-separated values\n";
    return 2;
  }
  if (!stage_time_shift_prior_sigmas.empty() &&
      stage_time_shift_prior_sigmas.size() != stage_count) {
    std::cerr << "--stage-time-shift-prior-sigmas expects " << stage_count
              << " comma-separated values\n";
    return 2;
  }
  if (!stage_solver_initial_trust_region_radii.empty() &&
      stage_solver_initial_trust_region_radii.size() != stage_count) {
    std::cerr << "--stage-solver-initial-trust-region-radii expects "
              << stage_count << " comma-separated values\n";
    return 2;
  }
  if (!stage_solver_max_trust_region_radii.empty() &&
      stage_solver_max_trust_region_radii.size() != stage_count) {
    std::cerr << "--stage-solver-max-trust-region-radii expects " << stage_count
              << " comma-separated values\n";
    return 2;
  }
  if (!stage_solver_min_trust_region_radii.empty() &&
      stage_solver_min_trust_region_radii.size() != stage_count) {
    std::cerr << "--stage-solver-min-trust-region-radii expects " << stage_count
              << " comma-separated values\n";
    return 2;
  }
  if (!stage_solver_min_relative_decreases.empty() &&
      stage_solver_min_relative_decreases.size() != stage_count) {
    std::cerr << "--stage-solver-min-relative-decreases expects " << stage_count
              << " comma-separated values\n";
    return 2;
  }
  if (!stage_solver_absolute_cost_change_tolerances.empty() &&
      stage_solver_absolute_cost_change_tolerances.size() != stage_count) {
    std::cerr << "--stage-solver-absolute-cost-change-tolerances expects "
              << stage_count << " comma-separated values\n";
    return 2;
  }
  if (!stage_solver_absolute_step_tolerances.empty() &&
      stage_solver_absolute_step_tolerances.size() != stage_count) {
    std::cerr << "--stage-solver-absolute-step-tolerances expects "
              << stage_count << " comma-separated values\n";
    return 2;
  }
  if (!stage_solver_absolute_parameter_tolerances.empty() &&
      stage_solver_absolute_parameter_tolerances.size() != stage_count) {
    std::cerr << "--stage-solver-absolute-parameter-tolerances expects "
              << stage_count << " comma-separated values\n";
    return 2;
  }
  for (const int iterations : stage_iterations) {
    if (iterations < 0) {
      std::cerr << "--stage-iterations values must be non-negative\n";
      return 2;
    }
  }
  for (const double variance : stage_pose_translation_variances) {
    if (!(variance > 0.0)) {
      std::cerr
          << "--stage-pose-translation-variances values must be positive\n";
      return 2;
    }
  }
  for (const double variance : stage_pose_rotation_variances) {
    if (!(variance > 0.0)) {
      std::cerr << "--stage-pose-rotation-variances values must be positive\n";
      return 2;
    }
  }
  for (const int order : stage_pose_motion_orders) {
    if (order <= 0 || order >= options.spline_order) {
      std::cerr
          << "--stage-pose-motion-orders values must be in [1, spline_order)\n";
      return 2;
    }
  }
  for (const double sigma : stage_time_shift_prior_sigmas) {
    if (sigma < 0.0) {
      std::cerr
          << "--stage-time-shift-prior-sigmas values must be non-negative\n";
      return 2;
    }
  }
  for (const double radius : stage_solver_initial_trust_region_radii) {
    if (!(radius > 0.0)) {
      std::cerr << "--stage-solver-initial-trust-region-radii values must be "
                   "positive\n";
      return 2;
    }
  }
  for (const double radius : stage_solver_max_trust_region_radii) {
    if (!(radius > 0.0)) {
      std::cerr
          << "--stage-solver-max-trust-region-radii values must be positive\n";
      return 2;
    }
  }
  for (const double radius : stage_solver_min_trust_region_radii) {
    if (!(radius > 0.0)) {
      std::cerr
          << "--stage-solver-min-trust-region-radii values must be positive\n";
      return 2;
    }
  }
  for (const double decrease : stage_solver_min_relative_decreases) {
    if (decrease < 0.0) {
      std::cerr << "--stage-solver-min-relative-decreases values must be "
                   "non-negative\n";
      return 2;
    }
  }
  for (const double tolerance : stage_solver_absolute_cost_change_tolerances) {
    if (tolerance < -1.0) {
      std::cerr << "--stage-solver-absolute-cost-change-tolerances values "
                   "must be -1 or non-negative\n";
      return 2;
    }
  }
  for (const double tolerance : stage_solver_absolute_step_tolerances) {
    if (tolerance < -1.0) {
      std::cerr << "--stage-solver-absolute-step-tolerances values must be -1 "
                   "or non-negative\n";
      return 2;
    }
  }
  for (const double tolerance : stage_solver_absolute_parameter_tolerances) {
    if (tolerance < -1.0) {
      std::cerr << "--stage-solver-absolute-parameter-tolerances values must "
                   "be -1 or non-negative\n";
      return 2;
    }
  }
  for (std::size_t i = 0; i < stage_count; ++i) {
    const double min_radius = stage_solver_min_trust_region_radii.empty()
                                  ? options.solver_min_trust_region_radius
                                  : stage_solver_min_trust_region_radii.at(i);
    const double max_radius = stage_solver_max_trust_region_radii.empty()
                                  ? options.solver_max_trust_region_radius
                                  : stage_solver_max_trust_region_radii.at(i);
    if (min_radius > max_radius) {
      std::cerr << "stage solver min trust region radius must not exceed "
                   "max radius\n";
      return 2;
    }
  }

  std::vector<std::string> cam_yamls = argValues(argc, argv, "--cam");
  std::vector<std::string> corner_csvs = argValues(argc, argv, "--corners");
  if (cam_yamls.empty()) {
    cam_yamls.push_back(cam_yaml);
  }
  if (corner_csvs.empty()) {
    corner_csvs.push_back(corners_csv);
  }
  if (corner_csvs.empty()) {
    std::cerr << "at least one --corners CSV is required\n";
    return 2;
  }
  if (cam_yamls.size() != 1 && cam_yamls.size() != corner_csvs.size()) {
    std::cerr << "use one shared --cam camchain YAML or one --cam per "
                 "--corners CSV\n";
    return 2;
  }
  const bool shared_camchain_yaml =
      cam_yamls.size() == 1 && corner_csvs.size() > 1;
  std::vector<ceres_cam_imu::CameraObservationDataset> cameras;
  cameras.reserve(corner_csvs.size());
  for (std::size_t camera_index = 0; camera_index < corner_csvs.size();
       ++camera_index) {
    const std::string &camera_yaml =
        shared_camchain_yaml ? cam_yamls.front() : cam_yamls[camera_index];
    ceres_cam_imu::CameraObservationDataset camera;
    camera.intrinsics = ceres_cam_imu::readCameraIntrinsics(
        camera_yaml, shared_camchain_yaml ? static_cast<int>(camera_index) : 0);
    camera.images =
        ceres_cam_imu::readCornerCsv(corner_csvs[camera_index],
                                     options.max_frames);
    cameras.push_back(std::move(camera));
  }
  const bool multi_camera = cameras.size() > 1;
  const ceres_cam_imu::CameraIntrinsics &intrinsics =
      cameras.front().intrinsics;
  const std::vector<ceres_cam_imu::ImageObservation> &images =
      cameras.front().images;
  const ceres_cam_imu::ImuNoise imu_noise =
      ceres_cam_imu::readImuNoise(imu_yaml);
  const std::vector<ceres_cam_imu::ImuSample> raw_imu_samples =
      ceres_cam_imu::readImuCsv(imu_data);
  const std::vector<ceres_cam_imu::ImuSample> imu_samples =
      ceres_cam_imu::trimImuSamplesKalibr(raw_imu_samples, imu_trim_edge_count);
  (void)ceres_cam_imu::readAprilGridConfig(target_yaml);

  const std::string corner_poses_csv = argValue(argc, argv, "--corner-poses");
  std::vector<ceres_cam_imu::PoseObservation> poses;
  if (!corner_poses_csv.empty()) {
    poses = ceres_cam_imu::readPoseCsv(corner_poses_csv);
  }

  const std::string kalibr_result_path =
      argValue(argc, argv, "--kalibr-result");
  ceres_cam_imu::KalibrResult kalibr;
  const bool have_kalibr_result = !kalibr_result_path.empty();
  if (have_kalibr_result) {
    kalibr = ceres_cam_imu::readKalibrResult(kalibr_result_path);
  }
  const std::string init_result_path =
      argValue(argc, argv, "--init-from-result");
  ceres_cam_imu::CalibrationResultFile init_result;
  const bool init_from_result = !init_result_path.empty();
  if (init_from_result) {
    init_result = ceres_cam_imu::readCalibrationResultYaml(init_result_path);
    if (multi_camera && init_result.camera_T_c_b.size() < cameras.size()) {
      std::cerr << "--init-from-result requires a complete camera_chain for "
                << cameras.size() << " cameras; found "
                << init_result.camera_T_c_b.size() << " entries in "
                << init_result_path << "\n";
      return 2;
    }
  }
  if (options.trace_iteration_state && have_kalibr_result) {
    options.trace_has_reference_state = true;
    options.trace_reference_T_c_b = kalibr.T_ci;
    options.trace_reference_time_shift_s = kalibr.timeshift_cam_to_imu_s;
    options.trace_reference_gravity = kalibr.gravity;
  }

  const bool requested_init_from_kalibr =
      hasFlag(argc, argv, "--init-from-kalibr");
  const bool requested_init_from_camchain =
      hasFlag(argc, argv, "--init-from-camchain");
  if (requested_init_from_kalibr && !have_kalibr_result) {
    std::cerr << "--init-from-kalibr requires --kalibr-result\n";
    return 2;
  }
  const bool init_from_kalibr =
      requested_init_from_kalibr && have_kalibr_result;
  ceres_cam_imu::CameraExtrinsicBlock initial_T_c_b;
  std::vector<ceres_cam_imu::CameraExtrinsicBlock> initial_camera_extrinsics(
      cameras.size());
  std::vector<double> initial_camera_time_shifts(cameras.size(),
                                                 options.initial_camera_time_shift_s);
  bool have_initial_camera_blocks = false;
  ceres_cam_imu::Vec3 initial_gravity = ceres_cam_imu::Vec3::Zero();
  bool have_initial_gravity = false;
  if (init_from_result) {
    options.initial_camera_time_shift_s = init_result.time_shift_s;
    initial_camera_time_shifts[0] = init_result.time_shift_s;
    const ceres_cam_imu::Vec6 T_c_b =
        ceres_cam_imu::matrixToPose6(init_result.T_c_b);
    for (int i = 0; i < 6; ++i) {
      initial_T_c_b.values[static_cast<std::size_t>(i)] = T_c_b(i);
    }
    initial_camera_extrinsics[0] = initial_T_c_b;
    for (std::size_t camera_index = 0;
         camera_index < init_result.camera_T_c_b.size() &&
         camera_index < initial_camera_extrinsics.size();
         ++camera_index) {
      const ceres_cam_imu::Vec6 camera_T_c_b =
          ceres_cam_imu::matrixToPose6(init_result.camera_T_c_b[camera_index]);
      for (int i = 0; i < 6; ++i) {
        initial_camera_extrinsics[camera_index]
            .values[static_cast<std::size_t>(i)] = camera_T_c_b(i);
      }
    }
    for (std::size_t camera_index = 0;
         camera_index < init_result.camera_time_shift_s.size() &&
         camera_index < initial_camera_time_shifts.size();
         ++camera_index) {
      initial_camera_time_shifts[camera_index] =
          init_result.camera_time_shift_s[camera_index];
    }
    have_initial_camera_blocks = true;
    initial_gravity = init_result.gravity;
    have_initial_gravity = true;
    const std::streamsize old_precision = std::cout.precision();
    std::cout << std::setprecision(17)
              << "initialized from Ceres result: time_shift_s="
              << options.initial_camera_time_shift_s
              << " translation_m=" << initial_T_c_b.values[0] << " "
              << initial_T_c_b.values[1] << " " << initial_T_c_b.values[2]
              << " gravity_m_s2=" << initial_gravity.transpose() << "\n";
    std::cout.precision(old_precision);
  }
  if (init_from_kalibr) {
    options.initial_camera_time_shift_s = kalibr.timeshift_cam_to_imu_s;
    initial_camera_time_shifts[0] = kalibr.timeshift_cam_to_imu_s;
    const ceres_cam_imu::Vec6 T_c_b = ceres_cam_imu::matrixToPose6(kalibr.T_ci);
    for (int i = 0; i < 6; ++i) {
      initial_T_c_b.values[static_cast<std::size_t>(i)] = T_c_b(i);
    }
    initial_camera_extrinsics[0] = initial_T_c_b;
    have_initial_camera_blocks = true;
    initial_gravity = kalibr.gravity;
    have_initial_gravity = true;
  }
  if (requested_init_from_camchain) {
    for (std::size_t camera_index = 0; camera_index < cameras.size();
         ++camera_index) {
      const std::string &camera_yaml =
          shared_camchain_yaml ? cam_yamls.front() : cam_yamls[camera_index];
      const ceres_cam_imu::CamchainImuPrior camchain_prior =
          ceres_cam_imu::readCamchainImuPrior(
              camera_yaml, shared_camchain_yaml ? static_cast<int>(camera_index)
                                                : 0);
      if (!camchain_prior.has_T_cam_imu) {
        std::cerr << "--init-from-camchain requires T_cam_imu for camera "
                  << camera_index << "\n";
        return 2;
      }
      if (camchain_prior.has_timeshift_cam_imu) {
        initial_camera_time_shifts[camera_index] =
            camchain_prior.timeshift_cam_imu_s;
      }
      const ceres_cam_imu::Vec6 T_c_b =
          ceres_cam_imu::matrixToPose6(camchain_prior.T_cam_imu);
      for (int i = 0; i < 6; ++i) {
        initial_camera_extrinsics[camera_index]
            .values[static_cast<std::size_t>(i)] = T_c_b(i);
      }
    }
    initial_T_c_b = initial_camera_extrinsics[0];
    options.initial_camera_time_shift_s = initial_camera_time_shifts[0];
    have_initial_camera_blocks = true;
    const std::streamsize old_precision = std::cout.precision();
    std::cout << std::setprecision(17)
              << "initialized from camchain: time_shift_s="
              << options.initial_camera_time_shift_s
              << " translation_m=" << initial_T_c_b.values[0] << " "
              << initial_T_c_b.values[1] << " " << initial_T_c_b.values[2];
    if (have_kalibr_result) {
      const ceres_cam_imu::Mat4 initial_T =
          ceres_cam_imu::pose6ToMatrix(initial_T_c_b);
      std::cout << " kalibr_translation_delta_m="
                << (initial_T.block<3, 1>(0, 3) -
                    kalibr.T_ci.block<3, 1>(0, 3))
                       .norm()
                << " kalibr_time_delta_s="
                << (options.initial_camera_time_shift_s -
                    kalibr.timeshift_cam_to_imu_s);
    }
    std::cout << "\n";
    std::cout.precision(old_precision);
  }

  if (hasFlag(argc, argv, "--estimate-time-shift-prior")) {
    if (poses.empty()) {
      std::cerr
          << "--estimate-time-shift-prior requires --corner-poses poses.csv\n";
      return 2;
    }
    ceres_cam_imu::TimeShiftPriorOptions time_shift_options;
    time_shift_options.pose_knots_per_second =
        doubleArg(argc, argv, "--time-shift-pose-kps",
                  time_shift_options.pose_knots_per_second);
    time_shift_options.pose_fit_regularization =
        doubleArg(argc, argv, "--time-shift-fit-lambda",
                  time_shift_options.pose_fit_regularization);
    const ceres_cam_imu::TimeShiftPriorEstimate time_shift =
        ceres_cam_imu::estimateCameraImuTimeShiftPrior(
            poses, imu_samples, initial_T_c_b, time_shift_options);
    options.initial_camera_time_shift_s = time_shift.shift_s;
    if (!initial_camera_time_shifts.empty()) {
      initial_camera_time_shifts[0] = time_shift.shift_s;
    }
    const std::streamsize old_precision = std::cout.precision();
    std::cout << std::setprecision(17)
              << "estimated time shift prior: shift_s=" << time_shift.shift_s
              << " pose_kps=" << time_shift_options.pose_knots_per_second
              << " fit_lambda=" << time_shift_options.pose_fit_regularization
              << " discrete_shift_samples=" << time_shift.discrete_shift_samples
              << " sample_dt_s=" << time_shift.sample_dt_s
              << " samples=" << time_shift.num_samples
              << " peak_correlation=" << time_shift.peak_correlation
              << " predicted_norm_rms=" << time_shift.predicted_norm_rms
              << " measured_norm_rms=" << time_shift.measured_norm_rms;
    if (have_kalibr_result) {
      std::cout << " kalibr_delta_s="
                << (time_shift.shift_s - kalibr.timeshift_cam_to_imu_s);
    }
    std::cout << "\n";
    std::cout.precision(old_precision);
  }

  if (hasFlag(argc, argv, "--estimate-orientation-gravity-prior")) {
    if (poses.empty()) {
      std::cerr << "--estimate-orientation-gravity-prior requires "
                   "--corner-poses poses.csv\n";
      return 2;
    }
    ceres_cam_imu::OrientationGravityInitializerOptions orientation_options;
    orientation_options.pose_knots_per_second =
        doubleArg(argc, argv, "--orientation-prior-pose-kps",
                  orientation_options.pose_knots_per_second);
    orientation_options.pose_fit_regularization =
        doubleArg(argc, argv, "--orientation-prior-fit-lambda",
                  orientation_options.pose_fit_regularization);
    orientation_options.pose_fit_boundary_anchors =
        !hasFlag(argc, argv, "--no-orientation-prior-boundary-anchors");
    orientation_options.refine_with_ceres =
        !hasFlag(argc, argv, "--no-orientation-prior-ceres-refine");
    if (orientation_options.pose_knots_per_second <= 0.0 ||
        orientation_options.pose_fit_regularization < 0.0) {
      std::cerr << "orientation prior kps must be positive and fit lambda "
                   "must be non-negative\n";
      return 2;
    }
    const ceres_cam_imu::OrientationGravityInitializerResult orientation =
        ceres_cam_imu::estimateOrientationGravityAndGyroBiasPrior(
            poses, imu_samples, initial_T_c_b,
            options.initial_camera_time_shift_s, orientation_options);
    initial_T_c_b = orientation.T_c_b;
    initial_camera_extrinsics[0] = initial_T_c_b;
    have_initial_camera_blocks = true;
    initial_gravity = orientation.gravity_m_s2;
    have_initial_gravity = true;
    options.initial_gyro_bias_rad_s = orientation.gyro_bias_rad_s;

    const std::streamsize old_precision = std::cout.precision();
    std::cout << std::setprecision(17)
              << "estimated orientation/gravity prior: samples="
              << orientation.num_samples
              << " pose_kps=" << orientation_options.pose_knots_per_second
              << " fit_lambda=" << orientation_options.pose_fit_regularization
              << " boundary_anchors="
              << (orientation_options.pose_fit_boundary_anchors ? 1 : 0)
              << " ceres_refine="
              << (orientation_options.refine_with_ceres ? 1 : 0)
              << " gyro_bias_rad_s=" << orientation.gyro_bias_rad_s.transpose()
              << " gravity_m_s2=" << orientation.gravity_m_s2.transpose()
              << " gravity_mean_norm_m_s2="
              << orientation.gravity_mean_norm_m_s2
              << " gyro_rms_rad_s=" << orientation.gyro_rms_rad_s
              << " singular_values=" << orientation.singular_values.transpose()
              << " pose_fit_rms_translation_m="
              << orientation.pose_fit_rms_translation_m
              << " pose_fit_rms_rotation_rad="
              << orientation.pose_fit_rms_rotation_rad
              << " pose_fit_boundary_anchor_observations="
              << orientation.pose_fit_boundary_anchor_observations
              << " refine_iterations=" << orientation.refine_iterations
              << " refine_final_cost=" << orientation.refine_final_cost;
    if (have_kalibr_result) {
      const ceres_cam_imu::Mat4 T_c_b_matrix =
          ceres_cam_imu::pose6ToMatrix(orientation.T_c_b);
      const ceres_cam_imu::Mat3 dR = T_c_b_matrix.block<3, 3>(0, 0) *
                                     kalibr.T_ci.block<3, 3>(0, 0).transpose();
      const double cos_angle = std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0);
      constexpr double kPi = 3.14159265358979323846;
      const double angle_deg = std::acos(cos_angle) * 180.0 / kPi;
      std::cout << " kalibr_rotation_delta_deg=" << angle_deg
                << " kalibr_gravity_delta_norm="
                << (orientation.gravity_m_s2 - kalibr.gravity).norm();
    }
    std::cout << "\n";
    std::cout.precision(old_precision);
  }

  if (options.add_time_shift_prior || !stage_time_shift_prior_sigmas.empty()) {
    options.time_shift_prior_s = options.initial_camera_time_shift_s;
  }
  if (options.add_time_shift_prior) {
    std::cout << "time shift prior residual: prior_s="
              << options.time_shift_prior_s
              << " sigma_s=" << options.time_shift_prior_sigma_s << "\n";
  } else if (!stage_time_shift_prior_sigmas.empty()) {
    std::cout << "stage time shift prior center: prior_s="
              << options.time_shift_prior_s << "\n";
  }
  if (corner_defaults) {
    std::cout << "corner defaults active: pose_kps="
              << options.pose_knots_per_second
              << " bias_kps=" << options.bias_knots_per_second
              << " max_iterations=" << options.max_iterations
              << " timeoffset_padding_s=" << options.time_padding_s
              << " imu_trim_edge_count=" << imu_trim_edge_count
              << " cauchy_widths=" << options.camera_loss_width << ","
              << options.gyro_loss_width << "," << options.accel_loss_width
              << "\n";
  }

  ceres_cam_imu::CalibrationState state =
      multi_camera
          ? ceres_cam_imu::initializeCalibrationState(cameras, imu_samples,
                                                      options)
          : ceres_cam_imu::initializeCalibrationState(images, imu_samples,
                                                      options);

  state.T_c_b = initial_T_c_b;
  if (have_initial_camera_blocks) {
    if (state.camera_extrinsics.size() < cameras.size()) {
      state.camera_extrinsics.resize(cameras.size());
    }
    if (state.camera_time_shifts.size() < cameras.size()) {
      state.camera_time_shifts.resize(cameras.size());
    }
    for (std::size_t camera_index = 0; camera_index < cameras.size();
         ++camera_index) {
      state.camera_extrinsics[camera_index] =
          initial_camera_extrinsics[camera_index];
      state.camera_time_shifts[camera_index].value =
          initial_camera_time_shifts[camera_index];
    }
    state.T_c_b = state.camera_extrinsics[0];
    state.camera_time_shift_s.value = state.camera_time_shifts[0].value;
  }
  if (have_initial_gravity) {
    for (int i = 0; i < 3; ++i) {
      state.gravity.values[static_cast<std::size_t>(i)] = initial_gravity(i);
    }
  }
  if (init_from_result) {
    initializeImuIntrinsicsFromResult(init_result, &state);
  }
  if (init_from_kalibr) {
    initializeImuIntrinsicsFromKalibr(kalibr, &state);
  }

  if (!poses.empty()) {
    const ceres_cam_imu::PoseInitializationSummary pose_init =
        ceres_cam_imu::initializePoseControlsFromCameraPoses(poses, state.T_c_b,
                                                             options, &state);
    std::cout << "initialized pose controls: used="
              << pose_init.used_observations
              << " skipped=" << pose_init.skipped_observations
              << " boundary_anchors=" << pose_init.boundary_anchor_observations
              << " coeffs=" << pose_init.num_coefficients
              << " rms_translation_m=" << pose_init.rms_translation_m
              << " rms_rotation_rad=" << pose_init.rms_rotation_rad
              << " fit_diag_lambda=" << options.pose_fit_diagonal_regularization
              << " fit_motion_lambda=" << options.pose_fit_motion_regularization
              << "\n";
  }

  if (staged) {
    if (multi_camera) {
      std::cerr << "--staged multi-camera calibration is not implemented yet; "
                   "use joint optimization for multi-camera runs\n";
      return 2;
    }
    std::vector<ceres_cam_imu::CalibrationStage> stages =
        stage_free_masks.empty()
            ? ceres_cam_imu::makeConservativeCalibrationStages(options,
                                                               stage_iterations)
            : ceres_cam_imu::makeCalibrationStagesFromFreeMasks(
                  options, stage_iterations, stage_free_masks);
    ceres_cam_imu::applyStagePoseMotionVariances(
        stage_pose_translation_variances, stage_pose_rotation_variances,
        &stages);
    ceres_cam_imu::applyStagePoseMotionOrders(stage_pose_motion_orders,
                                              &stages);
    ceres_cam_imu::applyStageTimeShiftPriorSigmas(stage_time_shift_prior_sigmas,
                                                  &stages);
    ceres_cam_imu::applyStageSolverOptions(
        stage_solver_initial_trust_region_radii,
        stage_solver_max_trust_region_radii,
        stage_solver_min_trust_region_radii,
        stage_solver_min_relative_decreases,
        stage_solver_absolute_cost_change_tolerances,
        stage_solver_absolute_step_tolerances,
        stage_solver_absolute_parameter_tolerances, &stages);
    if (hasFlag(argc, argv, "--dry-run")) {
      for (const ceres_cam_imu::CalibrationStage &stage : stages) {
        ceres::Problem stage_problem;
        const ceres_cam_imu::CalibrationBuildSummary stage_build =
            ceres_cam_imu::buildCalibrationProblem(
                intrinsics, imu_noise, images, imu_samples, stage.options,
                &state, &stage_problem);
        printBuildSummary(
            "stage built [" + stage.name + " iterations=" +
                std::to_string(stage.options.max_iterations) + " pose_order=" +
                std::to_string(stage.options.pose_motion_derivative_order) +
                " solver_initial_radius=" +
                std::to_string(
                    stage.options.solver_initial_trust_region_radius) +
                " solver_max_radius=" +
                std::to_string(stage.options.solver_max_trust_region_radius) +
                " solver_min_relative_decrease=" +
                std::to_string(stage.options.solver_min_relative_decrease) +
                " solver_abs_cost_tol=" +
                std::to_string(
                    stage.options.solver_absolute_cost_change_tolerance) +
                " solver_abs_step_tol=" +
                std::to_string(stage.options.solver_absolute_step_tolerance) +
                " solver_abs_param_tol=" +
                std::to_string(
                    stage.options.solver_absolute_parameter_tolerance) +
                "]: ",
            stage_build);
      }
      return 0;
    }

    bool all_stages_usable = true;
    for (const ceres_cam_imu::CalibrationStage &stage : stages) {
      std::cout << "stage begin: " << stage.name
                << " iterations=" << stage.options.max_iterations
                << " pose_order=" << stage.options.pose_motion_derivative_order
                << " solver_initial_radius="
                << stage.options.solver_initial_trust_region_radius
                << " solver_max_radius="
                << stage.options.solver_max_trust_region_radius
                << " solver_min_relative_decrease="
                << stage.options.solver_min_relative_decrease
                << " solver_abs_cost_tol="
                << stage.options.solver_absolute_cost_change_tolerance
                << " solver_abs_step_tol="
                << stage.options.solver_absolute_step_tolerance
                << " solver_abs_param_tol="
                << stage.options.solver_absolute_parameter_tolerance << "\n";
      const ceres_cam_imu::CalibrationStageResult stage_result =
          ceres_cam_imu::solveCalibrationStage(intrinsics, imu_noise, images,
                                               imu_samples, stage, &state);
      printBuildSummary("stage built [" + stage_result.name + "]: ",
                        stage_result.build);
      const std::streamsize old_precision = std::cout.precision();
      std::cout << std::setprecision(17) << "stage state [" << stage_result.name
                << "]: decision="
                << ceres_cam_imu::calibrationStageStateDecisionName(
                       stage_result.state_decision)
                << " restored=" << (stage_result.state_restored ? 1 : 0)
                << " initial_cost=" << stage_result.solver.initial_cost
                << " final_cost=" << stage_result.solver.final_cost
                << " cost_change=" << stage_result.state_cost_change
                << " usable="
                << (stage_result.solver.IsSolutionUsable() ? 1 : 0) << "\n";
      std::cout.precision(old_precision);
      std::cout << "stage complete [" << stage_result.name
                << "]: " << stage_result.solver.BriefReport() << "\n";
      if (!stage_result.solver.IsSolutionUsable() && stop_on_stage_failure) {
        std::cerr << "stage failure stopped calibration: " << stage_result.name
                  << "\n";
        all_stages_usable = false;
        break;
      }
      all_stages_usable =
          all_stages_usable && stage_result.solver.IsSolutionUsable();
    }
    printFinalState(state, have_kalibr_result, kalibr);
    const ceres_cam_imu::CalibrationResidualStatistics residual_stats =
        printFinalResidualStatistics(intrinsics, imu_noise, images, imu_samples,
                                     options, state);
    for (const double inspect_time_s : inspect_times_s) {
      printLocalTimeDiagnostics(inspect_time_s, inspect_window_s, poses,
                                imu_samples, state);
    }
    if (!output_result_path.empty()) {
      ceres_cam_imu::CalibrationResultWriterOptions writer_options;
      writer_options.include_kalibr_comparison = have_kalibr_result;
      writer_options.include_spline_controls = export_spline_controls;
      writer_options.kalibr_result = kalibr;
      ceres_cam_imu::writeCalibrationResultYaml(output_result_path, state,
                                                residual_stats, writer_options);
      std::cout << "wrote calibration result: " << output_result_path << "\n";
    }
    if (!imu_diagnostics_path.empty()) {
      ceres_cam_imu::writeImuDiagnosticsCsv(imu_diagnostics_path, imu_samples,
                                            options, state);
      std::cout << "wrote IMU diagnostics: " << imu_diagnostics_path << "\n";
    }
    return all_stages_usable ? 0 : 1;
  }

  ceres::Problem problem;
  const ceres_cam_imu::CalibrationBuildSummary build =
      multi_camera
          ? ceres_cam_imu::buildCalibrationProblem(cameras, imu_noise,
                                                   imu_samples, options, &state,
                                                   &problem)
          : ceres_cam_imu::buildCalibrationProblem(intrinsics, imu_noise,
                                                   images, imu_samples, options,
                                                   &state, &problem);

  printBuildSummary("problem built: ", build);

  if (hasFlag(argc, argv, "--dry-run")) {
    return 0;
  }

  const ceres::Solver::Summary summary =
      ceres_cam_imu::solveCalibrationProblem(options, &state, &problem);
  std::cout << summary.BriefReport() << "\n";
  if (multi_camera && !state.camera_extrinsics.empty() &&
      !state.camera_time_shifts.empty()) {
    state.camera_extrinsics[0] = state.T_c_b;
    state.camera_time_shifts[0] = state.camera_time_shift_s;
  }
  printFinalState(state, have_kalibr_result, kalibr);
  if (multi_camera) {
    for (std::size_t camera_index = 0;
         camera_index < state.camera_extrinsics.size(); ++camera_index) {
      const ceres_cam_imu::Mat4 T_c_b =
          ceres_cam_imu::pose6ToMatrix(state.camera_extrinsics[camera_index]);
      const double time_shift =
          camera_index < state.camera_time_shifts.size()
              ? state.camera_time_shifts[camera_index].value
              : 0.0;
      std::cout << "camera_chain_state camera=" << camera_index
                << " time_shift_s=" << time_shift << " translation_m="
                << T_c_b(0, 3) << " " << T_c_b(1, 3) << " " << T_c_b(2, 3)
                << "\n";
    }
  }
  const ceres_cam_imu::CalibrationResidualStatistics residual_stats =
      multi_camera ? printFinalResidualStatistics(cameras, imu_noise,
                                                  imu_samples, options, state)
                   : printFinalResidualStatistics(intrinsics, imu_noise,
                                                  images, imu_samples, options,
                                                  state);
  for (const double inspect_time_s : inspect_times_s) {
    printLocalTimeDiagnostics(inspect_time_s, inspect_window_s, poses,
                              imu_samples, state);
  }
  if (!output_result_path.empty()) {
    ceres_cam_imu::CalibrationResultWriterOptions writer_options;
    writer_options.include_kalibr_comparison = have_kalibr_result;
    writer_options.include_spline_controls = export_spline_controls;
    writer_options.kalibr_result = kalibr;
    ceres_cam_imu::writeCalibrationResultYaml(output_result_path, state,
                                              residual_stats, writer_options);
    std::cout << "wrote calibration result: " << output_result_path << "\n";
  }
  if (!imu_diagnostics_path.empty()) {
    ceres_cam_imu::writeImuDiagnosticsCsv(imu_diagnostics_path, imu_samples,
                                          options, state);
    std::cout << "wrote IMU diagnostics: " << imu_diagnostics_path << "\n";
  }
  return summary.IsSolutionUsable() ? 0 : 1;
}
