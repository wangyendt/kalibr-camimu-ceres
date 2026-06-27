#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ceres/ceres.h>

#include "ceres_cam_imu/camera/camera_model.h"
#include "ceres_cam_imu/core/se3.h"
#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/initialization/camera_translation_initializer.h"
#include "ceres_cam_imu/initialization/multi_imu_initializer.h"
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

std::string robustLossTypeName(const ceres_cam_imu::RobustLossType type) {
  switch (type) {
  case ceres_cam_imu::RobustLossType::kNone:
    return "none";
  case ceres_cam_imu::RobustLossType::kCauchy:
    return "cauchy";
  case ceres_cam_imu::RobustLossType::kHuber:
    return "huber";
  }
  return "unknown";
}

enum class CornerDefaultTopology {
  kOneCameraOneImu,
  kOneCameraMultiImu,
  kMultiCameraOneImu,
  kMultiCameraMultiImu,
};

CornerDefaultTopology inferCornerDefaultTopology(std::size_t camera_count,
                                                 std::size_t imu_count) {
  const bool multi_camera = camera_count > 1;
  const bool multi_imu = imu_count > 1;
  if (!multi_camera && !multi_imu) {
    return CornerDefaultTopology::kOneCameraOneImu;
  }
  if (!multi_camera && multi_imu) {
    return CornerDefaultTopology::kOneCameraMultiImu;
  }
  if (multi_camera && !multi_imu) {
    return CornerDefaultTopology::kMultiCameraOneImu;
  }
  return CornerDefaultTopology::kMultiCameraMultiImu;
}

std::string cornerDefaultTopologyName(const CornerDefaultTopology topology) {
  switch (topology) {
  case CornerDefaultTopology::kOneCameraOneImu:
    return "1cam+1imu";
  case CornerDefaultTopology::kOneCameraMultiImu:
    return "1cam+Nimu";
  case CornerDefaultTopology::kMultiCameraOneImu:
    return "Mcam+1imu";
  case CornerDefaultTopology::kMultiCameraMultiImu:
    return "Mcam+Nimu";
  }
  return "unknown";
}

void applyCornerCommonDefaults(ceres_cam_imu::CalibrationOptions *options) {
  options->pose_knots_per_second = 100.0;
  options->bias_knots_per_second = 50.0;
  options->time_padding_s = 0.04;
  options->camera_time_offset_buffer_s = 0.0;
  options->camera_loss_type = ceres_cam_imu::RobustLossType::kCauchy;
  options->gyro_loss_type = ceres_cam_imu::RobustLossType::kCauchy;
  options->accel_loss_type = ceres_cam_imu::RobustLossType::kCauchy;
  options->camera_loss_width = 10.0;
  options->gyro_loss_width = 10.0;
  options->accel_loss_width = 10.0;
}

void applyProductionSolverDefaults(ceres_cam_imu::CalibrationOptions *options) {
  options->max_iterations = 150;
  options->solver_function_tolerance = 0.0;
  options->solver_gradient_tolerance = 0.0;
  options->solver_parameter_tolerance = 0.0;
  options->solver_max_trust_region_radius = 1e7;
  options->solver_absolute_cost_change_tolerance = -1.0;
  options->solver_absolute_step_tolerance = 0.02;
  options->solver_absolute_parameter_tolerance = -1.0;
  options->solver_use_nonmonotonic_steps = true;
  options->solver_max_consecutive_nonmonotonic_steps = 20;
}

CornerDefaultTopology
applyCornerDefaults(const std::size_t camera_count, const std::size_t imu_count,
                    ceres_cam_imu::CalibrationOptions *options) {
  const CornerDefaultTopology topology =
      inferCornerDefaultTopology(camera_count, imu_count);
  applyCornerCommonDefaults(options);
  // Keep the first production preset conservative across topologies. More
  // specialized multi-camera/multi-IMU presets can narrow this later after
  // dedicated benchmarks, but bare --corner-defaults should not expose the old
  // 30-iteration early-stop behavior.
  applyProductionSolverDefaults(options);
  if (topology == CornerDefaultTopology::kOneCameraOneImu) {
    options->solver_absolute_cost_change_tolerance = 0.005;
  }
  return topology;
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

void printYellowWarning(const std::string &message) {
  std::cerr << "\033[33mwarning: " << message << "\033[0m\n";
}

long long timestampKeyNs(const double timestamp_s) {
  return static_cast<long long>(std::llround(timestamp_s * 1e9));
}

ceres_cam_imu::CameraExtrinsicBlock
matrixToExtrinsicBlock(const ceres_cam_imu::Mat4 &T) {
  ceres_cam_imu::CameraExtrinsicBlock block;
  const ceres_cam_imu::Vec6 pose = ceres_cam_imu::matrixToPose6(T);
  for (int i = 0; i < 6; ++i) {
    block.values[static_cast<std::size_t>(i)] = pose(i);
  }
  return block;
}

std::vector<ceres_cam_imu::Mat4> cameraChainPriorsFromExtrinsics(
    const std::vector<ceres_cam_imu::CameraExtrinsicBlock> &extrinsics) {
  std::vector<ceres_cam_imu::Mat4> priors(
      extrinsics.size(), ceres_cam_imu::Mat4::Identity());
  if (extrinsics.empty()) {
    return priors;
  }
  const ceres_cam_imu::Mat4 T_c0_b =
      ceres_cam_imu::pose6ToMatrix(extrinsics.front());
  const ceres_cam_imu::Mat4 T_b_c0 = T_c0_b.inverse();
  for (std::size_t camera_index = 1; camera_index < extrinsics.size();
       ++camera_index) {
    priors[camera_index] =
        ceres_cam_imu::pose6ToMatrix(extrinsics[camera_index]) * T_b_c0;
  }
  return priors;
}

void applyCameraChainPriorsToInitialExtrinsics(
    const std::vector<ceres_cam_imu::Mat4> &camera_chain_T_ci_c0,
    std::vector<ceres_cam_imu::CameraExtrinsicBlock> *extrinsics) {
  if (!extrinsics || extrinsics->empty() ||
      camera_chain_T_ci_c0.size() < extrinsics->size()) {
    return;
  }
  const ceres_cam_imu::Mat4 T_c0_b =
      ceres_cam_imu::pose6ToMatrix(extrinsics->front());
  for (std::size_t camera_index = 1; camera_index < extrinsics->size();
       ++camera_index) {
    (*extrinsics)[camera_index] =
        matrixToExtrinsicBlock(camera_chain_T_ci_c0[camera_index] * T_c0_b);
  }
}

class TargetPoseProjectionFunctor {
public:
  TargetPoseProjectionFunctor(ceres_cam_imu::CameraIntrinsics intrinsics,
                              ceres_cam_imu::CornerMeasurement corner)
      : camera_(std::move(intrinsics)), corner_(std::move(corner)) {}

  bool operator()(const double *const T_t_c_params, double *residuals) const {
    ceres_cam_imu::Vec6 pose;
    for (int i = 0; i < 6; ++i) {
      pose(i) = T_t_c_params[i];
    }
    const ceres_cam_imu::Mat4 T_t_c = ceres_cam_imu::pose6ToMatrix(pose);
    const ceres_cam_imu::Mat3 R_t_c = T_t_c.block<3, 3>(0, 0);
    const ceres_cam_imu::Vec3 t_t_c = T_t_c.block<3, 1>(0, 3);
    const ceres_cam_imu::Vec3 p_c =
        R_t_c.transpose() * (corner_.target_point - t_t_c);
    ceres_cam_imu::Vec2 pixel;
    if (!camera_.projectWithJacobian(p_c, &pixel, nullptr)) {
      return false;
    }
    residuals[0] = corner_.pixel.x() - pixel.x();
    residuals[1] = corner_.pixel.y() - pixel.y();
    return true;
  }

private:
  ceres_cam_imu::CameraModel camera_;
  ceres_cam_imu::CornerMeasurement corner_;
};

bool estimateTargetToCameraPose(
    const ceres_cam_imu::CameraIntrinsics &intrinsics,
    const ceres_cam_imu::ImageObservation &image,
    const ceres_cam_imu::Mat4 &initial_T_t_c, ceres_cam_imu::Mat4 *T_t_c,
    double *rms_px) {
  if (!T_t_c || image.corners.size() < 12) {
    return false;
  }
  ceres_cam_imu::Vec6 pose = ceres_cam_imu::matrixToPose6(initial_T_t_c);
  ceres::Problem problem;
  for (const ceres_cam_imu::CornerMeasurement &corner : image.corners) {
    problem.AddResidualBlock(
        new ceres::NumericDiffCostFunction<TargetPoseProjectionFunctor,
                                           ceres::CENTRAL, 2, 6>(
            new TargetPoseProjectionFunctor(intrinsics, corner)),
        nullptr, pose.data());
  }
  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.num_threads = 1;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.IsSolutionUsable()) {
    return false;
  }

  *T_t_c = ceres_cam_imu::pose6ToMatrix(pose);
  double sum_sq = 0.0;
  int count = 0;
  const ceres_cam_imu::CameraModel camera(intrinsics);
  const ceres_cam_imu::Mat3 R_t_c = T_t_c->block<3, 3>(0, 0);
  const ceres_cam_imu::Vec3 t_t_c = T_t_c->block<3, 1>(0, 3);
  for (const ceres_cam_imu::CornerMeasurement &corner : image.corners) {
    const ceres_cam_imu::Vec3 p_c =
        R_t_c.transpose() * (corner.target_point - t_t_c);
    ceres_cam_imu::Vec2 pixel;
    if (!camera.projectWithJacobian(p_c, &pixel, nullptr)) {
      continue;
    }
    sum_sq += (corner.pixel - pixel).squaredNorm();
    ++count;
  }
  if (count <= 0) {
    return false;
  }
  if (rms_px) {
    *rms_px = std::sqrt(sum_sq / static_cast<double>(count));
  }
  return true;
}

bool estimateCameraBaselineFromObservations(
    const ceres_cam_imu::CameraObservationDataset &camera,
    const std::vector<ceres_cam_imu::PoseObservation> &cam0_poses,
    ceres_cam_imu::Mat4 *T_ci_c0, int *used_frames, double *mean_rms_px) {
  if (!T_ci_c0 || cam0_poses.empty()) {
    return false;
  }
  std::map<long long, ceres_cam_imu::Mat4> cam0_pose_by_time;
  for (const ceres_cam_imu::PoseObservation &pose : cam0_poses) {
    cam0_pose_by_time[timestampKeyNs(pose.timestamp_s)] = pose.T_t_c;
  }

  std::vector<ceres_cam_imu::Mat4> baselines;
  std::vector<double> rms_values;
  baselines.reserve(50);
  rms_values.reserve(50);
  for (const ceres_cam_imu::ImageObservation &image : camera.images) {
    if (baselines.size() >= 50) {
      break;
    }
    const auto cam0_pose_it = cam0_pose_by_time.find(
        timestampKeyNs(image.timestamp_s));
    if (cam0_pose_it == cam0_pose_by_time.end()) {
      continue;
    }
    ceres_cam_imu::Mat4 T_t_ci = ceres_cam_imu::Mat4::Identity();
    double rms_px = 0.0;
    if (!estimateTargetToCameraPose(camera.intrinsics, image,
                                    cam0_pose_it->second, &T_t_ci, &rms_px)) {
      continue;
    }
    if (!std::isfinite(rms_px) || rms_px > 5.0) {
      continue;
    }
    baselines.push_back(T_t_ci.inverse() * cam0_pose_it->second);
    rms_values.push_back(rms_px);
  }
  if (baselines.size() < 3) {
    return false;
  }

  ceres_cam_imu::Vec3 translation = ceres_cam_imu::Vec3::Zero();
  Eigen::Vector4d quat_sum = Eigen::Vector4d::Zero();
  Eigen::Quaterniond reference_q(baselines.front().block<3, 3>(0, 0));
  for (const ceres_cam_imu::Mat4 &baseline : baselines) {
    translation += baseline.block<3, 1>(0, 3);
    Eigen::Quaterniond q(baseline.block<3, 3>(0, 0));
    if (q.coeffs().dot(reference_q.coeffs()) < 0.0) {
      q.coeffs() *= -1.0;
    }
    quat_sum += q.coeffs();
  }
  translation /= static_cast<double>(baselines.size());
  Eigen::Quaterniond mean_q(quat_sum(3), quat_sum(0), quat_sum(1),
                            quat_sum(2));
  mean_q.normalize();
  *T_ci_c0 = ceres_cam_imu::Mat4::Identity();
  T_ci_c0->block<3, 3>(0, 0) = mean_q.toRotationMatrix();
  T_ci_c0->block<3, 1>(0, 3) = translation;
  if (used_frames) {
    *used_frames = static_cast<int>(baselines.size());
  }
  if (mean_rms_px) {
    double sum = 0.0;
    for (const double rms : rms_values) {
      sum += rms;
    }
    *mean_rms_px = sum / static_cast<double>(rms_values.size());
  }
  return true;
}

void usage() {
  std::cout
      << "calibrate_cam_imu --cam camchain.yaml --imu imu.yaml --target "
         "aprilgrid.yaml "
         "--imu-data data.csv --corners corners.csv [--imu imu2.yaml "
         "--imu-data data2.csv ...] [--kalibr-result "
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
         "[--no-solver-use-nonmonotonic-steps] "
         "[--solver-max-consecutive-nonmonotonic-steps N] "
         "[--solver-restore-best-state] "
         "[--no-solver-restore-best-state] "
         "[--trace-iteration-state] "
         "[--pose-fit-diagonal-lambda L] [--pose-fit-motion-lambda L] "
         "[--pose-fit-boundary-anchors] "
         "[--stage-iterations N0,N1,N2,N3] [--stage-free MASK[,MASK...]] "
         "[--stop-on-stage-failure] "
         "[--time-padding S|--timeoffset-padding S] "
         "[--camera-time-offset-buffer S] "
         "[--fix-poses] [--fix-biases] [--fix-camera-extrinsic] "
         "[--fix-camera-chain-extrinsics] "
         "[--fix-time-shift] [--fix-gravity] [--fix-imu-extrinsics] "
         "[--imu-extrinsic-translation-bound-m M] "
         "[--imu-extrinsic-rotation-bound-rad R] "
         "[--extrinsic-manifold] "
         "[--estimate-gravity-length] "
         "[--imu-model calibrated|scale-misalignment|"
         "scale-misalignment-size-effect] [--fix-imu-intrinsics] "
         "[--estimate-time-shift-prior] "
         "[--initial-time-shift-s S] "
         "[--time-shift-pose-kps K] [--time-shift-fit-lambda L] "
         "[--estimate-orientation-gravity-prior] "
         "[--estimate-multi-imu-orientation-gravity-prior] "
         "[--orientation-prior-pose-kps K] [--orientation-prior-fit-lambda L] "
         "[--no-orientation-prior-boundary-anchors] "
         "[--no-orientation-prior-ceres-refine] "
         "[--estimate-imu-chain-lever-prior] "
         "[--no-estimate-imu-chain-lever-prior] "
         "[--imu-delay-correction auto|on|off] "
         "[--no-imu-delay-correction] "
         "[--optimize-imu-time-offsets] [--fix-imu-time-offsets] "
         "[--imu-time-offset-bound-s S] "
         "[--imu-chain-prior-max-lever-m M] "
         "[--imu-chain-prior-max-lever-accel-rms M] "
         "[--estimate-camera-translation-prior] "
         "[--no-estimate-camera-translation-prior] "
         "[--estimate-multi-imu-translation-prior] "
         "[--camera-translation-prior-pose-kps K] "
         "[--camera-translation-prior-fit-lambda L] "
         "[--camera-translation-prior-stride N] "
         "[--camera-translation-prior-min-lever-norm V] "
         "[--camera-translation-prior-max-norm M] "
         "[--multi-imu-translation-prior-max-lever-m M] "
         "[--multi-imu-translation-prior-camera-sigma-m S] "
         "[--multi-imu-translation-prior-lever-sigma-m S] "
         "[--multi-imu-translation-prior-accel-bias-sigma M] "
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
         "[--stage-imu-extrinsic-translation-bounds-m B0,B1,...] "
         "[--stage-imu-extrinsic-rotation-bounds-rad B0,B1,...] "
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
      << "  --camera-time-offset-buffer controls the camera residual control "
         "point buffer when camera time shift is active. Negative reuses time "
         "padding; zero uses the legacy fixed-segment fast path.\n";
  std::cout
      << "  --corner-defaults sets the standard corner-file defaults before "
         "parsing explicit overrides. It applies common corner defaults "
         "(pose/bias kps 100/50, time padding 0.04, camera time-offset "
         "buffer 0, IMU edge trim 1000, Cauchy width 10), infers the "
         "camera/IMU topology, and applies the current production solver "
         "defaults: max-iter 150, Ceres function/gradient/parameter "
         "tolerances 0, absolute step 0.02, absolute parameter disabled, "
         "absolute cost disabled except 1cam+1imu uses 0.005, and "
         "nonmonotonic steps enabled with max consecutive steps 20. "
         "--kalibr-corner-defaults is accepted as a deprecated alias.\n";
  std::cout
      << "  --pose-fit-motion-lambda adds Kalibr-style derivative-integral "
         "regularization to camera-pose spline initialization; "
         "--pose-fit-boundary-anchors duplicates the first/last camera pose at "
         "the padded spline boundaries.\n";
  std::cout
      << "  --estimate-time-shift-prior uses the Kalibr-style gyro-norm "
         "cross-correlation initializer: time-shift pose kps defaults to 100 "
         "time-shift fit lambda defaults to 1e-4, and "
         "--time-shift-max-search-s defaults to 0.05.\n";
  std::cout
      << "  --initial-time-shift-s sets an explicit cold-start camera-to-IMU "
         "time shift before orientation/gravity initialization; it overrides "
         "the gyro-norm initializer when both are provided.\n";
  std::cout
      << "  --init-from-camchain reads T_cam_imu and timeshift_cam_imu from "
         "the --cam YAML when those keys are present. If a later camera has "
         "T_cn_cnm1 but no T_cam_imu, its initial T_cam_imu is composed from "
         "the previous camera. If camchain extrinsics are incomplete, the "
         "program warns in yellow and attempts an observation-based camera "
         "chain fallback when cam0 poses are available.\n";
  std::cout
      << "  --fix-camera-chain-extrinsics keeps the initialized multi-camera "
         "baseline fixed relative to cam0 with tight relative extrinsic "
         "priors while still allowing cam0-to-IMU to optimize.\n";
  std::cout
      << "  --imu and --imu-data may be repeated for multi-IMU joint "
         "optimization. Counts must match, order matters, and the first IMU "
         "is the fixed reference IMU by default.\n";
  std::cout
      << "  Multi-IMU cold starts estimate non-reference IMU rotations from "
         "gyro correlation by default and apply the same gyro-correlation "
         "time offsets as fixed IMU delay correction. Multi-IMU "
         "--corner-defaults also tries non-reference lever-arm initialization "
         "from accelerometer differences with an RMS gate; outside "
         "corner-defaults it is available through "
         "--estimate-imu-chain-lever-prior. Use "
         "--no-estimate-imu-chain-prior to disable the whole chain prior, "
         "--no-imu-delay-correction to keep all IMU residual timestamps "
         "uncorrected, "
         "--optimize-imu-time-offsets to refine non-reference IMU delays as "
         "Ceres variables, --fix-imu-time-offsets to request the default "
         "fixed-correction mode explicitly, "
         "--imu-time-offset-bound-s S to bound the Ceres refinement window, "
         "--imu-extrinsic-translation-bound-m M and "
         "--imu-extrinsic-rotation-bound-rad R to bound non-reference IMU "
         "extrinsics around their current stage-start values (-1 disables, "
         "positive values enable), "
         "--imu-chain-prior-max-offset-s S to override multi-IMU "
         "corner-defaults full-overlap time search with a bounded search, and "
         "--imu-chain-prior-stride N to downsample initialization samples. "
         "--imu-chain-prior-max-lever-accel-rms rejects noisy lever-arm "
         "candidates while keeping the rotation/time chain prior. "
         "--imu-chain-prior-refine-with-accel is an experimental local "
         "refinement of non-reference IMU rotation, lever, and accel-bias "
         "delta from accelerometer residuals; the weight, rotation bound, "
         "lever sigma, and accel-bias sigma flags tune that refinement. "
         "--imu-chain-prior-refine-rotation-after-translation-prior first "
         "uses the non-accel chain prior for multi-IMU translation "
         "initialization, then reapplies only the accel-refined chain "
         "rotations while preserving the translation/lever seed.\n";
  std::cout
      << "  --estimate-orientation-gravity-prior estimates camera-IMU "
         "rotation, gyro bias, and gravity from camera-pose angular velocity "
         "and IMU samples before building the main problem.\n";
  std::cout
      << "  --estimate-multi-imu-orientation-gravity-prior reruns the "
         "orientation/gravity initializer after the IMU chain prior and "
         "averages all IMUs in the reference body frame for multi-IMU cold "
         "starts.\n";
  std::cout
      << "  --estimate-camera-translation-prior estimates cam0 translation "
         "from camera-pose acceleration and reference-IMU accelerometer data "
         "after the orientation/gravity prior. It is experimental and stays "
         "off unless requested.\n";
  std::cout
      << "  --estimate-multi-imu-translation-prior runs after the IMU chain "
         "rotation/time prior and jointly initializes cam0 translation, "
         "non-reference IMU lever arms, and per-IMU accelerometer biases from "
         "accelerometer data. The optional camera/lever/bias sigma flags add "
         "weak Tikhonov priors to test physical regularization. It is "
         "experimental and off unless requested.\n";
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
         "free variables: p=pose, b=bias, e=camera extrinsic, t=camera time "
         "shift and explicit non-reference IMU time-offset variables, "
         "g=gravity, i=non-reference IMU "
         "extrinsics; use '-' or 'none' for an evaluation-only stage. When no "
         "non-empty stage mask contains i, IMU extrinsics keep the legacy "
         "always-free behavior for non-empty stages.\n";
  std::cout
      << "  --stage-imu-extrinsic-translation-bounds-m and "
         "--stage-imu-extrinsic-rotation-bounds-rad override the global "
         "non-reference IMU extrinsic bounds per stage; use -1 to disable a "
         "bound for a stage.\n";
  std::cout
      << "  --extrinsic-manifold enables an experimental SO(3) update for "
         "camera/IMU extrinsics. The default keeps the legacy additive "
         "rotation-vector update until the SO(3) path is validated.\n";
  std::cout
      << "  --pose-control-manifold enables an experimental SO(3) update for "
         "pose spline control rotations. The default keeps the legacy additive "
         "rotation-vector update for benchmark compatibility.\n";
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
      << "  Nonmonotonic Ceres trust-region steps are enabled by default "
         "with max_consecutive_nonmonotonic_steps=20; use "
         "--no-solver-use-nonmonotonic-steps to reproduce monotonic runs.\n";
  std::cout
      << "  --solver-restore-best-state records the lowest accepted cost state "
         "inside each solve and restores it if a later nonmonotonic accepted "
         "state finishes with higher cost.\n";
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
            << " camera_chain_priors=" << build.camera_chain_priors
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

void printSolverTiming(const std::string &prefix,
                       const ceres::Solver::Summary &summary) {
  const std::streamsize old_precision = std::cout.precision();
  std::cout << std::setprecision(17) << prefix
            << " total_time_s=" << summary.total_time_in_seconds
            << " preprocessor_time_s="
            << summary.preprocessor_time_in_seconds
            << " minimizer_time_s=" << summary.minimizer_time_in_seconds
            << " postprocessor_time_s="
            << summary.postprocessor_time_in_seconds
            << " num_successful_steps=" << summary.num_successful_steps
            << " num_unsuccessful_steps=" << summary.num_unsuccessful_steps
            << "\n";
  std::cout.precision(old_precision);
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

void setBiasControls(const ceres_cam_imu::Vec3 &bias,
                     std::vector<ceres_cam_imu::BiasControlBlock> *controls) {
  if (!controls) {
    return;
  }
  for (ceres_cam_imu::BiasControlBlock &control : *controls) {
    control.values = {bias.x(), bias.y(), bias.z()};
  }
}

ceres_cam_imu::ImuExtrinsicBlock imuExtrinsicFromKalibrTib(
    const ceres_cam_imu::Mat4 &T_i_b) {
  ceres_cam_imu::ImuExtrinsicBlock block;
  const ceres_cam_imu::Mat3 R_i_b = T_i_b.block<3, 3>(0, 0);
  const ceres_cam_imu::Vec3 t_i_b = T_i_b.block<3, 1>(0, 3);
  const ceres_cam_imu::Vec3 r_b = -R_i_b.transpose() * t_i_b;
  const ceres_cam_imu::Vec3 r_i_b =
      ceres_cam_imu::rotationMatrixToVector(R_i_b);
  for (int i = 0; i < 3; ++i) {
    block.values[static_cast<std::size_t>(i)] = r_b(i);
    block.values[static_cast<std::size_t>(i + 3)] = r_i_b(i);
  }
  return block;
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

void initializeImuExtrinsicsFromKalibr(
    const ceres_cam_imu::KalibrResult &kalibr, const std::size_t imu_count,
    ceres_cam_imu::CalibrationState *state) {
  if (!state || kalibr.imu_T_i_b.size() < imu_count) {
    return;
  }
  if (state->imu_extrinsics.size() < imu_count) {
    state->imu_extrinsics.resize(imu_count);
  }
  for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
    const ceres_cam_imu::ImuExtrinsicBlock block =
        imuExtrinsicFromKalibrTib(kalibr.imu_T_i_b[imu_index]);
    if (imu_index == 0) {
      state->imu_extrinsic = block;
    }
    state->imu_extrinsics[imu_index] = block;
  }
  std::cout << "initialized IMU chain extrinsics from Kalibr: count="
            << imu_count << "\n";
}

void initializeImuExtrinsicsFromResult(
    const ceres_cam_imu::CalibrationResultFile &result,
    const std::size_t imu_count, ceres_cam_imu::CalibrationState *state) {
  if (!state || result.imu_extrinsics.size() < imu_count) {
    return;
  }
  if (state->imu_extrinsics.size() < imu_count) {
    state->imu_extrinsics.resize(imu_count);
  }
  for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
    ceres_cam_imu::ImuExtrinsicBlock block;
    const ceres_cam_imu::CalibrationResultImuExtrinsic &source =
        result.imu_extrinsics[imu_index];
    for (int i = 0; i < 3; ++i) {
      block.values[static_cast<std::size_t>(i)] = source.r_b(i);
      block.values[static_cast<std::size_t>(i + 3)] = source.r_i_b(i);
    }
    if (imu_index == 0) {
      state->imu_extrinsic = block;
    }
    state->imu_extrinsics[imu_index] = block;
  }
  std::cout << "initialized IMU chain extrinsics from Ceres result: count="
            << imu_count << "\n";
}

void initializeImuTimeOffsets(
    const std::vector<double> &time_offsets_s, const std::size_t imu_count,
    const std::string &source, ceres_cam_imu::CalibrationState *state) {
  if (!state || imu_count == 0 || time_offsets_s.empty()) {
    return;
  }
  state->imu_time_offsets_s.assign(imu_count, 0.0);
  const std::size_t count = std::min(imu_count, time_offsets_s.size());
  for (std::size_t imu_index = 0; imu_index < count; ++imu_index) {
    state->imu_time_offsets_s[imu_index] = time_offsets_s[imu_index];
  }
  state->imu_time_offsets_s[0] = 0.0;
  const std::streamsize old_precision = std::cout.precision();
  std::cout << std::setprecision(17)
            << "initialized IMU time offsets from " << source << ":";
  for (std::size_t imu_index = 0; imu_index < state->imu_time_offsets_s.size();
       ++imu_index) {
    std::cout << " imu" << imu_index << "="
              << state->imu_time_offsets_s[imu_index];
  }
  std::cout << "\n";
  std::cout.precision(old_precision);
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
  if (state.imu_time_offsets_s.size() > 1) {
    const std::streamsize old_precision = std::cout.precision();
    std::cout << std::setprecision(17);
    for (std::size_t imu_index = 0; imu_index < state.imu_time_offsets_s.size();
         ++imu_index) {
      const double offset_s = state.imu_time_offsets_s[imu_index];
      std::cout << "imu_time_offset_state imu=" << imu_index
                << " time_offset_s=" << offset_s
                << " camera0_effective_time_shift_s="
                << (state.camera_time_shift_s.value - offset_s) << "\n";
    }
    std::cout.precision(old_precision);
  }
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

ceres_cam_imu::CalibrationResidualStatistics printFinalResidualStatistics(
    const std::vector<ceres_cam_imu::CameraObservationDataset> &cameras,
    const std::vector<ceres_cam_imu::ImuObservationDataset> &imus,
    const ceres_cam_imu::CalibrationOptions &options,
    const ceres_cam_imu::CalibrationState &state) {
  const ceres_cam_imu::CalibrationResidualStatistics stats =
      ceres_cam_imu::evaluateCalibrationResidualStatistics(cameras, imus,
                                                           options, state);
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
  std::vector<std::string> topology_corner_csvs =
      argValues(argc, argv, "--corners");
  if (topology_corner_csvs.empty()) {
    topology_corner_csvs.push_back(corners_csv);
  }
  std::vector<std::string> topology_imu_data_csvs =
      argValues(argc, argv, "--imu-data");
  if (topology_imu_data_csvs.empty()) {
    topology_imu_data_csvs.push_back(imu_data);
  }
  const std::size_t topology_camera_count = topology_corner_csvs.size();
  const std::size_t topology_imu_count = topology_imu_data_csvs.size();
  ceres_cam_imu::CalibrationOptions options;
  CornerDefaultTopology corner_default_topology =
      CornerDefaultTopology::kOneCameraOneImu;
  if (corner_defaults) {
    corner_default_topology =
        applyCornerDefaults(topology_camera_count, topology_imu_count,
                            &options);
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
  if (hasFlag(argc, argv, "--solver-use-nonmonotonic-steps")) {
    options.solver_use_nonmonotonic_steps = true;
  }
  if (hasFlag(argc, argv, "--no-solver-use-nonmonotonic-steps")) {
    options.solver_use_nonmonotonic_steps = false;
  }
  if (hasFlag(argc, argv, "--solver-restore-best-state")) {
    options.solver_restore_best_state = true;
  }
  if (hasFlag(argc, argv, "--no-solver-restore-best-state")) {
    options.solver_restore_best_state = false;
  }
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
      << " function_tolerance=" << options.solver_function_tolerance
      << " gradient_tolerance=" << options.solver_gradient_tolerance
      << " parameter_tolerance=" << options.solver_parameter_tolerance
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
      << options.solver_max_consecutive_nonmonotonic_steps
      << " restore_best_state=" << options.solver_restore_best_state << "\n";
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
  options.camera_time_offset_buffer_s =
      doubleArg(argc, argv, "--camera-time-offset-buffer",
                options.camera_time_offset_buffer_s);
  if (options.time_padding_s < 0.0 ||
      options.camera_time_offset_buffer_s < -1.0) {
    std::cerr << "--time-padding must be non-negative and "
                 "--camera-time-offset-buffer must be -1 or non-negative\n";
    return 2;
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
  const bool estimate_imu_chain_prior =
      !hasFlag(argc, argv, "--no-estimate-imu-chain-prior");
  ceres_cam_imu::ImuChainInitializerOptions imu_chain_prior_options;
  const std::string imu_chain_prior_max_offset_arg =
      argValue(argc, argv, "--imu-chain-prior-max-offset-s");
  const bool corner_default_multi_imu =
      corner_default_topology == CornerDefaultTopology::kOneCameraMultiImu ||
      corner_default_topology == CornerDefaultTopology::kMultiCameraMultiImu;
  if (corner_defaults &&
      corner_default_multi_imu &&
      imu_chain_prior_max_offset_arg.empty()) {
    imu_chain_prior_options.use_full_overlap_time_offset_search = true;
  }
  if (!imu_chain_prior_max_offset_arg.empty()) {
    imu_chain_prior_options.max_time_offset_search_s =
        std::stod(imu_chain_prior_max_offset_arg);
    imu_chain_prior_options.use_full_overlap_time_offset_search = false;
  }
  imu_chain_prior_options.sample_stride =
      intArg(argc, argv, "--imu-chain-prior-stride",
             imu_chain_prior_options.sample_stride);
  imu_chain_prior_options.min_samples =
      intArg(argc, argv, "--imu-chain-prior-min-samples",
             imu_chain_prior_options.min_samples);
  imu_chain_prior_options.min_rotation_excitation =
      doubleArg(argc, argv, "--imu-chain-prior-min-excitation",
                imu_chain_prior_options.min_rotation_excitation);
  const bool explicitly_estimate_imu_chain_lever_prior =
      hasFlag(argc, argv, "--estimate-imu-chain-lever-prior");
  const bool explicitly_disable_imu_chain_lever_prior =
      hasFlag(argc, argv, "--no-estimate-imu-chain-lever-prior");
  imu_chain_prior_options.estimate_lever_arms =
      (explicitly_estimate_imu_chain_lever_prior ||
       (corner_defaults && corner_default_multi_imu)) &&
      !explicitly_disable_imu_chain_lever_prior;
  imu_chain_prior_options.min_lever_excitation =
      doubleArg(argc, argv, "--imu-chain-prior-min-lever-excitation",
                imu_chain_prior_options.min_lever_excitation);
  imu_chain_prior_options.max_lever_arm_norm_m =
      doubleArg(argc, argv, "--imu-chain-prior-max-lever-m",
                imu_chain_prior_options.max_lever_arm_norm_m);
  const std::string imu_chain_prior_max_lever_accel_rms_arg =
      argValue(argc, argv, "--imu-chain-prior-max-lever-accel-rms");
  if (corner_defaults && corner_default_multi_imu &&
      imu_chain_prior_options.estimate_lever_arms &&
      imu_chain_prior_max_lever_accel_rms_arg.empty()) {
    imu_chain_prior_options.max_lever_accel_rms_m_s2 = 0.5;
  }
  if (!imu_chain_prior_max_lever_accel_rms_arg.empty()) {
    imu_chain_prior_options.max_lever_accel_rms_m_s2 =
        std::stod(imu_chain_prior_max_lever_accel_rms_arg);
  }
  imu_chain_prior_options.refine_with_ceres =
      !hasFlag(argc, argv, "--no-imu-chain-prior-ceres-refine");
  imu_chain_prior_options.refine_max_iterations =
      intArg(argc, argv, "--imu-chain-prior-refine-iterations",
             imu_chain_prior_options.refine_max_iterations);
  imu_chain_prior_options.refine_with_accel =
      hasFlag(argc, argv, "--imu-chain-prior-refine-with-accel");
  imu_chain_prior_options.refine_gyro_weight =
      doubleArg(argc, argv, "--imu-chain-prior-refine-gyro-weight",
                imu_chain_prior_options.refine_gyro_weight);
  imu_chain_prior_options.refine_accel_weight =
      doubleArg(argc, argv, "--imu-chain-prior-refine-accel-weight",
                imu_chain_prior_options.refine_accel_weight);
  imu_chain_prior_options.refine_rotation_bound_rad =
      doubleArg(argc, argv, "--imu-chain-prior-refine-rotation-bound-rad",
                imu_chain_prior_options.refine_rotation_bound_rad);
  imu_chain_prior_options.refine_lever_prior_sigma_m =
      doubleArg(argc, argv, "--imu-chain-prior-refine-lever-sigma-m",
                imu_chain_prior_options.refine_lever_prior_sigma_m);
  imu_chain_prior_options.refine_accel_bias_prior_sigma_m_s2 =
      doubleArg(argc, argv, "--imu-chain-prior-refine-accel-bias-sigma",
                imu_chain_prior_options.refine_accel_bias_prior_sigma_m_s2);
  const bool refine_imu_chain_rotation_after_translation_prior =
      hasFlag(argc, argv,
              "--imu-chain-prior-refine-rotation-after-translation-prior");
  if (refine_imu_chain_rotation_after_translation_prior &&
      !imu_chain_prior_options.refine_with_accel) {
    std::cerr << "--imu-chain-prior-refine-rotation-after-translation-prior "
                 "requires --imu-chain-prior-refine-with-accel\n";
    return 2;
  }
  if ((!imu_chain_prior_options.use_full_overlap_time_offset_search &&
       imu_chain_prior_options.max_time_offset_search_s < 0.0) ||
      imu_chain_prior_options.sample_stride <= 0 ||
      imu_chain_prior_options.min_samples <= 0 ||
      imu_chain_prior_options.min_rotation_excitation < 0.0 ||
      imu_chain_prior_options.min_lever_excitation < 0.0 ||
      imu_chain_prior_options.max_lever_arm_norm_m <= 0.0 ||
      imu_chain_prior_options.max_lever_accel_rms_m_s2 < -1.0 ||
      imu_chain_prior_options.refine_max_iterations < 0 ||
      imu_chain_prior_options.refine_gyro_weight <= 0.0 ||
      imu_chain_prior_options.refine_accel_weight <= 0.0 ||
      imu_chain_prior_options.refine_rotation_bound_rad < -1.0 ||
      (imu_chain_prior_options.refine_lever_prior_sigma_m != -1.0 &&
       imu_chain_prior_options.refine_lever_prior_sigma_m <= 0.0) ||
      (imu_chain_prior_options.refine_accel_bias_prior_sigma_m_s2 != -1.0 &&
       imu_chain_prior_options.refine_accel_bias_prior_sigma_m_s2 <= 0.0)) {
    std::cerr << "IMU chain prior options must use non-negative bounded offset/"
                 "excitation, positive stride/min-samples/max-lever, and "
                 "non-negative refine iterations; max lever accel RMS must be "
                 "-1 or non-negative; accel-refine weights must be positive, "
                 "rotation bound must be -1 or non-negative, and accel-refine "
                 "sigmas must be -1 or positive\n";
    return 2;
  }
  options.fix_pose_controls = hasFlag(argc, argv, "--fix-poses");
  options.fix_bias_controls = hasFlag(argc, argv, "--fix-biases");
  options.fix_camera_extrinsic = hasFlag(argc, argv, "--fix-camera-extrinsic");
  options.fix_camera_chain_extrinsics =
      hasFlag(argc, argv, "--fix-camera-chain-extrinsics");
  options.camera_chain_translation_sigma_m =
      doubleArg(argc, argv, "--camera-chain-translation-sigma-m",
                options.camera_chain_translation_sigma_m);
  options.camera_chain_rotation_sigma_rad =
      doubleArg(argc, argv, "--camera-chain-rotation-sigma-rad",
                options.camera_chain_rotation_sigma_rad);
  if (options.camera_chain_translation_sigma_m <= 0.0 ||
      options.camera_chain_rotation_sigma_rad <= 0.0) {
    std::cerr << "camera-chain prior sigmas must be positive\n";
    return 2;
  }
  options.fix_imu_extrinsics = hasFlag(argc, argv, "--fix-imu-extrinsics");
  options.imu_extrinsic_translation_bound_m =
      doubleArg(argc, argv, "--imu-extrinsic-translation-bound-m",
                options.imu_extrinsic_translation_bound_m);
  options.imu_extrinsic_rotation_bound_rad =
      doubleArg(argc, argv, "--imu-extrinsic-rotation-bound-rad",
                options.imu_extrinsic_rotation_bound_rad);
  if ((options.imu_extrinsic_translation_bound_m != -1.0 &&
       options.imu_extrinsic_translation_bound_m <= 0.0) ||
      (options.imu_extrinsic_rotation_bound_rad != -1.0 &&
       options.imu_extrinsic_rotation_bound_rad <= 0.0)) {
    std::cerr << "IMU extrinsic bounds must be -1 or positive\n";
    return 2;
  }
  options.use_extrinsic_manifold = hasFlag(argc, argv, "--extrinsic-manifold");
  options.use_pose_control_manifold =
      hasFlag(argc, argv, "--pose-control-manifold");
  options.fix_time_shift = hasFlag(argc, argv, "--fix-time-shift");
  options.fix_gravity = hasFlag(argc, argv, "--fix-gravity");
  options.imu_model = ceres_cam_imu::parseImuCalibrationModel(
      argValue(argc, argv, "--imu-model", "calibrated"));
  options.imu_time_offset_bound_s =
      doubleArg(argc, argv, "--imu-time-offset-bound-s",
                options.imu_time_offset_bound_s);
  if (options.imu_time_offset_bound_s < 0.0) {
    std::cerr << "--imu-time-offset-bound-s must be non-negative\n";
    return 2;
  }
  options.fix_imu_intrinsics = hasFlag(argc, argv, "--fix-imu-intrinsics");
  options.estimate_gravity_length =
      hasFlag(argc, argv, "--estimate-gravity-length");
  options.camera_loss_type =
      parseLossType(argValue(argc, argv, "--camera-loss",
                             robustLossTypeName(options.camera_loss_type)));
  options.gyro_loss_type = parseLossType(
      argValue(argc, argv, "--gyro-loss",
               robustLossTypeName(options.gyro_loss_type)));
  options.accel_loss_type = parseLossType(
      argValue(argc, argv, "--accel-loss",
               robustLossTypeName(options.accel_loss_type)));
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
  const std::vector<double> stage_imu_extrinsic_translation_bounds_m =
      parseDoubleList(argValue(
          argc, argv, "--stage-imu-extrinsic-translation-bounds-m"));
  const std::vector<double> stage_imu_extrinsic_rotation_bounds_rad =
      parseDoubleList(
          argValue(argc, argv, "--stage-imu-extrinsic-rotation-bounds-rad"));
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
  if ((!stage_imu_extrinsic_translation_bounds_m.empty() ||
       !stage_imu_extrinsic_rotation_bounds_rad.empty()) &&
      !staged) {
    std::cerr << "--stage-imu-extrinsic-*-bounds require --staged\n";
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
  if (!stage_imu_extrinsic_translation_bounds_m.empty() &&
      stage_imu_extrinsic_translation_bounds_m.size() != stage_count) {
    std::cerr << "--stage-imu-extrinsic-translation-bounds-m expects "
              << stage_count << " comma-separated values\n";
    return 2;
  }
  if (!stage_imu_extrinsic_rotation_bounds_rad.empty() &&
      stage_imu_extrinsic_rotation_bounds_rad.size() != stage_count) {
    std::cerr << "--stage-imu-extrinsic-rotation-bounds-rad expects "
              << stage_count << " comma-separated values\n";
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
  for (const double bound : stage_imu_extrinsic_translation_bounds_m) {
    if (bound < -1.0) {
      std::cerr << "--stage-imu-extrinsic-translation-bounds-m values must be "
                   "-1 or non-negative\n";
      return 2;
    }
  }
  for (const double bound : stage_imu_extrinsic_rotation_bounds_rad) {
    if (bound < -1.0) {
      std::cerr << "--stage-imu-extrinsic-rotation-bounds-rad values must be "
                   "-1 or non-negative\n";
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
  std::vector<std::string> imu_yamls = argValues(argc, argv, "--imu");
  std::vector<std::string> imu_data_csvs = argValues(argc, argv, "--imu-data");
  if (imu_yamls.empty()) {
    imu_yamls.push_back(imu_yaml);
  }
  if (imu_data_csvs.empty()) {
    imu_data_csvs.push_back(imu_data);
  }
  if (imu_yamls.size() != imu_data_csvs.size()) {
    std::cerr << "the number of --imu YAML files must match --imu-data CSV "
                 "files; first IMU is the reference IMU\n";
    return 2;
  }
  std::vector<ceres_cam_imu::ImuObservationDataset> imus;
  imus.reserve(imu_yamls.size());
  for (std::size_t imu_index = 0; imu_index < imu_yamls.size(); ++imu_index) {
    ceres_cam_imu::ImuObservationDataset imu;
    imu.noise = ceres_cam_imu::readImuNoise(imu_yamls[imu_index]);
    const std::vector<ceres_cam_imu::ImuSample> raw_samples =
        ceres_cam_imu::readImuCsv(imu_data_csvs[imu_index]);
    imu.samples =
        ceres_cam_imu::trimImuSamplesKalibr(raw_samples, imu_trim_edge_count);
    imu.label = "imu" + std::to_string(imu_index);
    imus.push_back(std::move(imu));
  }
  const bool multi_imu = imus.size() > 1;
  const ceres_cam_imu::ImuNoise &imu_noise = imus.front().noise;
  const std::vector<ceres_cam_imu::ImuSample> &imu_samples =
      imus.front().samples;
  const std::string imu_delay_correction_arg =
      argValue(argc, argv, "--imu-delay-correction", "auto");
  bool enable_imu_delay_correction =
      multi_imu && !hasFlag(argc, argv, "--no-imu-delay-correction");
  if (imu_delay_correction_arg == "off" ||
      imu_delay_correction_arg == "false" ||
      imu_delay_correction_arg == "0" || imu_delay_correction_arg == "no") {
    enable_imu_delay_correction = false;
  } else if (imu_delay_correction_arg == "on" ||
             imu_delay_correction_arg == "true" ||
             imu_delay_correction_arg == "1" ||
             imu_delay_correction_arg == "yes" ||
             imu_delay_correction_arg == "auto") {
    enable_imu_delay_correction =
        enable_imu_delay_correction && imu_delay_correction_arg != "off";
  } else {
    std::cerr << "--imu-delay-correction must be auto, on, or off\n";
    return 2;
  }
  const bool requested_imu_time_offset_optimization =
      multi_imu && hasFlag(argc, argv, "--optimize-imu-time-offsets") &&
      !hasFlag(argc, argv, "--fix-imu-time-offsets");
  if (requested_imu_time_offset_optimization &&
      options.imu_model != ceres_cam_imu::ImuCalibrationModel::kCalibrated) {
    if (hasFlag(argc, argv, "--optimize-imu-time-offsets")) {
      std::cerr << "--optimize-imu-time-offsets currently requires "
                   "--imu-model calibrated\n";
      return 2;
    }
    std::cerr << "warning: IMU time offset optimization is disabled for "
                 "non-calibrated IMU models; fixed delay correction remains "
                 "enabled\n";
  }
  options.optimize_imu_time_offsets =
      requested_imu_time_offset_optimization &&
      options.imu_model == ceres_cam_imu::ImuCalibrationModel::kCalibrated;
  if (options.optimize_imu_time_offsets &&
      options.imu_time_offset_bound_s <= 0.0) {
    std::cerr << "--imu-time-offset-bound-s must be positive when IMU time "
                 "offset optimization is enabled\n";
    return 2;
  }
  std::cout << "imu inputs: count=" << imus.size();
  for (std::size_t imu_index = 0; imu_index < imus.size(); ++imu_index) {
    std::cout << " " << imus[imu_index].label << "_samples="
              << imus[imu_index].samples.size();
  }
  std::cout << " delay_correction="
            << (enable_imu_delay_correction ? "enabled" : "disabled")
            << " imu_time_offset_optimization="
            << (options.optimize_imu_time_offsets ? "enabled" : "disabled")
            << " imu_time_offset_bound_s=" << options.imu_time_offset_bound_s
            << "\n";
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
  const std::string initial_time_shift_arg =
      argValue(argc, argv, "--initial-time-shift-s");
  const bool have_explicit_initial_time_shift =
      !initial_time_shift_arg.empty();
  const double explicit_initial_time_shift_s =
      have_explicit_initial_time_shift ? std::stod(initial_time_shift_arg)
                                       : 0.0;
  if (requested_init_from_kalibr && !have_kalibr_result) {
    std::cerr << "--init-from-kalibr requires --kalibr-result\n";
    return 2;
  }
  const bool init_from_kalibr =
      requested_init_from_kalibr && have_kalibr_result;
  if (init_from_kalibr && multi_camera &&
      kalibr.camera_T_ci.size() < cameras.size()) {
    std::cerr << "--init-from-kalibr requires a complete Kalibr camera chain "
              << "for " << cameras.size() << " cameras; found "
              << kalibr.camera_T_ci.size() << " camera T_ci entries\n";
    return 2;
  }
  if (init_from_kalibr && multi_imu && kalibr.imu_T_i_b.size() < imus.size()) {
    std::cerr << "--init-from-kalibr requires a complete Kalibr IMU chain for "
              << imus.size() << " IMUs; found " << kalibr.imu_T_i_b.size()
              << " T_ib entries\n";
    return 2;
  }
  const bool auto_init_from_camchain =
      multi_camera && !init_from_result && !init_from_kalibr &&
      !requested_init_from_camchain;
  const bool init_from_camchain =
      requested_init_from_camchain || auto_init_from_camchain;
  ceres_cam_imu::CameraExtrinsicBlock initial_T_c_b;
  std::vector<ceres_cam_imu::CameraExtrinsicBlock> initial_camera_extrinsics(
      cameras.size());
  std::vector<double> initial_camera_time_shifts(cameras.size(),
                                                 options.initial_camera_time_shift_s);
  std::vector<ceres_cam_imu::Mat4> initial_camera_chain_T_ci_c0(
      cameras.size(), ceres_cam_imu::Mat4::Identity());
  bool have_initial_camera_blocks = false;
  bool have_initial_camera_chain_prior = false;
  ceres_cam_imu::Vec3 initial_gravity = ceres_cam_imu::Vec3::Zero();
  bool have_initial_gravity = false;
  bool did_initialize_from_camchain = false;
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
    if (multi_camera) {
      initial_camera_chain_T_ci_c0 =
          cameraChainPriorsFromExtrinsics(initial_camera_extrinsics);
      have_initial_camera_chain_prior = true;
    }
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
    for (std::size_t camera_index = 0; camera_index < cameras.size();
         ++camera_index) {
      const ceres_cam_imu::Mat4 &camera_T_ci =
          camera_index < kalibr.camera_T_ci.size()
              ? kalibr.camera_T_ci[camera_index]
              : kalibr.T_ci;
      const ceres_cam_imu::Vec6 camera_T_c_b =
          ceres_cam_imu::matrixToPose6(camera_T_ci);
      for (int i = 0; i < 6; ++i) {
        initial_camera_extrinsics[camera_index]
            .values[static_cast<std::size_t>(i)] = camera_T_c_b(i);
      }
      initial_camera_time_shifts[camera_index] =
          camera_index < kalibr.camera_timeshift_cam_to_imu_s.size()
              ? kalibr.camera_timeshift_cam_to_imu_s[camera_index]
              : kalibr.timeshift_cam_to_imu_s;
    }
    initial_T_c_b = initial_camera_extrinsics[0];
    options.initial_camera_time_shift_s = initial_camera_time_shifts[0];
    have_initial_camera_blocks = true;
    if (multi_camera) {
      initial_camera_chain_T_ci_c0 =
          cameraChainPriorsFromExtrinsics(initial_camera_extrinsics);
      have_initial_camera_chain_prior = true;
    }
    initial_gravity = kalibr.gravity;
    have_initial_gravity = true;
  }
  if (init_from_camchain) {
    std::vector<ceres_cam_imu::CamchainImuPrior> camchain_priors(
        cameras.size());
    std::vector<ceres_cam_imu::Mat4> camera_T_c_b(
        cameras.size(), ceres_cam_imu::Mat4::Identity());
    std::vector<char> have_camera_T_c_b(cameras.size(), 0);
    int direct_initializations = 0;
    int chained_initializations = 0;
    int observation_fallback_initializations = 0;

    for (std::size_t camera_index = 0; camera_index < cameras.size();
         ++camera_index) {
      const std::string &camera_yaml =
          shared_camchain_yaml ? cam_yamls.front() : cam_yamls[camera_index];
      camchain_priors[camera_index] = ceres_cam_imu::readCamchainImuPrior(
          camera_yaml,
          shared_camchain_yaml ? static_cast<int>(camera_index) : 0);
      const ceres_cam_imu::CamchainImuPrior &camchain_prior =
          camchain_priors[camera_index];
      if (camchain_prior.has_timeshift_cam_imu) {
        initial_camera_time_shifts[camera_index] =
            camchain_prior.timeshift_cam_imu_s;
      }
      if (camchain_prior.has_T_cam_imu) {
        camera_T_c_b[camera_index] = camchain_prior.T_cam_imu;
        have_camera_T_c_b[camera_index] = 1;
        ++direct_initializations;
      }
    }

    for (std::size_t camera_index = 1; camera_index < cameras.size();
         ++camera_index) {
      if (!have_camera_T_c_b[camera_index] &&
          have_camera_T_c_b[camera_index - 1] &&
          camchain_priors[camera_index].has_T_cam_cam_prev) {
        camera_T_c_b[camera_index] =
            camchain_priors[camera_index].T_cam_cam_prev *
            camera_T_c_b[camera_index - 1];
        have_camera_T_c_b[camera_index] = 1;
        ++chained_initializations;
      }
    }

    if (!std::all_of(have_camera_T_c_b.begin(), have_camera_T_c_b.end(),
                     [](const char value) { return value != 0; }) &&
        !poses.empty() && have_camera_T_c_b[0]) {
      for (std::size_t camera_index = 1; camera_index < cameras.size();
           ++camera_index) {
        if (have_camera_T_c_b[camera_index]) {
          continue;
        }
        ceres_cam_imu::Mat4 T_ci_c0 = ceres_cam_imu::Mat4::Identity();
        int used_frames = 0;
        double mean_rms_px = 0.0;
        if (!estimateCameraBaselineFromObservations(
                cameras[camera_index], poses, &T_ci_c0, &used_frames,
                &mean_rms_px)) {
          continue;
        }
        printYellowWarning(
            "falling back to observation-based camera-chain initialization for "
            "camera " +
            std::to_string(camera_index) +
            " because camchain YAML does not provide T_cam_imu/T_cn_cnm1");
        camera_T_c_b[camera_index] = T_ci_c0 * camera_T_c_b[0];
        have_camera_T_c_b[camera_index] = 1;
        ++observation_fallback_initializations;
        const std::streamsize old_precision = std::cout.precision();
        std::cout << std::setprecision(17)
                  << "camera-chain observation fallback: camera="
                  << camera_index << " frames=" << used_frames
                  << " mean_pnp_rms_px=" << mean_rms_px
                  << " translation_m=" << T_ci_c0(0, 3) << " "
                  << T_ci_c0(1, 3) << " " << T_ci_c0(2, 3) << "\n";
        std::cout.precision(old_precision);
      }
    }

    const bool have_complete_camera_chain =
        std::all_of(have_camera_T_c_b.begin(), have_camera_T_c_b.end(),
                    [](const char value) { return value != 0; });
    if (have_camera_T_c_b[0]) {
      initial_T_c_b = matrixToExtrinsicBlock(camera_T_c_b[0]);
      initial_camera_extrinsics[0] = initial_T_c_b;
      options.initial_camera_time_shift_s = initial_camera_time_shifts[0];
    }
    if (have_complete_camera_chain) {
      for (std::size_t camera_index = 0; camera_index < cameras.size();
           ++camera_index) {
        initial_camera_extrinsics[camera_index] =
            matrixToExtrinsicBlock(camera_T_c_b[camera_index]);
      }
      initial_T_c_b = initial_camera_extrinsics[0];
      options.initial_camera_time_shift_s = initial_camera_time_shifts[0];
      have_initial_camera_blocks = true;
      initial_camera_chain_T_ci_c0 =
          cameraChainPriorsFromExtrinsics(initial_camera_extrinsics);
      have_initial_camera_chain_prior = multi_camera;
      did_initialize_from_camchain = true;
      const std::streamsize old_precision = std::cout.precision();
      std::cout << std::setprecision(17)
                << "initialized from camchain"
                << (auto_init_from_camchain ? " (auto multi-camera)" : "")
                << ": direct=" << direct_initializations
                << " chained=" << chained_initializations
                << " observation_fallback="
                << observation_fallback_initializations
                << " time_shift_s=" << options.initial_camera_time_shift_s
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
    } else {
      std::ostringstream missing;
      for (std::size_t camera_index = 0; camera_index < cameras.size();
           ++camera_index) {
        if (!have_camera_T_c_b[camera_index]) {
          if (missing.tellp() > 0) {
            missing << ",";
          }
          missing << camera_index;
        }
      }
      printYellowWarning(
          "camchain initialization is incomplete for camera(s) " +
          missing.str() +
          "; falling back to the regular cold-start path for missing camera "
          "extrinsics");
      if (options.fix_camera_chain_extrinsics) {
        std::cerr << "--fix-camera-chain-extrinsics requires complete "
                     "camera-chain initialization\n";
        return 2;
      }
    }
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
    time_shift_options.max_search_s =
        doubleArg(argc, argv, "--time-shift-max-search-s",
                  time_shift_options.max_search_s);
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
              << " max_search_s=" << time_shift_options.max_search_s
              << " discrete_shift_samples=" << time_shift.discrete_shift_samples
              << " sample_dt_s=" << time_shift.sample_dt_s
              << " samples=" << time_shift.num_samples
              << " peak_correlation=" << time_shift.peak_correlation
              << " second_best_discrete_shift_samples="
              << time_shift.second_best_discrete_shift_samples
              << " second_best_correlation="
              << time_shift.second_best_correlation
              << " zero_lag_correlation=" << time_shift.zero_lag_correlation
              << " boundary_peak_rejected="
              << time_shift.boundary_peak_rejected
              << " predicted_norm_rms=" << time_shift.predicted_norm_rms
              << " measured_norm_rms=" << time_shift.measured_norm_rms;
    if (have_kalibr_result) {
      std::cout << " kalibr_delta_s="
                << (time_shift.shift_s - kalibr.timeshift_cam_to_imu_s);
    }
    std::cout << "\n";
    std::cout.precision(old_precision);
  }
  if (have_explicit_initial_time_shift) {
    options.initial_camera_time_shift_s = explicit_initial_time_shift_s;
    if (!initial_camera_time_shifts.empty()) {
      initial_camera_time_shifts[0] = explicit_initial_time_shift_s;
    }
    const std::streamsize old_precision = std::cout.precision();
    std::cout << std::setprecision(17)
              << "using explicit initial time shift: shift_s="
              << explicit_initial_time_shift_s << "\n";
    std::cout.precision(old_precision);
  }

  ceres_cam_imu::Vec3 initial_reference_accel_bias =
      ceres_cam_imu::Vec3::Zero();
  bool have_initial_reference_accel_bias = false;
  const bool estimate_orientation_gravity_prior =
      hasFlag(argc, argv, "--estimate-orientation-gravity-prior");
  const bool estimate_multi_imu_orientation_gravity_prior =
      hasFlag(argc, argv, "--estimate-multi-imu-orientation-gravity-prior");
  const bool estimate_camera_translation_prior =
      hasFlag(argc, argv, "--estimate-camera-translation-prior") &&
      estimate_orientation_gravity_prior &&
      !hasFlag(argc, argv, "--no-estimate-camera-translation-prior") &&
      !init_from_kalibr && !init_from_result && !did_initialize_from_camchain;
  const bool have_multi_imu_translation_gravity_seed =
      estimate_orientation_gravity_prior || have_initial_gravity;
  const bool estimate_multi_imu_translation_prior =
      hasFlag(argc, argv, "--estimate-multi-imu-translation-prior") &&
      multi_imu && have_multi_imu_translation_gravity_seed &&
      !init_from_kalibr;
  if (refine_imu_chain_rotation_after_translation_prior &&
      !estimate_multi_imu_translation_prior) {
    std::cerr << "--imu-chain-prior-refine-rotation-after-translation-prior "
                 "requires --estimate-multi-imu-translation-prior and an "
                 "orientation/gravity or gravity seed\n";
    return 2;
  }

  if (estimate_orientation_gravity_prior) {
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

  if (estimate_camera_translation_prior) {
    try {
      ceres_cam_imu::CameraTranslationInitializerOptions
          translation_options;
      translation_options.pose_knots_per_second =
          doubleArg(argc, argv, "--camera-translation-prior-pose-kps",
                    translation_options.pose_knots_per_second);
      translation_options.pose_fit_regularization =
          doubleArg(argc, argv, "--camera-translation-prior-fit-lambda",
                    translation_options.pose_fit_regularization);
      translation_options.sample_stride =
          intArg(argc, argv, "--camera-translation-prior-stride",
                 translation_options.sample_stride);
      translation_options.min_lever_jacobian_norm =
          doubleArg(argc, argv, "--camera-translation-prior-min-lever-norm",
                    translation_options.min_lever_jacobian_norm);
      translation_options.max_translation_norm_m =
          doubleArg(argc, argv, "--camera-translation-prior-max-norm",
                    translation_options.max_translation_norm_m);
      if (translation_options.pose_knots_per_second <= 0.0 ||
          translation_options.pose_fit_regularization < 0.0 ||
          translation_options.sample_stride <= 0 ||
          translation_options.min_lever_jacobian_norm < 0.0 ||
          translation_options.max_translation_norm_m <= 0.0) {
        std::cerr << "camera translation prior options are out of range\n";
        return 2;
      }
      const ceres_cam_imu::CameraTranslationInitializerResult translation =
          ceres_cam_imu::estimateCameraTranslationAndAccelBiasPrior(
              poses, imu_samples, initial_T_c_b, initial_gravity,
              options.initial_camera_time_shift_s, translation_options);
      for (int i = 0; i < 3; ++i) {
        initial_T_c_b.values[static_cast<std::size_t>(i)] =
            translation.t_c_b_m(i);
      }
      initial_camera_extrinsics[0] = initial_T_c_b;
      have_initial_camera_blocks = true;
      initial_reference_accel_bias = translation.accel_bias_m_s2;
      have_initial_reference_accel_bias = true;

      const std::streamsize old_precision = std::cout.precision();
      std::cout << std::setprecision(17)
                << "estimated camera translation prior: samples="
                << translation.num_samples
                << " pose_kps="
                << translation_options.pose_knots_per_second
                << " fit_lambda="
                << translation_options.pose_fit_regularization
                << " stride=" << translation_options.sample_stride
                << " min_lever_norm="
                << translation_options.min_lever_jacobian_norm
                << " t_c_b_m=" << translation.t_c_b_m.transpose()
                << " accel_bias_m_s2="
                << translation.accel_bias_m_s2.transpose()
                << " accel_rms_m_s2=" << translation.accel_rms_m_s2
                << " singular_values="
                << translation.singular_values.transpose()
                << " pose_fit_rms_translation_m="
                << translation.pose_fit_rms_translation_m
                << " pose_fit_rms_rotation_rad="
                << translation.pose_fit_rms_rotation_rad
                << " pose_fit_boundary_anchor_observations="
                << translation.pose_fit_boundary_anchor_observations;
      if (have_kalibr_result) {
        const ceres_cam_imu::Mat4 initial_T =
            ceres_cam_imu::pose6ToMatrix(initial_T_c_b);
        std::cout << " kalibr_translation_delta_m="
                  << (initial_T.block<3, 1>(0, 3) -
                      kalibr.T_ci.block<3, 1>(0, 3))
                         .norm();
      }
      std::cout << "\n";
      std::cout.precision(old_precision);
    } catch (const std::exception &e) {
      std::cerr << "failed to estimate camera translation prior: "
                << e.what() << "\n";
    }
  }

  if (have_initial_camera_blocks && have_initial_camera_chain_prior) {
    initial_camera_extrinsics[0] = initial_T_c_b;
    applyCameraChainPriorsToInitialExtrinsics(initial_camera_chain_T_ci_c0,
                                             &initial_camera_extrinsics);
    if (options.fix_camera_chain_extrinsics) {
      options.camera_chain_T_ci_c0_prior = initial_camera_chain_T_ci_c0;
      std::cout << "camera-chain extrinsics fixed: priors="
                << (initial_camera_chain_T_ci_c0.size() > 0
                        ? initial_camera_chain_T_ci_c0.size() - 1
                        : 0)
                << " translation_sigma_m="
                << options.camera_chain_translation_sigma_m
                << " rotation_sigma_rad="
                << options.camera_chain_rotation_sigma_rad << "\n";
    }
  } else if (options.fix_camera_chain_extrinsics) {
    std::cerr << "--fix-camera-chain-extrinsics requires a complete "
                 "multi-camera initialization from camchain, Kalibr, result, or "
                 "observation fallback\n";
    return 2;
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
    std::ostringstream imu_chain_prior_offset_search;
    if (imu_chain_prior_options.use_full_overlap_time_offset_search) {
      imu_chain_prior_offset_search << "full-overlap";
    } else {
      imu_chain_prior_offset_search
          << "bounded:" << imu_chain_prior_options.max_time_offset_search_s;
    }
    std::cout << "corner defaults active: topology="
              << cornerDefaultTopologyName(corner_default_topology)
              << " cameras=" << cameras.size() << " imus=" << imus.size()
              << " pose_kps="
              << options.pose_knots_per_second
              << " bias_kps=" << options.bias_knots_per_second
              << " max_iterations=" << options.max_iterations
              << " timeoffset_padding_s=" << options.time_padding_s
              << " camera_time_offset_buffer_s="
              << options.camera_time_offset_buffer_s
              << " absolute_cost_tolerance="
              << options.solver_absolute_cost_change_tolerance
              << " absolute_step_tolerance="
              << options.solver_absolute_step_tolerance
              << " use_nonmonotonic_steps="
              << options.solver_use_nonmonotonic_steps
              << " max_consecutive_nonmonotonic_steps="
              << options.solver_max_consecutive_nonmonotonic_steps
              << " imu_extrinsic_translation_bound_m="
              << options.imu_extrinsic_translation_bound_m
              << " imu_extrinsic_rotation_bound_rad="
              << options.imu_extrinsic_rotation_bound_rad
              << " imu_chain_prior_offset_search="
              << imu_chain_prior_offset_search.str()
              << " imu_chain_lever_prior="
              << imu_chain_prior_options.estimate_lever_arms
              << " imu_chain_lever_accel_rms_gate="
              << imu_chain_prior_options.max_lever_accel_rms_m_s2
              << " imu_chain_accel_refine="
              << imu_chain_prior_options.refine_with_accel
              << " imu_trim_edge_count=" << imu_trim_edge_count
              << " losses="
              << robustLossTypeName(options.camera_loss_type) << ":"
              << options.camera_loss_width << ","
              << robustLossTypeName(options.gyro_loss_type) << ":"
              << options.gyro_loss_width << ","
              << robustLossTypeName(options.accel_loss_type) << ":"
              << options.accel_loss_width
              << "\n";
  }

  ceres_cam_imu::CalibrationState state =
      multi_imu
          ? ceres_cam_imu::initializeCalibrationState(cameras, imus, options)
          : multi_camera
                ? ceres_cam_imu::initializeCalibrationState(cameras,
                                                            imu_samples,
                                                            options)
                : ceres_cam_imu::initializeCalibrationState(images,
                                                            imu_samples,
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
  if (have_initial_reference_accel_bias) {
    setBiasControls(initial_reference_accel_bias, &state.accel_bias_controls);
    if (!state.accel_bias_controls_by_imu.empty()) {
      setBiasControls(initial_reference_accel_bias,
                      &state.accel_bias_controls_by_imu[0]);
    }
  }
  if (init_from_result) {
    initializeImuIntrinsicsFromResult(init_result, &state);
    if (multi_imu) {
      initializeImuExtrinsicsFromResult(init_result, imus.size(), &state);
    }
    if (enable_imu_delay_correction) {
      initializeImuTimeOffsets(init_result.imu_time_offsets_s, imus.size(),
                               "Ceres result", &state);
    }
  }
  if (init_from_kalibr) {
    initializeImuIntrinsicsFromKalibr(kalibr, &state);
    if (multi_imu) {
      initializeImuExtrinsicsFromKalibr(kalibr, imus.size(), &state);
      if (enable_imu_delay_correction) {
        initializeImuTimeOffsets(kalibr.imu_time_offset_s, imus.size(),
                                 "Kalibr result", &state);
      }
    }
  }
  if (multi_imu && estimate_imu_chain_prior && !init_from_kalibr) {
    // The gyro-correlation chain prior also runs when seeding from a Ceres
    // result, so that non-reference IMUs not covered by the result (e.g. a
    // single-IMU cam+imu0 seed extended to 4 IMUs) still get a rotation/lever
    // initial guess. IMUs already provided by the result are left untouched.
    const std::size_t result_imu_count =
        init_from_result ? init_result.imu_extrinsics.size() : 0;
    try {
      ceres_cam_imu::ImuChainInitializerOptions initial_imu_chain_options =
          imu_chain_prior_options;
      if (refine_imu_chain_rotation_after_translation_prior) {
        initial_imu_chain_options.refine_with_accel = false;
      }
      const ceres_cam_imu::ImuChainInitializerResult imu_chain_prior =
          ceres_cam_imu::estimateImuChainPrior(imus,
                                               initial_imu_chain_options);
      for (const ceres_cam_imu::ImuChainInitializerPairResult &imu_result :
           imu_chain_prior.imu_results) {
        if (imu_result.imu_index >= state.imu_extrinsics.size()) {
          continue;
        }
        const bool keep_result_extrinsic =
            init_from_result && imu_result.imu_index < result_imu_count;
        if (!keep_result_extrinsic) {
          state.imu_extrinsics[imu_result.imu_index] =
              ceres_cam_imu::imuExtrinsicFromRotationAndLever(
                  imu_result.R_i_b, imu_result.r_b);
        }
        if (enable_imu_delay_correction) {
          if (state.imu_time_offsets_s.size() < imus.size()) {
            state.imu_time_offsets_s.resize(imus.size(), 0.0);
          }
          if (imu_result.imu_index >= init_result.imu_time_offsets_s.size()) {
            state.imu_time_offsets_s[imu_result.imu_index] =
                imu_result.time_offset_s;
          }
          state.imu_time_offsets_s[0] = 0.0;
        }
        const std::streamsize old_precision = std::cout.precision();
        std::cout << std::setprecision(17)
                  << "estimated IMU chain prior: imu="
                  << imu_result.imu_index
                  << " time_offset_s=" << imu_result.time_offset_s
                  << " delay_correction_applied="
                  << (enable_imu_delay_correction ? 1 : 0)
                  << " time_offset_search_radius_s="
                  << imu_result.time_offset_search_radius_s
                  << " max_search_lag_samples="
                  << imu_result.max_search_lag_samples
                  << " discrete_shift_samples="
                  << imu_result.discrete_shift_samples
                  << " sample_dt_s=" << imu_result.sample_dt_s
                  << " matched_samples=" << imu_result.matched_samples
                  << " peak_correlation=" << imu_result.peak_correlation
                  << " time_offset_boundary_peak_rejected="
                  << (imu_result.time_offset_boundary_peak_rejected ? 1 : 0)
                  << " rejected_discrete_shift_samples="
                  << imu_result.rejected_discrete_shift_samples
                  << " rejected_matched_samples="
                  << imu_result.rejected_matched_samples
                  << " rejected_peak_correlation="
                  << imu_result.rejected_peak_correlation
                  << " r_i_b=" << imu_result.r_i_b.transpose()
                  << " lever_estimated="
                  << (imu_result.lever_arm_estimated ? 1 : 0)
                  << " r_b_m=" << imu_result.r_b.transpose()
                  << " gyro_bias_rad_s="
                  << imu_result.gyro_bias_rad_s.transpose()
                  << " singular_values="
                  << imu_result.singular_values.transpose()
                  << " gyro_rms_rad_s=" << imu_result.gyro_rms_rad_s
                  << " accel_bias_delta_body_m_s2="
                  << imu_result.accel_bias_delta_body_m_s2.transpose()
                  << " lever_singular_values="
                  << imu_result.lever_singular_values.transpose()
                  << " accel_rms_m_s2=" << imu_result.accel_rms_m_s2
                  << " refine_iterations="
                  << imu_result.refine_iterations
                  << " refine_final_cost=" << imu_result.refine_final_cost
                  << " accel_refined=" << (imu_result.accel_refined ? 1 : 0)
                  << " accel_refine_rms_m_s2="
                  << imu_result.accel_refine_rms_m_s2
                  << "\n";
        std::cout.precision(old_precision);
      }
    } catch (const std::exception &e) {
      std::cerr << "failed to estimate IMU chain prior: " << e.what()
                << "\n";
      return 2;
    }
  }

  if (multi_imu && estimate_multi_imu_orientation_gravity_prior &&
      !init_from_kalibr) {
    if (poses.empty()) {
      std::cerr << "--estimate-multi-imu-orientation-gravity-prior requires "
                   "--corner-poses poses.csv\n";
      return 2;
    }
    try {
      ceres_cam_imu::OrientationGravityInitializerOptions
          orientation_options;
      orientation_options.pose_knots_per_second =
          doubleArg(argc, argv, "--orientation-prior-pose-kps",
                    orientation_options.pose_knots_per_second);
      orientation_options.pose_fit_regularization =
          doubleArg(argc, argv, "--orientation-prior-fit-lambda",
                    orientation_options.pose_fit_regularization);
      orientation_options.pose_fit_boundary_anchors =
          !hasFlag(argc, argv, "--no-orientation-prior-boundary-anchors");
      orientation_options.refine_with_ceres = false;
      if (orientation_options.pose_knots_per_second <= 0.0 ||
          orientation_options.pose_fit_regularization < 0.0) {
        std::cerr << "multi-IMU orientation prior kps must be positive and "
                     "fit lambda must be non-negative\n";
        return 2;
      }
      const ceres_cam_imu::OrientationGravityInitializerResult orientation =
          ceres_cam_imu::estimateMultiImuOrientationGravityAndGyroBiasPrior(
              poses, imus, state.imu_extrinsics, state.imu_time_offsets_s,
              state.T_c_b, options.initial_camera_time_shift_s,
              orientation_options);
      for (int i = 0; i < 3; ++i) {
        state.T_c_b.values[static_cast<std::size_t>(3 + i)] =
            orientation.T_c_b.values[static_cast<std::size_t>(3 + i)];
        if (!state.camera_extrinsics.empty()) {
          state.camera_extrinsics[0].values[static_cast<std::size_t>(3 + i)] =
              orientation.T_c_b.values[static_cast<std::size_t>(3 + i)];
        }
        state.gravity.values[static_cast<std::size_t>(i)] =
            orientation.gravity_m_s2(i);
      }
      initial_T_c_b = state.T_c_b;
      initial_gravity = orientation.gravity_m_s2;
      have_initial_gravity = true;
      options.initial_gyro_bias_rad_s = orientation.gyro_bias_rad_s;

      const std::streamsize old_precision = std::cout.precision();
      std::cout << std::setprecision(17)
                << "estimated multi-IMU orientation/gravity prior: samples="
                << orientation.num_samples
                << " pose_kps="
                << orientation_options.pose_knots_per_second
                << " fit_lambda="
                << orientation_options.pose_fit_regularization
                << " boundary_anchors="
                << (orientation_options.pose_fit_boundary_anchors ? 1 : 0)
                << " gyro_bias_rad_s="
                << orientation.gyro_bias_rad_s.transpose()
                << " gravity_m_s2="
                << orientation.gravity_m_s2.transpose()
                << " gravity_mean_norm_m_s2="
                << orientation.gravity_mean_norm_m_s2
                << " gyro_rms_rad_s=" << orientation.gyro_rms_rad_s
                << " singular_values="
                << orientation.singular_values.transpose()
                << " pose_fit_rms_translation_m="
                << orientation.pose_fit_rms_translation_m
                << " pose_fit_rms_rotation_rad="
                << orientation.pose_fit_rms_rotation_rad
                << " pose_fit_boundary_anchor_observations="
                << orientation.pose_fit_boundary_anchor_observations;
      if (have_kalibr_result) {
        const ceres_cam_imu::Mat4 T_c_b_matrix =
            ceres_cam_imu::pose6ToMatrix(orientation.T_c_b);
        const ceres_cam_imu::Mat3 dR =
            T_c_b_matrix.block<3, 3>(0, 0) *
            kalibr.T_ci.block<3, 3>(0, 0).transpose();
        const double cos_angle =
            std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0);
        constexpr double kPi = 3.14159265358979323846;
        const double angle_deg = std::acos(cos_angle) * 180.0 / kPi;
        std::cout << " kalibr_rotation_delta_deg=" << angle_deg
                  << " kalibr_gravity_delta_norm="
                  << (orientation.gravity_m_s2 - kalibr.gravity).norm();
      }
      std::cout << "\n";
      std::cout.precision(old_precision);
    } catch (const std::exception &e) {
      std::cerr << "failed to estimate multi-IMU orientation/gravity prior: "
                << e.what() << "\n";
      return 2;
    }
  }

  if (estimate_multi_imu_translation_prior) {
    if (poses.empty()) {
      std::cerr << "--estimate-multi-imu-translation-prior requires "
                   "--corner-poses poses.csv\n";
      return 2;
    }
    try {
      ceres_cam_imu::MultiImuTranslationInitializerOptions
          translation_options;
      translation_options.pose_knots_per_second =
          doubleArg(argc, argv, "--camera-translation-prior-pose-kps",
                    translation_options.pose_knots_per_second);
      translation_options.pose_fit_regularization =
          doubleArg(argc, argv, "--camera-translation-prior-fit-lambda",
                    translation_options.pose_fit_regularization);
      translation_options.sample_stride =
          intArg(argc, argv, "--camera-translation-prior-stride",
                 translation_options.sample_stride);
      translation_options.min_lever_jacobian_norm =
          doubleArg(argc, argv, "--camera-translation-prior-min-lever-norm",
                    translation_options.min_lever_jacobian_norm);
      translation_options.max_translation_norm_m =
          doubleArg(argc, argv, "--camera-translation-prior-max-norm",
                    translation_options.max_translation_norm_m);
      translation_options.max_lever_arm_norm_m =
          doubleArg(argc, argv, "--multi-imu-translation-prior-max-lever-m",
                    translation_options.max_lever_arm_norm_m);
      translation_options.camera_prior_sigma_m = doubleArg(
          argc, argv, "--multi-imu-translation-prior-camera-sigma-m",
          translation_options.camera_prior_sigma_m);
      translation_options.lever_prior_sigma_m = doubleArg(
          argc, argv, "--multi-imu-translation-prior-lever-sigma-m",
          translation_options.lever_prior_sigma_m);
      translation_options.accel_bias_prior_sigma_m_s2 =
          doubleArg(argc, argv,
                    "--multi-imu-translation-prior-accel-bias-sigma",
                    translation_options.accel_bias_prior_sigma_m_s2);
      if (translation_options.pose_knots_per_second <= 0.0 ||
          translation_options.pose_fit_regularization < 0.0 ||
          translation_options.sample_stride <= 0 ||
          translation_options.min_lever_jacobian_norm < 0.0 ||
          translation_options.max_translation_norm_m <= 0.0 ||
          translation_options.max_lever_arm_norm_m <= 0.0 ||
          (translation_options.camera_prior_sigma_m != -1.0 &&
           translation_options.camera_prior_sigma_m <= 0.0) ||
          (translation_options.lever_prior_sigma_m != -1.0 &&
           translation_options.lever_prior_sigma_m <= 0.0) ||
          (translation_options.accel_bias_prior_sigma_m_s2 != -1.0 &&
           translation_options.accel_bias_prior_sigma_m_s2 <= 0.0)) {
        std::cerr << "multi-IMU translation prior options are out of range\n";
        return 2;
      }
      const ceres_cam_imu::MultiImuTranslationInitializerResult translation =
          ceres_cam_imu::estimateMultiImuCameraTranslationAndLeverPrior(
              poses, imus, state.imu_extrinsics, state.imu_time_offsets_s,
              state.T_c_b, initial_gravity, options.initial_camera_time_shift_s,
              translation_options);
      for (int i = 0; i < 3; ++i) {
        state.T_c_b.values[static_cast<std::size_t>(i)] =
            translation.t_c_b_m(i);
        if (!state.camera_extrinsics.empty()) {
          state.camera_extrinsics[0].values[static_cast<std::size_t>(i)] =
              translation.t_c_b_m(i);
        }
      }
      for (std::size_t imu_index = 0;
           imu_index < translation.r_b_m.size() &&
           imu_index < state.imu_extrinsics.size();
           ++imu_index) {
        if (imu_index == 0) {
          continue;
        }
        for (int i = 0; i < 3; ++i) {
          state.imu_extrinsics[imu_index]
              .values[static_cast<std::size_t>(i)] =
              translation.r_b_m[imu_index](i);
        }
      }
      for (std::size_t imu_index = 0;
           imu_index < translation.accel_bias_m_s2.size(); ++imu_index) {
        if (imu_index == 0) {
          setBiasControls(translation.accel_bias_m_s2[imu_index],
                          &state.accel_bias_controls);
        }
        if (imu_index < state.accel_bias_controls_by_imu.size()) {
          setBiasControls(translation.accel_bias_m_s2[imu_index],
                          &state.accel_bias_controls_by_imu[imu_index]);
        }
      }

      const std::streamsize old_precision = std::cout.precision();
      std::cout << std::setprecision(17)
                << "estimated multi-IMU translation prior: samples="
                << translation.num_samples
                << " pose_kps="
                << translation_options.pose_knots_per_second
                << " fit_lambda="
                << translation_options.pose_fit_regularization
                << " stride=" << translation_options.sample_stride
                << " min_lever_norm="
                << translation_options.min_lever_jacobian_norm
                << " camera_prior_sigma_m="
                << translation_options.camera_prior_sigma_m
                << " lever_prior_sigma_m="
                << translation_options.lever_prior_sigma_m
                << " accel_bias_prior_sigma_m_s2="
                << translation_options.accel_bias_prior_sigma_m_s2
                << " t_c_b_m=" << translation.t_c_b_m.transpose()
                << " accel_rms_m_s2=" << translation.accel_rms_m_s2
                << " singular_values="
                << translation.singular_values.transpose()
                << " pose_fit_rms_translation_m="
                << translation.pose_fit_rms_translation_m
                << " pose_fit_rms_rotation_rad="
                << translation.pose_fit_rms_rotation_rad
                << " pose_fit_boundary_anchor_observations="
                << translation.pose_fit_boundary_anchor_observations;
      for (std::size_t imu_index = 0; imu_index < translation.r_b_m.size();
           ++imu_index) {
        std::cout << " imu" << imu_index
                  << "_r_b_m=" << translation.r_b_m[imu_index].transpose()
                  << " imu" << imu_index << "_accel_bias_m_s2="
                  << translation.accel_bias_m_s2[imu_index].transpose();
      }
      if (have_kalibr_result) {
        const ceres_cam_imu::Mat4 initial_T =
            ceres_cam_imu::pose6ToMatrix(state.T_c_b);
        std::cout << " kalibr_translation_delta_m="
                  << (initial_T.block<3, 1>(0, 3) -
                      kalibr.T_ci.block<3, 1>(0, 3))
                         .norm();
      }
      std::cout << "\n";
      std::cout.precision(old_precision);
    } catch (const std::exception &e) {
      std::cerr << "failed to estimate multi-IMU translation prior: "
                << e.what() << "\n";
      return 2;
    }
  }

  if (multi_imu && estimate_imu_chain_prior &&
      refine_imu_chain_rotation_after_translation_prior && !init_from_kalibr) {
    try {
      const ceres_cam_imu::ImuChainInitializerResult imu_chain_prior =
          ceres_cam_imu::estimateImuChainPrior(imus,
                                               imu_chain_prior_options);
      for (const ceres_cam_imu::ImuChainInitializerPairResult &imu_result :
           imu_chain_prior.imu_results) {
        if (imu_result.imu_index >= state.imu_extrinsics.size()) {
          continue;
        }
        for (int i = 0; i < 3; ++i) {
          state.imu_extrinsics[imu_result.imu_index]
              .values[static_cast<std::size_t>(3 + i)] =
              imu_result.r_i_b(i);
        }
        if (enable_imu_delay_correction) {
          if (state.imu_time_offsets_s.size() < imus.size()) {
            state.imu_time_offsets_s.resize(imus.size(), 0.0);
          }
          state.imu_time_offsets_s[imu_result.imu_index] =
              imu_result.time_offset_s;
          state.imu_time_offsets_s[0] = 0.0;
        }
        const std::streamsize old_precision = std::cout.precision();
        std::cout << std::setprecision(17)
                  << "estimated post-translation IMU chain rotation prior: imu="
                  << imu_result.imu_index
                  << " time_offset_s=" << imu_result.time_offset_s
                  << " delay_correction_applied="
                  << (enable_imu_delay_correction ? 1 : 0)
                  << " time_offset_search_radius_s="
                  << imu_result.time_offset_search_radius_s
                  << " max_search_lag_samples="
                  << imu_result.max_search_lag_samples
                  << " discrete_shift_samples="
                  << imu_result.discrete_shift_samples
                  << " sample_dt_s=" << imu_result.sample_dt_s
                  << " matched_samples=" << imu_result.matched_samples
                  << " peak_correlation=" << imu_result.peak_correlation
                  << " time_offset_boundary_peak_rejected="
                  << (imu_result.time_offset_boundary_peak_rejected ? 1 : 0)
                  << " r_i_b=" << imu_result.r_i_b.transpose()
                  << " preserved_translation_m="
                  << ceres_cam_imu::Vec3(
                         state.imu_extrinsics[imu_result.imu_index].values[0],
                         state.imu_extrinsics[imu_result.imu_index].values[1],
                         state.imu_extrinsics[imu_result.imu_index].values[2])
                         .transpose()
                  << " gyro_bias_rad_s="
                  << imu_result.gyro_bias_rad_s.transpose()
                  << " gyro_rms_rad_s=" << imu_result.gyro_rms_rad_s
                  << " accel_refined=" << (imu_result.accel_refined ? 1 : 0)
                  << " accel_refine_rms_m_s2="
                  << imu_result.accel_refine_rms_m_s2
                  << "\n";
        std::cout.precision(old_precision);
      }
    } catch (const std::exception &e) {
      std::cerr << "failed to estimate post-translation IMU chain rotation "
                   "prior: "
                << e.what() << "\n";
      return 2;
    }
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
    ceres_cam_imu::applyStageImuExtrinsicBounds(
        stage_imu_extrinsic_translation_bounds_m,
        stage_imu_extrinsic_rotation_bounds_rad, &stages);
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
            multi_imu
                ? ceres_cam_imu::buildCalibrationProblem(
                      cameras, imus, stage.options, &state, &stage_problem)
                : multi_camera
                ? ceres_cam_imu::buildCalibrationProblem(
                      cameras, imu_noise, imu_samples, stage.options, &state,
                      &stage_problem)
                : ceres_cam_imu::buildCalibrationProblem(
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
                " imu_translation_bound_m=" +
                std::to_string(
                    stage.options.imu_extrinsic_translation_bound_m) +
                " imu_rotation_bound_rad=" +
                std::to_string(stage.options.imu_extrinsic_rotation_bound_rad) +
                "]: ",
            stage_build);
      }
      return 0;
    }

    bool all_stages_usable = true;
    for (std::size_t stage_index = 0; stage_index < stages.size();
         ++stage_index) {
      const ceres_cam_imu::CalibrationStage &stage = stages[stage_index];
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
                << stage.options.solver_absolute_parameter_tolerance
                << " imu_translation_bound_m="
                << stage.options.imu_extrinsic_translation_bound_m
                << " imu_rotation_bound_rad="
                << stage.options.imu_extrinsic_rotation_bound_rad
                << " solver_restore_best_state="
                << stage.options.solver_restore_best_state << "\n";
      const ceres_cam_imu::CalibrationStageResult stage_result =
          multi_imu
              ? ceres_cam_imu::solveCalibrationStage(cameras, imus, stage,
                                                     &state)
              : multi_camera
              ? ceres_cam_imu::solveCalibrationStage(
                    cameras, std::vector<ceres_cam_imu::ImuObservationDataset>{
                                 imus.front()},
                    stage, &state)
              : ceres_cam_imu::solveCalibrationStage(intrinsics, imu_noise,
                                                     images, imu_samples, stage,
                                                     &state);
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
      printSolverTiming("stage timing [" + stage_result.name + "]: ",
                        stage_result.solver);
      if (!stage_result.solver.IsSolutionUsable() && stop_on_stage_failure) {
        std::cerr << "stage failure stopped calibration: " << stage_result.name
                  << "\n";
        all_stages_usable = false;
        break;
      }
      all_stages_usable =
          all_stages_usable && stage_result.solver.IsSolutionUsable();
    }
    // Sync the first-camera vector slots from the optimized scalar blocks (see
    // note in the single-stage path) so residual statistics use the solved
    // extrinsic/time-shift instead of the stale initial seed.
    if (!state.camera_extrinsics.empty()) {
      state.camera_extrinsics[0] = state.T_c_b;
    }
    if (!state.camera_time_shifts.empty()) {
      state.camera_time_shifts[0] = state.camera_time_shift_s;
    }
    printFinalState(state, have_kalibr_result, kalibr);
    const ceres_cam_imu::CalibrationResidualStatistics residual_stats =
        multi_imu ? printFinalResidualStatistics(cameras, imus, options, state)
                  : multi_camera ? printFinalResidualStatistics(cameras,
                                                                imu_noise,
                                                                imu_samples,
                                                                options, state)
                                 : printFinalResidualStatistics(
                                       intrinsics, imu_noise, images,
                                       imu_samples, options, state);
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
      multi_imu
          ? ceres_cam_imu::buildCalibrationProblem(cameras, imus, options,
                                                   &state, &problem)
          : multi_camera
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
  printSolverTiming("solver timing: ", summary);
  // The first camera always optimizes the scalar state.T_c_b /
  // camera_time_shift_s blocks (single-camera build path), even in multi-IMU
  // runs. The camera_extrinsics[0] / camera_time_shifts[0] vector slots are
  // only seeded with the initial values at setup, so they must be synced back
  // from the optimized scalars after solving; otherwise downstream consumers
  // that read the vector (e.g. multi-camera/multi-IMU residual statistics) use
  // the stale initial extrinsic/time-shift and report inflated reprojection.
  if (!state.camera_extrinsics.empty()) {
    state.camera_extrinsics[0] = state.T_c_b;
  }
  if (!state.camera_time_shifts.empty()) {
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
  if (multi_imu) {
    for (std::size_t imu_index = 0; imu_index < state.imu_extrinsics.size();
         ++imu_index) {
      const ceres_cam_imu::ImuExtrinsicBlock &imu_extrinsic =
          imu_index == 0 ? state.imu_extrinsic
                         : state.imu_extrinsics[imu_index];
      const double imu_time_offset_s =
          imu_index < state.imu_time_offsets_s.size()
              ? state.imu_time_offsets_s[imu_index]
              : 0.0;
      std::cout << "imu_chain_state imu=" << imu_index
                << " r_b_m=" << imu_extrinsic.values[0] << " "
                << imu_extrinsic.values[1] << " " << imu_extrinsic.values[2]
                << " r_i_b=" << imu_extrinsic.values[3] << " "
                << imu_extrinsic.values[4] << " " << imu_extrinsic.values[5]
                << " time_offset_s=" << imu_time_offset_s
                << " camera0_effective_time_shift_s="
                << (state.camera_time_shift_s.value - imu_time_offset_s)
                << "\n";
    }
  }
  const ceres_cam_imu::CalibrationResidualStatistics residual_stats =
      multi_imu ? printFinalResidualStatistics(cameras, imus, options, state)
                : multi_camera ? printFinalResidualStatistics(cameras,
                                                              imu_noise,
                                                              imu_samples,
                                                              options, state)
                               : printFinalResidualStatistics(
                                     intrinsics, imu_noise, images,
                                     imu_samples, options, state);
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
