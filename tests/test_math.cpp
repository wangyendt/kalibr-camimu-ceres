#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ceres_cam_imu/camera/camera_model.h"
#include "ceres_cam_imu/camera/pinhole_radtan.h"
#include "ceres_cam_imu/core/se3.h"
#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/initialization/orientation_gravity_initializer.h"
#include "ceres_cam_imu/initialization/pose_spline_fit.h"
#include "ceres_cam_imu/initialization/time_shift_initializer.h"
#include "ceres_cam_imu/io/calibration_result_reader.h"
#include "ceres_cam_imu/io/calibration_result_writer.h"
#include "ceres_cam_imu/io/config_reader.h"
#include "ceres_cam_imu/io/imu_csv_reader.h"
#include "ceres_cam_imu/optimizer/calibration_problem.h"
#include "ceres_cam_imu/optimizer/staged_optimizer.h"
#include "ceres_cam_imu/optimizer/state_snapshot.h"
#include "ceres_cam_imu/processing/dataset_processing.h"
#include "ceres_cam_imu/residuals/accelerometer_residual.h"
#include "ceres_cam_imu/residuals/bias_motion_prior.h"
#include "ceres_cam_imu/residuals/camera_reprojection_residual.h"
#include "ceres_cam_imu/residuals/gyroscope_residual.h"
#include "ceres_cam_imu/residuals/imu_model.h"
#include "ceres_cam_imu/residuals/pose_motion_prior.h"
#include "ceres_cam_imu/residuals/time_shift_prior.h"
#include "ceres_cam_imu/trajectory/spline_eval.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"
#include "ceres_cam_imu/variables/imu_intrinsics.h"

int main() {
  const Eigen::Vector3d r = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d R = ceres_cam_imu::rotationVectorToMatrix(r);
  assert((R - Eigen::Matrix3d::Identity()).norm() < 1e-12);

  ceres_cam_imu::CameraIntrinsics intr;
  intr.fx = 100.0;
  intr.fy = 100.0;
  intr.cx = 320.0;
  intr.cy = 200.0;
  ceres_cam_imu::PinholeRadtanCamera camera(intr);
  const ceres_cam_imu::Vec2 px =
      camera.project(ceres_cam_imu::Vec3(0.0, 0.0, 1.0));
  assert(std::abs(px.x() - 320.0) < 1e-12);
  assert(std::abs(px.y() - 200.0) < 1e-12);

  auto make_camera_intrinsics = [](const std::string &camera_model,
                                   const std::string &distortion_model,
                                   std::vector<double> intrinsics,
                                   std::vector<double> distortion_coeffs) {
    ceres_cam_imu::CameraIntrinsics model;
    model.camera_model = camera_model;
    model.distortion_model = distortion_model;
    model.intrinsics = intrinsics;
    model.distortion_coeffs = distortion_coeffs;
    if (camera_model == "pinhole") {
      model.fx = intrinsics[0];
      model.fy = intrinsics[1];
      model.cx = intrinsics[2];
      model.cy = intrinsics[3];
    } else if (camera_model == "omni") {
      model.xi = intrinsics[0];
      model.fx = intrinsics[1];
      model.fy = intrinsics[2];
      model.cx = intrinsics[3];
      model.cy = intrinsics[4];
    } else if (camera_model == "eucm") {
      model.alpha = intrinsics[0];
      model.beta = intrinsics[1];
      model.fx = intrinsics[2];
      model.fy = intrinsics[3];
      model.cx = intrinsics[4];
      model.cy = intrinsics[5];
    } else if (camera_model == "ds") {
      model.xi = intrinsics[0];
      model.alpha = intrinsics[1];
      model.fx = intrinsics[2];
      model.fy = intrinsics[3];
      model.cx = intrinsics[4];
      model.cy = intrinsics[5];
    }
    if (distortion_model == "radtan" || distortion_model == "equidistant") {
      model.k1 = distortion_coeffs[0];
      model.k2 = distortion_coeffs[1];
      model.p1 = distortion_coeffs[2];
      model.p2 = distortion_coeffs[3];
    } else if (distortion_model == "fov") {
      model.k1 = distortion_coeffs[0];
    }
    model.width = 640;
    model.height = 400;
    return model;
  };

  auto check_camera_jacobian =
      [](const ceres_cam_imu::CameraIntrinsics &camera_intrinsics) {
        const ceres_cam_imu::CameraModel model(camera_intrinsics);
        const ceres_cam_imu::Vec3 point(0.21, -0.13, 1.7);
        ceres_cam_imu::Vec2 pixel;
        ceres_cam_imu::Mat23 analytic;
        assert(model.projectWithJacobian(point, &pixel, &analytic));
        ceres_cam_imu::Mat23 numeric;
        constexpr double eps = 1e-6;
        for (int col = 0; col < 3; ++col) {
          ceres_cam_imu::Vec3 plus = point;
          ceres_cam_imu::Vec3 minus = point;
          plus(col) += eps;
          minus(col) -= eps;
          ceres_cam_imu::Vec2 pixel_plus;
          ceres_cam_imu::Vec2 pixel_minus;
          assert(model.projectWithJacobian(plus, &pixel_plus, nullptr));
          assert(model.projectWithJacobian(minus, &pixel_minus, nullptr));
          numeric.col(col) = (pixel_plus - pixel_minus) / (2.0 * eps);
        }
        assert((analytic - numeric).norm() < 1e-4);
      };

  const ceres_cam_imu::CameraIntrinsics pinhole_radtan =
      make_camera_intrinsics("pinhole", "radtan", {100.0, 101.0, 320.0, 200.0},
                             {0.01, -0.001, 0.0002, -0.0003});
  const ceres_cam_imu::CameraModel generic_pinhole_radtan(pinhole_radtan);
  ceres_cam_imu::Vec2 generic_px;
  assert(generic_pinhole_radtan.projectWithJacobian(
      ceres_cam_imu::Vec3(0.0, 0.0, 1.0), &generic_px, nullptr));
  assert(std::abs(generic_px.x() - 320.0) < 1e-12);
  assert(std::abs(generic_px.y() - 200.0) < 1e-12);

  check_camera_jacobian(pinhole_radtan);
  check_camera_jacobian(make_camera_intrinsics(
      "pinhole", "none", {100.0, 101.0, 320.0, 200.0}, {}));
  check_camera_jacobian(make_camera_intrinsics(
      "pinhole", "equidistant", {100.0, 101.0, 320.0, 200.0},
      {0.01, -0.002, 0.0003, -0.00004}));
  check_camera_jacobian(make_camera_intrinsics(
      "pinhole", "fov", {100.0, 101.0, 320.0, 200.0}, {0.85}));
  check_camera_jacobian(make_camera_intrinsics(
      "omni", "none", {0.7, 100.0, 101.0, 320.0, 200.0}, {}));
  check_camera_jacobian(make_camera_intrinsics(
      "omni", "radtan", {0.7, 100.0, 101.0, 320.0, 200.0},
      {0.01, -0.001, 0.0002, -0.0003}));
  check_camera_jacobian(make_camera_intrinsics(
      "eucm", "none", {0.55, 1.1, 100.0, 101.0, 320.0, 200.0}, {}));
  check_camera_jacobian(make_camera_intrinsics(
      "ds", "none", {-0.25, 0.55, 100.0, 101.0, 320.0, 200.0}, {}));
  assert(ceres_cam_imu::canonicalDistortionModelName("equi") == "equidistant");
  assert(ceres_cam_imu::parseImuCalibrationModel(
             "scale-misalignment-size-effect") ==
         ceres_cam_imu::ImuCalibrationModel::kScaleMisalignmentSizeEffect);

  const ceres_cam_imu::Mat3 R_i_b = ceres_cam_imu::rotationVectorToMatrix(
      ceres_cam_imu::Vec3(0.1, -0.2, 0.05));
  const ceres_cam_imu::Vec3 omega_b(0.3, -0.1, 0.2);
  const ceres_cam_imu::Vec3 alpha_b(0.05, 0.02, -0.04);
  const ceres_cam_imu::Vec3 h_b(0.2, -0.3, 9.5);
  const ceres_cam_imu::Vec3 r_b(0.01, -0.02, 0.03);
  const ceres_cam_imu::Vec3 imu_bias(0.001, -0.002, 0.003);
  const ceres_cam_imu::Vec3 lever =
      ceres_cam_imu::commonLeverAcceleration(omega_b, alpha_b, r_b);
  assert((ceres_cam_imu::predictScaleMisalignedGyroscope(
              R_i_b, ceres_cam_imu::Mat3::Identity(),
              ceres_cam_imu::Mat3::Identity(), ceres_cam_imu::Mat3::Zero(),
              omega_b, h_b + lever, imu_bias) -
          ceres_cam_imu::predictCalibratedGyroscope(R_i_b, omega_b, imu_bias))
             .norm() < 1e-12);
  assert((ceres_cam_imu::predictScaleMisalignedAccelerometer(
              R_i_b, ceres_cam_imu::Mat3::Identity(), h_b, lever, imu_bias) -
          ceres_cam_imu::predictCalibratedAccelerometer(R_i_b, h_b, lever,
                                                        imu_bias))
             .norm() < 1e-12);
  assert((ceres_cam_imu::predictSizeEffectAccelerometer(
              R_i_b, ceres_cam_imu::Mat3::Identity(), h_b, r_b,
              ceres_cam_imu::Vec3::Zero(), ceres_cam_imu::Vec3::Zero(),
              ceres_cam_imu::Vec3::Zero(), omega_b, alpha_b, imu_bias) -
          ceres_cam_imu::predictCalibratedAccelerometer(R_i_b, h_b, lever,
                                                        imu_bias))
             .norm() < 1e-12);

  ceres_cam_imu::Mat3 M_accel;
  M_accel << 1.02, 0.0, 0.0, -0.01, 0.98, 0.0, 0.015, -0.02, 1.01;
  ceres_cam_imu::Mat3 M_gyro;
  M_gyro << 0.99, 0.0, 0.0, 0.02, 1.01, 0.0, -0.015, 0.01, 0.97;
  ceres_cam_imu::Mat3 A_gyro;
  A_gyro << 0.001, -0.002, 0.003, 0.0005, -0.001, 0.002,
      -0.0007, 0.0009, 0.0012;
  const ceres_cam_imu::Mat3 R_gyro_i =
      ceres_cam_imu::rotationVectorToMatrix(ceres_cam_imu::Vec3(0.02, -0.01,
                                                                0.015));
  const ceres_cam_imu::Vec3 a_b = h_b + lever;
  const ceres_cam_imu::Vec3 kalibr_scale_gyro =
      M_gyro * (R_gyro_i * R_i_b * omega_b) +
      A_gyro * (R_gyro_i * R_i_b * a_b) + imu_bias;
  assert((ceres_cam_imu::predictScaleMisalignedGyroscope(
              R_i_b, R_gyro_i, M_gyro, A_gyro, omega_b, a_b, imu_bias) -
          kalibr_scale_gyro)
             .norm() < 1e-12);
  const ceres_cam_imu::Vec3 kalibr_scale_accel =
      M_accel * (R_i_b * (h_b + lever)) + imu_bias;
  assert((ceres_cam_imu::predictScaleMisalignedAccelerometer(
              R_i_b, M_accel, h_b, lever, imu_bias) -
          kalibr_scale_accel)
             .norm() < 1e-12);
  const ceres_cam_imu::Vec3 rx_i(0.002, -0.001, 0.0005);
  const ceres_cam_imu::Vec3 ry_i(-0.0007, 0.0025, 0.001);
  const ceres_cam_imu::Vec3 rz_i(0.0011, -0.0004, 0.003);
  const ceres_cam_imu::Vec3 rx_b = r_b + R_i_b.transpose() * rx_i;
  const ceres_cam_imu::Vec3 ry_b = r_b + R_i_b.transpose() * ry_i;
  const ceres_cam_imu::Vec3 rz_b = r_b + R_i_b.transpose() * rz_i;
  const ceres_cam_imu::Vec3 lever_x =
      R_i_b *
      ceres_cam_imu::commonLeverAcceleration(omega_b, alpha_b, rx_b);
  const ceres_cam_imu::Vec3 lever_y =
      R_i_b *
      ceres_cam_imu::commonLeverAcceleration(omega_b, alpha_b, ry_b);
  const ceres_cam_imu::Vec3 lever_z =
      R_i_b *
      ceres_cam_imu::commonLeverAcceleration(omega_b, alpha_b, rz_b);
  ceres_cam_imu::Vec3 kalibr_size_axis = R_i_b * h_b;
  kalibr_size_axis.x() += lever_x.x();
  kalibr_size_axis.y() += lever_y.y();
  kalibr_size_axis.z() += lever_z.z();
  const ceres_cam_imu::Vec3 kalibr_size_accel =
      M_accel * kalibr_size_axis + imu_bias;
  assert((ceres_cam_imu::predictSizeEffectAccelerometer(
              R_i_b, M_accel, h_b, r_b, rx_i, ry_i, rz_i, omega_b, alpha_b,
              imu_bias) -
          kalibr_size_accel)
             .norm() < 1e-12);

  ceres_cam_imu::UniformBSpline spline(1, 6, 0.0, 1.0, 10);
  std::vector<Eigen::VectorXd> coeffs(
      static_cast<std::size_t>(spline.numCoefficients()),
      Eigen::VectorXd::Ones(1));
  const Eigen::VectorXd value = spline.evaluate(coeffs, 0.5, 0);
  assert(std::abs(value(0) - 1.0) < 1e-9);

  const ceres_cam_imu::UniformBSpline kalibr_padded_spline =
      ceres_cam_imu::makeSplineForTimes(6, 6, 0.0, 48.53452786, 100.0, 0.04);
  assert(kalibr_padded_spline.numSegments() == 4869);
  assert(kalibr_padded_spline.numCoefficients() == 4874);
  assert(std::abs(kalibr_padded_spline.tMin() + 0.08) < 1e-12);
  assert(std::abs(kalibr_padded_spline.tMax() - 48.61452786) < 1e-12);

  const char *trim_csv_path = "/tmp/ceres_cam_imu_trim_reader_test.csv";
  {
    std::ofstream trim_csv(trim_csv_path);
    trim_csv << "#timestamp [ns],gx,gy,gz,ax,ay,az\n";
    for (int i = 0; i < 10; ++i) {
      trim_csv << i << ",0,0," << i << ",1,2,3\n";
    }
  }
  const std::vector<ceres_cam_imu::ImuSample> raw_imu =
      ceres_cam_imu::readImuCsv(trim_csv_path);
  assert(raw_imu.size() == 10);
  ceres_cam_imu::ImuTrimSummary trim_summary;
  const std::vector<ceres_cam_imu::ImuSample> trimmed_imu =
      ceres_cam_imu::trimImuSamplesKalibr(raw_imu, 2, &trim_summary);
  assert(trimmed_imu.size() == 7);
  assert(std::abs(trimmed_imu.front().timestamp_s - 2e-9) < 1e-18);
  assert(std::abs(trimmed_imu.back().timestamp_s - 8e-9) < 1e-18);
  assert(trim_summary.input_samples == 10);
  assert(trim_summary.output_samples == 7);
  assert(trim_summary.first_kept_index == 2);
  assert(trim_summary.last_kept_index == 8);
  assert(trim_summary.applied);
  const std::vector<ceres_cam_imu::ImuSample> reader_trimmed_imu =
      ceres_cam_imu::readImuCsv(trim_csv_path, 2);
  assert(reader_trimmed_imu.size() == trimmed_imu.size());
  ceres_cam_imu::ImuTrimSummary short_trim_summary;
  const std::vector<ceres_cam_imu::ImuSample> short_trim_imu =
      ceres_cam_imu::trimImuSamplesKalibr(raw_imu, 5, &short_trim_summary);
  assert(short_trim_imu.size() == raw_imu.size());
  assert(!short_trim_summary.applied);
  std::remove(trim_csv_path);

  const char *camchain_yaml_path =
      "/tmp/ceres_cam_imu_camchain_prior_test.yaml";
  {
    std::ofstream camchain_yaml(camchain_yaml_path);
    camchain_yaml << "cam0:\n"
                  << "  T_cam_imu:\n"
                  << "  - [1, 0, 0, 0.015]\n"
                  << "  - [0, 1, 0, 0.029]\n"
                  << "  - [0, 0, 1, 0.004]\n"
                  << "  - [0, 0, 0, 1]\n"
                  << "  camera_model: pinhole\n"
                  << "  distortion_coeffs: [0, 0, 0, 0]\n"
                  << "  distortion_model: radtan\n"
                  << "  intrinsics: [100, 101, 50, 51]\n"
                  << "  resolution: [640, 400]\n"
                  << "  timeshift_cam_imu: -0.5465\n";
  }
  const ceres_cam_imu::CamchainImuPrior camchain_prior =
      ceres_cam_imu::readCamchainImuPrior(camchain_yaml_path);
  assert(camchain_prior.has_T_cam_imu);
  assert(camchain_prior.has_timeshift_cam_imu);
  assert(std::abs(camchain_prior.T_cam_imu(0, 3) - 0.015) < 1e-12);
  assert(std::abs(camchain_prior.T_cam_imu(1, 3) - 0.029) < 1e-12);
  assert(std::abs(camchain_prior.T_cam_imu(2, 3) - 0.004) < 1e-12);
  assert(std::abs(camchain_prior.timeshift_cam_imu_s + 0.5465) < 1e-12);
  std::remove(camchain_yaml_path);

  std::vector<ceres_cam_imu::ImageObservation> processing_images(3);
  processing_images[0].timestamp_s = 1.0;
  processing_images[0].corners.resize(2);
  processing_images[1].timestamp_s = 2.0;
  processing_images[1].corners.resize(3);
  processing_images[2].timestamp_s = 3.0;
  processing_images[2].corners.resize(4);
  assert(ceres_cam_imu::countCornerMeasurements(processing_images) == 9);
  const std::vector<ceres_cam_imu::ImageObservation> limited_images =
      ceres_cam_imu::limitImageObservations(processing_images, 2);
  assert(limited_images.size() == 2);
  assert(ceres_cam_imu::countCornerMeasurements(limited_images) == 5);
  const ceres_cam_imu::TimeRange image_range =
      ceres_cam_imu::timeRange(processing_images);
  assert(image_range.valid);
  assert(image_range.start_s == 1.0);
  assert(image_range.end_s == 3.0);

  ceres_cam_imu::CalibrationOptions gravity_options;
  gravity_options.add_bias_motion_prior = false;
  gravity_options.add_pose_motion_prior = false;
  ceres_cam_imu::CalibrationState gravity_state;
  ceres::Problem gravity_problem;
  const ceres_cam_imu::CalibrationBuildSummary gravity_build =
      ceres_cam_imu::buildCalibrationProblem(intr, ceres_cam_imu::ImuNoise{},
                                             {}, {}, gravity_options,
                                             &gravity_state, &gravity_problem);
  assert(gravity_build.gravity_tangent_size == 2);
  assert(gravity_build.residual_blocks == 0);
  assert(gravity_build.scalar_residuals == 0);
  assert(gravity_build.parameter_blocks == 4);
  assert(gravity_build.active_parameter_blocks == 3);
  assert(gravity_build.ambient_parameters == 16);
  assert(gravity_build.tangent_parameters == 9);
  assert(gravity_build.kalibr_style_error_terms == 2);
  assert(gravity_problem.HasManifold(gravity_state.gravity.data()));
  assert(gravity_problem.ParameterBlockTangentSize(
             gravity_state.gravity.data()) == 2);

  ceres_cam_imu::CalibrationOptions gravity_length_options = gravity_options;
  gravity_length_options.estimate_gravity_length = true;
  ceres_cam_imu::CalibrationState gravity_length_state;
  ceres::Problem gravity_length_problem;
  const ceres_cam_imu::CalibrationBuildSummary gravity_length_build =
      ceres_cam_imu::buildCalibrationProblem(
          intr, ceres_cam_imu::ImuNoise{}, {}, {}, gravity_length_options,
          &gravity_length_state, &gravity_length_problem);
  assert(gravity_length_build.gravity_tangent_size == 3);
  assert(gravity_length_build.active_parameter_blocks == 3);
  assert(gravity_length_build.tangent_parameters == 10);
  assert(gravity_length_build.kalibr_style_error_terms == 3);
  assert(
      !gravity_length_problem.HasManifold(gravity_length_state.gravity.data()));
  assert(gravity_length_problem.ParameterBlockTangentSize(
             gravity_length_state.gravity.data()) == 3);

  ceres_cam_imu::CalibrationOptions fixed_gravity_options = gravity_options;
  fixed_gravity_options.fix_gravity = true;
  ceres_cam_imu::CalibrationState fixed_gravity_state;
  ceres::Problem fixed_gravity_problem;
  const ceres_cam_imu::CalibrationBuildSummary fixed_gravity_build =
      ceres_cam_imu::buildCalibrationProblem(
          intr, ceres_cam_imu::ImuNoise{}, {}, {}, fixed_gravity_options,
          &fixed_gravity_state, &fixed_gravity_problem);
  assert(fixed_gravity_build.gravity_tangent_size == 0);
  assert(fixed_gravity_build.active_parameter_blocks == 2);
  assert(fixed_gravity_build.tangent_parameters == 7);
  assert(fixed_gravity_build.kalibr_style_error_terms == 0);

  ceres_cam_imu::CalibrationOptions scale_imu_options = gravity_options;
  scale_imu_options.imu_model =
      ceres_cam_imu::ImuCalibrationModel::kScaleMisalignment;
  ceres_cam_imu::CalibrationState scale_imu_state;
  ceres::Problem scale_imu_problem;
  const ceres_cam_imu::CalibrationBuildSummary scale_imu_build =
      ceres_cam_imu::buildCalibrationProblem(
          intr, ceres_cam_imu::ImuNoise{}, {}, {}, scale_imu_options,
          &scale_imu_state, &scale_imu_problem);
  assert(scale_imu_build.parameter_blocks == 8);
  assert(scale_imu_build.active_parameter_blocks == 7);
  assert(scale_imu_build.ambient_parameters == 40);
  assert(scale_imu_build.tangent_parameters == 33);

  ceres_cam_imu::CalibrationOptions size_imu_options = gravity_options;
  size_imu_options.imu_model =
      ceres_cam_imu::ImuCalibrationModel::kScaleMisalignmentSizeEffect;
  ceres_cam_imu::CalibrationState size_imu_state;
  ceres::Problem size_imu_problem;
  const ceres_cam_imu::CalibrationBuildSummary size_imu_build =
      ceres_cam_imu::buildCalibrationProblem(
          intr, ceres_cam_imu::ImuNoise{}, {}, {}, size_imu_options,
          &size_imu_state, &size_imu_problem);
  assert(size_imu_build.parameter_blocks == 11);
  assert(size_imu_build.active_parameter_blocks == 9);
  assert(size_imu_build.ambient_parameters == 49);
  assert(size_imu_build.tangent_parameters == 39);
  assert(size_imu_problem.IsParameterBlockConstant(
      size_imu_state.imu_intrinsics.accel_axis_rx_i.data()));

  ceres_cam_imu::CalibrationState fit_state;
  fit_state.pose_spline = ceres_cam_imu::UniformBSpline(6, 6, 0.0, 1.0, 8);
  fit_state.pose_controls.resize(
      static_cast<std::size_t>(fit_state.pose_spline.numCoefficients()));
  std::vector<Eigen::VectorXd> true_controls(
      static_cast<std::size_t>(fit_state.pose_spline.numCoefficients()),
      Eigen::VectorXd::Zero(6));
  for (int i = 0; i < fit_state.pose_spline.numCoefficients(); ++i) {
    true_controls[static_cast<std::size_t>(i)](0) = 0.02 * i;
    true_controls[static_cast<std::size_t>(i)](1) = -0.01 * i;
    true_controls[static_cast<std::size_t>(i)](2) = 0.005 * i;
    true_controls[static_cast<std::size_t>(i)](3) = 0.002 * i;
    true_controls[static_cast<std::size_t>(i)](4) = -0.001 * i;
    true_controls[static_cast<std::size_t>(i)](5) = 0.0015 * i;
  }

  std::vector<ceres_cam_imu::PoseObservation> pose_observations;
  for (int i = 0; i < 80; ++i) {
    const double t = (static_cast<double>(i) + 0.5) / 80.0;
    const ceres_cam_imu::Vec6 pose =
        fit_state.pose_spline.evaluate(true_controls, t, 0);
    ceres_cam_imu::PoseObservation observation;
    observation.timestamp_s = t;
    observation.T_t_c = ceres_cam_imu::pose6ToMatrix(pose);
    pose_observations.push_back(observation);
  }
  ceres_cam_imu::CameraExtrinsicBlock identity_extrinsic;
  identity_extrinsic.values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const ceres_cam_imu::PoseInitializationSummary fit_summary =
      ceres_cam_imu::initializePoseControlsFromCameraPoses(
          pose_observations, identity_extrinsic, &fit_state);
  assert(fit_summary.used_observations == 80);
  assert(fit_summary.skipped_observations == 0);
  assert(fit_summary.rms_translation_m < 1e-6);
  assert(fit_summary.rms_rotation_rad < 1e-6);

  ceres_cam_imu::PoseSplineFitOptions anchored_fit_options;
  anchored_fit_options.regularization = 1e-12;
  anchored_fit_options.motion_regularization = 1e-4;
  anchored_fit_options.motion_regularization_order = 2;
  anchored_fit_options.add_boundary_anchors = true;
  std::vector<ceres_cam_imu::PoseControlBlock> anchored_pose_controls;
  const ceres_cam_imu::PoseSplineFitSummary anchored_fit_summary =
      ceres_cam_imu::fitPoseSplineControlsFromCameraPoses(
          pose_observations, identity_extrinsic, 0.0, fit_state.pose_spline,
          anchored_fit_options, &anchored_pose_controls);
  assert(anchored_fit_summary.boundary_anchor_observations == 2);
  assert(anchored_fit_summary.used_observations == 82);
  assert(anchored_fit_summary.rms_translation_m < 5e-2);
  assert(anchored_fit_summary.rms_rotation_rad < 5e-2);

  bool invalid_pose_fit_order_threw = false;
  try {
    anchored_fit_options.motion_regularization_order =
        fit_state.pose_spline.order();
    (void)ceres_cam_imu::fitPoseSplineControlsFromCameraPoses(
        pose_observations, identity_extrinsic, 0.0, fit_state.pose_spline,
        anchored_fit_options, &anchored_pose_controls);
  } catch (const std::invalid_argument &) {
    invalid_pose_fit_order_threw = true;
  }
  assert(invalid_pose_fit_order_threw);

  ceres_cam_imu::UniformBSpline shift_spline(6, 6, 0.0, 2.0, 20);
  std::vector<Eigen::VectorXd> shift_controls(
      static_cast<std::size_t>(shift_spline.numCoefficients()),
      Eigen::VectorXd::Zero(6));
  for (int i = 0; i < shift_spline.numCoefficients(); ++i) {
    shift_controls[static_cast<std::size_t>(i)](0) = 0.01 * std::sin(0.4 * i);
    shift_controls[static_cast<std::size_t>(i)](1) = 0.01 * std::cos(0.5 * i);
    shift_controls[static_cast<std::size_t>(i)](2) = 0.005 * i;
    shift_controls[static_cast<std::size_t>(i)](3) = 0.15 * std::sin(0.7 * i);
    shift_controls[static_cast<std::size_t>(i)](4) = 0.11 * std::cos(0.37 * i);
    shift_controls[static_cast<std::size_t>(i)](5) =
        0.13 * std::sin(0.53 * i + 0.2);
  }

  auto angular_velocity_at = [&](const double t) {
    const ceres_cam_imu::Vec6 curve =
        shift_spline.evaluate(shift_controls, t, 0);
    const ceres_cam_imu::Vec6 curve_dot =
        shift_spline.evaluate(shift_controls, t, 1);
    return ceres_cam_imu::bodyAngularVelocityFromCurve(curve, curve_dot);
  };

  std::vector<ceres_cam_imu::PoseObservation> shift_pose_observations;
  for (int i = 0; i <= 200; ++i) {
    const double t = 2.0 * static_cast<double>(i) / 200.0;
    const ceres_cam_imu::Vec6 pose =
        shift_spline.evaluate(shift_controls, t, 0);
    ceres_cam_imu::PoseObservation observation;
    observation.timestamp_s = t;
    observation.T_t_c = ceres_cam_imu::pose6ToMatrix(pose);
    shift_pose_observations.push_back(observation);
  }

  constexpr double true_shift_s = 0.04;
  std::vector<ceres_cam_imu::ImuSample> shifted_imu;
  for (int i = 0; i <= 160; ++i) {
    const double t_imu = 0.2 + 0.01 * static_cast<double>(i);
    ceres_cam_imu::ImuSample sample;
    sample.timestamp_s = t_imu;
    sample.gyro_rad_s = angular_velocity_at(t_imu - true_shift_s);
    shifted_imu.push_back(sample);
  }

  ceres_cam_imu::TimeShiftPriorOptions shift_options;
  shift_options.pose_knots_per_second = 10.0;
  shift_options.pose_fit_regularization = 1e-10;
  const ceres_cam_imu::TimeShiftPriorEstimate shift_estimate =
      ceres_cam_imu::estimateCameraImuTimeShiftPrior(
          shift_pose_observations, shifted_imu, identity_extrinsic,
          shift_options);
  assert(shift_estimate.num_samples == static_cast<int>(shifted_imu.size()));
  assert(std::abs(shift_estimate.shift_s - true_shift_s) <=
         shift_estimate.sample_dt_s + 1e-12);

  const ceres_cam_imu::Vec3 true_r_c_i(0.12, -0.07, 0.09);
  const ceres_cam_imu::Mat3 true_R_c_i =
      ceres_cam_imu::rotationVectorToMatrix(true_r_c_i);
  const ceres_cam_imu::Mat3 true_R_i_c = true_R_c_i.transpose();
  const ceres_cam_imu::Vec3 true_gyro_bias(0.015, -0.008, 0.006);
  const ceres_cam_imu::Vec3 true_gravity(0.1, -9.805, 0.15);
  const ceres_cam_imu::Vec3 normalized_gravity =
      true_gravity / true_gravity.norm() * 9.80655;
  std::vector<ceres_cam_imu::ImuSample> orientation_imu;
  for (int i = 0; i <= 160; ++i) {
    const double t = 0.2 + 0.01 * static_cast<double>(i);
    const ceres_cam_imu::Vec6 pose =
        shift_spline.evaluate(shift_controls, t, 0);
    const ceres_cam_imu::Mat3 R_w_c =
        ceres_cam_imu::rotationVectorToMatrix(pose.tail<3>());
    const ceres_cam_imu::Mat3 R_w_i = R_w_c * true_R_c_i;
    ceres_cam_imu::ImuSample sample;
    sample.timestamp_s = t;
    sample.gyro_rad_s = true_R_i_c * angular_velocity_at(t) + true_gyro_bias;
    sample.accel_m_s2 = -R_w_i.transpose() * true_gravity;
    orientation_imu.push_back(sample);
  }

  ceres_cam_imu::OrientationGravityInitializerOptions orientation_options;
  orientation_options.pose_knots_per_second = 20.0;
  orientation_options.pose_fit_regularization = 1e-12;
  const ceres_cam_imu::OrientationGravityInitializerResult orientation_prior =
      ceres_cam_imu::estimateOrientationGravityAndGyroBiasPrior(
          shift_pose_observations, orientation_imu, identity_extrinsic, 0.0,
          orientation_options);
  const ceres_cam_imu::Mat3 estimated_R_c_i =
      ceres_cam_imu::pose6ToMatrix(orientation_prior.T_c_b).block<3, 3>(0, 0);
  assert(orientation_prior.num_samples ==
         static_cast<int>(orientation_imu.size()));
  assert((estimated_R_c_i - true_R_c_i).norm() < 5e-3);
  assert((orientation_prior.gyro_bias_rad_s - true_gyro_bias).norm() < 5e-3);
  assert((orientation_prior.gravity_m_s2 - normalized_gravity).norm() < 5e-3);

  ceres_cam_imu::CalibrationOptions bias_init_options;
  bias_init_options.initial_gyro_bias_rad_s = true_gyro_bias;
  bias_init_options.initial_accel_bias_m_s2 =
      ceres_cam_imu::Vec3(0.1, 0.2, 0.3);
  bias_init_options.add_bias_motion_prior = false;
  bias_init_options.add_pose_motion_prior = false;
  std::vector<ceres_cam_imu::ImageObservation> bias_init_images(2);
  bias_init_images[0].timestamp_s = 0.0;
  bias_init_images[1].timestamp_s = 1.0;
  std::vector<ceres_cam_imu::ImuSample> bias_init_samples(2);
  bias_init_samples[0].timestamp_s = 0.0;
  bias_init_samples[1].timestamp_s = 1.0;
  const ceres_cam_imu::CalibrationState bias_init_state =
      ceres_cam_imu::initializeCalibrationState(
          bias_init_images, bias_init_samples, bias_init_options);
  assert((ceres_cam_imu::Vec3(
              bias_init_state.gyro_bias_controls.front().values[0],
              bias_init_state.gyro_bias_controls.front().values[1],
              bias_init_state.gyro_bias_controls.front().values[2]) -
          true_gyro_bias)
             .norm() < 1e-12);

  ceres_cam_imu::CalibrationState snapshot_state = bias_init_state;
  snapshot_state.T_c_b.values = {1.0, 2.0, 3.0, 0.1, -0.2, 0.3};
  snapshot_state.imu_extrinsic.values = {-1.0, -2.0, -3.0, -0.1, 0.2, -0.3};
  snapshot_state.imu_intrinsics.accel_M.values = {1.01,  0.02, 0.99,
                                                  -0.01, 0.03, 1.02};
  snapshot_state.imu_intrinsics.gyro_M.values = {0.98, -0.02, 1.03,
                                                 0.01, -0.04, 0.97};
  snapshot_state.imu_intrinsics.gyro_accel_sensitivity.values = {
      0.001, 0.002, 0.003, -0.001, -0.002, -0.003, 0.004, 0.005, 0.006};
  snapshot_state.imu_intrinsics.gyro_sensing_rotation.values = {0.01, -0.02,
                                                                0.03};
  snapshot_state.imu_intrinsics.accel_axis_rx_i.values = {0.001, 0.0, 0.0};
  snapshot_state.imu_intrinsics.accel_axis_ry_i.values = {0.0, 0.002, 0.0};
  snapshot_state.imu_intrinsics.accel_axis_rz_i.values = {0.0, 0.0, 0.003};
  snapshot_state.gravity.values = {0.1, -9.7, 0.2};
  snapshot_state.camera_time_shift_s.value = -0.123;
  snapshot_state.pose_controls.front().values = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  snapshot_state.gyro_bias_controls.front().values = {0.01, 0.02, 0.03};
  snapshot_state.accel_bias_controls.front().values = {-0.1, -0.2, -0.3};
  const ceres_cam_imu::CalibrationStateSnapshot state_snapshot =
      ceres_cam_imu::snapshotCalibrationState(snapshot_state);

  snapshot_state.T_c_b.values = {};
  snapshot_state.imu_extrinsic.values = {};
  snapshot_state.imu_intrinsics.accel_M.values = {};
  snapshot_state.imu_intrinsics.gyro_M.values = {};
  snapshot_state.imu_intrinsics.gyro_accel_sensitivity.values = {};
  snapshot_state.imu_intrinsics.gyro_sensing_rotation.values = {};
  snapshot_state.imu_intrinsics.accel_axis_rx_i.values = {};
  snapshot_state.imu_intrinsics.accel_axis_ry_i.values = {};
  snapshot_state.imu_intrinsics.accel_axis_rz_i.values = {};
  snapshot_state.gravity.values = {0.0, -9.80655, 0.0};
  snapshot_state.camera_time_shift_s.value = 0.0;
  snapshot_state.pose_controls.front().values = {};
  snapshot_state.gyro_bias_controls.front().values = {};
  snapshot_state.accel_bias_controls.front().values = {};
  assert(
      ceres_cam_imu::isCompatibleStateSnapshot(state_snapshot, snapshot_state));
  ceres_cam_imu::restoreCalibrationState(state_snapshot, &snapshot_state);
  assert(snapshot_state.T_c_b.values == state_snapshot.T_c_b.values);
  assert(snapshot_state.imu_extrinsic.values ==
         state_snapshot.imu_extrinsic.values);
  assert(snapshot_state.imu_intrinsics.accel_M.values ==
         state_snapshot.imu_intrinsics.accel_M.values);
  assert(snapshot_state.imu_intrinsics.gyro_M.values ==
         state_snapshot.imu_intrinsics.gyro_M.values);
  assert(snapshot_state.imu_intrinsics.gyro_accel_sensitivity.values ==
         state_snapshot.imu_intrinsics.gyro_accel_sensitivity.values);
  assert(snapshot_state.imu_intrinsics.gyro_sensing_rotation.values ==
         state_snapshot.imu_intrinsics.gyro_sensing_rotation.values);
  assert(snapshot_state.imu_intrinsics.accel_axis_rx_i.values ==
         state_snapshot.imu_intrinsics.accel_axis_rx_i.values);
  assert(snapshot_state.imu_intrinsics.accel_axis_ry_i.values ==
         state_snapshot.imu_intrinsics.accel_axis_ry_i.values);
  assert(snapshot_state.imu_intrinsics.accel_axis_rz_i.values ==
         state_snapshot.imu_intrinsics.accel_axis_rz_i.values);
  assert(snapshot_state.gravity.values == state_snapshot.gravity.values);
  assert(snapshot_state.camera_time_shift_s.value ==
         state_snapshot.camera_time_shift_s.value);
  assert(snapshot_state.pose_controls.front().values ==
         state_snapshot.pose_controls.front().values);
  assert(snapshot_state.gyro_bias_controls.front().values ==
         state_snapshot.gyro_bias_controls.front().values);
  assert(snapshot_state.accel_bias_controls.front().values ==
         state_snapshot.accel_bias_controls.front().values);

  ceres_cam_imu::CalibrationState incompatible_snapshot_state = snapshot_state;
  incompatible_snapshot_state.pose_controls.push_back(
      ceres_cam_imu::PoseControlBlock{});
  assert(!ceres_cam_imu::isCompatibleStateSnapshot(
      state_snapshot, incompatible_snapshot_state));
  bool incompatible_snapshot_threw = false;
  try {
    ceres_cam_imu::restoreCalibrationState(state_snapshot,
                                           &incompatible_snapshot_state);
  } catch (const std::invalid_argument &) {
    incompatible_snapshot_threw = true;
  }
  assert(incompatible_snapshot_threw);

  bool null_snapshot_restore_threw = false;
  try {
    ceres_cam_imu::restoreCalibrationState(state_snapshot, nullptr);
  } catch (const std::invalid_argument &) {
    null_snapshot_restore_threw = true;
  }
  assert(null_snapshot_restore_threw);

  ceres_cam_imu::CalibrationOptions stage_base;
  stage_base.max_iterations = 3;
  stage_base.max_frames = 17;
  stage_base.add_pose_motion_prior = true;
  const std::vector<ceres_cam_imu::CalibrationStage> stages =
      ceres_cam_imu::makeConservativeCalibrationStages(stage_base);
  assert(stages.size() == 4);
  assert(stages.front().name == "extrinsic_from_fixed_motion");
  assert(stages.front().options.fix_pose_controls);
  assert(stages.front().options.fix_bias_controls);
  assert(!stages.front().options.fix_camera_extrinsic);
  assert(stages.front().options.fix_time_shift);
  assert(stages.front().options.fix_gravity);
  assert(stages.back().options.fix_pose_controls);
  assert(!stages.back().options.fix_camera_extrinsic);
  assert(!stages.back().options.fix_time_shift);
  assert(stages.back().options.max_frames == 17);
  assert(stages.back().options.max_iterations == 3);
  assert(stages.back().options.add_pose_motion_prior);

  const std::vector<ceres_cam_imu::CalibrationStage> scheduled_stages =
      ceres_cam_imu::makeConservativeCalibrationStages(stage_base,
                                                       {0, 1, 2, 5});
  assert(scheduled_stages.at(0).options.max_iterations == 0);
  assert(scheduled_stages.at(1).options.max_iterations == 1);
  assert(scheduled_stages.at(2).options.max_iterations == 2);
  assert(scheduled_stages.at(3).options.max_iterations == 5);

  const std::vector<ceres_cam_imu::CalibrationStage> custom_stages =
      ceres_cam_imu::makeCalibrationStagesFromFreeMasks(stage_base, {0, 2},
                                                        {"e", "eb"});
  assert(custom_stages.size() == 2);
  assert(custom_stages.at(0).name == "custom_0_free_e");
  assert(custom_stages.at(0).options.max_iterations == 0);
  assert(custom_stages.at(0).options.fix_pose_controls);
  assert(custom_stages.at(0).options.fix_bias_controls);
  assert(!custom_stages.at(0).options.fix_camera_extrinsic);
  assert(custom_stages.at(0).options.fix_time_shift);
  assert(custom_stages.at(0).options.fix_gravity);
  assert(custom_stages.at(1).options.max_iterations == 2);
  assert(custom_stages.at(1).options.fix_pose_controls);
  assert(!custom_stages.at(1).options.fix_bias_controls);
  assert(!custom_stages.at(1).options.fix_camera_extrinsic);
  assert(custom_stages.at(1).options.fix_time_shift);
  assert(custom_stages.at(1).options.fix_gravity);

  std::vector<ceres_cam_imu::CalibrationStage> weighted_stages = custom_stages;
  ceres_cam_imu::applyStagePoseMotionVariances({100.0, 10.0}, {20.0, 2.0},
                                               &weighted_stages);
  assert(weighted_stages.at(0).options.add_pose_motion_prior);
  assert(weighted_stages.at(0).options.pose_motion_translation_variance ==
         100.0);
  assert(weighted_stages.at(0).options.pose_motion_rotation_variance == 20.0);
  assert(weighted_stages.at(1).options.add_pose_motion_prior);
  assert(weighted_stages.at(1).options.pose_motion_translation_variance ==
         10.0);
  assert(weighted_stages.at(1).options.pose_motion_rotation_variance == 2.0);

  ceres_cam_imu::applyStagePoseMotionOrders({1, 2}, &weighted_stages);
  assert(weighted_stages.at(0).options.add_pose_motion_prior);
  assert(weighted_stages.at(0).options.pose_motion_derivative_order == 1);
  assert(weighted_stages.at(1).options.add_pose_motion_prior);
  assert(weighted_stages.at(1).options.pose_motion_derivative_order == 2);

  bool invalid_stage_weight_count_threw = false;
  try {
    ceres_cam_imu::applyStagePoseMotionVariances({1.0}, {}, &weighted_stages);
  } catch (const std::invalid_argument &) {
    invalid_stage_weight_count_threw = true;
  }
  assert(invalid_stage_weight_count_threw);

  bool invalid_stage_weight_value_threw = false;
  try {
    ceres_cam_imu::applyStagePoseMotionVariances({1.0, 0.0}, {},
                                                 &weighted_stages);
  } catch (const std::invalid_argument &) {
    invalid_stage_weight_value_threw = true;
  }
  assert(invalid_stage_weight_value_threw);

  bool invalid_stage_order_count_threw = false;
  try {
    ceres_cam_imu::applyStagePoseMotionOrders({1}, &weighted_stages);
  } catch (const std::invalid_argument &) {
    invalid_stage_order_count_threw = true;
  }
  assert(invalid_stage_order_count_threw);

  bool invalid_stage_order_value_threw = false;
  try {
    ceres_cam_imu::applyStagePoseMotionOrders({1, 0}, &weighted_stages);
  } catch (const std::invalid_argument &) {
    invalid_stage_order_value_threw = true;
  }
  assert(invalid_stage_order_value_threw);

  bool invalid_stage_order_spline_threw = false;
  try {
    ceres_cam_imu::applyStagePoseMotionOrders({1, stage_base.spline_order},
                                              &weighted_stages);
  } catch (const std::invalid_argument &) {
    invalid_stage_order_spline_threw = true;
  }
  assert(invalid_stage_order_spline_threw);

  stage_base.fix_time_shift = true;
  stage_base.fix_camera_extrinsic = true;
  const std::vector<ceres_cam_imu::CalibrationStage> fixed_stages =
      ceres_cam_imu::makeConservativeCalibrationStages(stage_base);
  for (const ceres_cam_imu::CalibrationStage &stage : fixed_stages) {
    assert(stage.options.fix_time_shift);
    assert(stage.options.fix_camera_extrinsic);
  }

  const std::vector<ceres_cam_imu::CalibrationStage> globally_fixed_custom =
      ceres_cam_imu::makeCalibrationStagesFromFreeMasks(stage_base, {}, {"te"});
  assert(globally_fixed_custom.at(0).options.fix_time_shift);
  assert(globally_fixed_custom.at(0).options.fix_camera_extrinsic);

  bool invalid_stage_mask_threw = false;
  try {
    (void)ceres_cam_imu::makeCalibrationStagesFromFreeMasks(stage_base, {},
                                                            {"x"});
  } catch (const std::invalid_argument &) {
    invalid_stage_mask_threw = true;
  }
  assert(invalid_stage_mask_threw);

  bool invalid_stage_iteration_count_threw = false;
  try {
    (void)ceres_cam_imu::makeCalibrationStagesFromFreeMasks(stage_base, {1},
                                                            {"e", "t"});
  } catch (const std::invalid_argument &) {
    invalid_stage_iteration_count_threw = true;
  }
  assert(invalid_stage_iteration_count_threw);

  ceres::Solver::Summary stage_decision_summary;
  stage_decision_summary.termination_type = ceres::CONVERGENCE;
  stage_decision_summary.initial_cost = 10.0;
  stage_decision_summary.final_cost = 9.0;
  assert(ceres_cam_imu::decideCalibrationStageStateUpdate(
             stage_decision_summary) ==
         ceres_cam_imu::CalibrationStageStateDecision::kAccepted);
  assert(std::string(ceres_cam_imu::calibrationStageStateDecisionName(
             ceres_cam_imu::CalibrationStageStateDecision::kAccepted)) ==
         "accepted");

  stage_decision_summary.final_cost = 10.1;
  assert(ceres_cam_imu::decideCalibrationStageStateUpdate(
             stage_decision_summary) ==
         ceres_cam_imu::CalibrationStageStateDecision::kRestoredCostIncrease);

  stage_decision_summary.final_cost = std::numeric_limits<double>::infinity();
  assert(ceres_cam_imu::decideCalibrationStageStateUpdate(
             stage_decision_summary) ==
         ceres_cam_imu::CalibrationStageStateDecision::kRestoredNonFiniteCost);

  stage_decision_summary.termination_type = ceres::FAILURE;
  stage_decision_summary.final_cost = 9.0;
  assert(ceres_cam_imu::decideCalibrationStageStateUpdate(
             stage_decision_summary) ==
         ceres_cam_imu::CalibrationStageStateDecision::kRestoredSolverFailure);

  std::unique_ptr<ceres::CostFunction> time_shift_prior(
      ceres_cam_imu::createTimeShiftPrior(0.1, 0.02));
  std::array<double, 1> time_shift_block = {0.13};
  std::array<const double *, 1> time_shift_params = {time_shift_block.data()};
  std::array<double, 1> time_shift_residual{};
  std::array<double, 1> time_shift_jac_storage{};
  std::array<double *, 1> time_shift_jacobians = {
      time_shift_jac_storage.data()};
  assert(time_shift_prior->Evaluate(time_shift_params.data(),
                                    time_shift_residual.data(),
                                    time_shift_jacobians.data()));
  assert(std::abs(time_shift_residual[0] - 1.5) < 1e-12);
  assert(std::abs(time_shift_jac_storage[0] - 50.0) < 1e-12);

  auto check_cost_jacobians = [](ceres::CostFunction *cost,
                                 std::vector<std::vector<double>> blocks,
                                 const double tolerance) {
    const int num_residuals = cost->num_residuals();
    const std::vector<int32_t> &block_sizes = cost->parameter_block_sizes();
    assert(block_sizes.size() == blocks.size());
    std::vector<const double *> parameters(blocks.size(), nullptr);
    for (std::size_t block = 0; block < blocks.size(); ++block) {
      assert(static_cast<int>(blocks[block].size()) == block_sizes[block]);
      parameters[block] = blocks[block].data();
    }

    std::vector<double> residuals(static_cast<std::size_t>(num_residuals), 0.0);
    std::vector<std::vector<double>> jacobian_storage;
    std::vector<double *> jacobians(blocks.size(), nullptr);
    jacobian_storage.reserve(blocks.size());
    for (std::size_t block = 0; block < blocks.size(); ++block) {
      jacobian_storage.emplace_back(
          static_cast<std::size_t>(num_residuals * block_sizes[block]), 0.0);
      jacobians[block] = jacobian_storage.back().data();
    }
    assert(
        cost->Evaluate(parameters.data(), residuals.data(), jacobians.data()));

    constexpr double eps = 1e-6;
    for (std::size_t block = 0; block < blocks.size(); ++block) {
      for (int col = 0; col < block_sizes[block]; ++col) {
        blocks[block][static_cast<std::size_t>(col)] += eps;
        for (std::size_t i = 0; i < blocks.size(); ++i) {
          parameters[i] = blocks[i].data();
        }
        std::vector<double> plus(static_cast<std::size_t>(num_residuals), 0.0);
        assert(cost->Evaluate(parameters.data(), plus.data(), nullptr));

        blocks[block][static_cast<std::size_t>(col)] -= 2.0 * eps;
        for (std::size_t i = 0; i < blocks.size(); ++i) {
          parameters[i] = blocks[i].data();
        }
        std::vector<double> minus(static_cast<std::size_t>(num_residuals), 0.0);
        assert(cost->Evaluate(parameters.data(), minus.data(), nullptr));

        blocks[block][static_cast<std::size_t>(col)] += eps;
        for (int row = 0; row < num_residuals; ++row) {
          const double numeric = (plus[static_cast<std::size_t>(row)] -
                                  minus[static_cast<std::size_t>(row)]) /
                                 (2.0 * eps);
          const double analytic =
              jacobian_storage[block][static_cast<std::size_t>(
                  row * block_sizes[block] + col)];
          assert(std::abs(numeric - analytic) < tolerance);
        }
      }
    }
  };

  check_cost_jacobians(time_shift_prior.get(), {{0.13}}, 1e-8);

  ceres_cam_imu::UniformBSpline bias_spline(3, 6, 0.0, 1.0, 1);
  const ceres_cam_imu::SplineSegmentMeta6 bias_meta =
      bias_spline.segmentMeta6(0.0);
  constexpr double random_walk_sigma = 0.2;
  std::unique_ptr<ceres::CostFunction> bias_prior(
      ceres_cam_imu::createBiasMotionPrior(bias_meta, random_walk_sigma));
  std::array<std::array<double, 3>, 6> bias_blocks{};
  for (int i = 0; i < 6; ++i) {
    bias_blocks[static_cast<std::size_t>(i)] = {1.0, -2.0, 0.5};
  }
  std::array<const double *, 6> bias_params{};
  for (int i = 0; i < 6; ++i) {
    bias_params[static_cast<std::size_t>(i)] =
        bias_blocks[static_cast<std::size_t>(i)].data();
  }
  std::vector<double> bias_residuals(
      static_cast<std::size_t>(bias_prior->num_residuals()), 0.0);
  assert(
      bias_prior->Evaluate(bias_params.data(), bias_residuals.data(), nullptr));
  double constant_prior_norm = 0.0;
  for (const double residual : bias_residuals) {
    constant_prior_norm += residual * residual;
  }
  assert(constant_prior_norm < 1e-20);

  for (int i = 0; i < 6; ++i) {
    bias_blocks[static_cast<std::size_t>(i)] = {
        0.1 * std::sin(0.3 * i), 0.2 * std::cos(0.7 * i),
        -0.05 + 0.03 * static_cast<double>(i)};
  }
  assert(
      bias_prior->Evaluate(bias_params.data(), bias_residuals.data(), nullptr));
  double prior_residual_sq = 0.0;
  for (const double residual : bias_residuals) {
    prior_residual_sq += residual * residual;
  }

  double numeric_integral = 0.0;
  constexpr int kQuadratureSamples = 4000;
  for (int i = 0; i < kQuadratureSamples; ++i) {
    const double t = (static_cast<double>(i) + 0.5) /
                     static_cast<double>(kQuadratureSamples);
    const std::array<double, 6> weights = bias_meta.weights(t, 1);
    ceres_cam_imu::Vec3 bias_dot = ceres_cam_imu::Vec3::Zero();
    for (int k = 0; k < 6; ++k) {
      const auto &block = bias_blocks[static_cast<std::size_t>(k)];
      bias_dot += weights[static_cast<std::size_t>(k)] *
                  ceres_cam_imu::Vec3(block[0], block[1], block[2]);
    }
    numeric_integral += bias_dot.squaredNorm() /
                        (random_walk_sigma * random_walk_sigma) /
                        static_cast<double>(kQuadratureSamples);
  }
  assert(std::abs(prior_residual_sq - numeric_integral) /
             std::max(1.0, numeric_integral) <
         1e-6);
  std::vector<std::vector<double>> bias_jacobian_blocks;
  bias_jacobian_blocks.reserve(6);
  for (const auto &block : bias_blocks) {
    bias_jacobian_blocks.push_back({block[0], block[1], block[2]});
  }
  check_cost_jacobians(bias_prior.get(), bias_jacobian_blocks, 1e-8);

  std::unique_ptr<ceres::CostFunction> pose_prior(
      ceres_cam_imu::createPoseMotionPrior(bias_meta, 10.0, 2.0, 2));
  std::array<std::array<double, 6>, 6> pose_blocks{};
  for (int i = 0; i < 6; ++i) {
    pose_blocks[static_cast<std::size_t>(i)] = {1.0, -2.0, 0.5,
                                                0.1, -0.2, 0.05};
  }
  std::array<const double *, 6> pose_params{};
  for (int i = 0; i < 6; ++i) {
    pose_params[static_cast<std::size_t>(i)] =
        pose_blocks[static_cast<std::size_t>(i)].data();
  }
  std::vector<double> pose_residuals(
      static_cast<std::size_t>(pose_prior->num_residuals()), 0.0);
  assert(
      pose_prior->Evaluate(pose_params.data(), pose_residuals.data(), nullptr));
  double constant_pose_prior_norm = 0.0;
  for (const double residual : pose_residuals) {
    constant_pose_prior_norm += residual * residual;
  }
  assert(constant_pose_prior_norm < 1e-20);

  for (int i = 0; i < 6; ++i) {
    pose_blocks[static_cast<std::size_t>(i)] = {
        0.1 * std::sin(0.2 * i),  -0.04 + 0.03 * static_cast<double>(i),
        0.2 * std::cos(0.6 * i),  0.05 * std::sin(0.9 * i),
        0.04 * std::cos(0.4 * i), -0.02 + 0.015 * static_cast<double>(i)};
  }
  assert(
      pose_prior->Evaluate(pose_params.data(), pose_residuals.data(), nullptr));
  double pose_prior_residual_sq = 0.0;
  for (const double residual : pose_residuals) {
    pose_prior_residual_sq += residual * residual;
  }

  double pose_numeric_integral = 0.0;
  for (int i = 0; i < kQuadratureSamples; ++i) {
    const double t = (static_cast<double>(i) + 0.5) /
                     static_cast<double>(kQuadratureSamples);
    const std::array<double, 6> weights = bias_meta.weights(t, 2);
    ceres_cam_imu::Vec6 pose_ddot = ceres_cam_imu::Vec6::Zero();
    for (int k = 0; k < 6; ++k) {
      const auto &block = pose_blocks[static_cast<std::size_t>(k)];
      for (int dim = 0; dim < 6; ++dim) {
        pose_ddot(dim) += weights[static_cast<std::size_t>(k)] *
                          block[static_cast<std::size_t>(dim)];
      }
    }
    pose_numeric_integral += (pose_ddot.head<3>().squaredNorm() / 10.0 +
                              pose_ddot.tail<3>().squaredNorm() / 2.0) /
                             static_cast<double>(kQuadratureSamples);
  }
  assert(std::abs(pose_prior_residual_sq - pose_numeric_integral) /
             std::max(1.0, pose_numeric_integral) <
         1e-6);
  std::vector<std::vector<double>> pose_jacobian_blocks;
  pose_jacobian_blocks.reserve(6);
  for (const auto &block : pose_blocks) {
    pose_jacobian_blocks.push_back(
        {block[0], block[1], block[2], block[3], block[4], block[5]});
  }
  check_cost_jacobians(pose_prior.get(), pose_jacobian_blocks, 1e-8);

  ceres_cam_imu::CameraIntrinsics analytic_intr;
  analytic_intr.fx = 420.0;
  analytic_intr.fy = 415.0;
  analytic_intr.cx = 320.0;
  analytic_intr.cy = 200.0;
  analytic_intr.k1 = -0.08;
  analytic_intr.k2 = 0.015;
  analytic_intr.p1 = 0.001;
  analytic_intr.p2 = -0.002;

  ceres_cam_imu::UniformBSpline camera_spline(6, 6, 0.0, 1.0, 4);
  const double camera_time = 0.43;
  const ceres_cam_imu::SplineSegmentMeta6 camera_meta =
      camera_spline.segmentMeta6(camera_time);
  ceres_cam_imu::CornerMeasurement analytic_corner;
  analytic_corner.pixel = ceres_cam_imu::Vec2(300.0, 210.0);
  analytic_corner.target_point = ceres_cam_imu::Vec3(0.3, -0.2, 4.0);

  std::unique_ptr<ceres::CostFunction> camera_cost(
      ceres_cam_imu::createCameraReprojectionResidual(
          analytic_intr, analytic_corner, camera_time, camera_meta, 1.3));

  std::array<double, 6> analytic_T_c_b = {0.05, -0.03, 0.12, 0.03, -0.02, 0.04};
  std::array<double, 1> analytic_time_shift = {0.0};
  std::array<std::array<double, 6>, 6> analytic_controls{};
  for (int i = 0; i < 6; ++i) {
    analytic_controls[static_cast<std::size_t>(i)] = {0.02 * i,
                                                      -0.01 * i,
                                                      0.005 * i,
                                                      0.01 * std::sin(0.3 * i),
                                                      0.012 * std::cos(0.2 * i),
                                                      -0.008 *
                                                          std::sin(0.5 * i)};
  }

  auto make_camera_parameters = [&]() {
    std::array<double *, 8> parameters{};
    parameters[0] = analytic_T_c_b.data();
    parameters[1] = analytic_time_shift.data();
    for (int i = 0; i < 6; ++i) {
      parameters[static_cast<std::size_t>(2 + i)] =
          analytic_controls[static_cast<std::size_t>(i)].data();
    }
    return parameters;
  };

  std::array<double *, 8> camera_parameters = make_camera_parameters();
  std::array<double, 2> camera_residuals{};
  std::array<std::array<double, 12>, 8> analytic_jac_storage{};
  std::array<double *, 8> analytic_jacobians{};
  analytic_jacobians[0] = analytic_jac_storage[0].data();
  analytic_jacobians[1] = analytic_jac_storage[1].data();
  for (int i = 0; i < 6; ++i) {
    analytic_jacobians[static_cast<std::size_t>(2 + i)] =
        analytic_jac_storage[static_cast<std::size_t>(2 + i)].data();
  }
  assert(camera_cost->Evaluate(camera_parameters.data(),
                               camera_residuals.data(),
                               analytic_jacobians.data()));

  auto finite_difference_block = [&](const int block, const int block_size) {
    constexpr double kEps = 1e-7;
    for (int col = 0; col < block_size; ++col) {
      double *parameter = camera_parameters[static_cast<std::size_t>(block)];
      const double original = parameter[col];

      std::array<double, 2> plus{};
      parameter[col] = original + kEps;
      assert(camera_cost->Evaluate(camera_parameters.data(), plus.data(),
                                   nullptr));

      std::array<double, 2> minus{};
      parameter[col] = original - kEps;
      assert(camera_cost->Evaluate(camera_parameters.data(), minus.data(),
                                   nullptr));
      parameter[col] = original;

      for (int row = 0; row < 2; ++row) {
        const double numeric = (plus[static_cast<std::size_t>(row)] -
                                minus[static_cast<std::size_t>(row)]) /
                               (2.0 * kEps);
        const double analytic = analytic_jac_storage[static_cast<std::size_t>(
            block)][static_cast<std::size_t>(row * block_size + col)];
        const double tolerance = 2e-4 * std::max(1.0, std::abs(numeric));
        assert(std::abs(numeric - analytic) < tolerance);
      }
    }
  };

  finite_difference_block(0, 6);
  finite_difference_block(1, 1);
  for (int block = 2; block < 8; ++block) {
    finite_difference_block(block, 6);
  }

  ceres_cam_imu::UniformBSpline gyro_pose_spline(6, 6, 0.0, 1.0, 5);
  ceres_cam_imu::UniformBSpline gyro_bias_spline(3, 6, 0.0, 1.0, 5);
  const double gyro_time = 0.47;
  const ceres_cam_imu::SplineSegmentMeta6 gyro_pose_meta =
      gyro_pose_spline.segmentMeta6(gyro_time);
  const ceres_cam_imu::SplineSegmentMeta6 gyro_bias_meta =
      gyro_bias_spline.segmentMeta6(gyro_time);
  ceres_cam_imu::ImuSample gyro_sample;
  gyro_sample.timestamp_s = gyro_time;
  gyro_sample.gyro_rad_s = ceres_cam_imu::Vec3(0.08, -0.04, 0.11);
  ceres_cam_imu::ImuNoise gyro_noise;
  gyro_noise.update_rate_hz = 200.0;
  gyro_noise.gyroscope_noise_density = 0.015;
  std::unique_ptr<ceres::CostFunction> gyro_cost(
      ceres_cam_imu::createGyroscopeResidual(gyro_sample, gyro_noise,
                                             gyro_pose_meta, gyro_bias_meta));

  std::array<double, 6> gyro_imu_extrinsic = {0.01, -0.02, 0.03,
                                              0.04, -0.03, 0.02};
  std::array<std::array<double, 6>, 6> gyro_pose_controls{};
  std::array<std::array<double, 3>, 6> gyro_bias_controls{};
  for (int i = 0; i < 6; ++i) {
    gyro_pose_controls[static_cast<std::size_t>(i)] = {0.02 * i,
                                                       -0.015 * i,
                                                       0.01 * std::sin(0.2 * i),
                                                       0.04 * std::sin(0.3 * i),
                                                       -0.03 + 0.01 * i,
                                                       0.02 *
                                                           std::cos(0.4 * i)};
    gyro_bias_controls[static_cast<std::size_t>(i)] = {
        0.001 * i, -0.002 + 0.0007 * i, 0.0015 * std::sin(0.5 * i)};
  }

  auto make_gyro_parameters = [&]() {
    std::array<double *, 13> parameters{};
    parameters[0] = gyro_imu_extrinsic.data();
    for (int i = 0; i < 6; ++i) {
      parameters[static_cast<std::size_t>(1 + i)] =
          gyro_pose_controls[static_cast<std::size_t>(i)].data();
      parameters[static_cast<std::size_t>(7 + i)] =
          gyro_bias_controls[static_cast<std::size_t>(i)].data();
    }
    return parameters;
  };

  std::array<double *, 13> gyro_parameters = make_gyro_parameters();
  std::array<double, 3> gyro_residuals{};
  std::array<std::vector<double>, 13> gyro_jac_storage{};
  std::array<double *, 13> gyro_jacobians{};
  gyro_jac_storage[0].assign(18, 0.0);
  gyro_jacobians[0] = gyro_jac_storage[0].data();
  for (int i = 0; i < 6; ++i) {
    gyro_jac_storage[static_cast<std::size_t>(1 + i)].assign(18, 0.0);
    gyro_jac_storage[static_cast<std::size_t>(7 + i)].assign(9, 0.0);
    gyro_jacobians[static_cast<std::size_t>(1 + i)] =
        gyro_jac_storage[static_cast<std::size_t>(1 + i)].data();
    gyro_jacobians[static_cast<std::size_t>(7 + i)] =
        gyro_jac_storage[static_cast<std::size_t>(7 + i)].data();
  }
  assert(gyro_cost->Evaluate(gyro_parameters.data(), gyro_residuals.data(),
                             gyro_jacobians.data()));

  auto finite_difference_gyro_block = [&](const int block,
                                          const int block_size) {
    constexpr double kEps = 1e-7;
    for (int col = 0; col < block_size; ++col) {
      double *parameter = gyro_parameters[static_cast<std::size_t>(block)];
      const double original = parameter[col];

      std::array<double, 3> plus{};
      parameter[col] = original + kEps;
      assert(gyro_cost->Evaluate(gyro_parameters.data(), plus.data(), nullptr));

      std::array<double, 3> minus{};
      parameter[col] = original - kEps;
      assert(
          gyro_cost->Evaluate(gyro_parameters.data(), minus.data(), nullptr));
      parameter[col] = original;

      for (int row = 0; row < 3; ++row) {
        const double numeric = (plus[static_cast<std::size_t>(row)] -
                                minus[static_cast<std::size_t>(row)]) /
                               (2.0 * kEps);
        const double analytic =
            gyro_jac_storage[static_cast<std::size_t>(block)]
                            [static_cast<std::size_t>(row * block_size + col)];
        const double tolerance = 3e-4 * std::max(1.0, std::abs(numeric));
        assert(std::abs(numeric - analytic) < tolerance);
      }
    }
  };

  finite_difference_gyro_block(0, 6);
  for (int block = 1; block < 7; ++block) {
    finite_difference_gyro_block(block, 6);
  }
  for (int block = 7; block < 13; ++block) {
    finite_difference_gyro_block(block, 3);
  }

  std::unique_ptr<ceres::CostFunction> scale_gyro_cost(
      ceres_cam_imu::createScaleMisalignedGyroscopeResidual(
          gyro_sample, gyro_noise, gyro_pose_meta, gyro_bias_meta));
  std::vector<std::vector<double>> scale_gyro_blocks;
  scale_gyro_blocks.push_back({0.012, -0.018, 0.027, 0.035, -0.025, 0.017});
  scale_gyro_blocks.push_back({0.08, -9.79, 0.12});
  for (const auto &block : gyro_pose_controls) {
    scale_gyro_blocks.push_back({block[0], block[1], block[2],
                                 block[3], block[4], block[5]});
  }
  for (const auto &block : gyro_bias_controls) {
    scale_gyro_blocks.push_back({block[0], block[1], block[2]});
  }
  scale_gyro_blocks.push_back({0.012, -0.009, 0.015});
  scale_gyro_blocks.push_back({1.01, -0.015, 0.99, 0.02, -0.01, 1.02});
  scale_gyro_blocks.push_back({0.001, -0.002, 0.003, 0.0005, -0.0015,
                               0.0025, -0.0007, 0.0009, 0.0012});
  check_cost_jacobians(scale_gyro_cost.get(), scale_gyro_blocks, 2e-3);

  ceres_cam_imu::ImuSample accel_sample;
  accel_sample.timestamp_s = gyro_time;
  accel_sample.accel_m_s2 = ceres_cam_imu::Vec3(0.2, -9.6, 0.4);
  ceres_cam_imu::ImuNoise accel_noise;
  accel_noise.update_rate_hz = 200.0;
  accel_noise.accelerometer_noise_density = 0.08;
  std::unique_ptr<ceres::CostFunction> accel_cost(
      ceres_cam_imu::createAccelerometerResidual(
          accel_sample, accel_noise, gyro_pose_meta, gyro_bias_meta));

  std::array<double, 6> accel_imu_extrinsic = {0.03,  -0.015, 0.02,
                                               0.035, -0.025, 0.015};
  std::array<double, 3> accel_gravity = {0.1, -9.81, 0.05};
  std::array<std::array<double, 6>, 6> accel_pose_controls{};
  std::array<std::array<double, 3>, 6> accel_bias_controls{};
  for (int i = 0; i < 6; ++i) {
    accel_pose_controls[static_cast<std::size_t>(i)] = {
        0.03 * std::sin(0.2 * i), -0.02 + 0.01 * i,
        0.04 * std::cos(0.3 * i), 0.035 * std::sin(0.25 * i),
        -0.02 + 0.012 * i,        0.018 * std::cos(0.45 * i)};
    accel_bias_controls[static_cast<std::size_t>(i)] = {
        0.01 * std::sin(0.2 * i), -0.015 + 0.004 * i,
        0.006 * std::cos(0.5 * i)};
  }

  auto make_accel_parameters = [&]() {
    std::array<double *, 14> parameters{};
    parameters[0] = accel_imu_extrinsic.data();
    parameters[1] = accel_gravity.data();
    for (int i = 0; i < 6; ++i) {
      parameters[static_cast<std::size_t>(2 + i)] =
          accel_pose_controls[static_cast<std::size_t>(i)].data();
      parameters[static_cast<std::size_t>(8 + i)] =
          accel_bias_controls[static_cast<std::size_t>(i)].data();
    }
    return parameters;
  };

  std::array<double *, 14> accel_parameters = make_accel_parameters();
  std::array<double, 3> accel_residuals{};
  std::array<std::vector<double>, 14> accel_jac_storage{};
  std::array<double *, 14> accel_jacobians{};
  accel_jac_storage[0].assign(18, 0.0);
  accel_jac_storage[1].assign(9, 0.0);
  accel_jacobians[0] = accel_jac_storage[0].data();
  accel_jacobians[1] = accel_jac_storage[1].data();
  for (int i = 0; i < 6; ++i) {
    accel_jac_storage[static_cast<std::size_t>(2 + i)].assign(18, 0.0);
    accel_jac_storage[static_cast<std::size_t>(8 + i)].assign(9, 0.0);
    accel_jacobians[static_cast<std::size_t>(2 + i)] =
        accel_jac_storage[static_cast<std::size_t>(2 + i)].data();
    accel_jacobians[static_cast<std::size_t>(8 + i)] =
        accel_jac_storage[static_cast<std::size_t>(8 + i)].data();
  }
  assert(accel_cost->Evaluate(accel_parameters.data(), accel_residuals.data(),
                              accel_jacobians.data()));

  auto finite_difference_accel_block = [&](const int block,
                                           const int block_size) {
    constexpr double kEps = 1e-7;
    for (int col = 0; col < block_size; ++col) {
      double *parameter = accel_parameters[static_cast<std::size_t>(block)];
      const double original = parameter[col];

      std::array<double, 3> plus{};
      parameter[col] = original + kEps;
      assert(
          accel_cost->Evaluate(accel_parameters.data(), plus.data(), nullptr));

      std::array<double, 3> minus{};
      parameter[col] = original - kEps;
      assert(
          accel_cost->Evaluate(accel_parameters.data(), minus.data(), nullptr));
      parameter[col] = original;

      for (int row = 0; row < 3; ++row) {
        const double numeric = (plus[static_cast<std::size_t>(row)] -
                                minus[static_cast<std::size_t>(row)]) /
                               (2.0 * kEps);
        const double analytic =
            accel_jac_storage[static_cast<std::size_t>(block)]
                             [static_cast<std::size_t>(row * block_size + col)];
        const double tolerance = 8e-4 * std::max(1.0, std::abs(numeric));
        assert(std::abs(numeric - analytic) < tolerance);
      }
    }
  };

  finite_difference_accel_block(0, 6);
  finite_difference_accel_block(1, 3);
  for (int block = 2; block < 8; ++block) {
    finite_difference_accel_block(block, 6);
  }
  for (int block = 8; block < 14; ++block) {
    finite_difference_accel_block(block, 3);
  }

  std::unique_ptr<ceres::CostFunction> scale_accel_cost(
      ceres_cam_imu::createScaleMisalignedAccelerometerResidual(
          accel_sample, accel_noise, gyro_pose_meta, gyro_bias_meta));
  std::vector<std::vector<double>> scale_accel_blocks;
  scale_accel_blocks.push_back({0.031, -0.014, 0.019, 0.034, -0.026, 0.016});
  scale_accel_blocks.push_back({0.1, -9.81, 0.05});
  for (const auto &block : accel_pose_controls) {
    scale_accel_blocks.push_back({block[0], block[1], block[2],
                                  block[3], block[4], block[5]});
  }
  for (const auto &block : accel_bias_controls) {
    scale_accel_blocks.push_back({block[0], block[1], block[2]});
  }
  scale_accel_blocks.push_back({1.02, 0.01, 0.98, -0.015, 0.012, 1.01});
  check_cost_jacobians(scale_accel_cost.get(), scale_accel_blocks, 2e-3);

  std::unique_ptr<ceres::CostFunction> size_accel_cost(
      ceres_cam_imu::createSizeEffectAccelerometerResidual(
          accel_sample, accel_noise, gyro_pose_meta, gyro_bias_meta));
  std::vector<std::vector<double>> size_accel_blocks = scale_accel_blocks;
  size_accel_blocks.push_back({0.002, -0.001, 0.0005});
  size_accel_blocks.push_back({-0.0007, 0.0025, 0.001});
  size_accel_blocks.push_back({0.0011, -0.0004, 0.003});
  check_cost_jacobians(size_accel_cost.get(), size_accel_blocks, 3e-3);

  ceres_cam_imu::CalibrationState writer_state;
  writer_state.pose_spline = ceres_cam_imu::UniformBSpline(6, 6, 0.0, 1.0, 4);
  writer_state.gyro_bias_spline =
      ceres_cam_imu::UniformBSpline(3, 6, 0.0, 1.0, 2);
  writer_state.accel_bias_spline =
      ceres_cam_imu::UniformBSpline(3, 6, 0.0, 1.0, 2);
  writer_state.T_c_b.values = {0.01, -0.02, 0.03, 0.001, -0.002, 0.003};
  writer_state.camera_time_shift_s.value = -0.54;
  writer_state.gravity.values = {0.0, -9.81, 0.02};
  ceres_cam_imu::CalibrationResidualStatistics writer_stats;
  writer_stats.reprojection_px.count = 10;
  writer_stats.reprojection_px.mean = 0.2;
  writer_stats.gyro_rad_s.count = 4;
  writer_stats.gyro_rad_s.mean = 0.1;
  writer_stats.accel_m_s2.count = 4;
  writer_stats.accel_m_s2.mean = 0.8;
  writer_stats.top_accel_outliers.push_back({});
  writer_stats.top_accel_outliers.front().sample_index = 3;
  writer_stats.top_accel_outliers.front().timestamp_s = 1.25;
  writer_stats.top_accel_outliers.front().accel_error_m_s2 = 4.5;
  ceres_cam_imu::CalibrationResultWriterOptions writer_options;
  writer_options.include_kalibr_comparison = true;
  writer_options.kalibr_result.T_ci = ceres_cam_imu::Mat4::Identity();
  writer_options.kalibr_result.gravity = ceres_cam_imu::Vec3(0.0, -9.8, 0.0);
  writer_options.kalibr_result.timeshift_cam_to_imu_s = -0.55;
  const char *result_yaml_path = "/tmp/ceres_cam_imu_result_writer_test.yaml";
  ceres_cam_imu::writeCalibrationResultYaml(result_yaml_path, writer_state,
                                            writer_stats, writer_options);
  std::ifstream result_yaml(result_yaml_path);
  std::ostringstream result_buffer;
  result_buffer << result_yaml.rdbuf();
  const std::string result_contents = result_buffer.str();
  assert(result_contents.find("format_version: 1") != std::string::npos);
  assert(result_contents.find("T_c_b") != std::string::npos);
  assert(result_contents.find("time_shift_s: -0.540000") != std::string::npos);
  assert(result_contents.find("residual_statistics") != std::string::npos);
  assert(result_contents.find("kalibr_delta") != std::string::npos);
  assert(result_contents.find("sample_index: 3") != std::string::npos);
  const ceres_cam_imu::CalibrationResultFile read_result =
      ceres_cam_imu::readCalibrationResultYaml(result_yaml_path);
  assert(read_result.format_version == 1);
  assert(std::abs(read_result.time_shift_s + 0.54) < 1e-12);
  assert((read_result.gravity - ceres_cam_imu::Vec3(0.0, -9.81, 0.02)).norm() <
         1e-12);
  assert(std::abs(read_result.residuals.reprojection_mean_px - 0.2) < 1e-12);
  assert(std::abs(read_result.residuals.gyro_mean_rad_s - 0.1) < 1e-12);
  assert(std::abs(read_result.residuals.accel_mean_m_s2 - 0.8) < 1e-12);
  assert(read_result.has_kalibr_delta);
  assert(std::abs(read_result.kalibr_delta.time_shift_s - 0.01) < 1e-12);
  std::remove(result_yaml_path);

  std::cout << "test_math passed\n";
  return 0;
}
