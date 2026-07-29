#include <algorithm>

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

#include "ceres_cam_imu/core/se3.h"
#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/initialization/kalibr_style_multi_imu_initializer.h"
#include "ceres_cam_imu/optimizer/calibration_problem.h"

namespace {

using ceres_cam_imu::CameraExtrinsicBlock;
using ceres_cam_imu::ImuObservationDataset;
using ceres_cam_imu::ImuSample;
using ceres_cam_imu::Mat3;
using ceres_cam_imu::Mat4;
using ceres_cam_imu::PoseObservation;
using ceres_cam_imu::Vec3;
using ceres_cam_imu::Vec6;

constexpr double kDurationS = 6.0;
constexpr double kStartS = 1.0;
constexpr double kPoseDtS = 0.05;
constexpr double kImuDtS = 0.005;

struct SyntheticData {
  std::vector<std::vector<PoseObservation>> camera_poses;
  std::vector<ImuObservationDataset> imus;
  std::vector<CameraExtrinsicBlock> camera_extrinsics;
  std::vector<double> camera_time_shifts_s;
  std::vector<Mat3> imu_rotations;
  std::vector<double> imu_time_offsets_s;
};

double rotationDeltaRad(const Mat3 &lhs, const Mat3 &rhs) {
  const Mat3 delta = lhs * rhs.transpose();
  return std::acos(std::clamp((delta.trace() - 1.0) * 0.5, -1.0, 1.0));
}

Vec3 bodyRotationVectorAt(const double t) {
  return Vec3(0.24 * std::sin(0.73 * t) + 0.06 * std::sin(2.11 * t),
              0.19 * std::cos(0.91 * t) + 0.05 * std::sin(1.63 * t),
              0.31 * std::sin(1.17 * t) + 0.04 * std::cos(2.37 * t));
}

Vec3 bodyRotationVectorDerivativeAt(const double t) {
  return Vec3(
      0.24 * 0.73 * std::cos(0.73 * t) + 0.06 * 2.11 * std::cos(2.11 * t),
      -0.19 * 0.91 * std::sin(0.91 * t) + 0.05 * 1.63 * std::cos(1.63 * t),
      0.31 * 1.17 * std::cos(1.17 * t) - 0.04 * 2.37 * std::sin(2.37 * t));
}

Mat3 bodyRotationAt(const double t) {
  return ceres_cam_imu::rotationVectorToMatrix(bodyRotationVectorAt(t));
}

Vec3 bodyAngularVelocityAt(const double t) {
  const Vec3 r = bodyRotationVectorAt(t);
  return -bodyRotationAt(t).transpose() *
         ceres_cam_imu::rotationVectorSMatrix(r) *
         bodyRotationVectorDerivativeAt(t);
}

Mat4 bodyPoseAt(const double t) {
  Mat4 T_t_b = Mat4::Identity();
  T_t_b.block<3, 3>(0, 0) = bodyRotationAt(t);
  T_t_b.block<3, 1>(0, 3) =
      Vec3(0.03 * std::sin(0.37 * t), 0.02 * std::cos(0.43 * t), -2.0);
  return T_t_b;
}

CameraExtrinsicBlock cameraExtrinsic(const std::size_t camera_index) {
  CameraExtrinsicBlock block;
  if (camera_index == 1) {
    block.values = {-0.12, 0.0, 0.0, 0.0, 0.0, 0.01};
  }
  return block;
}

Mat4 extrinsicMatrix(const CameraExtrinsicBlock &block) {
  return ceres_cam_imu::pose6ToMatrix(Eigen::Map<const Vec6>(block.data()));
}

SyntheticData makeSyntheticData(const std::size_t camera_count,
                                const std::size_t imu_count) {
  if (camera_count == 0 || imu_count < 2 || camera_count > 2 || imu_count > 4) {
    throw std::invalid_argument("unsupported synthetic test topology");
  }

  SyntheticData data;
  data.camera_poses.resize(camera_count);
  data.camera_extrinsics.resize(camera_count);
  data.camera_time_shifts_s = {0.010, -0.015};
  data.camera_time_shifts_s.resize(camera_count);
  for (std::size_t camera_index = 0; camera_index < camera_count;
       ++camera_index) {
    data.camera_extrinsics[camera_index] = cameraExtrinsic(camera_index);
    const Mat4 T_b_ci =
        extrinsicMatrix(data.camera_extrinsics[camera_index]).inverse();
    for (double true_time = kStartS; true_time <= kStartS + kDurationS + 1e-12;
         true_time += kPoseDtS) {
      PoseObservation pose;
      pose.timestamp_s = true_time - data.camera_time_shifts_s[camera_index];
      pose.T_t_c = bodyPoseAt(true_time) * T_b_ci;
      data.camera_poses[camera_index].push_back(pose);
    }
  }

  const std::vector<Vec3> imu_rotation_vectors = {
      Vec3::Zero(), Vec3(0.012, -0.006, 0.018), Vec3(-0.010, 0.014, -0.009),
      Vec3(0.008, 0.011, 0.012)};
  data.imu_time_offsets_s = {0.0, 0.015, -0.010, 0.025};
  data.imu_time_offsets_s.resize(imu_count);
  data.imus.resize(imu_count);
  data.imu_rotations.resize(imu_count);
  const Vec3 gravity_world(0.0, 0.0, -9.80655);
  for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
    data.imus[imu_index].label = "imu" + std::to_string(imu_index);
    data.imu_rotations[imu_index] =
        ceres_cam_imu::rotationVectorToMatrix(imu_rotation_vectors[imu_index]);
    const Vec3 bias(0.0005 * static_cast<double>(imu_index + 1),
                    -0.0003 * static_cast<double>(imu_index),
                    0.0002 * static_cast<double>(imu_index + 1));
    for (double true_time = kStartS; true_time <= kStartS + kDurationS + 1e-12;
         true_time += kImuDtS) {
      ImuSample sample;
      sample.timestamp_s = true_time - data.imu_time_offsets_s[imu_index];
      const Vec3 omega_body = bodyAngularVelocityAt(true_time);
      const Vec3 accel_body =
          -bodyRotationAt(true_time).transpose() * gravity_world;
      sample.gyro_rad_s = data.imu_rotations[imu_index] * omega_body + bias;
      sample.accel_m_s2 = data.imu_rotations[imu_index] * accel_body;
      data.imus[imu_index].samples.push_back(sample);
    }
  }
  return data;
}

void writeMatrixRows(std::ostream &out, const Mat4 &matrix,
                     const std::string &indent) {
  out << std::setprecision(17);
  for (int row = 0; row < 4; ++row) {
    out << indent << "- [";
    for (int col = 0; col < 4; ++col) {
      if (col > 0) {
        out << ", ";
      }
      out << matrix(row, col);
    }
    out << "]\n";
  }
}

void writeFixture(const std::filesystem::path &root,
                  const std::size_t camera_count, const std::size_t imu_count) {
  const SyntheticData data = makeSyntheticData(camera_count, imu_count);
  std::filesystem::create_directories(root);

  {
    std::ofstream out(root / "aprilgrid.yaml");
    out << "target_type: aprilgrid\n"
           "tagCols: 3\n"
           "tagRows: 1\n"
           "tagSize: 0.06\n"
           "tagSpacing: 0.3\n";
  }
  {
    std::ofstream out(root / "camchain.yaml");
    for (std::size_t camera_index = 0; camera_index < camera_count;
         ++camera_index) {
      out << "cam" << camera_index << ":\n"
          << "  rostopic: /cam" << camera_index << "/image_raw\n"
          << "  camera_model: pinhole\n"
          << "  distortion_model: radtan\n"
          << "  intrinsics: [430, 428, 320, 240]\n"
          << "  distortion_coeffs: [0, 0, 0, 0]\n"
          << "  resolution: [640, 480]\n"
          << "  timeshift_cam_imu: "
          << 0.123 + 0.01 * static_cast<double>(camera_index) << "\n"
          << "  T_cam_imu:\n";
      writeMatrixRows(
          out, extrinsicMatrix(data.camera_extrinsics[camera_index]), "    ");
    }
  }

  for (std::size_t imu_index = 0; imu_index < imu_count; ++imu_index) {
    {
      std::ofstream out(root / ("imu" + std::to_string(imu_index) + ".yaml"));
      out << "rostopic: /imu" << imu_index << "\n"
          << "update_rate: 200\n"
          << "accelerometer_noise_density: 0.001\n"
          << "accelerometer_random_walk: 0.00002\n"
          << "gyroscope_noise_density: 0.0001\n"
          << "gyroscope_random_walk: 0.000002\n";
    }
    {
      std::ofstream out(root / ("imu" + std::to_string(imu_index) + ".csv"));
      out << std::setprecision(17)
          << "#timestamp_ns,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,"
             "accel_x_m_s2,accel_y_m_s2,accel_z_m_s2\n";
      for (const ImuSample &sample : data.imus[imu_index].samples) {
        out << std::llround(sample.timestamp_s * 1e9) << ","
            << sample.gyro_rad_s.x() << "," << sample.gyro_rad_s.y() << ","
            << sample.gyro_rad_s.z() << "," << sample.accel_m_s2.x() << ","
            << sample.accel_m_s2.y() << "," << sample.accel_m_s2.z() << "\n";
      }
    }
  }

  const std::vector<Vec3> target_points = {
      {-0.12, -0.08, 0.0}, {-0.04, -0.08, 0.0}, {0.04, -0.08, 0.0},
      {0.12, -0.08, 0.0},  {-0.12, 0.0, 0.0},   {-0.04, 0.0, 0.0},
      {0.04, 0.0, 0.0},    {0.12, 0.0, 0.0},    {-0.12, 0.08, 0.0},
      {-0.04, 0.08, 0.0},  {0.04, 0.08, 0.0},   {0.12, 0.08, 0.0}};
  for (std::size_t camera_index = 0; camera_index < camera_count;
       ++camera_index) {
    {
      std::ofstream out(
          root / ("cam" + std::to_string(camera_index) + "_corner_poses.csv"));
      out << std::setprecision(17)
          << "timestamp_ns,T00,T01,T02,T03,T10,T11,T12,T13,T20,T21,T22,T23,"
             "T30,T31,T32,T33\n";
      for (const PoseObservation &pose : data.camera_poses[camera_index]) {
        out << std::llround(pose.timestamp_s * 1e9);
        for (int row = 0; row < 4; ++row) {
          for (int col = 0; col < 4; ++col) {
            out << "," << pose.T_t_c(row, col);
          }
        }
        out << "\n";
      }
    }
    {
      std::ofstream out(
          root / ("cam" + std::to_string(camera_index) + "_corners.csv"));
      out << std::setprecision(17)
          << "timestamp_ns,corner_id,u_px,v_px,target_x_m,target_y_m,"
             "target_z_m\n";
      for (std::size_t pose_index = 0;
           pose_index < data.camera_poses[camera_index].size() &&
           pose_index < 20;
           ++pose_index) {
        const PoseObservation &pose =
            data.camera_poses[camera_index][pose_index];
        const Mat4 T_c_t = pose.T_t_c.inverse();
        for (std::size_t point_index = 0; point_index < target_points.size();
             ++point_index) {
          const Vec3 point_camera =
              T_c_t.block<3, 3>(0, 0) * target_points[point_index] +
              T_c_t.block<3, 1>(0, 3);
          assert(point_camera.z() > 0.5);
          const double u = 430.0 * point_camera.x() / point_camera.z() + 320.0;
          const double v = 428.0 * point_camera.y() / point_camera.z() + 240.0;
          out << std::llround(pose.timestamp_s * 1e9) << "," << point_index
              << "," << u << "," << v << "," << target_points[point_index].x()
              << "," << target_points[point_index].y() << ","
              << target_points[point_index].z() << "\n";
        }
      }
    }
  }
}

void runLibraryChecks() {
  const SyntheticData data = makeSyntheticData(2, 4);
  ceres_cam_imu::KalibrStyleMultiImuInitializerOptions options;
  assert(options.imu_chain.use_full_overlap_time_offset_search);
  options.imu_chain.min_samples = 100;
  options.camera_time_shift_seeds.resize(2);
  for (auto &seed : options.camera_time_shift_seeds) {
    seed.initial_time_shift_valid = true;
  }

  // Start from a deliberately wrong global camera-to-body transform while
  // keeping the inter-camera baseline exact. This distinguishes actual
  // camera-0 orientation recovery from an implementation that merely copies
  // the supplied absolute extrinsics.
  std::vector<CameraExtrinsicBlock> incorrect_camera_extrinsics =
      data.camera_extrinsics;
  const Mat4 global_seed_error = ceres_cam_imu::pose6ToMatrix(
      (Vec6() << 0.25, -0.15, 0.08, 0.10, -0.07, 0.05).finished());
  for (std::size_t camera_index = 0;
       camera_index < incorrect_camera_extrinsics.size(); ++camera_index) {
    const Mat4 incorrect =
        extrinsicMatrix(data.camera_extrinsics[camera_index]) *
        global_seed_error;
    const Vec6 pose = ceres_cam_imu::matrixToPose6(incorrect);
    std::copy(pose.data(), pose.data() + 6,
              incorrect_camera_extrinsics[camera_index].data());
  }

  const auto result = ceres_cam_imu::estimateKalibrStyleMultiImuInitialization(
      data.camera_poses, data.imus, incorrect_camera_extrinsics, options);
  assert(result.cameras.size() == 2);
  assert(result.imu_extrinsics.size() == 4);
  assert(result.imu_time_offsets_s.size() == 4);
  for (std::size_t camera_index = 0; camera_index < 2; ++camera_index) {
    assert(result.cameras[camera_index].time_shift_valid);
    assert(std::abs(result.cameras[camera_index].time_shift_s -
                    data.camera_time_shifts_s[camera_index]) <=
           kImuDtS + 1e-12);
  }
  assert(std::abs(result.cameras[0].T_c_b.values[0]) < 1e-15);
  assert(std::abs(result.cameras[0].T_c_b.values[1]) < 1e-15);
  assert(std::abs(result.cameras[0].T_c_b.values[2]) < 1e-15);
  const Mat4 initial_baseline =
      extrinsicMatrix(incorrect_camera_extrinsics[1]) *
      extrinsicMatrix(incorrect_camera_extrinsics[0]).inverse();
  const Mat4 estimated_baseline =
      extrinsicMatrix(result.cameras[1].T_c_b) *
      extrinsicMatrix(result.cameras[0].T_c_b).inverse();
  assert((initial_baseline - estimated_baseline).norm() < 1e-10);

  const Mat3 estimated_R_c_b = ceres_cam_imu::rotationVectorToMatrix(
      Vec3(result.cameras[0].T_c_b.values[3], result.cameras[0].T_c_b.values[4],
           result.cameras[0].T_c_b.values[5]));
  assert(rotationDeltaRad(estimated_R_c_b, Mat3::Identity()) < 0.03);
  assert((result.orientation_gravity.gravity_m_s2 - Vec3(0.0, 0.0, -9.80655))
             .norm() < 0.12);

  for (std::size_t imu_index = 0; imu_index < result.imu_extrinsics.size();
       ++imu_index) {
    const auto &extrinsic = result.imu_extrinsics[imu_index];
    assert(std::abs(extrinsic.values[0]) < 1e-15);
    assert(std::abs(extrinsic.values[1]) < 1e-15);
    assert(std::abs(extrinsic.values[2]) < 1e-15);
    const Mat3 estimated_R_i_b = ceres_cam_imu::rotationVectorToMatrix(
        Vec3(extrinsic.values[3], extrinsic.values[4], extrinsic.values[5]));
    assert(rotationDeltaRad(estimated_R_i_b, data.imu_rotations[imu_index]) <
           0.02);
    assert(std::abs(result.imu_time_offsets_s[imu_index] -
                    data.imu_time_offsets_s[imu_index]) <= kImuDtS + 1e-12);
  }

  ImuObservationDataset duplicate_reference = data.imus[0];
  ImuObservationDataset duplicate_target = data.imus[1];
  duplicate_reference.samples.insert(duplicate_reference.samples.begin() + 400,
                                     5, duplicate_reference.samples[400]);
  duplicate_target.samples.insert(duplicate_target.samples.begin() + 500, 4,
                                  duplicate_target.samples[500]);
  const auto duplicate_pair = ceres_cam_imu::estimateImuChainPairPrior(
      duplicate_reference, duplicate_target, 1, options.imu_chain);
  assert(std::abs(duplicate_pair.time_offset_s - data.imu_time_offsets_s[1]) <
         1e-6);

  // Each IMU may use an unrelated clock epoch. Full-overlap search must
  // recover the total timestamp mapping, not require the raw timestamp ranges
  // to intersect first.
  SyntheticData independent_clock_data = data;
  const std::vector<double> imu_epoch_shifts_s = {0.0, 500.003, -250.004,
                                                  1000.0015};
  for (std::size_t imu_index = 0;
       imu_index < independent_clock_data.imus.size(); ++imu_index) {
    for (ImuSample &sample : independent_clock_data.imus[imu_index].samples) {
      sample.timestamp_s += imu_epoch_shifts_s[imu_index];
    }
  }
  const auto independent_clock_result =
      ceres_cam_imu::estimateKalibrStyleMultiImuInitialization(
          independent_clock_data.camera_poses, independent_clock_data.imus,
          incorrect_camera_extrinsics, options);
  for (std::size_t imu_index = 0;
       imu_index < independent_clock_result.imu_time_offsets_s.size();
       ++imu_index) {
    const double expected_total_offset_s =
        data.imu_time_offsets_s[imu_index] - imu_epoch_shifts_s[imu_index];
    assert(std::abs(independent_clock_result.imu_time_offsets_s[imu_index] -
                    expected_total_offset_s) < 1e-6);
    if (imu_index > 0) {
      const auto &pair =
          independent_clock_result.imu_chain.imu_results[imu_index - 1];
      const double reconstructed_offset_s =
          static_cast<double>(pair.discrete_shift_samples) * pair.sample_dt_s +
          pair.discrete_shift_residual_s;
      assert(std::abs(pair.time_offset_s - reconstructed_offset_s) < 1e-12);
      assert(std::abs(pair.discrete_shift_residual_s) > 1e-5);
      assert(std::abs(pair.discrete_shift_residual_s) <=
             0.5 * pair.sample_dt_s + 1e-12);
    }
  }

  // The same time mappings must be applied before spline construction, not
  // only after initialization. Otherwise the unrelated raw epochs above
  // would create a roughly 1250-second spline and the joint problem would no
  // longer be a practical end-to-end path.
  std::vector<ceres_cam_imu::CameraObservationDataset> aligned_cameras(2);
  for (std::size_t camera_index = 0; camera_index < aligned_cameras.size();
       ++camera_index) {
    aligned_cameras[camera_index].images.resize(2);
    aligned_cameras[camera_index].images[0].timestamp_s =
        data.camera_poses[camera_index].front().timestamp_s;
    aligned_cameras[camera_index].images[1].timestamp_s =
        data.camera_poses[camera_index].back().timestamp_s;
  }
  ceres_cam_imu::CalibrationOptions aligned_options;
  aligned_options.add_bias_motion_prior = false;
  aligned_options.max_imu_residuals = 20;
  aligned_options.optimize_imu_time_offsets = true;
  for (const auto &camera : independent_clock_result.cameras) {
    aligned_options.initial_camera_time_shifts_s.push_back(camera.time_shift_s);
  }
  aligned_options.initial_camera_time_shift_s =
      aligned_options.initial_camera_time_shifts_s.front();
  aligned_options.initial_imu_time_offsets_s =
      independent_clock_result.imu_time_offsets_s;
  ceres_cam_imu::CalibrationState aligned_state =
      ceres_cam_imu::initializeCalibrationState(
          aligned_cameras, independent_clock_data.imus, aligned_options);
  assert(aligned_state.pose_spline.numSegments() < 200);
  assert(aligned_state.imu_time_offsets_s ==
         independent_clock_result.imu_time_offsets_s);
  ceres::Problem aligned_problem;
  const auto aligned_summary = ceres_cam_imu::buildCalibrationProblem(
      aligned_cameras, independent_clock_data.imus, aligned_options,
      &aligned_state, &aligned_problem);
  assert(aligned_summary.gyro_residuals == 80);
  assert(aligned_summary.accel_residuals == 80);

  bool missing_camera_poses_threw = false;
  try {
    auto incomplete_camera_poses = data.camera_poses;
    incomplete_camera_poses[1].clear();
    (void)ceres_cam_imu::estimateKalibrStyleMultiImuInitialization(
        incomplete_camera_poses, data.imus, data.camera_extrinsics, options);
  } catch (const std::runtime_error &error) {
    missing_camera_poses_threw =
        std::string(error.what()).find("camera-1 poses") != std::string::npos;
  }
  assert(missing_camera_poses_threw);

  ImuObservationDataset overlap_boundary_reference;
  ImuObservationDataset overlap_boundary_target;
  for (double timestamp_s = 0.0; timestamp_s <= 6.0 + 1e-12;
       timestamp_s += kImuDtS) {
    ImuSample sample;
    sample.timestamp_s = timestamp_s;
    sample.gyro_rad_s = bodyAngularVelocityAt(timestamp_s + 1.0);
    overlap_boundary_reference.samples.push_back(sample);
  }
  for (double timestamp_s = 3.0; timestamp_s <= 9.0 + 1e-12;
       timestamp_s += kImuDtS) {
    ImuSample sample;
    sample.timestamp_s = timestamp_s;
    sample.gyro_rad_s = bodyAngularVelocityAt(timestamp_s + 1.0);
    overlap_boundary_target.samples.push_back(sample);
  }
  const auto overlap_boundary = ceres_cam_imu::estimateImuChainPairPrior(
      overlap_boundary_reference, overlap_boundary_target, 1,
      options.imu_chain);
  assert(overlap_boundary.time_offset_boundary_peak_rejected);
  assert(overlap_boundary.rejected_matched_samples >= 600);
  assert(overlap_boundary.rejected_peak_correlation > 0.999);
  assert(overlap_boundary.time_offset_s == 0.0);

  auto fixed_options = options;
  fixed_options.camera_time_shift_seeds[0].estimate = false;
  fixed_options.camera_time_shift_seeds[0].initial_time_shift_s = 0.001;
  const auto fixed_result =
      ceres_cam_imu::estimateKalibrStyleMultiImuInitialization(
          data.camera_poses, data.imus, data.camera_extrinsics, fixed_options);
  assert(fixed_result.cameras[0].time_shift_valid);
  assert(std::abs(fixed_result.cameras[0].time_shift_s - 0.001) < 1e-15);
  assert(std::abs(fixed_result.cameras[1].time_shift_s -
                  data.camera_time_shifts_s[1]) <= kImuDtS + 1e-12);
  const Mat3 fixed_R_c_b = ceres_cam_imu::rotationVectorToMatrix(
      Vec3(fixed_result.cameras[0].T_c_b.values[3],
           fixed_result.cameras[0].T_c_b.values[4],
           fixed_result.cameras[0].T_c_b.values[5]));
  assert(rotationDeltaRad(fixed_R_c_b, Mat3::Identity()) < 0.03);
  assert(
      (fixed_result.orientation_gravity.gravity_m_s2 - Vec3(0.0, 0.0, -9.80655))
          .norm() < 0.12);
  for (std::size_t imu_index = 1;
       imu_index < fixed_result.imu_extrinsics.size(); ++imu_index) {
    assert(std::abs(fixed_result.imu_time_offsets_s[imu_index] -
                    data.imu_time_offsets_s[imu_index]) <= kImuDtS + 1e-12);
    const auto &extrinsic = fixed_result.imu_extrinsics[imu_index];
    const Mat3 fixed_R_i_b = ceres_cam_imu::rotationVectorToMatrix(
        Vec3(extrinsic.values[3], extrinsic.values[4], extrinsic.values[5]));
    assert(rotationDeltaRad(fixed_R_i_b, data.imu_rotations[imu_index]) < 0.02);
  }

  auto boundary_options = options;
  boundary_options.camera_time_shift_seeds.resize(1);
  boundary_options.camera_time_shift.max_search_s = 0.010;
  boundary_options.camera_time_shift_seeds[0].initial_time_shift_s = 0.004;
  const auto boundary_result =
      ceres_cam_imu::estimateKalibrStyleMultiImuInitialization(
          std::vector<std::vector<PoseObservation>>{data.camera_poses[0]},
          data.imus,
          std::vector<CameraExtrinsicBlock>{incorrect_camera_extrinsics[0]},
          boundary_options);
  assert(boundary_result.cameras[0].time_shift_estimate.boundary_peak_rejected);
  assert(boundary_result.cameras[0].time_shift_valid);
  assert(std::abs(boundary_result.cameras[0].time_shift_s - 0.004) < 1e-15);

  auto rejected_imu_options = options;
  rejected_imu_options.imu_chain.use_full_overlap_time_offset_search = false;
  rejected_imu_options.imu_chain.max_time_offset_search_s = 0.015;
  bool threw = false;
  try {
    (void)ceres_cam_imu::estimateKalibrStyleMultiImuInitialization(
        data.camera_poses, data.imus, data.camera_extrinsics,
        rejected_imu_options);
  } catch (const std::runtime_error &error) {
    threw = std::string(error.what()).find("boundary time offset peak") !=
            std::string::npos;
  }
  assert(threw);
}

} // namespace

int main(int argc, char **argv) {
  if (argc == 5 && std::string(argv[1]) == "--generate-fixture") {
    writeFixture(argv[2], static_cast<std::size_t>(std::stoul(argv[3])),
                 static_cast<std::size_t>(std::stoul(argv[4])));
    return 0;
  }
  assert(argc == 1);
  runLibraryChecks();
  return 0;
}
