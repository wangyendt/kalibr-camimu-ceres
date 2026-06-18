#include "ceres_cam_imu/io/calibration_result_writer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/variables/imu_intrinsics.h"

namespace ceres_cam_imu {
namespace {

Mat4 inverseRigidTransform(const Mat4& T) {
  Mat4 inverse = Mat4::Identity();
  const Mat3 R_t = T.block<3, 3>(0, 0).transpose();
  inverse.block<3, 3>(0, 0) = R_t;
  inverse.block<3, 1>(0, 3) = -R_t * T.block<3, 1>(0, 3);
  return inverse;
}

void writeVector3(std::ostream& os, const Vec3& value) {
  os << "[" << value.x() << ", " << value.y() << ", " << value.z() << "]";
}

void writeMatrix4(std::ostream& os, const Mat4& value) {
  os << "[";
  for (int r = 0; r < 4; ++r) {
    if (r != 0) {
      os << ", ";
    }
    os << "[";
    for (int c = 0; c < 4; ++c) {
      if (c != 0) {
        os << ", ";
      }
      os << value(r, c);
    }
    os << "]";
  }
  os << "]";
}

void writeMatrix3(std::ostream& os, const Mat3& value) {
  os << "[";
  for (int r = 0; r < 3; ++r) {
    if (r != 0) {
      os << ", ";
    }
    os << "[";
    for (int c = 0; c < 3; ++c) {
      if (c != 0) {
        os << ", ";
      }
      os << value(r, c);
    }
    os << "]";
  }
  os << "]";
}

template <std::size_t N>
void writeStdArray(std::ostream& os, const std::array<double, N>& value) {
  os << "[";
  for (std::size_t i = 0; i < N; ++i) {
    if (i != 0) {
      os << ", ";
    }
    os << value[i];
  }
  os << "]";
}

template <typename Spline>
void writeSplineMetadata(std::ostream& os, const Spline& spline,
                         const std::string& indent) {
  os << indent << "order: " << spline.order() << "\n";
  os << indent << "t_min_s: " << spline.tMin() << "\n";
  os << indent << "t_max_s: " << spline.tMax() << "\n";
  os << indent << "dt_s: " << spline.dt() << "\n";
  os << indent << "num_segments: " << spline.numSegments() << "\n";
  os << indent << "num_coefficients: " << spline.numCoefficients() << "\n";
}

template <typename Blocks>
void writeControlBlocks(std::ostream& os, const Blocks& blocks,
                        const std::string& indent) {
  os << indent << "controls:\n";
  for (const auto& block : blocks) {
    os << indent << "  - ";
    writeStdArray(os, block.values);
    os << "\n";
  }
}

void writeResidualStats(std::ostream& os, const char* key,
                        const ResidualMagnitudeStats& stats) {
  os << "  " << key << ":\n";
  os << "    count: " << stats.count << "\n";
  os << "    mean: " << stats.mean << "\n";
  os << "    median: " << stats.median << "\n";
  os << "    std: " << stats.stddev << "\n";
  os << "    rms: " << stats.rms << "\n";
  os << "    max: " << stats.max << "\n";
}

double rotationDeltaDeg(const Mat4& lhs, const Mat4& rhs) {
  const Mat3 dR = lhs.block<3, 3>(0, 0) * rhs.block<3, 3>(0, 0).transpose();
  const double cos_angle =
      std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0);
  constexpr double kPi = 3.14159265358979323846;
  return std::acos(cos_angle) * 180.0 / kPi;
}

}  // namespace

void writeCalibrationResultYaml(
    const std::string& output_path, const CalibrationState& state,
    const CalibrationResidualStatistics& residual_stats,
    const CalibrationResultWriterOptions& options) {
  std::ofstream output(output_path);
  if (!output.is_open()) {
    throw std::runtime_error("failed to open calibration result output: " +
                             output_path);
  }
  output << std::setprecision(17);

  const Mat4 T_c_b = pose6ToMatrix(state.T_c_b);
  const Mat4 T_b_c = inverseRigidTransform(T_c_b);
  const Vec3 gravity(state.gravity.values[0], state.gravity.values[1],
                     state.gravity.values[2]);

  output << "format_version: 1\n";
  output << "camera_to_body:\n";
  output << "  T_c_b: ";
  writeMatrix4(output, T_c_b);
  output << "\n";
  output << "  T_b_c: ";
  writeMatrix4(output, T_b_c);
  output << "\n";
  output << "time_shift_s: " << state.camera_time_shift_s.value << "\n";
  output << "gravity: ";
  writeVector3(output, gravity);
  output << "\n";
  output << "imu_extrinsic:\n";
  output << "  r_b: ";
  writeVector3(output, Vec3(state.imu_extrinsic.values[0],
                            state.imu_extrinsic.values[1],
                            state.imu_extrinsic.values[2]));
  output << "\n";
  output << "  r_i_b: ";
  writeVector3(output, Vec3(state.imu_extrinsic.values[3],
                            state.imu_extrinsic.values[4],
                            state.imu_extrinsic.values[5]));
  output << "\n";
  if (!state.camera_extrinsics.empty()) {
    output << "camera_chain:\n";
    for (std::size_t camera_index = 0;
         camera_index < state.camera_extrinsics.size(); ++camera_index) {
      const Mat4 T_camera_body =
          camera_index == 0 ? T_c_b
                            : pose6ToMatrix(
                                  state.camera_extrinsics[camera_index]);
      const double time_shift =
          camera_index == 0
              ? state.camera_time_shift_s.value
              : camera_index < state.camera_time_shifts.size()
              ? state.camera_time_shifts[camera_index].value
              : 0.0;
      output << "  - camera_index: " << camera_index << "\n";
      output << "    T_c_b: ";
      writeMatrix4(output, T_camera_body);
      output << "\n";
      output << "    time_shift_s: " << time_shift << "\n";
    }
  }
  output << "imu_intrinsics:\n";
  output << "  accel_M: ";
  writeMatrix3(output, lowerTriangularMatrix(state.imu_intrinsics.accel_M.data()));
  output << "\n";
  output << "  gyro_M: ";
  writeMatrix3(output, lowerTriangularMatrix(state.imu_intrinsics.gyro_M.data()));
  output << "\n";
  output << "  gyro_accel_sensitivity: ";
  writeMatrix3(output,
               matrix3Block(state.imu_intrinsics.gyro_accel_sensitivity.data()));
  output << "\n";
  const Vec3 gyro_sensing_rotation_vector =
      vector3Block(state.imu_intrinsics.gyro_sensing_rotation.data());
  output << "  gyro_sensing_rotation: ";
  writeMatrix3(output, rotationVectorToMatrix(gyro_sensing_rotation_vector));
  output << "\n";
  output << "  gyro_sensing_rotation_vector: ";
  writeVector3(output, gyro_sensing_rotation_vector);
  output << "\n";
  output << "  accel_axis_rx_i: ";
  writeVector3(output, vector3Block(state.imu_intrinsics.accel_axis_rx_i.data()));
  output << "\n";
  output << "  accel_axis_ry_i: ";
  writeVector3(output, vector3Block(state.imu_intrinsics.accel_axis_ry_i.data()));
  output << "\n";
  output << "  accel_axis_rz_i: ";
  writeVector3(output, vector3Block(state.imu_intrinsics.accel_axis_rz_i.data()));
  output << "\n";

  output << "pose_spline:\n";
  writeSplineMetadata(output, state.pose_spline, "  ");
  if (options.include_spline_controls) {
    writeControlBlocks(output, state.pose_controls, "  ");
  }
  output << "bias_splines:\n";
  output << "  gyro_num_coefficients: "
         << state.gyro_bias_spline.numCoefficients() << "\n";
  output << "  accel_num_coefficients: "
         << state.accel_bias_spline.numCoefficients() << "\n";
  output << "  gyro:\n";
  writeSplineMetadata(output, state.gyro_bias_spline, "    ");
  if (options.include_spline_controls) {
    writeControlBlocks(output, state.gyro_bias_controls, "    ");
  }
  output << "  accel:\n";
  writeSplineMetadata(output, state.accel_bias_spline, "    ");
  if (options.include_spline_controls) {
    writeControlBlocks(output, state.accel_bias_controls, "    ");
  }

  output << "residual_statistics:\n";
  writeResidualStats(output, "reprojection_px",
                     residual_stats.reprojection_px);
  writeResidualStats(output, "reprojection_normalized",
                     residual_stats.reprojection_normalized);
  writeResidualStats(output, "gyro_rad_s", residual_stats.gyro_rad_s);
  writeResidualStats(output, "gyro_normalized",
                     residual_stats.gyro_normalized);
  writeResidualStats(output, "accel_m_s2", residual_stats.accel_m_s2);
  writeResidualStats(output, "accel_normalized",
                     residual_stats.accel_normalized);
  output << "  skipped:\n";
  output << "    camera_frames: " << residual_stats.skipped_camera_frames
         << "\n";
  output << "    camera_projections: "
         << residual_stats.skipped_camera_projections << "\n";
  output << "    imu_samples: " << residual_stats.skipped_imu_samples << "\n";

  output << "top_accel_outliers:\n";
  for (const ImuResidualOutlier& outlier :
       residual_stats.top_accel_outliers) {
    output << "  - sample_index: " << outlier.sample_index << "\n";
    output << "    timestamp_s: " << outlier.timestamp_s << "\n";
    output << "    accel_m_s2: " << outlier.accel_error_m_s2 << "\n";
    output << "    accel_normalized: " << outlier.accel_normalized << "\n";
    output << "    gyro_rad_s: " << outlier.gyro_error_rad_s << "\n";
    output << "    gyro_normalized: " << outlier.gyro_normalized << "\n";
    output << "    pose_accel_world_norm: "
           << outlier.pose_accel_world_norm << "\n";
    output << "    gravity_corrected_body_accel_norm: "
           << outlier.gravity_corrected_body_accel_norm << "\n";
  }

  if (options.include_kalibr_comparison) {
    const KalibrResult& kalibr = options.kalibr_result;
    output << "kalibr_delta:\n";
    output << "  rotation_deg: " << rotationDeltaDeg(T_c_b, kalibr.T_ci)
           << "\n";
    output << "  translation_m: "
           << (T_c_b.block<3, 1>(0, 3) - kalibr.T_ci.block<3, 1>(0, 3))
                  .norm()
           << "\n";
    output << "  time_shift_s: "
           << (state.camera_time_shift_s.value -
               kalibr.timeshift_cam_to_imu_s)
           << "\n";
    output << "  gravity_norm: " << (gravity - kalibr.gravity).norm()
           << "\n";
  }

  if (!output.good()) {
    throw std::runtime_error("failed to write calibration result output: " +
                             output_path);
  }
}

}  // namespace ceres_cam_imu
