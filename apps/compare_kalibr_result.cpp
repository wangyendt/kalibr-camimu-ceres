#include <iostream>
#include <string>

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "ceres_cam_imu/io/calibration_result_reader.h"
#include "ceres_cam_imu/io/kalibr_result_parser.h"

namespace {

std::string argValue(int argc, char** argv, const std::string& name) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      return argv[i + 1];
    }
  }
  return "";
}

double rotationDeltaDeg(const ceres_cam_imu::Mat4& lhs,
                        const ceres_cam_imu::Mat4& rhs) {
  const ceres_cam_imu::Mat3 dR =
      lhs.block<3, 3>(0, 0) * rhs.block<3, 3>(0, 0).transpose();
  const double cos_angle =
      std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0);
  constexpr double kPi = 3.14159265358979323846;
  return std::acos(cos_angle) * 180.0 / kPi;
}

double relativeFrobenius(const ceres_cam_imu::Mat3& delta,
                         const ceres_cam_imu::Mat3& reference) {
  const double denom = reference.norm();
  if (denom <= 0.0) {
    return delta.norm();
  }
  return delta.norm() / denom;
}

double relativeNorm(const ceres_cam_imu::Vec3& delta,
                    const ceres_cam_imu::Vec3& reference) {
  const double denom = reference.norm();
  if (denom <= 0.0) {
    return delta.norm();
  }
  return delta.norm() / denom;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string result_path = argValue(argc, argv, "--kalibr-result");
  const std::string ceres_result_path = argValue(argc, argv, "--ceres-result");
  if (result_path.empty()) {
    std::cout << "compare_kalibr_result --kalibr-result result.txt "
                 "[--ceres-result result.yaml]\n";
    return 2;
  }
  const ceres_cam_imu::KalibrResult result =
      ceres_cam_imu::readKalibrResult(result_path);
  std::cout << "T_ci:\n" << result.T_ci << "\n";
  std::cout << "T_ic:\n" << result.T_ic << "\n";
  std::cout << "timeshift_cam_to_imu_s: " << result.timeshift_cam_to_imu_s << "\n";
  std::cout << "gravity: " << result.gravity.transpose() << "\n";
  std::cout << "mean residuals: reproj_px="
            << result.residuals.reprojection_mean_px
            << " gyro_rad_s=" << result.residuals.gyro_mean_rad_s
            << " accel_m_s2=" << result.residuals.accel_mean_m_s2 << "\n";
  std::cout << "mean normalized residuals: reproj="
            << result.residuals.reprojection_normalized_mean
            << " gyro=" << result.residuals.gyro_normalized_mean
            << " accel=" << result.residuals.accel_normalized_mean << "\n";
  for (std::size_t camera_index = 0;
       camera_index < result.camera_T_ci.size(); ++camera_index) {
    const double time_shift =
        camera_index < result.camera_timeshift_cam_to_imu_s.size()
            ? result.camera_timeshift_cam_to_imu_s[camera_index]
            : 0.0;
    const double reproj =
        camera_index < result.camera_reprojection_mean_px.size()
            ? result.camera_reprojection_mean_px[camera_index]
            : 0.0;
    std::cout << "kalibr_camera_chain camera=" << camera_index
              << " time_shift_s=" << std::setprecision(17) << time_shift
              << " reproj_px=" << reproj << " translation_m="
              << result.camera_T_ci[camera_index](0, 3) << " "
              << result.camera_T_ci[camera_index](1, 3) << " "
              << result.camera_T_ci[camera_index](2, 3) << "\n";
  }
  if (!ceres_result_path.empty()) {
    const ceres_cam_imu::CalibrationResultFile ceres_result =
        ceres_cam_imu::readCalibrationResultYaml(ceres_result_path);
    const double rotation_deg =
        rotationDeltaDeg(ceres_result.T_c_b, result.T_ci);
    const double translation_m =
        (ceres_result.T_c_b.block<3, 1>(0, 3) -
         result.T_ci.block<3, 1>(0, 3))
            .norm();
    const double time_shift_delta_s =
        ceres_result.time_shift_s - result.timeshift_cam_to_imu_s;
    const double gravity_norm =
        (ceres_result.gravity - result.gravity).norm();
    std::cout << "ceres T_c_b:\n" << ceres_result.T_c_b << "\n";
    std::cout << "ceres time_shift_s: " << ceres_result.time_shift_s << "\n";
    std::cout << "ceres gravity: " << ceres_result.gravity.transpose()
              << "\n";
    std::cout << "ceres mean residuals: reproj_px="
              << ceres_result.residuals.reprojection_mean_px
              << " gyro_rad_s=" << ceres_result.residuals.gyro_mean_rad_s
              << " accel_m_s2="
              << ceres_result.residuals.accel_mean_m_s2 << "\n";
    std::cout << "ceres_vs_kalibr: rotation_deg=" << rotation_deg
              << " translation_m=" << translation_m
              << " time_shift_s=" << time_shift_delta_s
              << " gravity_norm=" << gravity_norm << "\n";
    std::cout << "residual_delta_ceres_minus_kalibr: reproj_px="
              << (ceres_result.residuals.reprojection_mean_px -
                  result.residuals.reprojection_mean_px)
              << " gyro_rad_s="
              << (ceres_result.residuals.gyro_mean_rad_s -
                  result.residuals.gyro_mean_rad_s)
              << " accel_m_s2="
              << (ceres_result.residuals.accel_mean_m_s2 -
                  result.residuals.accel_mean_m_s2)
              << "\n";
    const std::size_t camera_count =
        std::min(ceres_result.camera_T_c_b.size(), result.camera_T_ci.size());
    for (std::size_t camera_index = 0; camera_index < camera_count;
         ++camera_index) {
      const double camera_rotation_deg =
          rotationDeltaDeg(ceres_result.camera_T_c_b[camera_index],
                           result.camera_T_ci[camera_index]);
      const double camera_translation_m =
          (ceres_result.camera_T_c_b[camera_index].block<3, 1>(0, 3) -
           result.camera_T_ci[camera_index].block<3, 1>(0, 3))
              .norm();
      const double ceres_time_shift =
          camera_index < ceres_result.camera_time_shift_s.size()
              ? ceres_result.camera_time_shift_s[camera_index]
              : 0.0;
      const double kalibr_time_shift =
          camera_index < result.camera_timeshift_cam_to_imu_s.size()
              ? result.camera_timeshift_cam_to_imu_s[camera_index]
              : 0.0;
      std::cout << "camera_chain_delta_ceres_minus_kalibr: camera="
                << camera_index << " rotation_deg=" << camera_rotation_deg
                << " translation_m=" << camera_translation_m
                << " time_shift_s=" << (ceres_time_shift - kalibr_time_shift)
                << "\n";
    }
    if (ceres_result.has_accel_M && result.has_accel_M &&
        ceres_result.has_gyro_M && result.has_gyro_M) {
      const ceres_cam_imu::Mat3 accel_M_delta =
          ceres_result.accel_M - result.accel_M;
      const ceres_cam_imu::Mat3 gyro_M_delta =
          ceres_result.gyro_M - result.gyro_M;
      std::cout << "imu_intrinsics_delta_ceres_minus_kalibr: "
                << "accel_M_fro=" << accel_M_delta.norm()
                << " accel_M_rel="
                << relativeFrobenius(accel_M_delta, result.accel_M)
                << " gyro_M_fro=" << gyro_M_delta.norm()
                << " gyro_M_rel="
                << relativeFrobenius(gyro_M_delta, result.gyro_M);
      if (ceres_result.has_gyro_accel_sensitivity &&
          result.has_gyro_accel_sensitivity) {
        const ceres_cam_imu::Mat3 gyro_A_delta =
            ceres_result.gyro_accel_sensitivity -
            result.gyro_accel_sensitivity;
        std::cout << " gyro_A_fro=" << gyro_A_delta.norm()
                  << " gyro_A_rel="
                  << relativeFrobenius(gyro_A_delta,
                                       result.gyro_accel_sensitivity);
      }
      if (ceres_result.has_gyro_sensing_rotation &&
          result.has_gyro_sensing_rotation) {
        const ceres_cam_imu::Mat3 gyro_C_delta =
            ceres_result.gyro_sensing_rotation -
            result.gyro_sensing_rotation;
        std::cout << " gyro_C_fro=" << gyro_C_delta.norm()
                  << " gyro_C_rel="
                  << relativeFrobenius(gyro_C_delta,
                                       result.gyro_sensing_rotation);
      }
      if (ceres_result.has_accel_axis_rx_i && result.has_accel_axis_rx_i &&
          ceres_result.has_accel_axis_ry_i && result.has_accel_axis_ry_i &&
          ceres_result.has_accel_axis_rz_i && result.has_accel_axis_rz_i) {
        const ceres_cam_imu::Vec3 rx_delta =
            ceres_result.accel_axis_rx_i - result.accel_axis_rx_i;
        const ceres_cam_imu::Vec3 ry_delta =
            ceres_result.accel_axis_ry_i - result.accel_axis_ry_i;
        const ceres_cam_imu::Vec3 rz_delta =
            ceres_result.accel_axis_rz_i - result.accel_axis_rz_i;
        std::cout << " accel_rx_norm=" << rx_delta.norm()
                  << " accel_rx_rel="
                  << relativeNorm(rx_delta, result.accel_axis_rx_i)
                  << " accel_ry_norm=" << ry_delta.norm()
                  << " accel_ry_rel="
                  << relativeNorm(ry_delta, result.accel_axis_ry_i)
                  << " accel_rz_norm=" << rz_delta.norm()
                  << " accel_rz_rel="
                  << relativeNorm(rz_delta, result.accel_axis_rz_i);
      }
      std::cout << "\n";
    }
    if (ceres_result.has_kalibr_delta) {
      std::cout << "embedded_kalibr_delta: rotation_deg="
                << ceres_result.kalibr_delta.rotation_deg
                << " translation_m="
                << ceres_result.kalibr_delta.translation_m
                << " time_shift_s="
                << ceres_result.kalibr_delta.time_shift_s
                << " gravity_norm="
                << ceres_result.kalibr_delta.gravity_norm << "\n";
    }
  }
  return 0;
}
