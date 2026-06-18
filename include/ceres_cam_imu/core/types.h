#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ceres_cam_imu {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat23 = Eigen::Matrix<double, 2, 3>;
using Mat26 = Eigen::Matrix<double, 2, 6>;
using Mat36 = Eigen::Matrix<double, 3, 6>;

enum class ImuCalibrationModel {
  kCalibrated,
  kScaleMisalignment,
  kScaleMisalignmentSizeEffect,
};

struct CameraIntrinsics {
  std::vector<double> intrinsics;
  std::vector<double> distortion_coeffs;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double xi = 0.0;
  double alpha = 0.0;
  double beta = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  int width = 0;
  int height = 0;
  std::string camera_model = "pinhole";
  std::string distortion_model = "radtan";
};

struct ImuNoise {
  double update_rate_hz = 0.0;
  double accelerometer_noise_density = 0.0;
  double accelerometer_random_walk = 0.0;
  double gyroscope_noise_density = 0.0;
  double gyroscope_random_walk = 0.0;

  double accelDiscreteSigma() const;
  double gyroDiscreteSigma() const;
};

struct AprilGridConfig {
  int tag_cols = 0;
  int tag_rows = 0;
  double tag_size_m = 0.0;
  double tag_spacing_ratio = 0.0;
};

struct ImuSample {
  double timestamp_s = 0.0;
  Vec3 gyro_rad_s = Vec3::Zero();
  Vec3 accel_m_s2 = Vec3::Zero();
};

struct CornerMeasurement {
  int corner_id = -1;
  Vec2 pixel = Vec2::Zero();
  Vec3 target_point = Vec3::Zero();
};

struct ImageObservation {
  double timestamp_s = 0.0;
  std::vector<CornerMeasurement> corners;
};

struct PoseObservation {
  double timestamp_s = 0.0;
  Mat4 T_t_c = Mat4::Identity();
};

struct KalibrResidualStats {
  double reprojection_mean_px = 0.0;
  double gyro_mean_rad_s = 0.0;
  double accel_mean_m_s2 = 0.0;
  double reprojection_normalized_mean = 0.0;
  double gyro_normalized_mean = 0.0;
  double accel_normalized_mean = 0.0;
};

struct KalibrResult {
  Mat4 T_ci = Mat4::Identity();
  Mat4 T_ic = Mat4::Identity();
  Vec3 gravity = Vec3::Zero();
  double timeshift_cam_to_imu_s = 0.0;
  KalibrResidualStats residuals;
  std::vector<Mat4> camera_T_ci;
  std::vector<Mat4> camera_T_ic;
  std::vector<double> camera_timeshift_cam_to_imu_s;
  std::vector<double> camera_reprojection_mean_px;
  std::vector<double> camera_reprojection_normalized_mean;
  bool has_accel_M = false;
  bool has_gyro_M = false;
  bool has_gyro_accel_sensitivity = false;
  bool has_gyro_sensing_rotation = false;
  bool has_accel_axis_rx_i = false;
  bool has_accel_axis_ry_i = false;
  bool has_accel_axis_rz_i = false;
  Mat3 accel_M = Mat3::Identity();
  Mat3 gyro_M = Mat3::Identity();
  Mat3 gyro_accel_sensitivity = Mat3::Zero();
  Mat3 gyro_sensing_rotation = Mat3::Identity();
  Vec3 accel_axis_rx_i = Vec3::Zero();
  Vec3 accel_axis_ry_i = Vec3::Zero();
  Vec3 accel_axis_rz_i = Vec3::Zero();
};

struct DatasetPaths {
  std::string camera_yaml;
  std::string imu_yaml;
  std::string target_yaml;
  std::string imu_data_csv;
  std::string corners_csv;
  std::string kalibr_result_txt;
};

inline bool hasPath(const std::string &path) { return !path.empty(); }

} // namespace ceres_cam_imu
