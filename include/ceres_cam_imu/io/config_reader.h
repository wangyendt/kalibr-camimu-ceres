#pragma once

#include <string>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

struct CamchainImuPrior {
  Mat4 T_cam_imu = Mat4::Identity();
  double timeshift_cam_imu_s = 0.0;
  bool has_T_cam_imu = false;
  bool has_timeshift_cam_imu = false;
};

CameraIntrinsics readCameraIntrinsics(const std::string& yaml_path);
CameraIntrinsics readCameraIntrinsics(const std::string& yaml_path,
                                      int camera_index);
ImuNoise readImuNoise(const std::string& yaml_path);
AprilGridConfig readAprilGridConfig(const std::string& yaml_path);
CamchainImuPrior readCamchainImuPrior(const std::string& yaml_path);
CamchainImuPrior readCamchainImuPrior(const std::string& yaml_path,
                                      int camera_index);
int readCameraCount(const std::string& yaml_path);

}  // namespace ceres_cam_imu
