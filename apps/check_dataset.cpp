#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>

#include "ceres_cam_imu/io/config_reader.h"
#include "ceres_cam_imu/io/corner_csv_reader.h"
#include "ceres_cam_imu/io/imu_csv_reader.h"
#include "ceres_cam_imu/io/kalibr_result_parser.h"
#include "ceres_cam_imu/io/pose_csv_reader.h"
#include "ceres_cam_imu/processing/dataset_processing.h"

namespace {

std::string argValue(int argc, char **argv, const std::string &name,
                     const std::string &default_value = "") {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      return argv[i + 1];
    }
  }
  return default_value;
}

bool hasFlag(int argc, char **argv, const std::string &name) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == name) {
      return true;
    }
  }
  return false;
}

void usage() {
  std::cout << "check_dataset --cam camchain.yaml --imu imu.yaml --target "
               "aprilgrid.yaml "
               "--imu-data data.csv [--imu-trim-edge-count N] [--corners "
               "corners.csv] "
               "[--kalibr-result result.txt]\n";
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
  if (cam_yaml.empty() || imu_yaml.empty() || target_yaml.empty() ||
      imu_data.empty()) {
    usage();
    return 2;
  }

  const ceres_cam_imu::CameraIntrinsics cam =
      ceres_cam_imu::readCameraIntrinsics(cam_yaml);
  const ceres_cam_imu::ImuNoise imu = ceres_cam_imu::readImuNoise(imu_yaml);
  const ceres_cam_imu::AprilGridConfig target =
      ceres_cam_imu::readAprilGridConfig(target_yaml);
  const int imu_trim_edge_count = std::max(
      0, std::stoi(argValue(argc, argv, "--imu-trim-edge-count", "0")));
  const std::vector<ceres_cam_imu::ImuSample> raw_imu_samples =
      ceres_cam_imu::readImuCsv(imu_data);
  ceres_cam_imu::ImuTrimSummary imu_trim_summary;
  const std::vector<ceres_cam_imu::ImuSample> imu_samples =
      ceres_cam_imu::trimImuSamplesKalibr(raw_imu_samples, imu_trim_edge_count,
                                          &imu_trim_summary);

  std::cout << "camera: " << cam.width << "x" << cam.height
            << " model=" << cam.camera_model
            << " distortion=" << cam.distortion_model << " fx=" << cam.fx
            << " fy=" << cam.fy << " cx=" << cam.cx << " cy=" << cam.cy << "\n";
  std::cout << "imu: " << imu_samples.size()
            << " samples, update_rate=" << imu.update_rate_hz
            << "Hz gyro_sigma_discrete=" << imu.gyroDiscreteSigma()
            << " accel_sigma_discrete=" << imu.accelDiscreteSigma() << "\n";
  if (imu_trim_edge_count > 0) {
    std::cout << "imu trim: input=" << imu_trim_summary.input_samples
              << " output=" << imu_trim_summary.output_samples
              << " first_kept_index=" << imu_trim_summary.first_kept_index
              << " last_kept_index=" << imu_trim_summary.last_kept_index
              << " applied=" << (imu_trim_summary.applied ? "true" : "false")
              << " rule=kalibr_inclusive_upper_bound\n";
  }
  std::cout << "target: " << target.tag_rows << "x" << target.tag_cols
            << " tag_size=" << target.tag_size_m
            << " spacing_ratio=" << target.tag_spacing_ratio << "\n";
  if (!imu_samples.empty()) {
    std::cout << "imu span: " << imu_samples.front().timestamp_s << " -> "
              << imu_samples.back().timestamp_s << " s\n";
  }

  const std::string corners_csv = argValue(argc, argv, "--corners");
  if (!corners_csv.empty()) {
    const std::vector<ceres_cam_imu::ImageObservation> images =
        ceres_cam_imu::readCornerCsv(corners_csv);
    const std::size_t corner_count =
        ceres_cam_imu::countCornerMeasurements(images);
    std::cout << "corners: " << images.size() << " frames, " << corner_count
              << " corner observations\n";
  }

  const std::string kalibr_result = argValue(argc, argv, "--kalibr-result");
  if (!kalibr_result.empty()) {
    const ceres_cam_imu::KalibrResult result =
        ceres_cam_imu::readKalibrResult(kalibr_result);
    std::cout << "kalibr time shift: " << result.timeshift_cam_to_imu_s
              << " s\n";
    std::cout << "kalibr gravity: " << result.gravity.transpose() << "\n";
    std::cout << "kalibr T_ci:\n" << result.T_ci << "\n";
  }

  const std::string poses_csv = argValue(argc, argv, "--corner-poses");
  if (!poses_csv.empty()) {
    const std::vector<ceres_cam_imu::PoseObservation> poses =
        ceres_cam_imu::readPoseCsv(poses_csv);
    std::cout << "corner poses: " << poses.size() << " frames\n";
  }
  return 0;
}
