#include "ceres_cam_imu/io/imu_csv_reader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ceres_cam_imu/processing/dataset_processing.h"

namespace ceres_cam_imu {
namespace {

std::vector<double> parseCsvNumbers(const std::string& line) {
  std::vector<double> values;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) {
      values.push_back(std::stod(item));
    }
  }
  return values;
}

}  // namespace

std::vector<ImuSample> readImuCsv(const std::string& csv_path,
                                  const int trim_edge_count) {
  std::ifstream input(csv_path);
  if (!input) {
    throw std::runtime_error("failed to open IMU csv: " + csv_path);
  }

  std::vector<ImuSample> samples;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const std::vector<double> values = parseCsvNumbers(line);
    if (values.size() < 7) {
      continue;
    }
    ImuSample sample;
    sample.timestamp_s = values[0] * 1e-9;
    sample.gyro_rad_s = Vec3(values[1], values[2], values[3]);
    sample.accel_m_s2 = Vec3(values[4], values[5], values[6]);
    samples.push_back(sample);
  }

  if (trim_edge_count > 0) {
    return trimImuSamplesKalibr(samples, trim_edge_count);
  }
  return samples;
}

}  // namespace ceres_cam_imu
