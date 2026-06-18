#include "ceres_cam_imu/io/pose_csv_reader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ceres_cam_imu {
namespace {

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

bool isHeaderLine(const std::vector<std::string>& fields) {
  return fields.empty() || fields[0].empty() ||
         fields[0].find_first_not_of("0123456789") != std::string::npos;
}

}  // namespace

std::vector<PoseObservation> readPoseCsv(const std::string& csv_path) {
  std::ifstream input(csv_path);
  if (!input) {
    throw std::runtime_error("failed to open pose csv: " + csv_path);
  }

  std::vector<PoseObservation> poses;
  std::string line;
  while (std::getline(input, line)) {
    const std::vector<std::string> fields = splitCsv(line);
    if (isHeaderLine(fields)) {
      continue;
    }
    if (fields.size() < 17) {
      throw std::runtime_error("pose csv row must contain timestamp_ns plus 16 matrix values");
    }
    PoseObservation pose;
    pose.timestamp_s = static_cast<double>(std::stoll(fields[0])) * 1e-9;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        pose.T_t_c(r, c) = std::stod(fields[1 + r * 4 + c]);
      }
    }
    poses.push_back(pose);
  }
  return poses;
}

}  // namespace ceres_cam_imu
