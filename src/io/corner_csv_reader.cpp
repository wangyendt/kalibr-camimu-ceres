#include "ceres_cam_imu/io/corner_csv_reader.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
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

std::vector<ImageObservation> readCornerCsv(const std::string& csv_path,
                                            const int max_frames) {
  std::ifstream input(csv_path);
  if (!input) {
    throw std::runtime_error("failed to open corner csv: " + csv_path);
  }

  std::vector<ImageObservation> observations;
  long long current_timestamp_ns = -1;
  std::string line;
  while (std::getline(input, line)) {
    const std::vector<std::string> fields = splitCsv(line);
    if (isHeaderLine(fields)) {
      continue;
    }
    if (fields.size() < 7) {
      throw std::runtime_error("corner csv row must have at least 7 columns");
    }

    const long long timestamp_ns = std::stoll(fields[0]);
    if (timestamp_ns != current_timestamp_ns) {
      if (max_frames > 0 && static_cast<int>(observations.size()) >= max_frames) {
        break;
      }
      current_timestamp_ns = timestamp_ns;
      ImageObservation obs;
      obs.timestamp_s = static_cast<double>(timestamp_ns) * 1e-9;
      observations.push_back(std::move(obs));
    }

    CornerMeasurement corner;
    corner.corner_id = std::stoi(fields[1]);
    corner.pixel = Vec2(std::stod(fields[2]), std::stod(fields[3]));
    corner.target_point =
        Vec3(std::stod(fields[4]), std::stod(fields[5]), std::stod(fields[6]));
    observations.back().corners.push_back(corner);
  }

  return observations;
}

}  // namespace ceres_cam_imu
