#include "ceres_cam_imu/io/calibration_result_reader.h"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ceres_cam_imu {
namespace {

std::string trimAscii(const std::string& value) {
  std::size_t begin = 0;
  while (begin < value.size() &&
         std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

bool startsWith(const std::string& value, const char* prefix) {
  return value.rfind(prefix, 0) == 0;
}

std::string valueAfterColon(const std::string& line) {
  const std::size_t pos = line.find(':');
  if (pos == std::string::npos) {
    return "";
  }
  return trimAscii(line.substr(pos + 1));
}

std::vector<double> parseDoubles(const std::string& text) {
  std::vector<double> values;
  const char* cursor = text.c_str();
  while (*cursor != '\0') {
    if (std::isdigit(static_cast<unsigned char>(*cursor)) || *cursor == '-' ||
        *cursor == '+' || *cursor == '.') {
      char* end = nullptr;
      const double value = std::strtod(cursor, &end);
      if (end != cursor) {
        values.push_back(value);
        cursor = end;
        continue;
      }
    }
    ++cursor;
  }
  return values;
}

double parseScalarAfterColon(const std::string& line) {
  const std::vector<double> values = parseDoubles(valueAfterColon(line));
  if (values.empty()) {
    throw std::runtime_error("expected scalar value in calibration result line: "
                             + line);
  }
  return values.front();
}

Mat4 parseMatrix4AfterColon(const std::string& line) {
  const std::vector<double> values = parseDoubles(valueAfterColon(line));
  if (values.size() != 16) {
    throw std::runtime_error("expected 16 matrix values in calibration result");
  }
  Mat4 matrix = Mat4::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      matrix(r, c) = values[static_cast<std::size_t>(4 * r + c)];
    }
  }
  return matrix;
}

Mat3 parseMatrix3AfterColon(const std::string& line) {
  const std::vector<double> values = parseDoubles(valueAfterColon(line));
  if (values.size() != 9) {
    throw std::runtime_error("expected 9 matrix values in calibration result");
  }
  Mat3 matrix = Mat3::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      matrix(r, c) = values[static_cast<std::size_t>(3 * r + c)];
    }
  }
  return matrix;
}

Vec3 parseVector3AfterColon(const std::string& line) {
  const std::vector<double> values = parseDoubles(valueAfterColon(line));
  if (values.size() != 3) {
    throw std::runtime_error("expected 3 vector values in calibration result");
  }
  return Vec3(values[0], values[1], values[2]);
}

void assignResidualMean(const std::string& key, const double mean,
                        KalibrResidualStats* stats) {
  if (!stats) {
    return;
  }
  if (key == "reprojection_px") {
    stats->reprojection_mean_px = mean;
  } else if (key == "reprojection_normalized") {
    stats->reprojection_normalized_mean = mean;
  } else if (key == "gyro_rad_s") {
    stats->gyro_mean_rad_s = mean;
  } else if (key == "gyro_normalized") {
    stats->gyro_normalized_mean = mean;
  } else if (key == "accel_m_s2") {
    stats->accel_mean_m_s2 = mean;
  } else if (key == "accel_normalized") {
    stats->accel_normalized_mean = mean;
  }
}

template <typename T>
void ensureVectorSize(std::vector<T>* values, const int index,
                      const T& fill_value) {
  if (!values || index < 0) {
    return;
  }
  const std::size_t required = static_cast<std::size_t>(index) + 1;
  if (values->size() < required) {
    values->resize(required, fill_value);
  }
}

}  // namespace

CalibrationResultFile readCalibrationResultYaml(
    const std::string& result_yaml_path) {
  std::ifstream input(result_yaml_path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open calibration result yaml: " +
                             result_yaml_path);
  }

  CalibrationResultFile result;
  bool have_T_c_b = false;
  bool have_time_shift = false;
  bool have_gravity = false;
  bool in_residual_statistics = false;
  bool in_kalibr_delta = false;
  bool in_camera_chain = false;
  int camera_chain_index = -1;
  std::string residual_key;

  std::string line;
  while (std::getline(input, line)) {
    const std::string trimmed = trimAscii(line);
    if (trimmed.empty()) {
      continue;
    }

    if (startsWith(trimmed, "format_version:")) {
      result.format_version = static_cast<int>(parseScalarAfterColon(trimmed));
      continue;
    }
    if (trimmed == "camera_chain:") {
      in_camera_chain = true;
      in_residual_statistics = false;
      in_kalibr_delta = false;
      camera_chain_index = -1;
      continue;
    }
    if (in_camera_chain) {
      if (!line.empty() && line[0] != ' ') {
        in_camera_chain = false;
        camera_chain_index = -1;
      } else {
        if (startsWith(trimmed, "- camera_index:")) {
          camera_chain_index =
              static_cast<int>(parseScalarAfterColon(trimmed));
          ensureVectorSize(&result.camera_T_c_b, camera_chain_index,
                           Mat4(Mat4::Identity()));
          ensureVectorSize(&result.camera_time_shift_s, camera_chain_index,
                           0.0);
        } else if (startsWith(trimmed, "T_c_b:") &&
                   camera_chain_index >= 0) {
          result.camera_T_c_b[static_cast<std::size_t>(camera_chain_index)] =
              parseMatrix4AfterColon(trimmed);
        } else if (startsWith(trimmed, "time_shift_s:") &&
                   camera_chain_index >= 0) {
          result.camera_time_shift_s
              [static_cast<std::size_t>(camera_chain_index)] =
                  parseScalarAfterColon(trimmed);
        }
        continue;
      }
    }
    if (startsWith(line, "  T_c_b:")) {
      result.T_c_b = parseMatrix4AfterColon(trimmed);
      have_T_c_b = true;
      continue;
    }
    if (startsWith(line, "  T_b_c:")) {
      result.T_b_c = parseMatrix4AfterColon(trimmed);
      continue;
    }
    if (startsWith(trimmed, "time_shift_s:") &&
        (line.empty() || line[0] != ' ')) {
      result.time_shift_s = parseScalarAfterColon(trimmed);
      have_time_shift = true;
      continue;
    }
    if (startsWith(trimmed, "gravity:")) {
      result.gravity = parseVector3AfterColon(trimmed);
      have_gravity = true;
      continue;
    }
    if (startsWith(trimmed, "accel_M:")) {
      result.accel_M = parseMatrix3AfterColon(trimmed);
      result.has_accel_M = true;
      continue;
    }
    if (startsWith(trimmed, "gyro_M:")) {
      result.gyro_M = parseMatrix3AfterColon(trimmed);
      result.has_gyro_M = true;
      continue;
    }
    if (startsWith(trimmed, "gyro_accel_sensitivity:")) {
      result.gyro_accel_sensitivity = parseMatrix3AfterColon(trimmed);
      result.has_gyro_accel_sensitivity = true;
      continue;
    }
    if (startsWith(trimmed, "gyro_sensing_rotation:")) {
      result.gyro_sensing_rotation = parseMatrix3AfterColon(trimmed);
      result.has_gyro_sensing_rotation = true;
      continue;
    }
    if (startsWith(trimmed, "accel_axis_rx_i:")) {
      result.accel_axis_rx_i = parseVector3AfterColon(trimmed);
      result.has_accel_axis_rx_i = true;
      continue;
    }
    if (startsWith(trimmed, "accel_axis_ry_i:")) {
      result.accel_axis_ry_i = parseVector3AfterColon(trimmed);
      result.has_accel_axis_ry_i = true;
      continue;
    }
    if (startsWith(trimmed, "accel_axis_rz_i:")) {
      result.accel_axis_rz_i = parseVector3AfterColon(trimmed);
      result.has_accel_axis_rz_i = true;
      continue;
    }
    if (trimmed == "residual_statistics:") {
      in_residual_statistics = true;
      in_kalibr_delta = false;
      in_camera_chain = false;
      residual_key.clear();
      continue;
    }
    if (trimmed == "kalibr_delta:") {
      in_residual_statistics = false;
      in_kalibr_delta = true;
      in_camera_chain = false;
      result.has_kalibr_delta = true;
      continue;
    }
    if (!line.empty() && line[0] != ' ') {
      in_residual_statistics = false;
      in_kalibr_delta = false;
      in_camera_chain = false;
      residual_key.clear();
    }

    if (in_residual_statistics) {
      if (startsWith(line, "  ") && !startsWith(line, "    ") &&
          trimmed.back() == ':') {
        residual_key = trimmed.substr(0, trimmed.size() - 1);
      } else if (!residual_key.empty() && startsWith(trimmed, "mean:")) {
        assignResidualMean(residual_key, parseScalarAfterColon(trimmed),
                           &result.residuals);
      }
      continue;
    }

    if (in_kalibr_delta) {
      if (startsWith(trimmed, "rotation_deg:")) {
        result.kalibr_delta.rotation_deg = parseScalarAfterColon(trimmed);
      } else if (startsWith(trimmed, "translation_m:")) {
        result.kalibr_delta.translation_m = parseScalarAfterColon(trimmed);
      } else if (startsWith(trimmed, "time_shift_s:")) {
        result.kalibr_delta.time_shift_s = parseScalarAfterColon(trimmed);
      } else if (startsWith(trimmed, "gravity_norm:")) {
        result.kalibr_delta.gravity_norm = parseScalarAfterColon(trimmed);
      }
    }
  }

  if (result.format_version <= 0) {
    throw std::runtime_error("calibration result yaml missing format_version");
  }
  if (!have_T_c_b || !have_time_shift || !have_gravity) {
    throw std::runtime_error(
        "calibration result yaml missing T_c_b, time_shift_s, or gravity");
  }
  return result;
}

}  // namespace ceres_cam_imu
