#include "ceres_cam_imu/io/kalibr_result_parser.h"

#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace ceres_cam_imu {
namespace {

std::vector<double> extractDoubles(const std::string& line) {
  static const std::regex number_re(
      R"([-+]?(?:(?:\d+\.\d*)|(?:\.\d+)|(?:\d+))(?:[eE][-+]?\d+)?)");
  std::vector<double> values;
  for (auto it = std::sregex_iterator(line.begin(), line.end(), number_re);
       it != std::sregex_iterator(); ++it) {
    values.push_back(std::stod(it->str()));
  }
  return values;
}

Mat4 readMatrix4(const std::vector<std::string>& lines, std::size_t first_row) {
  Mat4 matrix = Mat4::Identity();
  for (int r = 0; r < 4; ++r) {
    const std::vector<double> values = extractDoubles(lines.at(first_row + r));
    if (values.size() < 4) {
      throw std::runtime_error("failed to parse 4x4 matrix row from Kalibr result");
    }
    for (int c = 0; c < 4; ++c) {
      matrix(r, c) = values[static_cast<std::size_t>(c)];
    }
  }
  return matrix;
}

Mat3 readMatrix3(const std::vector<std::string>& lines, std::size_t first_row) {
  Mat3 matrix = Mat3::Identity();
  for (int r = 0; r < 3; ++r) {
    const std::vector<double> values = extractDoubles(lines.at(first_row + r));
    if (values.size() < 3) {
      throw std::runtime_error("failed to parse 3x3 matrix row from Kalibr result");
    }
    for (int c = 0; c < 3; ++c) {
      matrix(r, c) = values[static_cast<std::size_t>(c)];
    }
  }
  return matrix;
}

bool tryReadVector3(const std::vector<std::string>& lines,
                    std::size_t value_row, Vec3* value) {
  if (!value || value_row >= lines.size()) {
    return false;
  }
  const std::vector<double> values = extractDoubles(lines.at(value_row));
  if (values.size() < 3) {
    return false;
  }
  *value = Vec3(values[0], values[1], values[2]);
  return true;
}

double readMeanAfterToken(const std::string& line) {
  const std::size_t pos = line.find("mean");
  if (pos == std::string::npos) {
    return 0.0;
  }
  const std::vector<double> values = extractDoubles(line.substr(pos));
  return values.empty() ? 0.0 : values.front();
}

int cameraIndexFromLine(const std::string& line) {
  static const std::regex camera_re(R"(cam([0-9]+))");
  std::smatch match;
  if (!std::regex_search(line, match, camera_re) || match.size() < 2) {
    return -1;
  }
  return std::stoi(match[1].str());
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

KalibrResult readKalibrResult(const std::string& result_txt_path) {
  std::ifstream input(result_txt_path);
  if (!input) {
    throw std::runtime_error("failed to open Kalibr result txt: " + result_txt_path);
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }

  KalibrResult result;
  bool in_gyro_section = false;
  bool in_accel_section = false;
  int active_camera_index = -1;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    const std::string& l = lines[i];
    if (l.find("Reprojection error (cam") != std::string::npos &&
        l.find("[px]") == std::string::npos) {
      const int camera_index = cameraIndexFromLine(l);
      const double mean = readMeanAfterToken(l);
      ensureVectorSize(&result.camera_reprojection_normalized_mean,
                       camera_index, 0.0);
      if (camera_index >= 0) {
        result.camera_reprojection_normalized_mean
            [static_cast<std::size_t>(camera_index)] = mean;
      }
      if (camera_index == 0) {
        result.residuals.reprojection_normalized_mean = mean;
      }
    } else if (l.find("Gyroscope error (imu0):") != std::string::npos) {
      result.residuals.gyro_normalized_mean = readMeanAfterToken(l);
    } else if (l.find("Accelerometer error (imu0):") != std::string::npos) {
      result.residuals.accel_normalized_mean = readMeanAfterToken(l);
    } else if (l.find("Reprojection error (cam") != std::string::npos &&
               l.find("[px]") != std::string::npos) {
      const int camera_index = cameraIndexFromLine(l);
      const double mean = readMeanAfterToken(l);
      ensureVectorSize(&result.camera_reprojection_mean_px, camera_index, 0.0);
      if (camera_index >= 0) {
        result.camera_reprojection_mean_px
            [static_cast<std::size_t>(camera_index)] = mean;
      }
      if (camera_index == 0) {
        result.residuals.reprojection_mean_px = mean;
      }
    } else if (l.find("Gyroscope error (imu0) [rad/s]") != std::string::npos) {
      result.residuals.gyro_mean_rad_s = readMeanAfterToken(l);
    } else if (l.find("Accelerometer error (imu0) [m/s^2]") != std::string::npos) {
      result.residuals.accel_mean_m_s2 = readMeanAfterToken(l);
    } else if (l.find("Transformation (cam") != std::string::npos) {
      active_camera_index = cameraIndexFromLine(l);
    } else if (l.find("T_ci:") != std::string::npos && i + 4 < lines.size()) {
      const Mat4 T_ci = readMatrix4(lines, i + 1);
      ensureVectorSize(&result.camera_T_ci, active_camera_index,
                       Mat4(Mat4::Identity()));
      if (active_camera_index >= 0) {
        result.camera_T_ci[static_cast<std::size_t>(active_camera_index)] =
            T_ci;
      }
      if (active_camera_index <= 0) {
        result.T_ci = T_ci;
      }
    } else if (l.find("T_ic:") != std::string::npos && i + 4 < lines.size()) {
      const Mat4 T_ic = readMatrix4(lines, i + 1);
      ensureVectorSize(&result.camera_T_ic, active_camera_index,
                       Mat4(Mat4::Identity()));
      if (active_camera_index >= 0) {
        result.camera_T_ic[static_cast<std::size_t>(active_camera_index)] =
            T_ic;
      }
      if (active_camera_index <= 0) {
        result.T_ic = T_ic;
      }
    } else if (l.find("timeshift cam") != std::string::npos &&
               i + 1 < lines.size()) {
      const std::vector<double> values = extractDoubles(lines[i + 1]);
      if (!values.empty()) {
        const int camera_index = cameraIndexFromLine(l);
        ensureVectorSize(&result.camera_timeshift_cam_to_imu_s,
                         camera_index, 0.0);
        if (camera_index >= 0) {
          result.camera_timeshift_cam_to_imu_s
              [static_cast<std::size_t>(camera_index)] = values.front();
        }
        if (camera_index == 0) {
          result.timeshift_cam_to_imu_s = values.front();
        }
      }
    } else if (l.find("Gravity vector") != std::string::npos &&
               i + 1 < lines.size()) {
      const std::vector<double> values = extractDoubles(lines[i + 1]);
      if (values.size() >= 3) {
        result.gravity = Vec3(values[0], values[1], values[2]);
      }
    } else if (l.find("Gyroscope:") != std::string::npos) {
      in_gyro_section = true;
      in_accel_section = false;
    } else if (l.find("Accelerometer:") != std::string::npos) {
      in_gyro_section = false;
      in_accel_section = true;
    } else if (l.find("M:") != std::string::npos && i + 3 < lines.size()) {
      if (in_gyro_section) {
        result.gyro_M = readMatrix3(lines, i + 1);
        result.has_gyro_M = true;
      } else if (in_accel_section) {
        result.accel_M = readMatrix3(lines, i + 1);
        result.has_accel_M = true;
      }
    } else if (l.find("A [(rad/s)/(m/s^2)]") != std::string::npos &&
               i + 3 < lines.size()) {
      result.gyro_accel_sensitivity = readMatrix3(lines, i + 1);
      result.has_gyro_accel_sensitivity = true;
    } else if (l.find("C_gyro_i:") != std::string::npos &&
               i + 3 < lines.size()) {
      result.gyro_sensing_rotation = readMatrix3(lines, i + 1);
      result.has_gyro_sensing_rotation = true;
    } else if (l.find("rx_i [m]:") != std::string::npos &&
               tryReadVector3(lines, i + 1, &result.accel_axis_rx_i)) {
      result.has_accel_axis_rx_i = true;
    } else if (l.find("ry_i [m]:") != std::string::npos &&
               tryReadVector3(lines, i + 1, &result.accel_axis_ry_i)) {
      result.has_accel_axis_ry_i = true;
    } else if (l.find("rz_i [m]:") != std::string::npos &&
               tryReadVector3(lines, i + 1, &result.accel_axis_rz_i)) {
      result.has_accel_axis_rz_i = true;
    }
  }

  return result;
}

}  // namespace ceres_cam_imu
