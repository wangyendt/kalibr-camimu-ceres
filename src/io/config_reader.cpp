#include "ceres_cam_imu/io/config_reader.h"

#include <algorithm>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "ceres_cam_imu/camera/camera_model.h"

namespace ceres_cam_imu {
namespace {

std::vector<std::string> readLines(const std::string &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open yaml file: " + path);
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    lines.push_back(line);
  }
  return lines;
}

bool isCameraHeaderLine(const std::string &line, int *camera_index) {
  static const std::regex header_re(R"(^cam([0-9]+)[[:space:]]*:[[:space:]]*$)");
  std::smatch match;
  if (!std::regex_match(line, match, header_re)) {
    return false;
  }
  if (camera_index) {
    *camera_index = std::stoi(match[1].str());
  }
  return true;
}

std::vector<std::string> cameraSectionLines(
    const std::vector<std::string> &lines, const int camera_index) {
  const std::string header = "cam" + std::to_string(camera_index) + ":";
  std::size_t begin = lines.size();
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (lines[i] == header) {
      begin = i + 1;
      break;
    }
  }
  if (begin == lines.size()) {
    if (camera_index == 0) {
      return lines;
    }
    throw std::runtime_error("missing camera section in yaml: cam" +
                             std::to_string(camera_index));
  }
  std::vector<std::string> section;
  for (std::size_t i = begin; i < lines.size(); ++i) {
    if (isCameraHeaderLine(lines[i], nullptr)) {
      break;
    }
    section.push_back(lines[i]);
  }
  return section;
}

std::vector<double> extractNumbers(const std::string &text) {
  static const std::regex number_re(
      R"([-+]?(?:(?:\d+\.\d*)|(?:\.\d+)|(?:\d+))(?:[eE][-+]?\d+)?)");
  std::vector<double> values;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), number_re);
       it != std::sregex_iterator(); ++it) {
    values.push_back(std::stod(it->str()));
  }
  return values;
}

std::string valueTextForKey(const std::vector<std::string> &lines,
                            const std::string &key) {
  const std::regex key_re("(^|[[:space:]])" + key + "[[:space:]]*:");
  for (std::size_t line_index = 0; line_index < lines.size(); ++line_index) {
    const std::string &line = lines[line_index];
    std::smatch match;
    if (!std::regex_search(line, match, key_re)) {
      continue;
    }
    const std::size_t start = static_cast<std::size_t>(match.position());
    const std::size_t colon = line.find(':', start);
    if (colon != std::string::npos) {
      std::string value = line.substr(colon + 1);
      if (value.find('[') != std::string::npos &&
          value.find(']') == std::string::npos) {
        for (std::size_t next = line_index + 1; next < lines.size(); ++next) {
          value += " " + lines[next];
          if (lines[next].find(']') != std::string::npos) {
            break;
          }
        }
      }
      return value;
    }
  }
  throw std::runtime_error("missing yaml key: " + key);
}

std::vector<double> numericSequence(const std::vector<std::string> &lines,
                                    const std::string &key,
                                    const std::size_t expected) {
  const std::vector<double> values =
      extractNumbers(valueTextForKey(lines, key));
  if (values.size() < expected) {
    throw std::runtime_error("yaml key has too few numeric values: " + key);
  }
  return values;
}

std::vector<double> numericSequenceAny(const std::vector<std::string> &lines,
                                       const std::string &key) {
  return extractNumbers(valueTextForKey(lines, key));
}

std::vector<double>
optionalNumericSequenceAny(const std::vector<std::string> &lines,
                           const std::string &key) {
  try {
    return numericSequenceAny(lines, key);
  } catch (const std::runtime_error &) {
    return {};
  }
}

double numericScalar(const std::vector<std::string> &lines,
                     const std::string &key) {
  const std::vector<double> values =
      extractNumbers(valueTextForKey(lines, key));
  if (values.empty()) {
    throw std::runtime_error("yaml key is not numeric: " + key);
  }
  return values.front();
}

bool tryNumericScalar(const std::vector<std::string> &lines,
                      const std::string &key, double *value) {
  try {
    *value = numericScalar(lines, key);
    return true;
  } catch (const std::runtime_error &) {
    return false;
  }
}

bool tryMatrix4AfterKey(const std::vector<std::string> &lines,
                        const std::string &key, Mat4 *matrix) {
  const std::regex key_re("(^|[[:space:]])" + key + "[[:space:]]*:");
  for (std::size_t line_index = 0; line_index < lines.size(); ++line_index) {
    std::smatch match;
    if (!std::regex_search(lines[line_index], match, key_re)) {
      continue;
    }
    if (line_index + 4 >= lines.size()) {
      throw std::runtime_error("yaml matrix has too few rows: " + key);
    }
    Mat4 parsed = Mat4::Identity();
    for (int row = 0; row < 4; ++row) {
      const std::vector<double> values = extractNumbers(
          lines.at(line_index + 1 + static_cast<std::size_t>(row)));
      if (values.size() < 4) {
        throw std::runtime_error("failed to parse yaml matrix row: " + key);
      }
      for (int col = 0; col < 4; ++col) {
        parsed(row, col) = values[static_cast<std::size_t>(col)];
      }
    }
    *matrix = parsed;
    return true;
  }
  return false;
}

std::string stringScalar(const std::vector<std::string> &lines,
                         const std::string &key,
                         const std::string &default_value) {
  try {
    const std::string value = valueTextForKey(lines, key);
    const std::size_t first = value.find_first_not_of(" \t'\"");
    const std::size_t last = value.find_last_not_of(" \t'\"");
    if (first == std::string::npos || last == std::string::npos ||
        last < first) {
      return default_value;
    }
    return value.substr(first, last - first + 1);
  } catch (const std::runtime_error &) {
    return default_value;
  }
}

} // namespace

CameraIntrinsics readCameraIntrinsics(const std::string &yaml_path,
                                      const int camera_index) {
  const std::vector<std::string> lines =
      cameraSectionLines(readLines(yaml_path), camera_index);

  CameraIntrinsics intrinsics;
  intrinsics.camera_model =
      canonicalCameraModelName(stringScalar(lines, "camera_model", "pinhole"));
  intrinsics.distortion_model = canonicalDistortionModelName(
      stringScalar(lines, "distortion_model", "radtan"));

  intrinsics.intrinsics = numericSequenceAny(lines, "intrinsics");
  const std::vector<double> &intr = intrinsics.intrinsics;
  if (intrinsics.camera_model == "pinhole") {
    if (intr.size() < 4) {
      throw std::runtime_error("pinhole intrinsics require 4 values");
    }
    intrinsics.fx = intr[0];
    intrinsics.fy = intr[1];
    intrinsics.cx = intr[2];
    intrinsics.cy = intr[3];
  } else if (intrinsics.camera_model == "omni") {
    if (intr.size() < 5) {
      throw std::runtime_error("omni intrinsics require 5 values");
    }
    intrinsics.xi = intr[0];
    intrinsics.fx = intr[1];
    intrinsics.fy = intr[2];
    intrinsics.cx = intr[3];
    intrinsics.cy = intr[4];
  } else if (intrinsics.camera_model == "eucm") {
    if (intr.size() < 6) {
      throw std::runtime_error("eucm intrinsics require 6 values");
    }
    intrinsics.alpha = intr[0];
    intrinsics.beta = intr[1];
    intrinsics.fx = intr[2];
    intrinsics.fy = intr[3];
    intrinsics.cx = intr[4];
    intrinsics.cy = intr[5];
  } else if (intrinsics.camera_model == "ds") {
    if (intr.size() < 6) {
      throw std::runtime_error("ds intrinsics require 6 values");
    }
    intrinsics.xi = intr[0];
    intrinsics.alpha = intr[1];
    intrinsics.fx = intr[2];
    intrinsics.fy = intr[3];
    intrinsics.cx = intr[4];
    intrinsics.cy = intr[5];
  } else {
    throw std::runtime_error("unsupported camera_model: " +
                             intrinsics.camera_model);
  }

  intrinsics.distortion_coeffs =
      optionalNumericSequenceAny(lines, "distortion_coeffs");
  const std::vector<double> &dist = intrinsics.distortion_coeffs;
  if (intrinsics.distortion_model == "radtan" ||
      intrinsics.distortion_model == "equidistant") {
    if (dist.size() < 4) {
      throw std::runtime_error(intrinsics.distortion_model +
                               " distortion_coeffs require 4 values");
    }
    intrinsics.k1 = dist[0];
    intrinsics.k2 = dist[1];
    intrinsics.p1 = dist[2];
    intrinsics.p2 = dist[3];
  } else if (intrinsics.distortion_model == "fov") {
    if (dist.empty()) {
      throw std::runtime_error("fov distortion_coeffs require 1 value");
    }
    intrinsics.k1 = dist[0];
  } else if (intrinsics.distortion_model != "none") {
    throw std::runtime_error("unsupported distortion_model: " +
                             intrinsics.distortion_model);
  }

  const std::vector<double> resolution =
      numericSequence(lines, "resolution", 2);
  intrinsics.width = static_cast<int>(resolution[0]);
  intrinsics.height = static_cast<int>(resolution[1]);
  (void)CameraModel(intrinsics);
  return intrinsics;
}

CameraIntrinsics readCameraIntrinsics(const std::string &yaml_path) {
  return readCameraIntrinsics(yaml_path, 0);
}

ImuNoise readImuNoise(const std::string &yaml_path) {
  const std::vector<std::string> lines = readLines(yaml_path);
  ImuNoise noise;
  noise.update_rate_hz = numericScalar(lines, "update_rate");
  noise.accelerometer_noise_density =
      numericScalar(lines, "accelerometer_noise_density");
  noise.accelerometer_random_walk =
      numericScalar(lines, "accelerometer_random_walk");
  noise.gyroscope_noise_density =
      numericScalar(lines, "gyroscope_noise_density");
  noise.gyroscope_random_walk = numericScalar(lines, "gyroscope_random_walk");
  return noise;
}

AprilGridConfig readAprilGridConfig(const std::string &yaml_path) {
  const std::vector<std::string> lines = readLines(yaml_path);
  AprilGridConfig config;
  config.tag_cols = static_cast<int>(numericScalar(lines, "tagCols"));
  config.tag_rows = static_cast<int>(numericScalar(lines, "tagRows"));
  config.tag_size_m = numericScalar(lines, "tagSize");
  config.tag_spacing_ratio = numericScalar(lines, "tagSpacing");
  return config;
}

CamchainImuPrior readCamchainImuPrior(const std::string &yaml_path) {
  return readCamchainImuPrior(yaml_path, 0);
}

CamchainImuPrior readCamchainImuPrior(const std::string &yaml_path,
                                      const int camera_index) {
  const std::vector<std::string> lines =
      cameraSectionLines(readLines(yaml_path), camera_index);
  CamchainImuPrior prior;
  prior.has_T_cam_imu =
      tryMatrix4AfterKey(lines, "T_cam_imu", &prior.T_cam_imu);
  prior.has_timeshift_cam_imu =
      tryNumericScalar(lines, "timeshift_cam_imu", &prior.timeshift_cam_imu_s);
  return prior;
}

int readCameraCount(const std::string &yaml_path) {
  const std::vector<std::string> lines = readLines(yaml_path);
  int max_index = -1;
  for (const std::string &line : lines) {
    int camera_index = -1;
    if (isCameraHeaderLine(line, &camera_index)) {
      max_index = std::max(max_index, camera_index);
    }
  }
  return max_index + 1;
}

} // namespace ceres_cam_imu
