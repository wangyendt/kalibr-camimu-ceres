#include "ceres_cam_imu/variables/imu_intrinsics.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace ceres_cam_imu {

Mat3 lowerTriangularMatrix(const double *block) {
  Mat3 matrix = Mat3::Zero();
  matrix(0, 0) = block[0];
  matrix(1, 0) = block[1];
  matrix(1, 1) = block[2];
  matrix(2, 0) = block[3];
  matrix(2, 1) = block[4];
  matrix(2, 2) = block[5];
  return matrix;
}

Mat3 matrix3Block(const double *block) {
  Mat3 matrix;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      matrix(r, c) = block[r * 3 + c];
    }
  }
  return matrix;
}

Vec3 vector3Block(const double *block) { return Eigen::Map<const Vec3>(block); }

const char *imuCalibrationModelName(const ImuCalibrationModel model) {
  switch (model) {
  case ImuCalibrationModel::kCalibrated:
    return "calibrated";
  case ImuCalibrationModel::kScaleMisalignment:
    return "scale-misalignment";
  case ImuCalibrationModel::kScaleMisalignmentSizeEffect:
    return "scale-misalignment-size-effect";
  }
  return "unknown";
}

ImuCalibrationModel parseImuCalibrationModel(const std::string &model_text) {
  std::string model = model_text;
  std::transform(model.begin(), model.end(), model.begin(),
                 [](const unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  if (model.empty() || model == "calibrated") {
    return ImuCalibrationModel::kCalibrated;
  }
  if (model == "scale-misalignment" || model == "scale_misalignment") {
    return ImuCalibrationModel::kScaleMisalignment;
  }
  if (model == "scale-misalignment-size-effect" ||
      model == "scale_misalignment_size_effect" || model == "size-effect" ||
      model == "size_effect") {
    return ImuCalibrationModel::kScaleMisalignmentSizeEffect;
  }
  throw std::invalid_argument("unknown IMU calibration model: " + model);
}

} // namespace ceres_cam_imu
