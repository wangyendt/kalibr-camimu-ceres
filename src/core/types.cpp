#include "ceres_cam_imu/core/types.h"

#include <cmath>

namespace ceres_cam_imu {

double ImuNoise::accelDiscreteSigma() const {
  return accelerometer_noise_density * std::sqrt(update_rate_hz);
}

double ImuNoise::gyroDiscreteSigma() const {
  return gyroscope_noise_density * std::sqrt(update_rate_hz);
}

}  // namespace ceres_cam_imu
