#pragma once

#include <ceres/cost_function.h>

namespace ceres_cam_imu {

ceres::CostFunction* createTimeShiftPrior(double prior_s, double sigma_s);

}  // namespace ceres_cam_imu
