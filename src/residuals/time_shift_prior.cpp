#include "ceres_cam_imu/residuals/time_shift_prior.h"

#include <algorithm>

#include <ceres/sized_cost_function.h>

namespace ceres_cam_imu {
namespace {

class TimeShiftPriorCost final : public ceres::SizedCostFunction<1, 1> {
 public:
  TimeShiftPriorCost(const double prior_s, const double sigma_s)
      : prior_s_(prior_s), sqrt_information_(1.0 / std::max(1e-12, sigma_s)) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    residuals[0] = sqrt_information_ * (parameters[0][0] - prior_s_);
    if (jacobians && jacobians[0]) {
      jacobians[0][0] = sqrt_information_;
    }
    return true;
  }

 private:
  double prior_s_ = 0.0;
  double sqrt_information_ = 1.0;
};

}  // namespace

ceres::CostFunction* createTimeShiftPrior(const double prior_s,
                                          const double sigma_s) {
  return new TimeShiftPriorCost(prior_s, sigma_s);
}

}  // namespace ceres_cam_imu
