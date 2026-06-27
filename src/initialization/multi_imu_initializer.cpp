#include "ceres_cam_imu/initialization/multi_imu_initializer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <ceres/problem.h>
#include <ceres/sized_cost_function.h>
#include <ceres/solver.h>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/so3_jacobians.h"

namespace ceres_cam_imu {
namespace {

void writeMat3RowMajor(const Mat3& matrix, double* jacobian) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      jacobian[row * 3 + col] = matrix(row, col);
    }
  }
}

class RelativeGyroRotationCost final
    : public ceres::SizedCostFunction<3, 3, 3> {
 public:
  RelativeGyroRotationCost(Vec3 omega_reference, Vec3 omega_target,
                           const double weight = 1.0)
      : omega_reference_(std::move(omega_reference)),
        omega_target_(std::move(omega_target)),
        weight_(weight) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const Eigen::Map<const Vec3> r_i_b(parameters[0]);
    const Eigen::Map<const Vec3> bias(parameters[1]);
    const Mat3 R_i_b = rotationVectorToMatrix(r_i_b);
    const Vec3 residual =
        weight_ * (R_i_b * omega_reference_ + bias - omega_target_);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (!jacobians) {
      return true;
    }
    if (jacobians[0]) {
      const Mat3 J =
          weight_ * R_i_b * skew(omega_reference_) *
          leftJacobianSO3(r_i_b);
      writeMat3RowMajor(J, jacobians[0]);
    }
    if (jacobians[1]) {
      writeMat3RowMajor(weight_ * Mat3::Identity(), jacobians[1]);
    }
    return true;
  }

 private:
  Vec3 omega_reference_;
  Vec3 omega_target_;
  double weight_;
};

class RelativeAccelLeverCost final
    : public ceres::SizedCostFunction<3, 3, 3, 3> {
 public:
  RelativeAccelLeverCost(Vec3 omega_reference, Vec3 alpha_reference,
                         Vec3 accel_reference, Vec3 accel_target,
                         const double weight)
      : omega_reference_(std::move(omega_reference)),
        alpha_reference_(std::move(alpha_reference)),
        accel_reference_(std::move(accel_reference)),
        accel_target_(std::move(accel_target)),
        weight_(weight) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const Eigen::Map<const Vec3> r_i_b(parameters[0]);
    const Eigen::Map<const Vec3> lever_b(parameters[1]);
    const Eigen::Map<const Vec3> accel_bias_delta_b(parameters[2]);
    const Mat3 R_i_b = rotationVectorToMatrix(r_i_b);
    const Mat3 lever_jacobian =
        skew(alpha_reference_) + skew(omega_reference_) *
                                     skew(omega_reference_);
    const Vec3 accel_delta_b =
        R_i_b.transpose() * accel_target_ - accel_reference_;
    const Vec3 residual =
        weight_ * (lever_jacobian * lever_b + accel_bias_delta_b -
                   accel_delta_b);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (!jacobians) {
      return true;
    }
    if (jacobians[0]) {
      const Mat3 J =
          -weight_ *
          rotationTransposeTimesVectorDerivative(r_i_b, accel_target_);
      writeMat3RowMajor(J, jacobians[0]);
    }
    if (jacobians[1]) {
      writeMat3RowMajor(weight_ * lever_jacobian, jacobians[1]);
    }
    if (jacobians[2]) {
      writeMat3RowMajor(weight_ * Mat3::Identity(), jacobians[2]);
    }
    return true;
  }

 private:
  Vec3 omega_reference_;
  Vec3 alpha_reference_;
  Vec3 accel_reference_;
  Vec3 accel_target_;
  double weight_;
};

class VectorPriorCost final : public ceres::SizedCostFunction<3, 3> {
 public:
  VectorPriorCost(Vec3 prior, const double sigma)
      : prior_(std::move(prior)), inv_sigma_(1.0 / sigma) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const Eigen::Map<const Vec3> value(parameters[0]);
    const Vec3 residual = inv_sigma_ * (value - prior_);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();
    if (jacobians && jacobians[0]) {
      writeMat3RowMajor(inv_sigma_ * Mat3::Identity(), jacobians[0]);
    }
    return true;
  }

 private:
  Vec3 prior_;
  double inv_sigma_;
};

double meanPositiveDt(const std::vector<ImuSample>& samples) {
  if (samples.size() < 2) {
    throw std::runtime_error(
        "at least two IMU samples are required for IMU chain initialization");
  }
  double total_dt = 0.0;
  int count = 0;
  for (std::size_t i = 1; i < samples.size(); ++i) {
    const double dt = samples[i].timestamp_s - samples[i - 1].timestamp_s;
    if (dt > 0.0) {
      total_dt += dt;
      ++count;
    }
  }
  if (count == 0) {
    throw std::runtime_error("IMU timestamps are not increasing");
  }
  return total_dt / static_cast<double>(count);
}

bool interpolateGyro(const std::vector<ImuSample>& samples,
                     const double timestamp_s, Vec3* gyro) {
  if (!gyro || samples.size() < 2) {
    return false;
  }
  if (timestamp_s < samples.front().timestamp_s ||
      timestamp_s > samples.back().timestamp_s) {
    return false;
  }
  const auto upper = std::lower_bound(
      samples.begin(), samples.end(), timestamp_s,
      [](const ImuSample& sample, const double t) {
        return sample.timestamp_s < t;
      });
  if (upper == samples.begin()) {
    *gyro = upper->gyro_rad_s;
    return true;
  }
  if (upper == samples.end()) {
    *gyro = samples.back().gyro_rad_s;
    return true;
  }
  const ImuSample& right = *upper;
  const ImuSample& left = *(upper - 1);
  const double dt = right.timestamp_s - left.timestamp_s;
  if (!(dt > 0.0)) {
    *gyro = left.gyro_rad_s;
    return true;
  }
  const double u = (timestamp_s - left.timestamp_s) / dt;
  *gyro = (1.0 - u) * left.gyro_rad_s + u * right.gyro_rad_s;
  return true;
}

bool interpolateImuSample(const std::vector<ImuSample>& samples,
                          const double timestamp_s, ImuSample* interpolated,
                          Vec3* angular_accel_rad_s2) {
  if (!interpolated || samples.size() < 2) {
    return false;
  }
  if (timestamp_s < samples.front().timestamp_s ||
      timestamp_s > samples.back().timestamp_s) {
    return false;
  }
  const auto upper = std::lower_bound(
      samples.begin(), samples.end(), timestamp_s,
      [](const ImuSample& sample, const double t) {
        return sample.timestamp_s < t;
      });
  if (upper == samples.begin()) {
    *interpolated = *upper;
    if (angular_accel_rad_s2) {
      const double dt = (upper + 1)->timestamp_s - upper->timestamp_s;
      if (!(dt > 0.0)) {
        return false;
      }
      *angular_accel_rad_s2 = ((upper + 1)->gyro_rad_s - upper->gyro_rad_s) / dt;
    }
    return true;
  }
  if (upper == samples.end()) {
    *interpolated = samples.back();
    if (angular_accel_rad_s2) {
      const ImuSample& right = samples.back();
      const ImuSample& left = *(samples.end() - 2);
      const double dt = right.timestamp_s - left.timestamp_s;
      if (!(dt > 0.0)) {
        return false;
      }
      *angular_accel_rad_s2 = (right.gyro_rad_s - left.gyro_rad_s) / dt;
    }
    return true;
  }
  const ImuSample& right = *upper;
  const ImuSample& left = *(upper - 1);
  const double dt = right.timestamp_s - left.timestamp_s;
  if (!(dt > 0.0)) {
    return false;
  }
  const double u = (timestamp_s - left.timestamp_s) / dt;
  interpolated->timestamp_s = timestamp_s;
  interpolated->gyro_rad_s = (1.0 - u) * left.gyro_rad_s + u * right.gyro_rad_s;
  interpolated->accel_m_s2 =
      (1.0 - u) * left.accel_m_s2 + u * right.accel_m_s2;
  if (angular_accel_rad_s2) {
    *angular_accel_rad_s2 = (right.gyro_rad_s - left.gyro_rad_s) / dt;
  }
  return true;
}

struct CorrelationResult {
  double time_offset_s = 0.0;
  int discrete_shift_samples = 0;
  double sample_dt_s = 0.0;
  double search_radius_s = 0.0;
  int max_lag_samples = 0;
  int matched_samples = 0;
  double peak_correlation = 0.0;
  bool boundary_peak_rejected = false;
  int rejected_discrete_shift_samples = 0;
  int rejected_matched_samples = 0;
  double rejected_peak_correlation = 0.0;
};

struct LagCorrelationStats {
  bool valid = false;
  int matched_samples = 0;
  double correlation = -std::numeric_limits<double>::infinity();
};

double rawOverlapDurationSeconds(const ImuObservationDataset& reference_imu,
                                 const ImuObservationDataset& target_imu) {
  const double overlap_start =
      std::max(reference_imu.samples.front().timestamp_s,
               target_imu.samples.front().timestamp_s);
  const double overlap_end =
      std::min(reference_imu.samples.back().timestamp_s,
               target_imu.samples.back().timestamp_s);
  if (!(overlap_end > overlap_start)) {
    throw std::runtime_error(
        "IMU time ranges do not overlap for full-offset correlation");
  }
  return overlap_end - overlap_start;
}

double correlationSearchRadiusSeconds(
    const ImuObservationDataset& reference_imu,
    const ImuObservationDataset& target_imu,
    const ImuChainInitializerOptions& options) {
  return options.use_full_overlap_time_offset_search
             ? rawOverlapDurationSeconds(reference_imu, target_imu)
             : options.max_time_offset_search_s;
}

int maxCorrelationLagSamples(const double search_radius_s, const double dt) {
  return std::max(0, static_cast<int>(std::round(search_radius_s / dt)));
}

LagCorrelationStats gyroNormCorrelationAtLag(
    const ImuObservationDataset& reference_imu,
    const ImuObservationDataset& target_imu, const int lag,
    const double dt, const int stride, const int min_samples) {
  const double offset_s = static_cast<double>(lag) * dt;
  double sum_reference = 0.0;
  double sum_target = 0.0;
  double sum_reference_sq = 0.0;
  double sum_target_sq = 0.0;
  double sum_cross = 0.0;
  int matched = 0;
  for (std::size_t i = 0; i < target_imu.samples.size();
       i += static_cast<std::size_t>(stride)) {
    const ImuSample& target = target_imu.samples[i];
    Vec3 reference_gyro = Vec3::Zero();
    if (!interpolateGyro(reference_imu.samples,
                         target.timestamp_s + offset_s, &reference_gyro)) {
      continue;
    }
    const double reference_norm = reference_gyro.norm();
    const double target_norm = target.gyro_rad_s.norm();
    sum_reference += reference_norm;
    sum_target += target_norm;
    sum_reference_sq += reference_norm * reference_norm;
    sum_target_sq += target_norm * target_norm;
    sum_cross += reference_norm * target_norm;
    ++matched;
  }
  LagCorrelationStats stats;
  stats.matched_samples = matched;
  if (matched < min_samples) {
    return stats;
  }
  const double inv_count = 1.0 / static_cast<double>(matched);
  const double covariance =
      sum_cross - sum_reference * sum_target * inv_count;
  const double reference_variance =
      sum_reference_sq - sum_reference * sum_reference * inv_count;
  const double target_variance =
      sum_target_sq - sum_target * sum_target * inv_count;
  if (reference_variance <= 0.0 || target_variance <= 0.0) {
    return stats;
  }
  stats.valid = true;
  stats.correlation =
      covariance / std::sqrt(reference_variance * target_variance);
  return stats;
}

CorrelationResult estimateTimeOffsetByGyroNorm(
    const ImuObservationDataset& reference_imu,
    const ImuObservationDataset& target_imu,
    const ImuChainInitializerOptions& options) {
  const double dt = meanPositiveDt(target_imu.samples);
  const double search_radius_s =
      correlationSearchRadiusSeconds(reference_imu, target_imu, options);
  const int max_lag_samples =
      maxCorrelationLagSamples(search_radius_s, dt);
  const int stride = std::max(1, options.sample_stride);

  CorrelationResult best;
  best.sample_dt_s = dt;
  best.search_radius_s = search_radius_s;
  best.max_lag_samples = max_lag_samples;
  best.peak_correlation = -std::numeric_limits<double>::infinity();
  for (int lag = -max_lag_samples; lag <= max_lag_samples; ++lag) {
    const LagCorrelationStats stats = gyroNormCorrelationAtLag(
        reference_imu, target_imu, lag, dt, stride, options.min_samples);
    if (!stats.valid) {
      continue;
    }
    if (stats.correlation > best.peak_correlation ||
        (stats.correlation == best.peak_correlation &&
         std::abs(lag) < std::abs(best.discrete_shift_samples))) {
      best.time_offset_s = static_cast<double>(lag) * dt;
      best.discrete_shift_samples = lag;
      best.matched_samples = stats.matched_samples;
      best.peak_correlation = stats.correlation;
    }
  }
  if (!std::isfinite(best.peak_correlation)) {
    throw std::runtime_error(
        "not enough overlapping IMU samples for gyro-norm correlation");
  }
  if (max_lag_samples > 0 &&
      std::abs(best.discrete_shift_samples) == max_lag_samples) {
    const LagCorrelationStats zero_lag = gyroNormCorrelationAtLag(
        reference_imu, target_imu, 0, dt, stride, options.min_samples);
    if (!zero_lag.valid) {
      throw std::runtime_error(
          "not enough zero-lag IMU samples for boundary-peak rejection");
    }
    best.boundary_peak_rejected = true;
    best.rejected_discrete_shift_samples = best.discrete_shift_samples;
    best.rejected_matched_samples = best.matched_samples;
    best.rejected_peak_correlation = best.peak_correlation;
    best.time_offset_s = 0.0;
    best.discrete_shift_samples = 0;
    best.matched_samples = zero_lag.matched_samples;
    best.peak_correlation = zero_lag.correlation;
  }
  return best;
}

void collectGyroPairs(const ImuObservationDataset& reference_imu,
                      const ImuObservationDataset& target_imu,
                      const double time_offset_s, const int stride,
                      std::vector<Vec3>* reference_gyro,
                      std::vector<Vec3>* target_gyro) {
  if (!reference_gyro || !target_gyro) {
    throw std::invalid_argument("gyro pair outputs must be non-null");
  }
  reference_gyro->clear();
  target_gyro->clear();
  const int safe_stride = std::max(1, stride);
  reference_gyro->reserve(target_imu.samples.size() /
                          static_cast<std::size_t>(safe_stride) + 1);
  target_gyro->reserve(reference_gyro->capacity());
  for (std::size_t i = 0; i < target_imu.samples.size();
       i += static_cast<std::size_t>(safe_stride)) {
    const ImuSample& target = target_imu.samples[i];
    Vec3 reference = Vec3::Zero();
    if (!interpolateGyro(reference_imu.samples,
                         target.timestamp_s + time_offset_s, &reference)) {
      continue;
    }
    reference_gyro->push_back(reference);
    target_gyro->push_back(target.gyro_rad_s);
  }
}

struct ImuPairSample {
  Vec3 omega_reference = Vec3::Zero();
  Vec3 omega_target = Vec3::Zero();
  Vec3 alpha_reference = Vec3::Zero();
  Vec3 accel_reference = Vec3::Zero();
  Vec3 accel_target = Vec3::Zero();
};

std::vector<ImuPairSample> collectImuPairSamples(
    const ImuObservationDataset& reference_imu,
    const ImuObservationDataset& target_imu, const double time_offset_s,
    const int stride) {
  std::vector<ImuPairSample> samples;
  const int safe_stride = std::max(1, stride);
  samples.reserve(target_imu.samples.size() /
                  static_cast<std::size_t>(safe_stride) + 1);
  for (std::size_t i = 0; i < target_imu.samples.size();
       i += static_cast<std::size_t>(safe_stride)) {
    const ImuSample& target = target_imu.samples[i];
    ImuSample reference;
    Vec3 alpha_reference = Vec3::Zero();
    if (!interpolateImuSample(reference_imu.samples,
                              target.timestamp_s + time_offset_s,
                              &reference, &alpha_reference)) {
      continue;
    }
    ImuPairSample pair;
    pair.omega_reference = reference.gyro_rad_s;
    pair.omega_target = target.gyro_rad_s;
    pair.alpha_reference = alpha_reference;
    pair.accel_reference = reference.accel_m_s2;
    pair.accel_target = target.accel_m_s2;
    samples.push_back(pair);
  }
  return samples;
}

Mat3 wahbaRotationReferenceToTarget(const std::vector<Vec3>& reference_gyro,
                                    const std::vector<Vec3>& target_gyro,
                                    Vec3* reference_mean, Vec3* target_mean,
                                    Vec3* singular_values) {
  Vec3 mean_reference = Vec3::Zero();
  Vec3 mean_target = Vec3::Zero();
  for (std::size_t i = 0; i < reference_gyro.size(); ++i) {
    mean_reference += reference_gyro[i];
    mean_target += target_gyro[i];
  }
  const double inv_n = 1.0 / static_cast<double>(reference_gyro.size());
  mean_reference *= inv_n;
  mean_target *= inv_n;

  Mat3 correlation = Mat3::Zero();
  for (std::size_t i = 0; i < reference_gyro.size(); ++i) {
    correlation +=
        (reference_gyro[i] - mean_reference) *
        (target_gyro[i] - mean_target).transpose();
  }

  Eigen::JacobiSVD<Mat3> svd(correlation,
                             Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 fix = Mat3::Identity();
  if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0) {
    fix(2, 2) = -1.0;
  }
  if (reference_mean) {
    *reference_mean = mean_reference;
  }
  if (target_mean) {
    *target_mean = mean_target;
  }
  if (singular_values) {
    *singular_values = svd.singularValues();
  }
  return svd.matrixV() * fix * svd.matrixU().transpose();
}

double gyroRms(const std::vector<Vec3>& reference_gyro,
               const std::vector<Vec3>& target_gyro, const Mat3& R_i_b,
               const Vec3& bias) {
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < reference_gyro.size(); ++i) {
    sum_sq += (R_i_b * reference_gyro[i] + bias - target_gyro[i]).squaredNorm();
  }
  return std::sqrt(sum_sq / static_cast<double>(reference_gyro.size()));
}

double accelLeverRms(const std::vector<ImuPairSample>& samples,
                     const Mat3& R_i_b, const Vec3& r_b,
                     const Vec3& accel_bias_delta_body) {
  if (samples.empty()) {
    return 0.0;
  }
  double sum_sq = 0.0;
  const Mat3 R_b_i = R_i_b.transpose();
  for (const ImuPairSample& sample : samples) {
    const Mat3 lever_jacobian =
        skew(sample.alpha_reference) + skew(sample.omega_reference) *
                                           skew(sample.omega_reference);
    const Vec3 accel_delta =
        R_b_i * sample.accel_target - sample.accel_reference;
    const Vec3 residual =
        lever_jacobian * r_b + accel_bias_delta_body - accel_delta;
    sum_sq += residual.squaredNorm();
  }
  return std::sqrt(sum_sq / static_cast<double>(samples.size()));
}

struct LeverArmEstimate {
  bool estimated = false;
  Vec3 r_b = Vec3::Zero();
  Vec3 bias_delta_body = Vec3::Zero();
  Vec3 singular_values = Vec3::Zero();
  double rms_m_s2 = 0.0;
};

LeverArmEstimate estimateLeverArmFromAccelDifference(
    const ImuObservationDataset& reference_imu,
    const ImuObservationDataset& target_imu, const double time_offset_s,
    const Mat3& R_i_b, const ImuChainInitializerOptions& options) {
  LeverArmEstimate result;
  const int stride = std::max(1, options.sample_stride);
  std::vector<Mat3> lever_jacobians;
  std::vector<Vec3> accel_deltas;
  lever_jacobians.reserve(target_imu.samples.size() /
                          static_cast<std::size_t>(stride) + 1);
  accel_deltas.reserve(lever_jacobians.capacity());
  const Mat3 R_b_i = R_i_b.transpose();
  for (std::size_t i = 0; i < target_imu.samples.size();
       i += static_cast<std::size_t>(stride)) {
    const ImuSample& target = target_imu.samples[i];
    ImuSample reference;
    Vec3 alpha_b = Vec3::Zero();
    if (!interpolateImuSample(reference_imu.samples,
                              target.timestamp_s + time_offset_s,
                              &reference, &alpha_b)) {
      continue;
    }
    const Vec3& omega_b = reference.gyro_rad_s;
    const Mat3 lever_jacobian =
        skew(alpha_b) + skew(omega_b) * skew(omega_b);
    lever_jacobians.push_back(lever_jacobian);
    accel_deltas.push_back(R_b_i * target.accel_m_s2 -
                           reference.accel_m_s2);
  }
  const std::size_t sample_count = accel_deltas.size();
  if (sample_count < static_cast<std::size_t>(options.min_samples)) {
    return result;
  }

  Mat3 mean_jacobian = Mat3::Zero();
  for (const Mat3& jacobian : lever_jacobians) {
    mean_jacobian += jacobian;
  }
  mean_jacobian /= static_cast<double>(sample_count);
  Mat3 excitation = Mat3::Zero();
  for (const Mat3& jacobian : lever_jacobians) {
    excitation += (jacobian - mean_jacobian).transpose() *
                  (jacobian - mean_jacobian);
  }
  Eigen::SelfAdjointEigenSolver<Mat3> excitation_solver(excitation);
  if (excitation_solver.info() != Eigen::Success) {
    return result;
  }
  const Vec3 excitation_values = excitation_solver.eigenvalues();
  result.singular_values =
      Vec3(std::sqrt(std::max(0.0, excitation_values(2))),
           std::sqrt(std::max(0.0, excitation_values(1))),
           std::sqrt(std::max(0.0, excitation_values(0))));
  if (result.singular_values(2) <= options.min_lever_excitation) {
    return result;
  }

  Eigen::MatrixXd A(static_cast<Eigen::Index>(3 * sample_count), 6);
  Eigen::VectorXd y(static_cast<Eigen::Index>(3 * sample_count));
  for (std::size_t i = 0; i < sample_count; ++i) {
    A.block<3, 3>(static_cast<Eigen::Index>(3 * i), 0) =
        lever_jacobians[i];
    A.block<3, 3>(static_cast<Eigen::Index>(3 * i), 3) = Mat3::Identity();
    y.segment<3>(static_cast<Eigen::Index>(3 * i)) = accel_deltas[i];
  }
  const Eigen::VectorXd x =
      A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
  result.r_b = x.segment<3>(0);
  result.bias_delta_body = x.segment<3>(3);
  if (result.r_b.norm() > options.max_lever_arm_norm_m) {
    return result;
  }
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < sample_count; ++i) {
    const Vec3 residual =
        lever_jacobians[i] * result.r_b + result.bias_delta_body -
        accel_deltas[i];
    sum_sq += residual.squaredNorm();
  }
  result.rms_m_s2 = std::sqrt(sum_sq / static_cast<double>(sample_count));
  if (options.max_lever_accel_rms_m_s2 >= 0.0 &&
      result.rms_m_s2 > options.max_lever_accel_rms_m_s2) {
    return result;
  }
  result.estimated = true;
  return result;
}

ceres::Solver::Summary refineRotationAndBias(
    const std::vector<Vec3>& reference_gyro,
    const std::vector<Vec3>& target_gyro,
    const ImuChainInitializerOptions& options, Mat3* R_i_b, Vec3* bias) {
  if (!R_i_b || !bias) {
    throw std::invalid_argument("rotation refinement outputs must be non-null");
  }
  Vec3 r_i_b = rotationMatrixToVector(*R_i_b);
  Vec3 mutable_bias = *bias;

  ceres::Problem problem;
  problem.AddParameterBlock(r_i_b.data(), 3);
  problem.AddParameterBlock(mutable_bias.data(), 3);
  for (std::size_t i = 0; i < reference_gyro.size(); ++i) {
    problem.AddResidualBlock(
        new RelativeGyroRotationCost(reference_gyro[i], target_gyro[i]),
        nullptr, r_i_b.data(), mutable_bias.data());
  }

  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = std::max(0, options.refine_max_iterations);
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.num_threads = 2;
  solver_options.minimizer_progress_to_stdout = false;
  solver_options.logging_type = ceres::SILENT;
  solver_options.parameter_tolerance = 1e-4;
  solver_options.function_tolerance = 1e-12;
  solver_options.gradient_tolerance = 1e-12;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  *R_i_b = rotationVectorToMatrix(r_i_b);
  *bias = mutable_bias;
  return summary;
}

ceres::Solver::Summary refineRotationLeverAndBiasWithAccel(
    const std::vector<ImuPairSample>& samples,
    const ImuChainInitializerOptions& options, Mat3* R_i_b, Vec3* gyro_bias,
    Vec3* r_b, Vec3* accel_bias_delta_body, double* accel_rms_m_s2) {
  if (!R_i_b || !gyro_bias || !r_b || !accel_bias_delta_body ||
      !accel_rms_m_s2) {
    throw std::invalid_argument("accel refinement outputs must be non-null");
  }
  Vec3 r_i_b = rotationMatrixToVector(*R_i_b);
  const Vec3 initial_r_i_b = r_i_b;
  Vec3 mutable_gyro_bias = *gyro_bias;
  Vec3 mutable_r_b = *r_b;
  Vec3 mutable_accel_bias = *accel_bias_delta_body;

  ceres::Problem problem;
  problem.AddParameterBlock(r_i_b.data(), 3);
  problem.AddParameterBlock(mutable_gyro_bias.data(), 3);
  problem.AddParameterBlock(mutable_r_b.data(), 3);
  problem.AddParameterBlock(mutable_accel_bias.data(), 3);
  if (options.refine_rotation_bound_rad > 0.0) {
    for (int i = 0; i < 3; ++i) {
      problem.SetParameterLowerBound(
          r_i_b.data(), i, initial_r_i_b(i) - options.refine_rotation_bound_rad);
      problem.SetParameterUpperBound(
          r_i_b.data(), i, initial_r_i_b(i) + options.refine_rotation_bound_rad);
    }
  }
  for (int i = 0; i < 3; ++i) {
    problem.SetParameterLowerBound(mutable_r_b.data(), i,
                                   -options.max_lever_arm_norm_m);
    problem.SetParameterUpperBound(mutable_r_b.data(), i,
                                   options.max_lever_arm_norm_m);
  }

  for (const ImuPairSample& sample : samples) {
    problem.AddResidualBlock(
        new RelativeGyroRotationCost(sample.omega_reference,
                                     sample.omega_target,
                                     options.refine_gyro_weight),
        nullptr, r_i_b.data(), mutable_gyro_bias.data());
    problem.AddResidualBlock(
        new RelativeAccelLeverCost(sample.omega_reference,
                                   sample.alpha_reference,
                                   sample.accel_reference,
                                   sample.accel_target,
                                   options.refine_accel_weight),
        nullptr, r_i_b.data(), mutable_r_b.data(),
        mutable_accel_bias.data());
  }
  if (options.refine_lever_prior_sigma_m > 0.0) {
    problem.AddResidualBlock(
        new VectorPriorCost(*r_b, options.refine_lever_prior_sigma_m),
        nullptr, mutable_r_b.data());
  }
  if (options.refine_accel_bias_prior_sigma_m_s2 > 0.0) {
    problem.AddResidualBlock(
        new VectorPriorCost(*accel_bias_delta_body,
                            options.refine_accel_bias_prior_sigma_m_s2),
        nullptr, mutable_accel_bias.data());
  }

  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = std::max(0, options.refine_max_iterations);
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.num_threads = 2;
  solver_options.minimizer_progress_to_stdout = false;
  solver_options.logging_type = ceres::SILENT;
  solver_options.parameter_tolerance = 1e-4;
  solver_options.function_tolerance = 1e-12;
  solver_options.gradient_tolerance = 1e-12;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  *R_i_b = rotationVectorToMatrix(r_i_b);
  *gyro_bias = mutable_gyro_bias;
  *r_b = mutable_r_b;
  *accel_bias_delta_body = mutable_accel_bias;
  *accel_rms_m_s2 =
      accelLeverRms(samples, *R_i_b, *r_b, *accel_bias_delta_body);
  return summary;
}

}  // namespace

ImuExtrinsicBlock imuExtrinsicFromRotationAndLever(const Mat3& R_i_b,
                                                   const Vec3& r_b) {
  ImuExtrinsicBlock block;
  const Vec3 r_i_b = rotationMatrixToVector(R_i_b);
  for (int i = 0; i < 3; ++i) {
    block.values[static_cast<std::size_t>(i)] = r_b(i);
    block.values[static_cast<std::size_t>(i + 3)] = r_i_b(i);
  }
  return block;
}

ImuExtrinsicBlock imuExtrinsicFromRotation(const Mat3& R_i_b) {
  return imuExtrinsicFromRotationAndLever(R_i_b, Vec3::Zero());
}

ImuChainInitializerPairResult estimateImuChainPairPrior(
    const ImuObservationDataset& reference_imu,
    const ImuObservationDataset& target_imu, const std::size_t imu_index,
    const ImuChainInitializerOptions& options) {
  if (reference_imu.samples.size() < 2 || target_imu.samples.size() < 2) {
    throw std::runtime_error(
        "reference and target IMUs need at least two samples");
  }
  if (!options.use_full_overlap_time_offset_search &&
      options.max_time_offset_search_s < 0.0) {
    throw std::runtime_error("IMU chain max offset search must be non-negative");
  }

  const CorrelationResult correlation =
      estimateTimeOffsetByGyroNorm(reference_imu, target_imu, options);

  std::vector<Vec3> reference_gyro;
  std::vector<Vec3> target_gyro;
  collectGyroPairs(reference_imu, target_imu, correlation.time_offset_s,
                   options.sample_stride, &reference_gyro, &target_gyro);
  if (reference_gyro.size() < static_cast<std::size_t>(options.min_samples)) {
    throw std::runtime_error(
        "not enough overlapping samples for IMU chain rotation initialization");
  }

  Vec3 reference_mean = Vec3::Zero();
  Vec3 target_mean = Vec3::Zero();
  Vec3 singular_values = Vec3::Zero();
  Mat3 R_i_b = wahbaRotationReferenceToTarget(
      reference_gyro, target_gyro, &reference_mean, &target_mean,
      &singular_values);
  if (singular_values.sum() <= options.min_rotation_excitation) {
    throw std::runtime_error(
        "gyro motion is too weak for IMU chain rotation initialization");
  }
  Vec3 bias = target_mean - R_i_b * reference_mean;
  ceres::Solver::Summary refine_summary;
  if (options.refine_with_ceres && options.refine_max_iterations > 0) {
    refine_summary =
        refineRotationAndBias(reference_gyro, target_gyro, options, &R_i_b,
                              &bias);
  }
  LeverArmEstimate lever;
  if (options.estimate_lever_arms) {
    lever = estimateLeverArmFromAccelDifference(
        reference_imu, target_imu, correlation.time_offset_s, R_i_b, options);
  }
  ceres::Solver::Summary accel_refine_summary;
  bool accel_refined = false;
  double accel_refine_rms_m_s2 = 0.0;
  if (options.refine_with_accel && options.refine_max_iterations > 0) {
    std::vector<ImuPairSample> pair_samples = collectImuPairSamples(
        reference_imu, target_imu, correlation.time_offset_s,
        options.sample_stride);
    if (pair_samples.size() < static_cast<std::size_t>(options.min_samples)) {
      throw std::runtime_error(
          "not enough overlapping samples for IMU chain accel refinement");
    }
    Vec3 refined_r_b = lever.estimated ? lever.r_b : Vec3::Zero();
    Vec3 refined_accel_bias =
        lever.estimated ? lever.bias_delta_body : Vec3::Zero();
    accel_refine_summary = refineRotationLeverAndBiasWithAccel(
        pair_samples, options, &R_i_b, &bias, &refined_r_b,
        &refined_accel_bias, &accel_refine_rms_m_s2);
    accel_refined = true;
    lever.r_b = refined_r_b;
    lever.bias_delta_body = refined_accel_bias;
    lever.rms_m_s2 = accel_refine_rms_m_s2;
    lever.estimated =
        options.estimate_lever_arms &&
        refined_r_b.norm() <= options.max_lever_arm_norm_m &&
        (options.max_lever_accel_rms_m_s2 < 0.0 ||
         accel_refine_rms_m_s2 <= options.max_lever_accel_rms_m_s2);
  }

  ImuChainInitializerPairResult result;
  result.imu_index = imu_index;
  result.time_offset_s = correlation.time_offset_s;
  result.discrete_shift_samples = correlation.discrete_shift_samples;
  result.sample_dt_s = correlation.sample_dt_s;
  result.time_offset_search_radius_s = correlation.search_radius_s;
  result.max_search_lag_samples = correlation.max_lag_samples;
  result.matched_samples = static_cast<int>(reference_gyro.size());
  result.peak_correlation = correlation.peak_correlation;
  result.time_offset_boundary_peak_rejected = correlation.boundary_peak_rejected;
  result.rejected_discrete_shift_samples =
      correlation.rejected_discrete_shift_samples;
  result.rejected_matched_samples = correlation.rejected_matched_samples;
  result.rejected_peak_correlation = correlation.rejected_peak_correlation;
  result.R_i_b = R_i_b;
  result.r_b = lever.estimated ? lever.r_b : Vec3::Zero();
  result.r_i_b = rotationMatrixToVector(R_i_b);
  result.gyro_bias_rad_s = bias;
  result.singular_values = singular_values;
  result.gyro_rms_rad_s = gyroRms(reference_gyro, target_gyro, R_i_b, bias);
  result.lever_arm_estimated = lever.estimated;
  result.accel_bias_delta_body_m_s2 = lever.bias_delta_body;
  result.lever_singular_values = lever.singular_values;
  result.accel_rms_m_s2 = lever.rms_m_s2;
  result.refine_iterations = accel_refined
                                 ? static_cast<int>(
                                       accel_refine_summary.iterations.size())
                                 : static_cast<int>(
                                       refine_summary.iterations.size());
  result.refine_final_cost = accel_refined ? accel_refine_summary.final_cost
                                           : refine_summary.final_cost;
  result.accel_refined = accel_refined;
  result.accel_refine_rms_m_s2 = accel_refine_rms_m_s2;
  return result;
}

ImuChainInitializerResult estimateImuChainPrior(
    const std::vector<ImuObservationDataset>& imus,
    const ImuChainInitializerOptions& options) {
  if (imus.size() < 2) {
    throw std::runtime_error(
        "at least two IMUs are required for IMU chain initialization");
  }
  ImuChainInitializerResult result;
  result.imu_results.reserve(imus.size() - 1);
  for (std::size_t imu_index = 1; imu_index < imus.size(); ++imu_index) {
    result.imu_results.push_back(
        estimateImuChainPairPrior(imus.front(), imus[imu_index], imu_index,
                                  options));
  }
  return result;
}

}  // namespace ceres_cam_imu
