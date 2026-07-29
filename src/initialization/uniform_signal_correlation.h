#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ceres_cam_imu::internal {

struct UniformScalarSignal {
  double start_time_s = 0.0;
  double sample_dt_s = 0.0;
  std::vector<double> values;
};

// Both current correlation paths maximize left[i] * right[i - lag], but they
// expose opposite applied-time conventions. Keeping that choice explicit here
// prevents a camera lag sign from leaking into the IMU-chain path again.
enum class AppliedShiftConvention {
  kRightMinusLeftMinusLag,
  kLeftMinusRightPlusLag,
};

inline int uniformSampleCount(const double start_time_s,
                              const double end_time_s,
                              const double sample_dt_s) {
  if (!(end_time_s >= start_time_s) || !(sample_dt_s > 0.0)) {
    return 0;
  }
  return static_cast<int>(
             std::floor((end_time_s - start_time_s) / sample_dt_s + 1e-9)) +
         1;
}

inline double appliedShiftForLag(const UniformScalarSignal &left,
                                 const UniformScalarSignal &right,
                                 const int lag,
                                 const AppliedShiftConvention convention) {
  const double left_minus_right = left.start_time_s - right.start_time_s;
  const double lag_seconds = static_cast<double>(lag) * left.sample_dt_s;
  return convention == AppliedShiftConvention::kLeftMinusRightPlusLag
             ? left_minus_right + lag_seconds
             : -left_minus_right - lag_seconds;
}

inline double continuousLagForAppliedShift(
    const UniformScalarSignal &left, const UniformScalarSignal &right,
    const double shift_s, const AppliedShiftConvention convention) {
  const double left_minus_right = left.start_time_s - right.start_time_s;
  const double signed_numerator =
      convention == AppliedShiftConvention::kLeftMinusRightPlusLag
          ? shift_s - left_minus_right
          : -shift_s - left_minus_right;
  return signed_numerator / left.sample_dt_s;
}

inline int lagForAppliedShift(const UniformScalarSignal &left,
                              const UniformScalarSignal &right,
                              const double shift_s,
                              const AppliedShiftConvention convention) {
  return static_cast<int>(std::llround(
      continuousLagForAppliedShift(left, right, shift_s, convention)));
}

struct CorrelationAtLag {
  double value = -std::numeric_limits<double>::infinity();
  int matched = 0;
};

inline CorrelationAtLag
normalizedCorrelationAtLag(const UniformScalarSignal &left,
                           const UniformScalarSignal &right, const int lag) {
  const int left_count = static_cast<int>(left.values.size());
  const int right_count = static_cast<int>(right.values.size());
  const int begin = std::max(0, lag);
  const int end = std::min(left_count, right_count + lag);
  CorrelationAtLag result;
  result.matched = std::max(0, end - begin);
  if (result.matched == 0) {
    return result;
  }

  double sum_left = 0.0;
  double sum_right = 0.0;
  double sum_left_sq = 0.0;
  double sum_right_sq = 0.0;
  double sum_cross = 0.0;
  for (int left_index = begin; left_index < end; ++left_index) {
    const int right_index = left_index - lag;
    const double left_value = left.values[static_cast<std::size_t>(left_index)];
    const double right_value =
        right.values[static_cast<std::size_t>(right_index)];
    sum_left += left_value;
    sum_right += right_value;
    sum_left_sq += left_value * left_value;
    sum_right_sq += right_value * right_value;
    sum_cross += left_value * right_value;
  }
  const double inv_count = 1.0 / static_cast<double>(result.matched);
  const double covariance = sum_cross - sum_left * sum_right * inv_count;
  const double left_variance = sum_left_sq - sum_left * sum_left * inv_count;
  const double right_variance =
      sum_right_sq - sum_right * sum_right * inv_count;
  if (left_variance > 0.0 && right_variance > 0.0) {
    result.value = covariance / std::sqrt(left_variance * right_variance);
  }
  return result;
}

inline int minimumOverlapCount(const UniformScalarSignal &left,
                               const UniformScalarSignal &right,
                               const double min_overlap_fraction) {
  return std::max(10, static_cast<int>(std::ceil(
                          min_overlap_fraction *
                          static_cast<double>(std::min(left.values.size(),
                                                       right.values.size())))));
}

inline std::pair<int, int> overlapLimitedLagRange(
    const UniformScalarSignal &left, const UniformScalarSignal &right,
    const double min_overlap_fraction, const double max_abs_shift_s,
    const AppliedShiftConvention convention) {
  const int min_overlap =
      minimumOverlapCount(left, right, min_overlap_fraction);
  int first_lag = min_overlap - static_cast<int>(right.values.size());
  int last_lag = static_cast<int>(left.values.size()) - min_overlap;
  if (max_abs_shift_s <= 0.0) {
    return {first_lag, last_lag};
  }

  const double first_bound =
      continuousLagForAppliedShift(left, right, -max_abs_shift_s, convention);
  const double last_bound =
      continuousLagForAppliedShift(left, right, max_abs_shift_s, convention);
  const double lower_bound = std::min(first_bound, last_bound);
  const double upper_bound = std::max(first_bound, last_bound);
  if (lower_bound > static_cast<double>(std::numeric_limits<int>::max()) ||
      upper_bound < static_cast<double>(std::numeric_limits<int>::min())) {
    return {1, 0};
  }
  const int bounded_first = static_cast<int>(std::ceil(
      std::max(lower_bound - 1e-9,
               static_cast<double>(std::numeric_limits<int>::min()))));
  const int bounded_last = static_cast<int>(std::floor(
      std::min(upper_bound + 1e-9,
               static_cast<double>(std::numeric_limits<int>::max()))));
  first_lag = std::max(first_lag, bounded_first);
  last_lag = std::min(last_lag, bounded_last);
  return {first_lag, last_lag};
}

struct CorrelationSearchResult {
  int best_lag = 0;
  int second_best_lag = 0;
  int first_tested_lag = 0;
  int last_tested_lag = 0;
  int best_matched = 0;
  double best_correlation = -std::numeric_limits<double>::infinity();
  double second_best_correlation = -std::numeric_limits<double>::infinity();
  bool have_tested_lag = false;
};

inline CorrelationSearchResult
searchCorrelation(const UniformScalarSignal &left,
                  const UniformScalarSignal &right, const int first_lag,
                  const int last_lag, const int min_overlap,
                  const AppliedShiftConvention convention) {
  CorrelationSearchResult result;
  for (int lag = first_lag; lag <= last_lag; ++lag) {
    const CorrelationAtLag stats = normalizedCorrelationAtLag(left, right, lag);
    if (stats.matched < min_overlap || !std::isfinite(stats.value)) {
      continue;
    }
    if (!result.have_tested_lag) {
      result.first_tested_lag = lag;
      result.have_tested_lag = true;
    }
    result.last_tested_lag = lag;
    const double shift_s = appliedShiftForLag(left, right, lag, convention);
    const double best_shift_s =
        appliedShiftForLag(left, right, result.best_lag, convention);
    if (stats.value > result.best_correlation ||
        (stats.value == result.best_correlation &&
         std::abs(shift_s) < std::abs(best_shift_s))) {
      result.second_best_correlation = result.best_correlation;
      result.second_best_lag = result.best_lag;
      result.best_correlation = stats.value;
      result.best_lag = lag;
      result.best_matched = stats.matched;
    } else {
      const double second_shift_s =
          appliedShiftForLag(left, right, result.second_best_lag, convention);
      if (stats.value > result.second_best_correlation ||
          (stats.value == result.second_best_correlation &&
           std::abs(shift_s) < std::abs(second_shift_s))) {
        result.second_best_correlation = stats.value;
        result.second_best_lag = lag;
      }
    }
  }
  return result;
}

struct CoarseToFineCorrelationResult {
  CorrelationSearchResult fine;
  int fine_global_first_lag = 0;
  int fine_global_last_lag = 0;
  double best_shift_s = 0.0;
  double second_best_shift_s = 0.0;
  bool global_boundary = false;
};

inline CoarseToFineCorrelationResult searchCoarseToFineCorrelation(
    const UniformScalarSignal &coarse_left,
    const UniformScalarSignal &coarse_right,
    const UniformScalarSignal &fine_left, const UniformScalarSignal &fine_right,
    const double min_overlap_fraction, const double max_abs_shift_s,
    const double fine_search_half_width_s,
    const AppliedShiftConvention convention) {
  const auto [coarse_first_lag, coarse_last_lag] =
      overlapLimitedLagRange(coarse_left, coarse_right, min_overlap_fraction,
                             max_abs_shift_s, convention);
  const CorrelationSearchResult coarse = searchCorrelation(
      coarse_left, coarse_right, coarse_first_lag, coarse_last_lag,
      minimumOverlapCount(coarse_left, coarse_right, min_overlap_fraction),
      convention);
  if (!coarse.have_tested_lag || !std::isfinite(coarse.best_correlation)) {
    throw std::runtime_error(
        "not enough samples for coarse full-range correlation");
  }

  const double coarse_shift_s = appliedShiftForLag(coarse_left, coarse_right,
                                                   coarse.best_lag, convention);
  const auto [fine_global_first_lag, fine_global_last_lag] =
      overlapLimitedLagRange(fine_left, fine_right, min_overlap_fraction,
                             max_abs_shift_s, convention);
  const int fine_center_lag =
      lagForAppliedShift(fine_left, fine_right, coarse_shift_s, convention);
  int fine_half_width =
      std::max(2, static_cast<int>(std::ceil(fine_search_half_width_s /
                                             fine_left.sample_dt_s)));
  int fine_first_lag =
      std::max(fine_global_first_lag, fine_center_lag - fine_half_width);
  int fine_last_lag =
      std::min(fine_global_last_lag, fine_center_lag + fine_half_width);
  const int fine_min_overlap =
      minimumOverlapCount(fine_left, fine_right, min_overlap_fraction);
  CorrelationSearchResult fine =
      searchCorrelation(fine_left, fine_right, fine_first_lag, fine_last_lag,
                        fine_min_overlap, convention);
  while (fine.have_tested_lag &&
         (fine.best_lag == fine.first_tested_lag ||
          fine.best_lag == fine.last_tested_lag) &&
         (fine_first_lag > fine_global_first_lag ||
          fine_last_lag < fine_global_last_lag)) {
    fine_half_width *= 2;
    fine_first_lag =
        std::max(fine_global_first_lag, fine_center_lag - fine_half_width);
    fine_last_lag =
        std::min(fine_global_last_lag, fine_center_lag + fine_half_width);
    fine = searchCorrelation(fine_left, fine_right, fine_first_lag,
                             fine_last_lag, fine_min_overlap, convention);
  }
  if (!fine.have_tested_lag || !std::isfinite(fine.best_correlation)) {
    throw std::runtime_error(
        "not enough samples for fine full-range correlation");
  }

  CoarseToFineCorrelationResult result;
  result.fine = fine;
  result.fine_global_first_lag = fine_global_first_lag;
  result.fine_global_last_lag = fine_global_last_lag;
  result.best_shift_s =
      appliedShiftForLag(fine_left, fine_right, fine.best_lag, convention);
  result.second_best_shift_s = appliedShiftForLag(
      fine_left, fine_right, fine.second_best_lag, convention);
  result.global_boundary = fine.best_lag == fine_global_first_lag ||
                           fine.best_lag == fine_global_last_lag;
  return result;
}

} // namespace ceres_cam_imu::internal
