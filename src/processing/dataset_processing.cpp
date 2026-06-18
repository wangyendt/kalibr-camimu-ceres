#include "ceres_cam_imu/processing/dataset_processing.h"

#include <algorithm>

namespace ceres_cam_imu {

std::vector<ImuSample> trimImuSamplesKalibr(
    const std::vector<ImuSample>& samples, const int trim_edge_count,
    ImuTrimSummary* summary) {
  const int trim = std::max(0, trim_edge_count);
  const int input_size = static_cast<int>(samples.size());
  int first = samples.empty() ? -1 : 0;
  int last = samples.empty() ? -1 : input_size - 1;
  bool applied = false;

  if (trim > 0 && input_size > 2 * trim) {
    first = trim;
    // Kalibr skips index > num_messages - trim, so this boundary stays kept.
    last = input_size - trim;
    applied = true;
  }

  std::vector<ImuSample> output;
  if (first >= 0 && last >= first) {
    output.assign(samples.begin() + first, samples.begin() + last + 1);
  }

  if (summary) {
    summary->input_samples = input_size;
    summary->output_samples = static_cast<int>(output.size());
    summary->trim_edge_count = trim;
    summary->first_kept_index = first;
    summary->last_kept_index = last;
    summary->applied = applied;
  }
  return output;
}

std::vector<ImageObservation> limitImageObservations(
    const std::vector<ImageObservation>& observations, const int max_frames) {
  if (max_frames <= 0 ||
      static_cast<int>(observations.size()) <= max_frames) {
    return observations;
  }
  return std::vector<ImageObservation>(
      observations.begin(), observations.begin() + max_frames);
}

std::size_t countCornerMeasurements(
    const std::vector<ImageObservation>& observations) {
  std::size_t count = 0;
  for (const ImageObservation& observation : observations) {
    count += observation.corners.size();
  }
  return count;
}

TimeRange timeRange(const std::vector<ImuSample>& samples) {
  TimeRange range;
  if (!samples.empty()) {
    range.valid = true;
    range.start_s = samples.front().timestamp_s;
    range.end_s = samples.back().timestamp_s;
  }
  return range;
}

TimeRange timeRange(const std::vector<ImageObservation>& observations) {
  TimeRange range;
  if (!observations.empty()) {
    range.valid = true;
    range.start_s = observations.front().timestamp_s;
    range.end_s = observations.back().timestamp_s;
  }
  return range;
}

}  // namespace ceres_cam_imu
