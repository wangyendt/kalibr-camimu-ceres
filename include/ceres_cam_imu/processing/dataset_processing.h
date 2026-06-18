#pragma once

#include <cstddef>
#include <vector>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

struct TimeRange {
  bool valid = false;
  double start_s = 0.0;
  double end_s = 0.0;
};

struct ImuTrimSummary {
  int input_samples = 0;
  int output_samples = 0;
  int trim_edge_count = 0;
  int first_kept_index = -1;
  int last_kept_index = -1;
  bool applied = false;
};

std::vector<ImuSample> trimImuSamplesKalibr(
    const std::vector<ImuSample>& samples, int trim_edge_count,
    ImuTrimSummary* summary = nullptr);

std::vector<ImageObservation> limitImageObservations(
    const std::vector<ImageObservation>& observations, int max_frames);

std::size_t countCornerMeasurements(
    const std::vector<ImageObservation>& observations);

TimeRange timeRange(const std::vector<ImuSample>& samples);

TimeRange timeRange(const std::vector<ImageObservation>& observations);

}  // namespace ceres_cam_imu
