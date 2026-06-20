#pragma once

#include <ceres/cost_function.h>

#include <vector>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"

namespace ceres_cam_imu {

ceres::CostFunction *
createAccelerometerResidual(const ImuSample &sample, const ImuNoise &noise,
                            const SplineSegmentMeta6 &pose_segment,
                            const SplineSegmentMeta6 &accel_bias_segment);

ceres::CostFunction *createAccelerometerTimeOffsetResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const std::vector<SplineSegmentMeta6> &pose_segments,
    const int pose_local_coeff_start,
    const std::vector<SplineSegmentMeta6> &accel_bias_segments,
    const int accel_bias_local_coeff_start, const double buffer_start_s,
    const double buffer_end_s);

ceres::CostFunction *createScaleMisalignedAccelerometerResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const SplineSegmentMeta6 &pose_segment,
    const SplineSegmentMeta6 &accel_bias_segment);

ceres::CostFunction *createSizeEffectAccelerometerResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const SplineSegmentMeta6 &pose_segment,
    const SplineSegmentMeta6 &accel_bias_segment);

} // namespace ceres_cam_imu
