#pragma once

#include <ceres/cost_function.h>

#include "ceres_cam_imu/core/types.h"
#include "ceres_cam_imu/trajectory/uniform_bspline.h"

namespace ceres_cam_imu {

ceres::CostFunction *
createGyroscopeResidual(const ImuSample &sample, const ImuNoise &noise,
                        const SplineSegmentMeta6 &pose_segment,
                        const SplineSegmentMeta6 &gyro_bias_segment);

ceres::CostFunction *createScaleMisalignedGyroscopeResidual(
    const ImuSample &sample, const ImuNoise &noise,
    const SplineSegmentMeta6 &pose_segment,
    const SplineSegmentMeta6 &gyro_bias_segment);

} // namespace ceres_cam_imu
