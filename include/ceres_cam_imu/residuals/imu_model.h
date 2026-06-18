#pragma once

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

struct ImuKinematics {
  Vec3 omega_b = Vec3::Zero();
  Vec3 alpha_b = Vec3::Zero();
  Vec3 h_b = Vec3::Zero();
};

Vec3 commonLeverAcceleration(const Vec3 &omega_b, const Vec3 &alpha_b,
                             const Vec3 &r_b);

Vec3 predictCalibratedGyroscope(const Mat3 &R_i_b, const Vec3 &omega_b,
                                const Vec3 &bias);

Vec3 predictScaleMisalignedGyroscope(const Mat3 &R_i_b, const Mat3 &R_gyro_i,
                                     const Mat3 &M_gyro,
                                     const Mat3 &A_gyro_accel,
                                     const Vec3 &omega_b, const Vec3 &a_b,
                                     const Vec3 &bias);

Vec3 predictCalibratedAccelerometer(const Mat3 &R_i_b, const Vec3 &h_b,
                                    const Vec3 &lever_b, const Vec3 &bias);

Vec3 predictScaleMisalignedAccelerometer(const Mat3 &R_i_b, const Mat3 &M_accel,
                                         const Vec3 &h_b, const Vec3 &lever_b,
                                         const Vec3 &bias);

Vec3 predictSizeEffectAccelerometer(const Mat3 &R_i_b, const Mat3 &M_accel,
                                    const Vec3 &h_b, const Vec3 &r_b,
                                    const Vec3 &rx_i, const Vec3 &ry_i,
                                    const Vec3 &rz_i, const Vec3 &omega_b,
                                    const Vec3 &alpha_b, const Vec3 &bias);

} // namespace ceres_cam_imu
