#include "ceres_cam_imu/residuals/imu_model.h"

#include "ceres_cam_imu/core/so3.h"

namespace ceres_cam_imu {

Vec3 commonLeverAcceleration(const Vec3 &omega_b, const Vec3 &alpha_b,
                             const Vec3 &r_b) {
  return alpha_b.cross(r_b) + omega_b.cross(omega_b.cross(r_b));
}

Vec3 predictCalibratedGyroscope(const Mat3 &R_i_b, const Vec3 &omega_b,
                                const Vec3 &bias) {
  return R_i_b * omega_b + bias;
}

Vec3 predictScaleMisalignedGyroscope(const Mat3 &R_i_b, const Mat3 &R_gyro_i,
                                     const Mat3 &M_gyro,
                                     const Mat3 &A_gyro_accel,
                                     const Vec3 &omega_b, const Vec3 &a_b,
                                     const Vec3 &bias) {
  const Mat3 R_gyro_b = R_gyro_i * R_i_b;
  return M_gyro * (R_gyro_b * omega_b) + A_gyro_accel * (R_gyro_b * a_b) + bias;
}

Vec3 predictCalibratedAccelerometer(const Mat3 &R_i_b, const Vec3 &h_b,
                                    const Vec3 &lever_b, const Vec3 &bias) {
  return R_i_b * (h_b + lever_b) + bias;
}

Vec3 predictScaleMisalignedAccelerometer(const Mat3 &R_i_b, const Mat3 &M_accel,
                                         const Vec3 &h_b, const Vec3 &lever_b,
                                         const Vec3 &bias) {
  return M_accel * (R_i_b * (h_b + lever_b)) + bias;
}

Vec3 predictSizeEffectAccelerometer(const Mat3 &R_i_b, const Mat3 &M_accel,
                                    const Vec3 &h_b, const Vec3 &r_b,
                                    const Vec3 &rx_i, const Vec3 &ry_i,
                                    const Vec3 &rz_i, const Vec3 &omega_b,
                                    const Vec3 &alpha_b, const Vec3 &bias) {
  const Mat3 R_b_i = R_i_b.transpose();
  const Vec3 rx_b = r_b + R_b_i * rx_i;
  const Vec3 ry_b = r_b + R_b_i * ry_i;
  const Vec3 rz_b = r_b + R_b_i * rz_i;
  const Vec3 lever_x = R_i_b * commonLeverAcceleration(omega_b, alpha_b, rx_b);
  const Vec3 lever_y = R_i_b * commonLeverAcceleration(omega_b, alpha_b, ry_b);
  const Vec3 lever_z = R_i_b * commonLeverAcceleration(omega_b, alpha_b, rz_b);
  Vec3 axis_specific = R_i_b * h_b;
  axis_specific.x() += lever_x.x();
  axis_specific.y() += lever_y.y();
  axis_specific.z() += lever_z.z();
  return M_accel * axis_specific + bias;
}

} // namespace ceres_cam_imu
