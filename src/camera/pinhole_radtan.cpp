#include "ceres_cam_imu/camera/pinhole_radtan.h"

#include <stdexcept>
#include <utility>

namespace ceres_cam_imu {

PinholeRadtanCamera::PinholeRadtanCamera(CameraIntrinsics intrinsics)
    : intrinsics_(std::move(intrinsics)) {}

Vec2 PinholeRadtanCamera::project(const Vec3& p_c) const {
  Vec2 pixel;
  project<double>(p_c, &pixel);
  return pixel;
}

bool PinholeRadtanCamera::projectWithJacobian(const Vec3& p_c, Vec2* pixel,
                                              Mat23* d_pixel_d_point) const {
  if (p_c.z() == 0.0) {
    return false;
  }

  const double X = p_c.x();
  const double Y = p_c.y();
  const double Z = p_c.z();
  const double inv_z = 1.0 / Z;
  const double inv_z2 = inv_z * inv_z;
  const double x = X * inv_z;
  const double y = Y * inv_z;
  const double r2 = x * x + y * y;
  const double r4 = r2 * r2;
  const double radial = 1.0 + intrinsics_.k1 * r2 + intrinsics_.k2 * r4;
  const double xy = x * y;
  const double xd = x * radial + 2.0 * intrinsics_.p1 * xy
                  + intrinsics_.p2 * (r2 + 2.0 * x * x);
  const double yd = y * radial + intrinsics_.p1 * (r2 + 2.0 * y * y)
                  + 2.0 * intrinsics_.p2 * xy;

  if (pixel) {
    *pixel = Vec2(intrinsics_.fx * xd + intrinsics_.cx,
                  intrinsics_.fy * yd + intrinsics_.cy);
  }

  if (!d_pixel_d_point) {
    return true;
  }

  const double d_radial_dx = 2.0 * intrinsics_.k1 * x
                           + 4.0 * intrinsics_.k2 * x * r2;
  const double d_radial_dy = 2.0 * intrinsics_.k1 * y
                           + 4.0 * intrinsics_.k2 * y * r2;

  const double d_xd_dx = radial + x * d_radial_dx
                       + 2.0 * intrinsics_.p1 * y
                       + 6.0 * intrinsics_.p2 * x;
  const double d_xd_dy = x * d_radial_dy
                       + 2.0 * intrinsics_.p1 * x
                       + 2.0 * intrinsics_.p2 * y;
  const double d_yd_dx = y * d_radial_dx
                       + 2.0 * intrinsics_.p1 * x
                       + 2.0 * intrinsics_.p2 * y;
  const double d_yd_dy = radial + y * d_radial_dy
                       + 6.0 * intrinsics_.p1 * y
                       + 2.0 * intrinsics_.p2 * x;

  Eigen::Matrix<double, 2, 2> d_pixel_d_norm;
  d_pixel_d_norm << intrinsics_.fx * d_xd_dx, intrinsics_.fx * d_xd_dy,
                    intrinsics_.fy * d_yd_dx, intrinsics_.fy * d_yd_dy;

  Eigen::Matrix<double, 2, 3> d_norm_d_point;
  d_norm_d_point << inv_z, 0.0, -X * inv_z2,
                    0.0, inv_z, -Y * inv_z2;
  *d_pixel_d_point = d_pixel_d_norm * d_norm_d_point;
  return true;
}

}  // namespace ceres_cam_imu
