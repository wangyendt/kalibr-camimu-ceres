#include "ceres_cam_imu/camera/camera_model.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ceres_cam_imu {
namespace {

constexpr double kEpsilon = 1e-12;

std::string lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

bool isPinhole(const CameraIntrinsics &intrinsics) {
  return canonicalCameraModelName(intrinsics.camera_model) == "pinhole";
}

bool isOmni(const CameraIntrinsics &intrinsics) {
  return canonicalCameraModelName(intrinsics.camera_model) == "omni";
}

bool isEucm(const CameraIntrinsics &intrinsics) {
  return canonicalCameraModelName(intrinsics.camera_model) == "eucm";
}

bool isDoubleSphere(const CameraIntrinsics &intrinsics) {
  return canonicalCameraModelName(intrinsics.camera_model) == "ds";
}

void validateModelPair(const CameraIntrinsics &intrinsics) {
  const std::string camera_model =
      canonicalCameraModelName(intrinsics.camera_model);
  const std::string distortion_model =
      canonicalDistortionModelName(intrinsics.distortion_model);
  const bool valid =
      (camera_model == "pinhole" &&
       (distortion_model == "none" || distortion_model == "radtan" ||
        distortion_model == "equidistant" || distortion_model == "fov")) ||
      (camera_model == "omni" &&
       (distortion_model == "none" || distortion_model == "radtan")) ||
      ((camera_model == "eucm" || camera_model == "ds") &&
       distortion_model == "none");
  if (!valid) {
    throw std::invalid_argument(
        "unsupported camera/distortion model pair: " + intrinsics.camera_model +
        "/" + intrinsics.distortion_model);
  }
}

bool distortNone(const Vec2 &y, Vec2 *yd, Mat2 *d_yd_d_y) {
  if (yd) {
    *yd = y;
  }
  if (d_yd_d_y) {
    d_yd_d_y->setIdentity();
  }
  return true;
}

bool distortRadtan(const CameraIntrinsics &intrinsics, const Vec2 &y, Vec2 *yd,
                   Mat2 *d_yd_d_y) {
  const double x = y.x();
  const double yy = y.y();
  const double x2 = x * x;
  const double y2 = yy * yy;
  const double xy = x * yy;
  const double r2 = x2 + y2;
  const double r4 = r2 * r2;
  const double radial = intrinsics.k1 * r2 + intrinsics.k2 * r4;
  if (yd) {
    (*yd).x() = x + x * radial + 2.0 * intrinsics.p1 * xy +
                intrinsics.p2 * (r2 + 2.0 * x2);
    (*yd).y() = yy + yy * radial + 2.0 * intrinsics.p2 * xy +
                intrinsics.p1 * (r2 + 2.0 * y2);
  }
  if (d_yd_d_y) {
    (*d_yd_d_y)(0, 0) = 1.0 + radial + 2.0 * intrinsics.k1 * x2 +
                        4.0 * intrinsics.k2 * r2 * x2 +
                        2.0 * intrinsics.p1 * yy + 6.0 * intrinsics.p2 * x;
    (*d_yd_d_y)(1, 0) = 2.0 * intrinsics.k1 * xy +
                        4.0 * intrinsics.k2 * r2 * xy +
                        2.0 * intrinsics.p1 * x + 2.0 * intrinsics.p2 * yy;
    (*d_yd_d_y)(0, 1) = (*d_yd_d_y)(1, 0);
    (*d_yd_d_y)(1, 1) = 1.0 + radial + 2.0 * intrinsics.k1 * y2 +
                        4.0 * intrinsics.k2 * r2 * y2 +
                        6.0 * intrinsics.p1 * yy + 2.0 * intrinsics.p2 * x;
  }
  return true;
}

bool distortEquidistant(const CameraIntrinsics &intrinsics, const Vec2 &y,
                        Vec2 *yd, Mat2 *d_yd_d_y) {
  const double x = y.x();
  const double yy = y.y();
  const double r2 = x * x + yy * yy;
  const double r = std::sqrt(r2);
  if (r <= 1e-8) {
    if (yd) {
      *yd = y;
    }
    if (d_yd_d_y) {
      d_yd_d_y->setIdentity();
    }
    return true;
  }
  const double theta = std::atan(r);
  const double theta2 = theta * theta;
  const double theta4 = theta2 * theta2;
  const double theta6 = theta4 * theta2;
  const double theta8 = theta4 * theta4;
  const double poly = 1.0 + intrinsics.k1 * theta2 + intrinsics.k2 * theta4 +
                      intrinsics.p1 * theta6 + intrinsics.p2 * theta8;
  const double theta_d = theta * poly;
  const double scale = theta_d / r;
  if (yd) {
    *yd = scale * y;
  }
  if (d_yd_d_y) {
    const double dpoly_dtheta = 2.0 * intrinsics.k1 * theta +
                                4.0 * intrinsics.k2 * theta * theta2 +
                                6.0 * intrinsics.p1 * theta * theta4 +
                                8.0 * intrinsics.p2 * theta * theta6;
    const double dtheta_d_r = 1.0 / (1.0 + r2);
    const double dtheta_d_dtheta = poly + theta * dpoly_dtheta;
    const double dtheta_d_dr = dtheta_d_dtheta * dtheta_d_r;
    const double dscale_dr = (dtheta_d_dr * r - theta_d) / (r * r);
    *d_yd_d_y =
        scale * Mat2::Identity() + (dscale_dr / r) * (y * y.transpose());
  }
  return true;
}

bool distortFov(const CameraIntrinsics &intrinsics, const Vec2 &y, Vec2 *yd,
                Mat2 *d_yd_d_y) {
  const double w = intrinsics.distortion_coeffs.empty()
                       ? intrinsics.k1
                       : intrinsics.distortion_coeffs.front();
  const double r = y.norm();
  const double tan_w_half = std::tan(0.5 * w);
  double scale = 1.0;
  if (w * w >= 1e-5) {
    if (r * r < 1e-5) {
      scale = 2.0 * tan_w_half / w;
    } else {
      scale = std::atan(2.0 * tan_w_half * r) / (r * w);
    }
  }
  if (yd) {
    *yd = scale * y;
  }
  if (!d_yd_d_y) {
    return true;
  }
  if (w * w < 1e-5) {
    d_yd_d_y->setIdentity();
    return true;
  }
  if (r * r < 1e-5) {
    *d_yd_d_y = scale * Mat2::Identity();
    return true;
  }
  const double u = y.x();
  const double v = y.y();
  const double r2 = r * r;
  const double r3 = r2 * r;
  const double tan2 = tan_w_half * tan_w_half;
  const double atan_term = std::atan(2.0 * tan_w_half * r);
  const double common = 2.0 * tan_w_half / (w * r2 * (4.0 * tan2 * r2 + 1.0));
  (*d_yd_d_y)(0, 0) =
      atan_term / (w * r) - u * u * atan_term / (w * r3) + u * u * common;
  (*d_yd_d_y)(0, 1) = u * v * common - u * v * atan_term / (w * r3);
  (*d_yd_d_y)(1, 0) = (*d_yd_d_y)(0, 1);
  (*d_yd_d_y)(1, 1) =
      atan_term / (w * r) - v * v * atan_term / (w * r3) + v * v * common;
  return true;
}

bool distortNormalized(const CameraIntrinsics &intrinsics, const Vec2 &y,
                       Vec2 *yd, Mat2 *d_yd_d_y) {
  const std::string distortion_model =
      canonicalDistortionModelName(intrinsics.distortion_model);
  if (distortion_model == "none") {
    return distortNone(y, yd, d_yd_d_y);
  }
  if (distortion_model == "radtan") {
    return distortRadtan(intrinsics, y, yd, d_yd_d_y);
  }
  if (distortion_model == "equidistant") {
    return distortEquidistant(intrinsics, y, yd, d_yd_d_y);
  }
  if (distortion_model == "fov") {
    return distortFov(intrinsics, y, yd, d_yd_d_y);
  }
  return false;
}

bool pinholeNormalized(const Vec3 &p_c, Vec2 *y, Mat23 *d_y_d_p) {
  const double z = p_c.z();
  if (std::abs(z) <= kEpsilon) {
    return false;
  }
  const double inv_z = 1.0 / z;
  const double inv_z2 = inv_z * inv_z;
  if (y) {
    *y = Vec2(p_c.x() * inv_z, p_c.y() * inv_z);
  }
  if (d_y_d_p) {
    (*d_y_d_p) << inv_z, 0.0, -p_c.x() * inv_z2, 0.0, inv_z, -p_c.y() * inv_z2;
  }
  return true;
}

bool omniNormalized(const CameraIntrinsics &intrinsics, const Vec3 &p_c,
                    Vec2 *y, Mat23 *d_y_d_p) {
  const double x = p_c.x();
  const double yy = p_c.y();
  const double z = p_c.z();
  const double d = p_c.norm();
  if (d <= kEpsilon) {
    return false;
  }
  const double denom = z + intrinsics.xi * d;
  if (std::abs(denom) <= kEpsilon) {
    return false;
  }
  const double inv = 1.0 / denom;
  if (y) {
    *y = Vec2(x * inv, yy * inv);
  }
  if (d_y_d_p) {
    const double inv2_over_d = inv * inv / d;
    (*d_y_d_p)(0, 0) =
        inv2_over_d * (d * z + intrinsics.xi * (yy * yy + z * z));
    (*d_y_d_p)(1, 0) = -inv2_over_d * intrinsics.xi * x * yy;
    (*d_y_d_p)(0, 1) = (*d_y_d_p)(1, 0);
    (*d_y_d_p)(1, 1) = inv2_over_d * (d * z + intrinsics.xi * (x * x + z * z));
    const double z_col_scale = inv2_over_d * (-intrinsics.xi * z - d);
    (*d_y_d_p)(0, 2) = x * z_col_scale;
    (*d_y_d_p)(1, 2) = yy * z_col_scale;
  }
  return true;
}

bool eucmNormalized(const CameraIntrinsics &intrinsics, const Vec3 &p_c,
                    Vec2 *y, Mat23 *d_y_d_p) {
  const double x = p_c.x();
  const double yy = p_c.y();
  const double z = p_c.z();
  const double d2 = intrinsics.beta * (x * x + yy * yy) + z * z;
  if (d2 <= kEpsilon) {
    return false;
  }
  const double d = std::sqrt(d2);
  const double norm = intrinsics.alpha * d + (1.0 - intrinsics.alpha) * z;
  if (std::abs(norm) <= kEpsilon) {
    return false;
  }
  const double inv = 1.0 / norm;
  if (y) {
    *y = Vec2(x * inv, yy * inv);
  }
  if (d_y_d_p) {
    const double denom = inv * inv / d;
    const double add = norm * d;
    const double mid = -(intrinsics.alpha * intrinsics.beta * x * yy) * denom;
    const double add_z = intrinsics.alpha * z + (1.0 - intrinsics.alpha) * d;
    (*d_y_d_p)(0, 0) =
        (add - x * x * intrinsics.alpha * intrinsics.beta) * denom;
    (*d_y_d_p)(1, 0) = mid;
    (*d_y_d_p)(0, 1) = mid;
    (*d_y_d_p)(1, 1) =
        (add - yy * yy * intrinsics.alpha * intrinsics.beta) * denom;
    (*d_y_d_p)(0, 2) = -x * add_z * denom;
    (*d_y_d_p)(1, 2) = -yy * add_z * denom;
  }
  return true;
}

bool doubleSphereNormalized(const CameraIntrinsics &intrinsics, const Vec3 &p_c,
                            Vec2 *y, Mat23 *d_y_d_p) {
  const double x = p_c.x();
  const double yy = p_c.y();
  const double z = p_c.z();
  const double r2 = x * x + yy * yy;
  const double d1_sq = r2 + z * z;
  if (d1_sq <= kEpsilon) {
    return false;
  }
  const double d1 = std::sqrt(d1_sq);
  const double d1_inv = 1.0 / d1;
  const double k = intrinsics.xi * d1 + z;
  const double d2_sq = r2 + k * k;
  if (d2_sq <= kEpsilon) {
    return false;
  }
  const double d2 = std::sqrt(d2_sq);
  const double d2_inv = 1.0 / d2;
  const double norm = intrinsics.alpha * d2 + (1.0 - intrinsics.alpha) * k;
  if (std::abs(norm) <= kEpsilon) {
    return false;
  }
  const double inv = 1.0 / norm;
  if (y) {
    *y = Vec2(x * inv, yy * inv);
  }
  if (d_y_d_p) {
    const double inv2 = inv * inv;
    const double xy = x * yy;
    const double d_norm_d_r2 =
        (intrinsics.xi * (1.0 - intrinsics.alpha) * d1_inv +
         intrinsics.alpha * (intrinsics.xi * k * d1_inv + 1.0) * d2_inv) *
        inv2;
    const double tt2 = intrinsics.xi * z * d1_inv + 1.0;
    const double tmp2 =
        ((1.0 - intrinsics.alpha) * tt2 + intrinsics.alpha * k * tt2 * d2_inv) *
        inv2;
    (*d_y_d_p)(0, 0) = inv - x * x * d_norm_d_r2;
    (*d_y_d_p)(1, 0) = -xy * d_norm_d_r2;
    (*d_y_d_p)(0, 1) = -xy * d_norm_d_r2;
    (*d_y_d_p)(1, 1) = inv - yy * yy * d_norm_d_r2;
    (*d_y_d_p)(0, 2) = -x * tmp2;
    (*d_y_d_p)(1, 2) = -yy * tmp2;
  }
  return true;
}

bool normalizedProjection(const CameraIntrinsics &intrinsics, const Vec3 &p_c,
                          Vec2 *y, Mat23 *d_y_d_p) {
  if (isPinhole(intrinsics)) {
    return pinholeNormalized(p_c, y, d_y_d_p);
  }
  if (isOmni(intrinsics)) {
    return omniNormalized(intrinsics, p_c, y, d_y_d_p);
  }
  if (isEucm(intrinsics)) {
    return eucmNormalized(intrinsics, p_c, y, d_y_d_p);
  }
  if (isDoubleSphere(intrinsics)) {
    return doubleSphereNormalized(intrinsics, p_c, y, d_y_d_p);
  }
  return false;
}

} // namespace

std::string canonicalCameraModelName(const std::string &model) {
  std::string normalized = lowercase(model);
  if (normalized == "double-sphere" || normalized == "double_sphere") {
    return "ds";
  }
  return normalized;
}

std::string canonicalDistortionModelName(const std::string &model) {
  std::string normalized = lowercase(model);
  if (normalized == "equi" || normalized == "equidistant") {
    return "equidistant";
  }
  if (normalized.empty()) {
    return "none";
  }
  return normalized;
}

CameraModel::CameraModel(CameraIntrinsics intrinsics)
    : intrinsics_(std::move(intrinsics)) {
  intrinsics_.camera_model = canonicalCameraModelName(intrinsics_.camera_model);
  intrinsics_.distortion_model =
      canonicalDistortionModelName(intrinsics_.distortion_model);
  validateModelPair(intrinsics_);
}

Vec2 CameraModel::project(const Vec3 &p_c) const {
  Vec2 pixel;
  if (!projectWithJacobian(p_c, &pixel, nullptr)) {
    throw std::runtime_error("camera projection failed");
  }
  return pixel;
}

bool CameraModel::projectWithJacobian(const Vec3 &p_c, Vec2 *pixel,
                                      Mat23 *d_pixel_d_point) const {
  Vec2 y;
  Mat23 d_y_d_p;
  if (!normalizedProjection(intrinsics_, p_c, &y, &d_y_d_p)) {
    return false;
  }

  Vec2 yd;
  Mat2 d_yd_d_y;
  if (!distortNormalized(intrinsics_, y, &yd, &d_yd_d_y)) {
    return false;
  }

  if (pixel) {
    *pixel = Vec2(intrinsics_.fx * yd.x() + intrinsics_.cx,
                  intrinsics_.fy * yd.y() + intrinsics_.cy);
  }
  if (d_pixel_d_point) {
    Mat2 d_pixel_d_yd = Mat2::Zero();
    d_pixel_d_yd(0, 0) = intrinsics_.fx;
    d_pixel_d_yd(1, 1) = intrinsics_.fy;
    *d_pixel_d_point = d_pixel_d_yd * d_yd_d_y * d_y_d_p;
  }
  return true;
}

} // namespace ceres_cam_imu
