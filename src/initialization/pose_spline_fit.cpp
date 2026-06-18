#include "ceres_cam_imu/initialization/pose_spline_fit.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/QR>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include "ceres_cam_imu/core/se3.h"
#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/residuals/spline_motion_prior.h"

namespace ceres_cam_imu {
namespace {

struct PoseFitRow {
  int coeff_start = 0;
  std::array<double, SplineSegmentMeta6::kOrder> weights{};
  Vec6 pose = Vec6::Zero();
};

Mat4 cameraExtrinsicToMatrix(const CameraExtrinsicBlock& pose) {
  Vec6 p;
  for (int i = 0; i < 6; ++i) {
    p(i) = pose.values[static_cast<std::size_t>(i)];
  }
  return pose6ToMatrix(p);
}

Vec6 matrixToPose6Local(const Mat4& T) {
  Vec6 pose;
  pose.head<3>() = T.block<3, 1>(0, 3);
  pose.tail<3>() = rotationMatrixToVector(T.block<3, 3>(0, 0));
  return pose;
}

void unwrapRotationVectors(std::vector<Vec6>* poses) {
  if (!poses || poses->size() < 2) {
    return;
  }

  constexpr double kTwoPi = 6.28318530717958647692;
  for (std::size_t i = 1; i < poses->size(); ++i) {
    const Vec3 previous = poses->at(i - 1).tail<3>();
    const Vec3 current = poses->at(i).tail<3>();
    const double angle = current.norm();
    if (angle < 1e-12) {
      continue;
    }

    const Vec3 axis = current / angle;
    Vec3 best = current;
    double best_distance = (best - previous).norm();
    for (int s = -3; s <= 3; ++s) {
      const Vec3 candidate = axis * (angle + kTwoPi * static_cast<double>(s));
      const double distance = (candidate - previous).norm();
      if (distance < best_distance) {
        best = candidate;
        best_distance = distance;
      }
    }
    poses->at(i).tail<3>() = best;
  }
}

Eigen::MatrixXd solvePoseFitNormalEquations(
    const std::vector<PoseFitRow>& rows, const UniformBSpline& pose_spline,
    const PoseSplineFitOptions& options) {
  using SparseMatrix = Eigen::SparseMatrix<double>;
  using Triplet = Eigen::Triplet<double>;

  const int num_coefficients = pose_spline.numCoefficients();
  std::vector<Triplet> triplets;
  triplets.reserve(rows.size() * SplineSegmentMeta6::kOrder);
  Eigen::MatrixXd targets(rows.size(), 6);
  for (int row_index = 0; row_index < static_cast<int>(rows.size()); ++row_index) {
    const PoseFitRow& row = rows[static_cast<std::size_t>(row_index)];
    targets.row(row_index) = row.pose.transpose();
    for (int k = 0; k < SplineSegmentMeta6::kOrder; ++k) {
      triplets.emplace_back(row_index, row.coeff_start + k,
                            row.weights[static_cast<std::size_t>(k)]);
    }
  }

  SparseMatrix design(static_cast<int>(rows.size()), num_coefficients);
  design.setFromTriplets(triplets.begin(), triplets.end());
  design.makeCompressed();

  SparseMatrix normal = design.transpose() * design;
  const double lambda = std::max(0.0, options.regularization);
  for (int i = 0; i < num_coefficients; ++i) {
    normal.coeffRef(i, i) += lambda;
  }
  const double motion_lambda = std::max(0.0, options.motion_regularization);
  if (motion_lambda > 0.0) {
    if (options.motion_regularization_order <= 0 ||
        options.motion_regularization_order >= SplineSegmentMeta6::kOrder) {
      throw std::invalid_argument(
          "pose fit motion regularization order must be in [1, spline_order)");
    }
    for (int segment = 0; segment < pose_spline.numSegments(); ++segment) {
      const SplineSegmentMeta6 meta = pose_spline.segmentMeta6(
          pose_spline.tMin() + static_cast<double>(segment) * pose_spline.dt());
      const SplineBasisMatrix6 q = segmentWeightDerivativeIntegral(
          meta, options.motion_regularization_order);
      for (int row = 0; row < SplineSegmentMeta6::kOrder; ++row) {
        for (int col = 0; col < SplineSegmentMeta6::kOrder; ++col) {
          normal.coeffRef(meta.coeff_start + row, meta.coeff_start + col) +=
              motion_lambda * q(row, col);
        }
      }
    }
  }
  normal.makeCompressed();

  const Eigen::MatrixXd rhs = design.transpose() * targets;
  Eigen::SimplicialLDLT<SparseMatrix> solver;
  solver.compute(normal);
  if (solver.info() == Eigen::Success) {
    const Eigen::MatrixXd solution = solver.solve(rhs);
    if (solver.info() == Eigen::Success && solution.allFinite()) {
      return solution;
    }
  }

  const Eigen::MatrixXd dense_design(design);
  return dense_design.colPivHouseholderQr().solve(targets);
}

}  // namespace

PoseSplineFitSummary fitPoseSplineControlsFromCameraPoses(
    const std::vector<PoseObservation>& pose_observations,
    const CameraExtrinsicBlock& T_c_b, const double camera_time_shift_s,
    const UniformBSpline& pose_spline, const PoseSplineFitOptions& options,
    std::vector<PoseControlBlock>* pose_controls) {
  if (!pose_controls) {
    throw std::invalid_argument("pose_controls must be non-null");
  }

  PoseSplineFitSummary summary;
  if (pose_observations.empty() || !pose_spline.valid()) {
    return summary;
  }

  summary.num_coefficients = pose_spline.numCoefficients();
  pose_controls->resize(static_cast<std::size_t>(summary.num_coefficients));

  const Mat4 T_c_b_matrix = cameraExtrinsicToMatrix(T_c_b);
  std::vector<double> query_times;
  std::vector<Vec6> body_poses;
  query_times.reserve(pose_observations.size());
  body_poses.reserve(pose_observations.size());
  for (const PoseObservation& observation : pose_observations) {
    const double query_time = observation.timestamp_s + camera_time_shift_s;
    if (!pose_spline.isValidTime(query_time)) {
      ++summary.skipped_observations;
      continue;
    }
    query_times.push_back(query_time);
    body_poses.push_back(matrixToPose6Local(observation.T_t_c * T_c_b_matrix));
  }

  if (options.add_boundary_anchors && !query_times.empty()) {
    const Vec6 first_pose = body_poses.front();
    const Vec6 last_pose = body_poses.back();
    query_times.insert(query_times.begin(), pose_spline.tMin());
    body_poses.insert(body_poses.begin(), first_pose);
    query_times.push_back(pose_spline.tMax());
    body_poses.push_back(last_pose);
    summary.boundary_anchor_observations = 2;
  }

  if (options.unwrap_rotation_vectors) {
    unwrapRotationVectors(&body_poses);
  }

  std::vector<PoseFitRow> rows;
  rows.reserve(body_poses.size());
  for (std::size_t i = 0; i < body_poses.size(); ++i) {
    const SplineSegmentMeta6 meta = pose_spline.segmentMeta6(query_times[i]);
    PoseFitRow row;
    row.coeff_start = meta.coeff_start;
    row.weights = meta.weights(query_times[i], 0);
    row.pose = body_poses[i];
    rows.push_back(row);
  }

  summary.used_observations = static_cast<int>(rows.size());
  if (rows.empty()) {
    return summary;
  }

  const Eigen::MatrixXd coefficients = solvePoseFitNormalEquations(
      rows, pose_spline, options);
  if (coefficients.rows() != pose_spline.numCoefficients() ||
      coefficients.cols() != 6 || !coefficients.allFinite()) {
    throw std::runtime_error("pose spline least-squares fit failed");
  }

  for (int i = 0; i < coefficients.rows(); ++i) {
    for (int k = 0; k < 6; ++k) {
      pose_controls->at(static_cast<std::size_t>(i))
          .values[static_cast<std::size_t>(k)] = coefficients(i, k);
    }
  }

  double translation_sq = 0.0;
  double rotation_sq = 0.0;
  for (const PoseFitRow& row : rows) {
    Vec6 fitted = Vec6::Zero();
    for (int k = 0; k < SplineSegmentMeta6::kOrder; ++k) {
      fitted += row.weights[static_cast<std::size_t>(k)]
              * coefficients.row(row.coeff_start + k).transpose();
    }
    const Vec6 residual = fitted - row.pose;
    translation_sq += residual.head<3>().squaredNorm();
    rotation_sq += residual.tail<3>().squaredNorm();
  }
  const double inv_count = 1.0 / static_cast<double>(rows.size());
  summary.rms_translation_m = std::sqrt(translation_sq * inv_count);
  summary.rms_rotation_rad = std::sqrt(rotation_sq * inv_count);
  return summary;
}

}  // namespace ceres_cam_imu
