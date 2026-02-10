#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace projection_plane {

using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using MatX3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using VecXd = Eigen::VectorXd;
using Vec3i = Eigen::Vector3i;

/**
 * @brief Normalize a vector to unit length
 * @param v Input vector
 * @return Unit vector
 * @throws std::runtime_error if norm is near zero
 */
inline Vec3d normalize(const Vec3d& v) {
  double norm = v.norm();
  if (norm < 1e-12) {
    throw std::runtime_error("Cannot normalize near-zero vector: norm=" +
                            std::to_string(norm));
  }
  return v / norm;
}

/**
 * @brief Validate plane equation and return normal vector
 * @param a, b, c, d Plane equation coefficients (ax + by + cz + d = 0)
 * @return Normal vector n = (a, b, c)
 * @throws std::runtime_error if plane is degenerate
 */
inline Vec3d validate_plane(double a, double b, double c, double /* d */) {
  Vec3d n(a, b, c);
  double n_norm = n.norm();
  if (n_norm < 1e-12) {
    throw std::runtime_error(
        "Degenerate plane: normal vector (a,b,c) has near-zero magnitude");
  }
  return n;
}

/**
 * @brief Choose an up hint vector for basis construction
 * @param n_hat Normalized plane normal
 * @param user_up_hint Optional user-specified up hint
 * @return Appropriate up_hint vector
 */
inline Vec3d choose_up_hint(const Vec3d& n_hat,
                            const Vec3d* user_up_hint = nullptr) {
  if (user_up_hint != nullptr) {
    Vec3d up = *user_up_hint;
    double up_norm = up.norm();
    if (up_norm > 1e-12) {
      up = up / up_norm;
      if (std::abs(n_hat.dot(up)) < 0.95) {
        return up;
      }
    }
  }

  // Default candidates: (0,0,1), (0,1,0), (1,0,0)
  std::vector<Vec3d> candidates = {
      Vec3d(0.0, 0.0, 1.0),
      Vec3d(0.0, 1.0, 0.0),
      Vec3d(1.0, 0.0, 0.0)
  };

  for (const auto& up : candidates) {
    if (std::abs(n_hat.dot(up)) < 0.95) {
      return up;
    }
  }

  // Fallback (should never reach here for valid planes)
  return candidates[0];
}

/**
 * @brief Build orthonormal basis (t1, t2) for the plane
 * @param n_hat Normalized plane normal
 * @param up_hint Hint vector for orientation
 * @return Tuple (t1, t2) orthonormal vectors on the plane
 * @throws std::runtime_error if basis construction fails
 */
inline std::pair<Vec3d, Vec3d> build_basis(const Vec3d& n_hat,
                                           const Vec3d& up_hint) {
  // t1 = normalize(cross(n_hat, up_hint))
  Vec3d t1 = n_hat.cross(up_hint);
  double t1_norm = t1.norm();
  if (t1_norm < 1e-12) {
    throw std::runtime_error(
        "Failed to build basis: n_hat and up_hint are parallel");
  }
  t1 = t1 / t1_norm;

  // t2 = cross(n_hat, t1)
  Vec3d t2 = n_hat.cross(t1);
  double t2_norm = t2.norm();
  if (t2_norm < 1e-12) {
    throw std::runtime_error("Failed to build basis: t2 is near-zero");
  }
  t2 = t2 / t2_norm;

  return {t1, t2};
}

/**
 * @brief Orthographically project points onto the plane
 * Projection formula: p_proj = p - ((n·p + d) / ||n||^2) * n
 * @param points (N, 3) array of 3D points
 * @param n Normal vector (a, b, c)
 * @param d Plane equation constant
 * @return (N, 3) array of projected points
 */
inline MatX3d project_points(const MatX3d& points, const Vec3d& n, double d) {
  double n_sq = n.dot(n);  // ||n||^2

  // Signed distance to plane: n·p + d for each point
  VecXd distances = points * n + VecXd::Constant(points.rows(), d);

  // p_proj = p - ((n·p + d) / ||n||^2) * n
  MatX3d projections = points;
  for (int i = 0; i < points.rows(); ++i) {
    projections.row(i) -= (distances(i) / n_sq) * n.transpose();
  }

  return projections;
}

/**
 * @brief Compute UV origin point on the plane
 * @param points_proj Projected points
 * @param n Normal vector
 * @param d Plane constant
 * @param mode 'mean' or 'closest'
 * @return Origin point on plane
 */
inline Vec3d compute_origin(const MatX3d& points_proj, const Vec3d& n,
                           double d, const std::string& mode) {
  if (mode == "closest") {
    // Closest point on plane to world origin: -(d / ||n||^2) * n
    double n_sq = n.dot(n);
    return -(d / n_sq) * n;
  } else {  // 'mean'
    return points_proj.colwise().mean();
  }
}

/**
 * @brief Map projected points to UV coordinates on the plane
 * @param points_proj (N, 3) projected points
 * @param origin Origin point for UV system
 * @param t1, t2 Basis vectors
 * @return Pair (u, v) where each is (N,) array
 */
inline std::pair<VecXd, VecXd> map_uv(const MatX3d& points_proj,
                                      const Vec3d& origin,
                                      const Vec3d& t1,
                                      const Vec3d& t2) {
  // Relative positions: rel = p_proj - origin
  int n = points_proj.rows();
  VecXd u(n), v(n);

  for (int i = 0; i < n; ++i) {
    Vec3d point = points_proj.row(i).transpose();  // Convert row to column vector
    Vec3d rel = point - origin;
    u(i) = rel.dot(t1);
    v(i) = rel.dot(t2);
  }

  return {u, v};
}

/**
 * @brief Compute depth values for z-buffer
 * Signed distance to plane: dist = (n·p + d) / ||n||
 * @param points Original 3D points
 * @param n Normal vector
 * @param d Plane constant
 * @param mode 'abs' or 'signed'
 * @return (N,) array of depth values
 */
inline VecXd compute_depth(const MatX3d& points, const Vec3d& n,
                          double d, const std::string& mode) {
  double n_norm = n.norm();
  VecXd signed_dist = (points * n + VecXd::Constant(points.rows(), d)) / n_norm;

  if (mode == "abs") {
    return signed_dist.cwiseAbs();
  } else {  // 'signed'
    return signed_dist;
  }
}

/**
 * @brief Compute image dimensions and UV bounds
 * @param u, v UV coordinates
 * @param pixels_per_unit Resolution factor
 * @param width_override, height_override Optional fixed dimensions
 * @param robust_range Use percentile-based range
 * @param percentiles (low, high) percentiles for robust range
 * @return Tuple (width, height, u_min, u_max, v_min, v_max)
 */
inline std::tuple<int, int, double, double, double, double>
compute_image_size(const VecXd& u, const VecXd& v,
                   double pixels_per_unit,
                   int width_override = -1,
                   int height_override = -1,
                   bool robust_range = false,
                   double percentile_low = 1.0,
                   double percentile_high = 99.0) {
  double u_min, u_max, v_min, v_max;

  if (robust_range) {
    // Compute percentiles
    auto compute_percentile = [](const VecXd& data, double p) {
      int n = data.size();
      std::vector<double> sorted(data.data(), data.data() + n);
      std::sort(sorted.begin(), sorted.end());
      int idx = static_cast<int>(n * p / 100.0);
      idx = std::max(0, std::min(n - 1, idx));
      return sorted[idx];
    };

    u_min = compute_percentile(u, percentile_low);
    u_max = compute_percentile(u, percentile_high);
    v_min = compute_percentile(v, percentile_low);
    v_max = compute_percentile(v, percentile_high);
  } else {
    u_min = u.minCoeff();
    u_max = u.maxCoeff();
    v_min = v.minCoeff();
    v_max = v.maxCoeff();
  }

  double u_range = u_max - u_min;
  double v_range = v_max - v_min;

  // Handle degenerate cases
  if (u_range < 1e-9) u_range = 1.0;
  if (v_range < 1e-9) v_range = 1.0;

  int auto_width = static_cast<int>(std::ceil(u_range * pixels_per_unit)) + 1;
  int auto_height = static_cast<int>(std::ceil(v_range * pixels_per_unit)) + 1;

  // Apply limits
  auto_width = std::max(100, std::min(8192, auto_width));
  auto_height = std::max(100, std::min(8192, auto_height));

  // Apply overrides
  int width = (width_override > 0) ? width_override : auto_width;
  int height = (height_override > 0) ? height_override : auto_height;

  return {width, height, u_min, u_max, v_min, v_max};
}

/**
 * @brief Bankers rounding (ties to even) matching numpy.round
 */
inline double bankers_round(double x) {
  double rounded = std::nearbyint(x);
  return rounded;
}

/**
 * @brief Round a double to int32 using bankers rounding
 */
inline int32_t round_to_int32(double x) {
  return static_cast<int32_t>(bankers_round(x));
}

}  // namespace projection_plane
