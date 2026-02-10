#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "projection_math.hpp"

namespace projection_plane {

/**
 * @brief Baseline rasterizer: sequential loop with per-pixel z-buffer
 * @param u, v UV coordinates
 * @param depth Depth values
 * @param colors (N, 3) BGR colors (uint8) or empty for gray
 * @param width, height Image dimensions
 * @param u_min, u_max, v_min, v_max UV bounds
 * @param pixels_per_unit Resolution factor
 * @param depth_priority_far If true, farther points win
 * @param point_size Pixel size per point (1 = single pixel)
 * @return Image as cv::Mat (BGR, uint8)
 */
inline cv::Mat rasterize_baseline(
    const VecXd& u,
    const VecXd& v,
    const VecXd& depth,
    const std::vector<cv::Vec3b>& colors,
    int width,
    int height,
    double u_min,
    double u_max,
    double v_min,
    double v_max,
    double /* pixels_per_unit */,
    bool depth_priority_far,
    int point_size = 1) {

  // Initialize image (BGR, uint8)
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

  // Initialize depth buffer
  std::vector<std::vector<float>> zbuf(
      height, std::vector<float>(width, depth_priority_far ? -std::numeric_limits<float>::infinity()
                                                              : std::numeric_limits<float>::infinity()));

  // Compute scales
  double u_range = u_max - u_min;
  double v_range = v_max - v_min;

  if (u_range < 1e-9) u_range = 1.0;
  if (v_range < 1e-9) v_range = 1.0;

  double scale_u = (width - 1.0) / u_range;
  double scale_v = (height - 1.0) / v_range;

  // Prepare default color (gray)
  cv::Vec3b default_color(128, 128, 128);

  // Process each point in original order
  int n = u.size();
  for (int i = 0; i < n; ++i) {
    // Compute pixel coordinates with bankers rounding
    double px_f = (u(i) - u_min) * scale_u;
    double py_f = (v_max - v(i)) * scale_v;

    int32_t px = round_to_int32(px_f);
    int32_t py = round_to_int32(py_f);

    // Clamp to bounds
    px = std::max(0, std::min(width - 1, (int)px));
    py = std::max(0, std::min(height - 1, (int)py));

    float d = static_cast<float>(depth(i));
    cv::Vec3b color = (colors.size() > 0) ? colors[i] : default_color;

    // Determine if we should write
    bool should_write = false;
    if (depth_priority_far) {
      should_write = (d > zbuf[py][px]);
    } else {
      should_write = (d < zbuf[py][px]);
    }

    if (should_write) {
      // Handle point_size: draw neighborhood
      int radius = (point_size - 1) / 2;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          int nx = px + dx;
          int ny = py + dy;

          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            // Check z-test for this neighboring pixel
            bool should_write_neighbor = false;
            if (depth_priority_far) {
              should_write_neighbor = (d > zbuf[ny][nx]);
            } else {
              should_write_neighbor = (d < zbuf[ny][nx]);
            }

            if (should_write_neighbor) {
              zbuf[ny][nx] = d;
              image.at<cv::Vec3b>(ny, nx) = color;
            }
          }
        }
      }

      // Also update the center pixel explicitly
      zbuf[py][px] = d;
      image.at<cv::Vec3b>(py, px) = color;
    }
  }

  return image;
}

/**
 * @brief Fast stable rasterizer using sorting
 * For point_size=1 only; produces identical results to baseline.
 * @param u, v UV coordinates
 * @param depth Depth values
 * @param colors (N, 3) BGR colors (uint8) or empty for gray
 * @param width, height Image dimensions
 * @param u_min, u_max, v_min, v_max UV bounds
 * @param pixels_per_unit Resolution factor
 * @param depth_priority_far If true, farther points win
 * @param point_size Pixel size per point
 * @param logger Optional logger for warnings
 * @return Image as cv::Mat (BGR, uint8)
 */
inline cv::Mat rasterize_fast_stable(
    const VecXd& u,
    const VecXd& v,
    const VecXd& depth,
    const std::vector<cv::Vec3b>& colors,
    int width,
    int height,
    double u_min,
    double u_max,
    double v_min,
    double v_max,
    double pixels_per_unit,
    bool depth_priority_far,
    int point_size = 1,
    rclcpp::Logger logger = rclcpp::get_logger("projection_plane")) {

  // Warn if point_size > 1
  if (point_size > 1) {
    RCLCPP_WARN(logger,
                "fast_stable mode with point_size > 1 not fully optimized. "
                "Falling back to baseline.");
    return rasterize_baseline(u, v, depth, colors, width, height,
                             u_min, u_max, v_min, v_max, pixels_per_unit,
                             depth_priority_far, point_size);
  }

  // Initialize image (BGR, uint8)
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

  // Compute scales
  double u_range = u_max - u_min;
  double v_range = v_max - v_min;

  if (u_range < 1e-9) u_range = 1.0;
  if (v_range < 1e-9) v_range = 1.0;

  double scale_u = (width - 1.0) / u_range;
  double scale_v = (height - 1.0) / v_range;

  // Prepare default color (gray)
  cv::Vec3b default_color(128, 128, 128);

  // Step 1: Compute px, py arrays
  int n = u.size();
  std::vector<int32_t> px_array(n), py_array(n);

  for (int i = 0; i < n; ++i) {
    double px_f = (u(i) - u_min) * scale_u;
    double py_f = (v_max - v(i)) * scale_v;

    int32_t px = round_to_int32(px_f);
    int32_t py = round_to_int32(py_f);

    px = std::max(0, std::min(width - 1, (int)px));
    py = std::max(0, std::min(height - 1, (int)py));

    px_array[i] = px;
    py_array[i] = py;
  }

  // Step 2: Build 1D pixel indices
  std::vector<int64_t> idx_array(n);
  for (int i = 0; i < n; ++i) {
    idx_array[i] = (int64_t)py_array[i] * width + px_array[i];
  }

  // Step 3: Create index array [0, 1, ..., n-1]
  std::vector<int> indices(n);
  std::iota(indices.begin(), indices.end(), 0);

  // Step 4: Stable sort by (idx, depth order, original_index)
  // Comparator ensures stable sort matching baseline semantics
  auto comparator = [&](int i, int j) {
    int64_t idx_i = idx_array[i];
    int64_t idx_j = idx_array[j];

    if (idx_i != idx_j) {
      return idx_i < idx_j;
    }

    // Same pixel: sort by depth
    float d_i = static_cast<float>(depth(i));
    float d_j = static_cast<float>(depth(j));

    if (depth_priority_far) {
      // Far wins: larger depth is better, so sort descending
      if (d_i != d_j) return d_i > d_j;
    } else {
      // Near wins: smaller depth is better, so sort ascending
      if (d_i != d_j) return d_i < d_j;
    }

    // Tie-break by original index
    return i < j;
  };

  std::stable_sort(indices.begin(), indices.end(), comparator);

  // Step 5: Traverse sorted list and take FIRST occurrence per pixel_idx as winner
  std::vector<bool> pixel_visited(width * height, false);
  for (int idx_in_sorted : indices) {
    int64_t pixel_idx = idx_array[idx_in_sorted];

    if (!pixel_visited[pixel_idx]) {
      int px = px_array[idx_in_sorted];
      int py = py_array[idx_in_sorted];
      cv::Vec3b color = (colors.size() > 0) ? colors[idx_in_sorted]
                                             : default_color;

      image.at<cv::Vec3b>(py, px) = color;
      pixel_visited[pixel_idx] = true;
    }
  }

  return image;
}

/**
 * @brief Unified rasterizer interface
 */
inline cv::Mat rasterize(
    const VecXd& u,
    const VecXd& v,
    const VecXd& depth,
    const std::vector<cv::Vec3b>& colors,
    int width,
    int height,
    double u_min,
    double u_max,
    double v_min,
    double v_max,
    double pixels_per_unit,
    bool depth_priority_far,
    const std::string& raster_mode = "baseline",
    int point_size = 1,
    rclcpp::Logger logger = rclcpp::get_logger("projection_plane")) {

  if (raster_mode == "fast_stable") {
    return rasterize_fast_stable(u, v, depth, colors, width, height,
                                u_min, u_max, v_min, v_max, pixels_per_unit,
                                depth_priority_far, point_size, logger);
  } else {
    return rasterize_baseline(u, v, depth, colors, width, height,
                             u_min, u_max, v_min, v_max, pixels_per_unit,
                             depth_priority_far, point_size);
  }
}

}  // namespace projection_plane
