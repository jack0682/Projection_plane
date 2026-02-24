#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <projection_msgs/msg/projection_contract.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <memory>
#include <chrono>

#include "projection_plane/projection_math.hpp"
#include "projection_plane/rasterizer.hpp"

namespace projection_plane {

class ProjectionPlaneNode : public rclcpp::Node {
public:
  ProjectionPlaneNode() : Node("projection_plane_node") {
    // Declare parameters
    this->declare_parameter("ply_path",
        "/home/jack/Last_point/pcd_file/241108_converted - Cloud.ply");
    this->declare_parameter("pixels_per_unit", 500.0);
    this->declare_parameter("width", -1);
    this->declare_parameter("height", -1);
    this->declare_parameter("depth_priority_far", false);
    this->declare_parameter("origin_mode", "mean");
    this->declare_parameter("depth_mode", "abs");
    this->declare_parameter("robust_range", false);
    this->declare_parameter("percentile_low", 1.0);
    this->declare_parameter("percentile_high", 99.0);
    this->declare_parameter("point_size", 1);
    this->declare_parameter("publish_rate_hz", 10.0);
    this->declare_parameter("raster_mode", "baseline");
    this->declare_parameter("up_hint_x", std::nan(""));
    this->declare_parameter("up_hint_y", std::nan(""));
    this->declare_parameter("up_hint_z", std::nan(""));
    this->declare_parameter("save_png_path", "");

    // FOV geometry parameters
    this->declare_parameter("hfov_deg", 87.0);           // RealSense D435 horizontal FOV
    this->declare_parameter("vfov_deg", 58.0);           // RealSense D435 vertical FOV
    this->declare_parameter("plane_distance_m", 2.0);    // Virtual plane distance from camera
    this->declare_parameter("lock_yaw", true);           // Lock yaw to camera x-axis projection
    this->declare_parameter("imgsz_px", 1092);           // Fixed image size (must be 1092)

    // Get parameters
    ply_path_ = this->get_parameter("ply_path").as_string();
    pixels_per_unit_ = this->get_parameter("pixels_per_unit").as_double();
    width_override_ = this->get_parameter("width").as_int();
    height_override_ = this->get_parameter("height").as_int();
    depth_priority_far_ = this->get_parameter("depth_priority_far").as_bool();
    origin_mode_ = this->get_parameter("origin_mode").as_string();
    depth_mode_ = this->get_parameter("depth_mode").as_string();
    robust_range_ = this->get_parameter("robust_range").as_bool();
    percentile_low_ = this->get_parameter("percentile_low").as_double();
    percentile_high_ = this->get_parameter("percentile_high").as_double();
    point_size_ = this->get_parameter("point_size").as_int();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    raster_mode_ = this->get_parameter("raster_mode").as_string();
    save_png_path_ = this->get_parameter("save_png_path").as_string();

    // Load FOV parameters
    hfov_deg_ = this->get_parameter("hfov_deg").as_double();
    vfov_deg_ = this->get_parameter("vfov_deg").as_double();
    plane_distance_m_ = this->get_parameter("plane_distance_m").as_double();
    lock_yaw_ = this->get_parameter("lock_yaw").as_bool();
    imgsz_px_ = this->get_parameter("imgsz_px").as_int();

    RCLCPP_INFO(this->get_logger(),
        "FOV: H=%.1f° V=%.1f°, dist=%.2fm, imgsz=%d, lock_yaw=%s",
        hfov_deg_, vfov_deg_, plane_distance_m_, imgsz_px_,
        lock_yaw_ ? "true" : "false");

    // Validate parameters
    if (hfov_deg_ <= 0.0 || hfov_deg_ > 180.0) {
        throw std::runtime_error("hfov_deg must be in (0, 180)");
    }
    if (vfov_deg_ <= 0.0 || vfov_deg_ > 180.0) {
        throw std::runtime_error("vfov_deg must be in (0, 180)");
    }
    if (plane_distance_m_ <= 0.0) {
        throw std::runtime_error("plane_distance_m must be > 0");
    }
    if (imgsz_px_ != 1092) {
        RCLCPP_WARN(this->get_logger(),
            "imgsz_px=%d (expected 1092), forcing 1092", imgsz_px_);
        imgsz_px_ = 1092;
    }

    // Get up_hint if provided
    double up_hint_x = this->get_parameter("up_hint_x").as_double();
    double up_hint_y = this->get_parameter("up_hint_y").as_double();
    double up_hint_z = this->get_parameter("up_hint_z").as_double();

    if (!std::isnan(up_hint_x) && !std::isnan(up_hint_y) &&
        !std::isnan(up_hint_z)) {
      user_up_hint_ = std::make_unique<Vec3d>(up_hint_x, up_hint_y, up_hint_z);
      RCLCPP_INFO(this->get_logger(), "Using user-specified up_hint: (%.3f, %.3f, %.3f)",
                  up_hint_x, up_hint_y, up_hint_z);
    }

    // Create publishers
    pub_cloud_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/projection/cloud_raw", rclcpp::QoS(1).transient_local());

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/projection/image", rclcpp::QoS(10));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/projection/camera_pose", rclcpp::QoS(10));

    pub_meta_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/projection/proj_meta", rclcpp::QoS(10));

    pub_contract_ = this->create_publisher<projection_msgs::msg::ProjectionContract>(
        "/projection/contract", rclcpp::QoS(10));

    // Create subscriptions
    sub_plane_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/projection/plane", rclcpp::QoS(10),
        std::bind(&ProjectionPlaneNode::plane_callback, this,
                  std::placeholders::_1));

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/camera/pose_in", rclcpp::QoS(10),
        std::bind(&ProjectionPlaneNode::pose_callback, this,
                  std::placeholders::_1));

    // Load point cloud
    try {
      load_point_cloud();
      RCLCPP_INFO(this->get_logger(),
                  "Loaded point cloud: %zu points", cloud_raw_.rows());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud: %s",
                   e.what());
      throw;
    }

    // Publish cloud once
    publish_cloud_raw();

    // Create timer for projection updates
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&ProjectionPlaneNode::timer_callback, this));

    // Start worker thread for projection computation
    worker_thread_ =
        std::thread(std::bind(&ProjectionPlaneNode::worker_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "ProjectionPlaneNode initialized successfully");
  }

  ~ProjectionPlaneNode() {
    // Signal worker thread to stop
    stop_worker_ = true;
    cv_worker_.notify_one();

    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  // Parameters
  std::string ply_path_;
  double pixels_per_unit_;
  int width_override_;
  int height_override_;
  bool depth_priority_far_;
  std::string origin_mode_;
  std::string depth_mode_;
  bool robust_range_;
  double percentile_low_;
  double percentile_high_;
  int point_size_;
  double publish_rate_hz_;
  std::string raster_mode_;
  std::string save_png_path_;
  std::unique_ptr<Vec3d> user_up_hint_;

  // FOV geometry parameters
  double hfov_deg_;
  double vfov_deg_;
  double plane_distance_m_;
  bool lock_yaw_;
  int imgsz_px_;

  // Plane geometry (updated by pose_callback)
  Vec3d plane_center_;
  Vec3d plane_normal_;
  Mat3d camera_rotation_;
  double plane_width_m_;
  double plane_height_m_;
  double sx_px_per_m_;
  double sy_px_per_m_;
  double ox_px_;
  double oy_px_;

  // Basis vectors (u, v, n) for deterministic yaw lock
  Vec3d basis_u_;
  Vec3d basis_v_;
  Vec3d basis_n_;

  // Point cloud data (cached)
  MatX3d cloud_raw_;
  std::vector<cv::Vec3b> colors_raw_;

  // Current plane
  struct PlaneData {
    double a, b, c, d;
  };

  std::mutex plane_mutex_;
  PlaneData current_plane_{1.0, 0.0, 0.0, 0.0};
  bool plane_updated_{false};

  // Last computed image and metadata
  std::mutex image_mutex_;
  cv::Mat last_image_;
  geometry_msgs::msg::PoseStamped last_pose_;

  // Worker thread synchronization
  std::thread worker_thread_;
  std::mutex worker_mutex_;
  std::condition_variable cv_worker_;
  std::atomic<bool> stop_worker_{false};
  std::atomic<bool> is_projecting_{false};
  PlaneData pending_plane_{1.0, 0.0, 0.0, 0.0};
  bool pending_plane_available_{false};

  // Publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_meta_;
  rclcpp::Publisher<projection_msgs::msg::ProjectionContract>::SharedPtr pub_contract_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_plane_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /**
   * @brief Load point cloud from PLY file
   */
  void load_point_cloud() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPLYFile(ply_path_, *cloud) == -1) {
      throw std::runtime_error("Failed to load PLY file: " + ply_path_);
    }

    if (cloud->empty()) {
      throw std::runtime_error("Point cloud is empty");
    }

    // Convert to Eigen matrix
    cloud_raw_.resize(cloud->size(), 3);
    colors_raw_.resize(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i) {
      cloud_raw_(i, 0) = cloud->points[i].x;
      cloud_raw_(i, 1) = cloud->points[i].y;
      cloud_raw_(i, 2) = cloud->points[i].z;

      // RGB to BGR
      uint8_t r = cloud->points[i].r;
      uint8_t g = cloud->points[i].g;
      uint8_t b = cloud->points[i].b;
      colors_raw_[i] = cv::Vec3b(b, g, r);
    }
  }

  /**
   * @brief Publish raw point cloud
   */
  void publish_cloud_raw() {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;

    for (int i = 0; i < cloud_raw_.rows(); ++i) {
      pcl::PointXYZRGB p;
      p.x = cloud_raw_(i, 0);
      p.y = cloud_raw_(i, 1);
      p.z = cloud_raw_(i, 2);
      p.r = colors_raw_[i][2];
      p.g = colors_raw_[i][1];
      p.b = colors_raw_[i][0];
      pcl_cloud.push_back(p);
    }

    pcl::toROSMsg(pcl_cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = this->get_clock()->now();
    pub_cloud_raw_->publish(msg);
  }

  /**
   * @brief Plane subscription callback
   */
  void plane_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid plane message: expected 4 values, got %zu",
                  msg->data.size());
      return;
    }

    PlaneData plane{msg->data[0], msg->data[1], msg->data[2], msg->data[3]};

    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      pending_plane_ = plane;
      pending_plane_available_ = true;
    }

    cv_worker_.notify_one();
  }

  /**
   * @brief Pose subscription callback (relay)
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    try {
      // Extract camera position
      Vec3d C(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

      // Extract camera orientation and convert to rotation matrix
      Eigen::Quaterniond q(
          msg->pose.orientation.w,
          msg->pose.orientation.x,
          msg->pose.orientation.y,
          msg->pose.orientation.z
      );
      Mat3d R = q.toRotationMatrix();

      // Compute plane normal from camera forward (Z-axis)
      // Convention: camera Z-axis is optical axis (forward)
      Vec3d n_cam = R.col(2);  // Camera z-axis

      // Validate: plane normal should not be near zero
      if (n_cam.norm() < 0.99) {
        RCLCPP_ERROR(this->get_logger(), "Invalid camera rotation matrix");
        return;
      }

      // Plane origin: camera position + distance * normal
      Vec3d P = C + plane_distance_m_ * n_cam;

      // Compute horizontal and vertical physical dimensions from FOV
      double hfov_rad = hfov_deg_ * M_PI / 180.0;
      double vfov_rad = vfov_deg_ * M_PI / 180.0;
      double Wm = 2.0 * plane_distance_m_ * std::tan(hfov_rad / 2.0);
      double Hm = 2.0 * plane_distance_m_ * std::tan(vfov_rad / 2.0);

      // Store FOV-derived dimensions (will be published in contract)
      plane_width_m_ = Wm;
      plane_height_m_ = Hm;

      // Compute scale factors (px/m)
      sx_px_per_m_ = imgsz_px_ / Wm;
      sy_px_per_m_ = imgsz_px_ / Hm;

      // Pixel origins (center-based mapping)
      ox_px_ = imgsz_px_ / 2.0;
      oy_px_ = imgsz_px_ / 2.0;

      // === DETERMINISTIC YAW LOCK (T2b) ===
      // Compute orthonormal basis (u, v, n) with camera x-axis projection
      Vec3d e_x_cam = R.col(0);  // Camera x-axis (right direction)

      // Project camera x-axis onto plane to get u-axis
      Vec3d u_raw = e_x_cam - (e_x_cam.dot(n_cam)) * n_cam;

      // Handle degenerate case: if camera x-axis is parallel to plane normal
      double u_raw_norm = u_raw.norm();
      if (u_raw_norm < 1e-6) {
        // Fallback 1: Use camera y-axis
        Vec3d e_y_cam = R.col(1);
        u_raw = e_y_cam - (e_y_cam.dot(n_cam)) * n_cam;
        u_raw_norm = u_raw.norm();

        // Fallback 2: Use world up if camera y-axis also fails
        if (u_raw_norm < 1e-6) {
          Vec3d world_up(0.0, 0.0, 1.0);
          u_raw = world_up - (world_up.dot(n_cam)) * n_cam;
          u_raw_norm = u_raw.norm();

          // Final fallback: arbitrary orthogonal direction
          if (u_raw_norm < 1e-6) {
            // Pick orthogonal direction to n_cam
            if (std::abs(n_cam(0)) < 0.9) {
              u_raw = Vec3d(1.0, 0.0, 0.0) - (n_cam(0)) * n_cam;
            } else {
              u_raw = Vec3d(0.0, 1.0, 0.0) - (n_cam(1)) * n_cam;
            }
            u_raw_norm = u_raw.norm();
          }
        }
      }

      // Normalize u-axis
      Vec3d u = u_raw / u_raw_norm;

      // Compute v-axis via right-hand rule cross product
      Vec3d v = n_cam.cross(u);
      v = v.normalized();  // Normalize v-axis

      // Normalize n_cam as well (should already be normalized, but ensure it)
      Vec3d n = n_cam.normalized();

      // === VALIDATION: Orthonormality checks ===
      double u_len = u.norm();
      double v_len = v.norm();
      double n_len = n.norm();
      double u_dot_v = u.dot(v);
      double v_dot_n = v.dot(n);
      double n_dot_u = n.dot(u);

      bool ortho_valid = (std::abs(u_len - 1.0) < 1e-4) &&
                         (std::abs(v_len - 1.0) < 1e-4) &&
                         (std::abs(n_len - 1.0) < 1e-4) &&
                         (std::abs(u_dot_v) < 1e-4) &&
                         (std::abs(v_dot_n) < 1e-4) &&
                         (std::abs(n_dot_u) < 1e-4);

      // Ensure right-handedness: u × v = n
      Vec3d u_cross_v = u.cross(v);
      double rh_check = u_cross_v.dot(n);  // Should be ≈ 1.0
      bool right_handed = rh_check > 0.99;

      if (!ortho_valid || !right_handed) {
        RCLCPP_WARN(this->get_logger(),
            "Basis orthonormality check failed: |u|=%.4f, |v|=%.4f, |n|=%.4f, "
            "u·v=%.6f, v·n=%.6f, n·u=%.6f, u×v·n=%.4f",
            u_len, v_len, n_len, u_dot_v, v_dot_n, n_dot_u, rh_check);
      }

      // Store basis vectors
      basis_u_ = u;
      basis_v_ = v;
      basis_n_ = n;

      RCLCPP_DEBUG(this->get_logger(),
          "Basis locked: u=[%.4f,%.4f,%.4f], v=[%.4f,%.4f,%.4f], "
          "n=[%.4f,%.4f,%.4f] (ortho check: %.4f, RH: %.4f)",
          u(0), u(1), u(2), v(0), v(1), v(2), n(0), n(1), n(2),
          u_dot_v, rh_check);

      // Signal worker thread to recompute projection
      {
        std::lock_guard<std::mutex> lock(plane_mutex_);
        plane_center_ = P;
        plane_normal_ = n_cam;  // Will compute u, v in worker
        camera_rotation_ = R;   // Store for yaw lock computation
        plane_updated_ = true;
      }

      // Also update pending_plane for worker_loop (worker_mutex_)
      // Compute plane equation from calculated normal and center
      double a = n_cam(0);
      double b = n_cam(1);
      double c = n_cam(2);
      double d = -(n_cam.dot(P));  // d = -(n · P)
      {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        pending_plane_.a = a;
        pending_plane_.b = b;
        pending_plane_.c = c;
        pending_plane_.d = d;
        pending_plane_available_ = true;
      }
      cv_worker_.notify_one();

      RCLCPP_DEBUG(this->get_logger(),
          "Pose→Plane: P=[%.2f,%.2f,%.2f], n=[%.3f,%.3f,%.3f], "
          "FOV=%.1f°x%.1f°, Wm=%.2f, Hm=%.2f",
          P(0), P(1), P(2), n_cam(0), n_cam(1), n_cam(2),
          hfov_deg_, vfov_deg_, Wm, Hm);

      // Also store pose for other uses
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        last_pose_ = *msg;
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "pose_callback error: %s", e.what());
    }
  }

  /**
   * @brief Timer callback for publishing
   */
  void timer_callback() {
    std::lock_guard<std::mutex> lock(image_mutex_);

    if (!last_image_.empty()) {
      // Publish image
      auto img_msg = cv_bridge::CvImage(
          std_msgs::msg::Header(), "bgr8", last_image_);
      img_msg.header.stamp = this->get_clock()->now();
      img_msg.header.frame_id = "camera";
      pub_image_->publish(*img_msg.toImageMsg());

      // Publish pose if available
      last_pose_.header.stamp = this->get_clock()->now();
      pub_pose_->publish(last_pose_);

      // Save PNG if requested
      if (!save_png_path_.empty()) {
        cv::imwrite(save_png_path_, last_image_);
        RCLCPP_INFO(this->get_logger(), "Saved image to %s",
                    save_png_path_.c_str());
      }
    }
  }

  /**
   * @brief Worker thread loop for projection computation
   */
  void worker_loop() {
    while (!stop_worker_) {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      cv_worker_.wait(lock, [this] {
        return pending_plane_available_ || stop_worker_;
      });

      if (stop_worker_) break;

      // Get latest plane
      if (!pending_plane_available_) continue;

      PlaneData plane = pending_plane_;
      pending_plane_available_ = false;

      // Mark that we're starting projection
      is_projecting_ = true;

      lock.unlock();

      // Do the actual projection
      try {
        do_projection(plane);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Projection error: %s", e.what());
      }

      // Mark projection complete and continue waiting for next plane
      {
        std::lock_guard<std::mutex> lock2(worker_mutex_);
        is_projecting_ = false;
      }
    }
  }

  /**
   * @brief Perform projection for a given plane
   */
  void do_projection(const PlaneData& plane) {
    try {
      // Validate plane
      Vec3d n = validate_plane(plane.a, plane.b, plane.c, plane.d);
      Vec3d n_hat = normalize(n);

      // Choose up hint
      Vec3d up_hint = choose_up_hint(n_hat, user_up_hint_.get());

      // Build basis
      auto [t1, t2] = build_basis(n_hat, up_hint);

      // Use raw cloud for projection
      const MatX3d& cloud = cloud_raw_;
      const std::vector<cv::Vec3b>& colors = colors_raw_;

      // Project points
      MatX3d cloud_proj = project_points(cloud, n, plane.d);

      // Compute origin
      Vec3d origin = compute_origin(cloud_proj, n, plane.d, origin_mode_);

      // Map to UV
      auto [u, v] = map_uv(cloud_proj, origin, t1, t2);

      // Compute depth
      VecXd depth = compute_depth(cloud, n, plane.d, depth_mode_);

      // Compute image size (for metadata and bounds)
      auto [width_computed, height_computed, u_min, u_max, v_min, v_max] = compute_image_size(
          u, v, pixels_per_unit_, width_override_, height_override_,
          robust_range_, percentile_low_, percentile_high_);

      // === FORCE 1092×1092 OUTPUT (T3) ===
      // Override computed size to always output 1092×1092 px
      int width = imgsz_px_;      // Always 1092 (enforced in constructor)
      int height = imgsz_px_;     // Always 1092 (enforced in constructor)

      // Rasterize
      cv::Mat image = rasterize(u, v, depth, colors, width, height, u_min,
                                u_max, v_min, v_max, pixels_per_unit_,
                                depth_priority_far_, raster_mode_,
                                point_size_, this->get_logger());

      // Store result
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        last_image_ = image;
      }

      // Publish projection metadata for tracker
      // Format: [width, height,
      //          a, b, c, d,
      //          origin_x, origin_y, origin_z,
      //          t1_x, t1_y, t1_z,
      //          t2_x, t2_y, t2_z,
      //          u_min, u_max, v_min, v_max,
      //          stamp_sec, stamp_nanosec]
      std_msgs::msg::Float64MultiArray meta_msg;
      meta_msg.data.resize(20);
      meta_msg.data[0] = static_cast<double>(width);
      meta_msg.data[1] = static_cast<double>(height);
      meta_msg.data[2] = plane.a;
      meta_msg.data[3] = plane.b;
      meta_msg.data[4] = plane.c;
      meta_msg.data[5] = plane.d;
      meta_msg.data[6] = origin(0);
      meta_msg.data[7] = origin(1);
      meta_msg.data[8] = origin(2);
      meta_msg.data[9] = t1(0);
      meta_msg.data[10] = t1(1);
      meta_msg.data[11] = t1(2);
      meta_msg.data[12] = t2(0);
      meta_msg.data[13] = t2(1);
      meta_msg.data[14] = t2(2);
      meta_msg.data[15] = u_min;
      meta_msg.data[16] = u_max;
      meta_msg.data[17] = v_min;
      meta_msg.data[18] = v_max;

      // Timestamp
      auto now = this->get_clock()->now();
      meta_msg.data[19] = static_cast<double>(now.seconds());
      // Note: nanosec is stored as second element in geometry_utils.py extract_metadata
      // For now we pack both into the data array

      pub_meta_->publish(meta_msg);

      // === PUBLISH PROJECTION CONTRACT (T5) ===
      // Sync contract with image using values from pose_callback
      {
        std::lock_guard<std::mutex> lock(plane_mutex_);

        projection_msgs::msg::ProjectionContract contract_msg;

        // Header with timestamp
        contract_msg.header.stamp = now;
        contract_msg.header.frame_id = "world";

        // Plane geometry (from pose_callback via T2b)
        contract_msg.plane_center.x = plane_center_(0);
        contract_msg.plane_center.y = plane_center_(1);
        contract_msg.plane_center.z = plane_center_(2);

        // Basis vectors (from T2b yaw lock computation)
        contract_msg.basis_u.x = basis_u_(0);
        contract_msg.basis_u.y = basis_u_(1);
        contract_msg.basis_u.z = basis_u_(2);

        contract_msg.basis_v.x = basis_v_(0);
        contract_msg.basis_v.y = basis_v_(1);
        contract_msg.basis_v.z = basis_v_(2);

        contract_msg.basis_n.x = basis_n_(0);
        contract_msg.basis_n.y = basis_n_(1);
        contract_msg.basis_n.z = basis_n_(2);

        // Plane dimensions (from T2 FOV computation)
        contract_msg.plane_width_m = plane_width_m_;
        contract_msg.plane_height_m = plane_height_m_;

        // Image size (fixed 1092×1092 from T3)
        contract_msg.image_width_px = static_cast<uint32_t>(width);
        contract_msg.image_height_px = static_cast<uint32_t>(height);

        // Scale factors and pixel origins (from T2)
        contract_msg.sx_px_per_m = sx_px_per_m_;
        contract_msg.sy_px_per_m = sy_px_per_m_;
        contract_msg.ox_px = ox_px_;
        contract_msg.oy_px = oy_px_;

        // Pixel convention: center-based (0,0) at image center
        contract_msg.pixel_convention = "center";

        pub_contract_->publish(contract_msg);

        RCLCPP_DEBUG(this->get_logger(),
            "Contract published: center=[%.2f,%.2f,%.2f], "
            "dims=%.2fx%.2fm, scale=%.1fx%.1f px/m",
            plane_center_(0), plane_center_(1), plane_center_(2),
            plane_width_m_, plane_height_m_, sx_px_per_m_, sy_px_per_m_);
      }

      RCLCPP_DEBUG(this->get_logger(),
                   "Projection complete: plane=[%.3f,%.3f,%.3f,%.3f] "
                   "image=%dx%d",
                   plane.a, plane.b, plane.c, plane.d, width, height);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Projection computation failed: %s",
                   e.what());
      throw;
    }
  }
};

}  // namespace projection_plane

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<projection_plane::ProjectionPlaneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
