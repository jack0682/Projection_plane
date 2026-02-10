#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
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
    this->declare_parameter("enable_downsample", false);
    this->declare_parameter("voxel_leaf_size", 0.01);
    this->declare_parameter("use_downsample_for_projection", true);
    this->declare_parameter("publish_downsample_cloud", true);
    this->declare_parameter("up_hint_x", std::nan(""));
    this->declare_parameter("up_hint_y", std::nan(""));
    this->declare_parameter("up_hint_z", std::nan(""));
    this->declare_parameter("save_png_path", "");

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
    enable_downsample_ = this->get_parameter("enable_downsample").as_bool();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    use_downsample_for_projection_ =
        this->get_parameter("use_downsample_for_projection").as_bool();
    publish_downsample_cloud_ =
        this->get_parameter("publish_downsample_cloud").as_bool();
    save_png_path_ = this->get_parameter("save_png_path").as_string();

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

    if (publish_downsample_cloud_) {
      pub_cloud_render_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/projection/cloud_render", rclcpp::QoS(1).transient_local());
    }

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/projection/image", rclcpp::QoS(10));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/projection/camera_pose", rclcpp::QoS(10));

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
    if (publish_downsample_cloud_) {
      publish_cloud_render();
    }

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
  bool enable_downsample_;
  double voxel_leaf_size_;
  bool use_downsample_for_projection_;
  bool publish_downsample_cloud_;
  std::string save_png_path_;
  std::unique_ptr<Vec3d> user_up_hint_;

  // Point cloud data (cached)
  MatX3d cloud_raw_;
  std::vector<cv::Vec3b> colors_raw_;
  MatX3d cloud_render_;  // Downsampled if enabled

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_render_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
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

    // Handle downsampling if enabled
    if (enable_downsample_) {
      downsample_cloud();
    } else {
      cloud_render_ = cloud_raw_;
    }
  }

  /**
   * @brief Downsample point cloud using voxel grid
   */
  void downsample_cloud() {
    // Use PCL voxel grid filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw_pcl(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert Eigen matrix back to PCL for filtering
    for (int i = 0; i < cloud_raw_.rows(); ++i) {
      pcl::PointXYZRGB p;
      p.x = cloud_raw_(i, 0);
      p.y = cloud_raw_(i, 1);
      p.z = cloud_raw_(i, 2);
      p.r = colors_raw_[i][2];
      p.g = colors_raw_[i][1];
      p.b = colors_raw_[i][0];
      cloud_raw_pcl->push_back(p);
    }

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_raw_pcl);
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    vg.filter(*cloud_downsampled);

    // Convert back to Eigen
    cloud_render_.resize(cloud_downsampled->size(), 3);
    std::vector<cv::Vec3b> colors_render(cloud_downsampled->size());

    for (size_t i = 0; i < cloud_downsampled->size(); ++i) {
      cloud_render_(i, 0) = cloud_downsampled->points[i].x;
      cloud_render_(i, 1) = cloud_downsampled->points[i].y;
      cloud_render_(i, 2) = cloud_downsampled->points[i].z;

      uint8_t r = cloud_downsampled->points[i].r;
      uint8_t g = cloud_downsampled->points[i].g;
      uint8_t b = cloud_downsampled->points[i].b;
      colors_render[i] = cv::Vec3b(b, g, r);
    }

    RCLCPP_INFO(this->get_logger(),
                "Downsampled cloud: %zu -> %zu points", cloud_raw_.rows(),
                cloud_render_.rows());
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
   * @brief Publish downsampled point cloud
   */
  void publish_cloud_render() {
    if (!publish_downsample_cloud_) return;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;

    // Create temporary colors for render cloud
    std::vector<cv::Vec3b> colors_render(cloud_render_.rows());
    if (!colors_raw_.empty()) {
      // Use raw cloud colors if available (simplified)
      for (size_t i = 0; i < std::min((size_t)cloud_render_.rows(),
                                      colors_raw_.size());
           ++i) {
        colors_render[i] = colors_raw_[i];
      }
    }

    for (int i = 0; i < cloud_render_.rows(); ++i) {
      pcl::PointXYZRGB p;
      p.x = cloud_render_(i, 0);
      p.y = cloud_render_(i, 1);
      p.z = cloud_render_(i, 2);
      p.r = colors_render[i][2];
      p.g = colors_render[i][1];
      p.b = colors_render[i][0];
      pcl_cloud.push_back(p);
    }

    pcl::toROSMsg(pcl_cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = this->get_clock()->now();
    pub_cloud_render_->publish(msg);
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
    std::lock_guard<std::mutex> lock(image_mutex_);
    last_pose_ = *msg;
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

      // Select cloud for projection
      const MatX3d& cloud =
          (enable_downsample_ && use_downsample_for_projection_) ? cloud_render_
                                                                  : cloud_raw_;
      const std::vector<cv::Vec3b>& colors =
          (enable_downsample_ && use_downsample_for_projection_)
              ? std::vector<cv::Vec3b>()
              : colors_raw_;

      // Project points
      MatX3d cloud_proj = project_points(cloud, n, plane.d);

      // Compute origin
      Vec3d origin = compute_origin(cloud_proj, n, plane.d, origin_mode_);

      // Map to UV
      auto [u, v] = map_uv(cloud_proj, origin, t1, t2);

      // Compute depth
      VecXd depth = compute_depth(cloud, n, plane.d, depth_mode_);

      // Compute image size
      auto [width, height, u_min, u_max, v_min, v_max] = compute_image_size(
          u, v, pixels_per_unit_, width_override_, height_override_,
          robust_range_, percentile_low_, percentile_high_);

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
