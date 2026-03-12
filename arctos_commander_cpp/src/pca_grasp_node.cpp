#include <atomic>
#include <cmath>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

inline geometry_msgs::msg::Quaternion toMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

}

class PCAGraspNode
{
public:
  explicit PCAGraspNode(const rclcpp::Node::SharedPtr & node)
  : node_(node), busy_(false)
  {
    pointcloud_topic_ = node_->declare_parameter<std::string>("pointcloud_topic", "/point_cloud");
    target_frame_ = node_->declare_parameter<std::string>("target_frame", "base_link");
    min_points_ = node_->declare_parameter<int>("min_points", 120);
    grasp_offset_x_ = node_->declare_parameter<double>("grasp_offset_x", 0.0);
    grasp_offset_y_ = node_->declare_parameter<double>("grasp_offset_y", 0.0);
    grasp_offset_z_ = node_->declare_parameter<double>("grasp_offset_z", 0.0);
    approach_offset_ = node_->declare_parameter<double>("approach_offset", -0.12);
    axis_length_ = node_->declare_parameter<double>("axis_length", 0.10);
    axis_width_ = node_->declare_parameter<double>("axis_width", 0.006);
    axis_head_width_ = node_->declare_parameter<double>("axis_head_width", 0.012);
    axis_head_length_ = node_->declare_parameter<double>("axis_head_length", 0.02);
    center_marker_scale_ = node_->declare_parameter<double>("center_marker_scale", 0.01);

    grasp_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pca_grasp/grasp_pose", 10);
    axis_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/pca_grasp/axis_markers", 10);

    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 10,
      std::bind(&PCAGraspNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(),
      "pca_grasp_node ready. topic=%s target_frame=%s min_points=%d",
      pointcloud_topic_.c_str(), target_frame_.c_str(), min_points_);
  }

private:
  bool estimatePcaframe(const sensor_msgs::msg::PointCloud2 & cloud,
    Eigen::Vector3d & center, Eigen::Quaterniond & orientation) const
  {
    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<size_t>(cloud.width * cloud.height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }
      points.emplace_back(*iter_x, *iter_y, *iter_z);
    }

    if (static_cast<int>(points.size()) < min_points_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "PCA input ignored: %ld finite points (minimum %d required).",
        points.size(), min_points_);
      return false;
    }

    center.setZero();
    for (const auto & p : points) {
      center += p;
    }
    center /= static_cast<double>(points.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto & p : points) {
      const Eigen::Vector3d d = p - center;
      covariance += d * d.transpose();
    }
    covariance /= static_cast<double>(points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      RCLCPP_ERROR(node_->get_logger(), "PCA eigen decomposition failed.");
      return false;
    }

    // Eigenvalues are sorted ascending. Use principal component as z-axis of grasp frame.
    const Eigen::Matrix3d eigvecs = solver.eigenvectors();
    const Eigen::Vector3d x_axis = eigvecs.col(0).normalized();
    const Eigen::Vector3d y_axis = eigvecs.col(1).normalized();
    const Eigen::Vector3d z_axis = eigvecs.col(2).normalized();

    Eigen::Matrix3d frame;
    frame <<
      x_axis.x(), y_axis.x(), z_axis.x(),
      x_axis.y(), y_axis.y(), z_axis.y(),
      x_axis.z(), y_axis.z(), z_axis.z();
    orientation = Eigen::Quaterniond(frame);
    if (orientation.norm() < 1e-6) {
      orientation = Eigen::Quaterniond::Identity();
    } else {
      orientation.normalize();
    }

    return true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "Still processing previous point cloud, ignoring new data.");
      return;
    }

    if (msg->header.frame_id != target_frame_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping cloud frame mismatch: received '%s' expected '%s'. "
        "Provide cloud in the target frame or extend node to transform externally.",
        msg->header.frame_id.c_str(), target_frame_.c_str());
      busy_ = false;
      return;
    }

    Eigen::Vector3d center;
    Eigen::Quaterniond orientation;

    if (!estimatePcaframe(*msg, center, orientation)) {
      busy_ = false;
      return;
    }

    const Eigen::Vector3d approach_axis = -Eigen::Vector3d(orientation.toRotationMatrix().col(2));
    const Eigen::Matrix3d frame = orientation.toRotationMatrix();
    const geometry_msgs::msg::PoseStamped grasp_pose_msg = buildPose(
      center + approach_axis * approach_offset_ +
      Eigen::Vector3d(grasp_offset_x_, grasp_offset_y_, grasp_offset_z_),
      orientation
    );
    grasp_pose_pub_->publish(grasp_pose_msg);
    publishAxisMarkers(center, frame);
    RCLCPP_INFO(node_->get_logger(), "Published PCA grasp pose.");
    busy_ = false;
  }

  static geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & p)
  {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    return point;
  }

  void publishAxisMarkers(const Eigen::Vector3d & center, const Eigen::Matrix3d & frame)
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.reserve(4);

    auto make_color = [](float r, float g, float b, float a) {
      std_msgs::msg::ColorRGBA c;
      c.r = r;
      c.g = g;
      c.b = b;
      c.a = a;
      return c;
    };

    const std::array<Eigen::Vector3d, 3> axes = {
      frame.col(0).normalized(),
      frame.col(1).normalized(),
      frame.col(2).normalized(),
    };
    const std::array<std_msgs::msg::ColorRGBA, 3> colors = {
      make_color(1.0f, 0.0f, 0.0f, 0.9f),
      make_color(0.0f, 1.0f, 0.0f, 0.9f),
      make_color(0.0f, 0.0f, 1.0f, 0.9f),
    };

    for (std::size_t i = 0; i < axes.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = target_frame_;
      marker.header.stamp = node_->now();
      marker.ns = "pca_axes";
      marker.id = static_cast<int32_t>(i);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = builtin_interfaces::msg::Duration{};
      marker.scale.x = axis_width_;
      marker.scale.y = axis_head_width_;
      marker.scale.z = axis_head_length_;
      marker.color = colors[i];

      marker.points.reserve(2);
      marker.points.push_back(toPoint(center));
      marker.points.push_back(toPoint(center + axes[i] * axis_length_));
      markers.markers.push_back(marker);
    }

    visualization_msgs::msg::Marker center_marker;
    center_marker.header.frame_id = target_frame_;
    center_marker.header.stamp = node_->now();
    center_marker.ns = "pca_axes";
    center_marker.id = 999;
    center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    center_marker.action = visualization_msgs::msg::Marker::ADD;
    center_marker.lifetime = builtin_interfaces::msg::Duration{};
    center_marker.pose.position = toPoint(center);
    center_marker.pose.orientation.w = 1.0;
    center_marker.scale.x = center_marker_scale_;
    center_marker.scale.y = center_marker_scale_;
    center_marker.scale.z = center_marker_scale_;
    center_marker.color = make_color(1.0f, 1.0f, 1.0f, 0.8f);
    markers.markers.push_back(center_marker);

    axis_marker_pub_->publish(markers);
  }

  geometry_msgs::msg::PoseStamped buildPose(const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = target_frame_;
    pose.header.stamp = node_->now();
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();
    pose.pose.orientation = toMsg(orientation);
    return pose;
  }

  rclcpp::Node::SharedPtr node_;

  std::string pointcloud_topic_;
  std::string target_frame_;
  int min_points_;
  double grasp_offset_x_;
  double grasp_offset_y_;
  double grasp_offset_z_;
  double approach_offset_;
  double axis_length_;
  double axis_width_;
  double axis_head_width_;
  double axis_head_length_;
  double center_marker_scale_;

  std::atomic<bool> busy_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr axis_marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pca_grasp_node");
  auto app = std::make_shared<PCAGraspNode>(node);
  rclcpp::spin(node);
  (void)app;
  rclcpp::shutdown();
  return 0;
}
