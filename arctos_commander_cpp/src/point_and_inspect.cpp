#include <atomic>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PointAndInspect
{
public:
  explicit PointAndInspect(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    arm_(node_, node_->declare_parameter<std::string>("planning_group", "arm")),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_),
    busy_(false)
  {
    input_topic_ = node_->declare_parameter<std::string>("input_topic", "/clicked_point");
    target_frame_ = node_->declare_parameter<std::string>("target_frame", "base_link");
    execute_ = node_->declare_parameter<bool>("execute", false);
    inspect_offset_x_ = node_->declare_parameter<double>("inspect_offset_x", 0.0);
    inspect_offset_y_ = node_->declare_parameter<double>("inspect_offset_y", 0.0);
    inspect_offset_z_ = node_->declare_parameter<double>("inspect_offset_z", 0.12);
    target_roll_ = node_->declare_parameter<double>("target_roll", 3.14159);
    target_pitch_ = node_->declare_parameter<double>("target_pitch", 0.0);
    target_yaw_ = node_->declare_parameter<double>("target_yaw", 0.0);
    velocity_scaling_ = node_->declare_parameter<double>("velocity_scaling", 0.2);
    acceleration_scaling_ = node_->declare_parameter<double>("acceleration_scaling", 0.2);
    planning_time_ = node_->declare_parameter<double>("planning_time", 6.0);

    arm_.setMaxVelocityScalingFactor(velocity_scaling_);
    arm_.setMaxAccelerationScalingFactor(acceleration_scaling_);
    arm_.setPlanningTime(planning_time_);

    target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/point_and_inspect/target_pose", 10);

    clicked_point_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      input_topic_, 10,
      std::bind(&PointAndInspect::clickedPointCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(),
      "point_and_inspect ready. topic=%s target_frame=%s execute=%s planning_frame=%s eef=%s",
      input_topic_.c_str(),
      target_frame_.c_str(),
      execute_ ? "true" : "false",
      arm_.getPlanningFrame().c_str(),
      arm_.getEndEffectorLink().c_str());
  }

private:
  void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "Still processing previous click, ignoring new point.");
      return;
    }

    geometry_msgs::msg::PointStamped point_in_target;
    try {
      point_in_target = tf_buffer_.transform(*msg, target_frame_, tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_->get_logger(), "TF transform failed from %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      busy_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = target_frame_;
    target_pose.header.stamp = node_->now();
    target_pose.pose.position.x = point_in_target.point.x + inspect_offset_x_;
    target_pose.pose.position.y = point_in_target.point.y + inspect_offset_y_;
    target_pose.pose.position.z = point_in_target.point.z + inspect_offset_z_;

    tf2::Quaternion q;
    q.setRPY(target_roll_, target_pitch_, target_yaw_);
    q.normalize();
    target_pose.pose.orientation = tf2::toMsg(q);

    target_pose_pub_->publish(target_pose);

    arm_.setStartStateToCurrentState();
    arm_.setPoseTarget(target_pose);

    MoveGroupInterface::Plan plan;
    const bool planned = (arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Planning failed for clicked point (%.3f, %.3f, %.3f) in %s",
        point_in_target.point.x, point_in_target.point.y, point_in_target.point.z, target_frame_.c_str());
      busy_ = false;
      return;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Plan succeeded to inspect pose at (%.3f, %.3f, %.3f)%s",
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z,
      execute_ ? ", executing..." : " (dry-run only)");

    if (execute_) {
      const auto result = arm_.execute(plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed.");
      }
    }

    busy_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  MoveGroupInterface arm_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_topic_;
  std::string target_frame_;
  bool execute_;
  double inspect_offset_x_;
  double inspect_offset_y_;
  double inspect_offset_z_;
  double target_roll_;
  double target_pitch_;
  double target_yaw_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double planning_time_;

  std::atomic<bool> busy_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_and_inspect");
  auto app = std::make_shared<PointAndInspect>(node);
  rclcpp::spin(node);
  (void)app;
  rclcpp::shutdown();
  return 0;
}
