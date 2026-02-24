#include <atomic>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class VisionGuidedPick
{
public:
  explicit VisionGuidedPick(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    arm_(node_, node_->declare_parameter<std::string>("planning_group", "arm")),
    gripper_(node_, node_->declare_parameter<std::string>("gripper_group", "gripper")),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_),
    busy_(false)
  {
    object_pose_topic_ = node_->declare_parameter<std::string>("object_pose_topic", "/detected_object_pose");
    target_frame_ = node_->declare_parameter<std::string>("target_frame", "base_link");
    execute_ = node_->declare_parameter<bool>("execute", false);
    use_fixed_grasp_orientation_ = node_->declare_parameter<bool>("use_fixed_grasp_orientation", false);

    grasp_offset_x_ = node_->declare_parameter<double>("grasp_offset_x", 0.0);
    grasp_offset_y_ = node_->declare_parameter<double>("grasp_offset_y", 0.0);
    grasp_offset_z_ = node_->declare_parameter<double>("grasp_offset_z", 0.0);
    pregrasp_offset_x_ = node_->declare_parameter<double>("pregrasp_offset_x", 0.0);
    pregrasp_offset_y_ = node_->declare_parameter<double>("pregrasp_offset_y", 0.0);
    pregrasp_offset_z_ = node_->declare_parameter<double>("pregrasp_offset_z", 0.10);
    retreat_offset_x_ = node_->declare_parameter<double>("retreat_offset_x", 0.0);
    retreat_offset_y_ = node_->declare_parameter<double>("retreat_offset_y", 0.0);
    retreat_offset_z_ = node_->declare_parameter<double>("retreat_offset_z", 0.12);

    target_roll_ = node_->declare_parameter<double>("target_roll", 3.14159);
    target_pitch_ = node_->declare_parameter<double>("target_pitch", 0.0);
    target_yaw_ = node_->declare_parameter<double>("target_yaw", 0.0);
    velocity_scaling_ = node_->declare_parameter<double>("velocity_scaling", 0.2);
    acceleration_scaling_ = node_->declare_parameter<double>("acceleration_scaling", 0.2);
    planning_time_ = node_->declare_parameter<double>("planning_time", 8.0);
    min_target_z_ = node_->declare_parameter<double>("min_target_z", 0.03);
    max_xy_reach_ = node_->declare_parameter<double>("max_xy_reach", 0.80);
    min_new_target_distance_ = node_->declare_parameter<double>("min_new_target_distance", 0.05);
    min_confirmations_ = node_->declare_parameter<int>("min_confirmations", 3);
    confirmation_distance_tolerance_ =
      node_->declare_parameter<double>("confirmation_distance_tolerance", 0.02);
    confirmation_timeout_sec_ = node_->declare_parameter<double>("confirmation_timeout_sec", 1.0);

    arm_.setMaxVelocityScalingFactor(velocity_scaling_);
    arm_.setMaxAccelerationScalingFactor(acceleration_scaling_);
    arm_.setPlanningTime(planning_time_);
    arm_.setNumPlanningAttempts(5);
    arm_.setGoalPositionTolerance(0.01);
    arm_.setGoalOrientationTolerance(0.20);

    target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/vision_guided_pick/grasp_pose", 10);
    pregrasp_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/vision_guided_pick/pregrasp_pose", 10);
    retreat_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/vision_guided_pick/retreat_pose", 10);

    object_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      object_pose_topic_, 10,
      std::bind(&VisionGuidedPick::objectPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(),
      "vision_guided_pick ready. topic=%s target_frame=%s execute=%s",
      object_pose_topic_.c_str(), target_frame_.c_str(), execute_ ? "true" : "false");
  }

private:
  bool planAndMaybeExecutePose(const geometry_msgs::msg::PoseStamped & pose, const std::string & stage_name)
  {
    arm_.setStartStateToCurrentState();
    arm_.setPoseTarget(pose);

    MoveGroupInterface::Plan plan;
    const bool planned = (arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed for stage '%s'.", stage_name.c_str());
      arm_.clearPoseTargets();
      return false;
    }

    if (execute_) {
      const auto result = arm_.execute(plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed for stage '%s'.", stage_name.c_str());
        arm_.clearPoseTargets();
        return false;
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "Stage '%s' planned (dry-run).", stage_name.c_str());
    }

    arm_.clearPoseTargets();
    return true;
  }

  bool operateGripper(const std::string & named_target)
  {
    gripper_.setStartStateToCurrentState();
    gripper_.setNamedTarget(named_target);
    MoveGroupInterface::Plan plan;
    const bool planned = (gripper_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_ERROR(node_->get_logger(), "Gripper planning failed for '%s'.", named_target.c_str());
      return false;
    }
    if (execute_) {
      const auto result = gripper_.execute(plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper execution failed for '%s'.", named_target.c_str());
        return false;
      }
    }
    return true;
  }

  geometry_msgs::msg::Quaternion makeFixedOrientation() const
  {
    tf2::Quaternion q;
    q.setRPY(target_roll_, target_pitch_, target_yaw_);
    q.normalize();
    return tf2::toMsg(q);
  }

  static double pointDistance(
    const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
  }

  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "Still processing previous object pose, ignoring new one.");
      return;
    }

    geometry_msgs::msg::PoseStamped object_pose_base;
    try {
      object_pose_base = tf_buffer_.transform(*msg, target_frame_, tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_->get_logger(), "TF transform failed from %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      busy_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped grasp_pose = object_pose_base;
    grasp_pose.header.stamp = node_->now();
    grasp_pose.pose.position.x += grasp_offset_x_;
    grasp_pose.pose.position.y += grasp_offset_y_;
    grasp_pose.pose.position.z += grasp_offset_z_;
    if (use_fixed_grasp_orientation_) {
      grasp_pose.pose.orientation = makeFixedOrientation();
    }

    const double xy = std::hypot(grasp_pose.pose.position.x, grasp_pose.pose.position.y);
    if (grasp_pose.pose.position.z < min_target_z_ || xy > max_xy_reach_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rejected target pose: x=%.3f y=%.3f z=%.3f (xy=%.3f). Limits: min_z=%.3f max_xy=%.3f",
        grasp_pose.pose.position.x,
        grasp_pose.pose.position.y,
        grasp_pose.pose.position.z,
        xy,
        min_target_z_,
        max_xy_reach_);
      busy_ = false;
      return;
    }

    const auto now = node_->now();
    const bool confirmation_timed_out = has_pending_grasp_pose_ &&
      ((now - pending_grasp_pose_time_).seconds() > confirmation_timeout_sec_);
    if (!has_pending_grasp_pose_ || confirmation_timed_out) {
      has_pending_grasp_pose_ = true;
      pending_grasp_pose_ = grasp_pose.pose.position;
      pending_grasp_pose_time_ = now;
      confirmation_count_ = 1;
      RCLCPP_INFO(
        node_->get_logger(),
        "Detection confirmation 1/%d received. Waiting for stable repeats.",
        min_confirmations_);
      busy_ = false;
      return;
    }

    const double dist_to_pending = pointDistance(grasp_pose.pose.position, pending_grasp_pose_);
    if (dist_to_pending > confirmation_distance_tolerance_) {
      pending_grasp_pose_ = grasp_pose.pose.position;
      pending_grasp_pose_time_ = now;
      confirmation_count_ = 1;
      RCLCPP_INFO(
        node_->get_logger(),
        "Detection jump (%.3f m > %.3f m). Restarting confirmation window.",
        dist_to_pending,
        confirmation_distance_tolerance_);
      busy_ = false;
      return;
    }

    confirmation_count_++;
    // Track the latest accepted detection so small frame-to-frame drift is tolerated.
    pending_grasp_pose_ = grasp_pose.pose.position;
    pending_grasp_pose_time_ = now;
    if (confirmation_count_ < min_confirmations_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Detection confirmation %d/%d received. Waiting for stable repeats.",
        confirmation_count_,
        min_confirmations_);
      busy_ = false;
      return;
    }
    has_pending_grasp_pose_ = false;
    confirmation_count_ = 0;

    const double dist_to_last = pointDistance(grasp_pose.pose.position, last_successful_grasp_);
    if (has_last_successful_grasp_ && dist_to_last < min_new_target_distance_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Ignoring duplicate target (distance=%.3f m < min_new_target_distance=%.3f m).",
        dist_to_last,
        min_new_target_distance_);
      busy_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped pregrasp_pose = grasp_pose;
    pregrasp_pose.pose.position.x += pregrasp_offset_x_;
    pregrasp_pose.pose.position.y += pregrasp_offset_y_;
    pregrasp_pose.pose.position.z += pregrasp_offset_z_;

    geometry_msgs::msg::PoseStamped retreat_pose = grasp_pose;
    retreat_pose.pose.position.x += retreat_offset_x_;
    retreat_pose.pose.position.y += retreat_offset_y_;
    retreat_pose.pose.position.z += retreat_offset_z_;

    pregrasp_pose_pub_->publish(pregrasp_pose);
    target_pose_pub_->publish(grasp_pose);
    retreat_pose_pub_->publish(retreat_pose);

    if (!operateGripper("open")) {
      busy_ = false;
      return;
    }
    if (!planAndMaybeExecutePose(pregrasp_pose, "pregrasp")) {
      // Fallback: use current EE orientation for better IK reachability.
      const auto ee_now = arm_.getCurrentPose().pose.orientation;
      pregrasp_pose.pose.orientation = ee_now;
      grasp_pose.pose.orientation = ee_now;
      retreat_pose.pose.orientation = ee_now;
      RCLCPP_WARN(
        node_->get_logger(),
        "Retrying pick stages with current EE orientation fallback.");
      if (!planAndMaybeExecutePose(pregrasp_pose, "pregrasp_fallback")) {
        busy_ = false;
        return;
      }
    }
    if (!planAndMaybeExecutePose(grasp_pose, "grasp")) {
      busy_ = false;
      return;
    }
    if (!operateGripper("close")) {
      busy_ = false;
      return;
    }
    if (!planAndMaybeExecutePose(retreat_pose, "retreat")) {
      busy_ = false;
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Vision-guided pick pipeline completed.");
    has_last_successful_grasp_ = true;
    last_successful_grasp_ = grasp_pose.pose.position;
    busy_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  MoveGroupInterface arm_;
  MoveGroupInterface gripper_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string object_pose_topic_;
  std::string target_frame_;
  bool execute_;
  bool use_fixed_grasp_orientation_;
  double grasp_offset_x_;
  double grasp_offset_y_;
  double grasp_offset_z_;
  double pregrasp_offset_x_;
  double pregrasp_offset_y_;
  double pregrasp_offset_z_;
  double retreat_offset_x_;
  double retreat_offset_y_;
  double retreat_offset_z_;
  double target_roll_;
  double target_pitch_;
  double target_yaw_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double planning_time_;
  double min_target_z_;
  double max_xy_reach_;
  double min_new_target_distance_;
  int min_confirmations_;
  double confirmation_distance_tolerance_;
  double confirmation_timeout_sec_;
  std::atomic<bool> busy_;
  bool has_pending_grasp_pose_{false};
  geometry_msgs::msg::Point pending_grasp_pose_;
  rclcpp::Time pending_grasp_pose_time_{0, 0, RCL_ROS_TIME};
  int confirmation_count_{0};
  bool has_last_successful_grasp_{false};
  geometry_msgs::msg::Point last_successful_grasp_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pregrasp_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr retreat_pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vision_guided_pick");
  auto app = std::make_shared<VisionGuidedPick>(node);
  rclcpp::spin(node);
  (void)app;
  rclcpp::shutdown();
  return 0;
}
