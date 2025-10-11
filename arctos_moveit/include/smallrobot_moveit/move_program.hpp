#pragma once

// this is necessary to plan and execute motions with MoveIt
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>
// #include <rclcpp/rclcpp.hpp>

namespace arctos_moveit {

// Utility function to create a default target pose
inline geometry_msgs::msg::Pose make_default_target_pose() {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
}

} // namespace arctos_moveit