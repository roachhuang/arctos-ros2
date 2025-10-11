#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread> 

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello, this is a test file for moveit");
    
    auto node = rclcpp::Node::make_shared("test_moveit_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_thread = std::thread([&executor]() { executor.spin(); });   

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setPlanningTime(10.0);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning frame: %s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "End effector: %s", arm.getEndEffectorLink().c_str());

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // Joint space goals - use smaller values
    // std::vector<double> joints = {1.5, 0.5, 0.0, 1.5, 0.0, -0.7};
    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);
    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (success1)
    //     arm.execute(plan1);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to joint target %s", success1 ? "SUCCEEDED" : "FAILED");

    // Named goals
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (success2)
    //     arm.execute(plan2);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to home %s", success2 ? "SUCCEEDED" : "FAILED"); 
    
    // Pose goals
    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // convert Roll, Pitch, Yaw to quaternion
    q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = node->now();  // ADD THIS LINE
    // target_pose.pose.position.x = 0.7;  // 0.7
    // target_pose.pose.position.y = 0.0;
    // target_pose.pose.position.z = 0.4;
    target_pose.pose.position.x = 0.3;  // 0.7
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.3;
    // target_pose.pose.orientation.x = q.getX();
    // target_pose.pose.orientation.y = q.getY();
    // target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = 1.0; //q.getW();
    
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success3)
        arm.execute(plan3);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to pose target %s", success3 ? "SUCCEEDED" : "FAILED"); 
    
    // Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z += -0.2;
    waypoints.push_back(pose1);
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.y += 0.2;
    waypoints.push_back(pose2);
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.y += -0.2;
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);   
    if(fraction == 1)
    {        
        arm.execute(trajectory);
    }

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}