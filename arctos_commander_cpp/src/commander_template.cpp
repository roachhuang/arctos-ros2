#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <example_interfaces/msg/bool.hpp>
#include <functional>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;

// composer
class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper");
        arm_->setMaxVelocityScalingFactor(0.6);
        arm_->setMaxAccelerationScalingFactor(0.6);
        arm_->setPlanningTime(10.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning frame: %s", arm_->getPlanningFrame().c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "End effector: %s", arm_->getEndEffectorLink().c_str());

        open_gripper_sub = node_->create_subscription<Bool>(
            "open_gripper", 10, std::bind(&Commander::OpenGripperCallback, this, std::placeholders::_1));
      
    }

    void moveToNamedTarget(const std::string &target_name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(target_name);
        planAndExecute(arm_);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to %s", target_name.c_str());
    }

    void moveToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to joint target");
    }   

    void moveToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path = false) 
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw); // convert Roll, Pitch, Yaw to quaternion
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.header.stamp = node_->now();        
        target_pose.pose.position.x = x;  
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();
        arm_->setStartStateToCurrentState();
        arm_->setPoseTarget(target_pose);
        if (!cartesian_path){
            planAndExecute(arm_);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to pose target");
        } else {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
            // double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
            if (fraction == 1){
                arm_->execute(trajectory);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to pose target");
            }
        }
    }

    // void moveToPoseTarget(const geometry_msgs::msg::PoseStamped &pose)
    // {
    //     arm_->setStartStateToCurrentState();
    //     arm_->setPoseTarget(pose);
    //     planAndExecute(arm_);
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move to pose target");
    // }
    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("open");
        planAndExecute(gripper_);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Open gripper");
    }   
    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("close");
        planAndExecute(gripper_);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Close gripper");
    }
private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
            interface->execute(plan);
    }  
    void OpenGripperCallback(const Bool::SharedPtr msg)
    {
        if (msg->data)
            openGripper();
        else
            closeGripper(); 
    }
    void CloseGripperCallback(const Bool::SharedPtr msg)
    {
        if (msg->data)
            closeGripper();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander_node");
    auto commander= Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
