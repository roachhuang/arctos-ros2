#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace
{
    constexpr double VELOCITY_SCALING = 0.1;
    constexpr double ACCELERATION_SCALING = 0.1;
    constexpr double PLANNING_TIME = 10.0;
    constexpr double CARTESIAN_STEP_SIZE = 0.01;
    
    constexpr double TARGET_X = 0.3;
    constexpr double TARGET_Y = 0.0;
    constexpr double TARGET_Z = 0.3;
    
    constexpr double WAYPOINT_Z_OFFSET = -0.2;
    constexpr double WAYPOINT_Y_OFFSET = 0.2;
    
    const std::string LOGGER_NAME = "test_moveit";
}

class MoveitTester
{
public:
    MoveitTester()
        : node_(rclcpp::Node::make_shared("test_moveit_node")),
          arm_(node_, "arm"),
          gripper_(node_, "gripper")
    {
        setupExecutor();
        configureArm();
        logArmInfo();
    }

    ~MoveitTester()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

    void runTests()
    {
        testPoseTarget();
        testCartesianPath();
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface arm_;
    moveit::planning_interface::MoveGroupInterface gripper_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;

    void setupExecutor()
    {
        executor_.add_node(node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });
    }

    void configureArm()
    {
        arm_.setMaxVelocityScalingFactor(VELOCITY_SCALING);
        arm_.setMaxAccelerationScalingFactor(ACCELERATION_SCALING);
        arm_.setPlanningTime(PLANNING_TIME);
    }

    void logArmInfo()
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                   "Planning frame: %s", arm_.getPlanningFrame().c_str());
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                   "End effector: %s", arm_.getEndEffectorLink().c_str());
    }

    void testPoseTarget()
    {
        geometry_msgs::msg::PoseStamped target_pose = createTargetPose();
        
        arm_.setStartStateToCurrentState();
        arm_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            arm_.execute(plan);
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                   "Pose target motion %s", success ? "SUCCEEDED" : "FAILED");
    }

    void testCartesianPath()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints = createWaypoints();
        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = arm_.computeCartesianPath(waypoints, CARTESIAN_STEP_SIZE, trajectory);
        
        if (fraction == 1.0)
        {
            arm_.execute(trajectory);
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Cartesian path SUCCEEDED");
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), 
                       "Cartesian path only %.2f%% complete", fraction * 100.0);
        }
    }

    geometry_msgs::msg::PoseStamped createTargetPose()
    {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.header.stamp = node_->now();
        target_pose.pose.position.x = TARGET_X;
        target_pose.pose.position.y = TARGET_Y;
        target_pose.pose.position.z = TARGET_Z;
        target_pose.pose.orientation.w = 1.0;
        
        return target_pose;
    }

    std::vector<geometry_msgs::msg::Pose> createWaypoints()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose current_pose = arm_.getCurrentPose().pose;
        
        // Waypoint 1: Move down
        geometry_msgs::msg::Pose waypoint1 = current_pose;
        waypoint1.position.z += WAYPOINT_Z_OFFSET;
        waypoints.push_back(waypoint1);
        
        // Waypoint 2: Move sideways
        geometry_msgs::msg::Pose waypoint2 = waypoint1;
        waypoint2.position.y += WAYPOINT_Y_OFFSET;
        waypoints.push_back(waypoint2);
        
        // Waypoint 3: Return and move up
        geometry_msgs::msg::Pose waypoint3 = waypoint2;
        waypoint3.position.y -= WAYPOINT_Y_OFFSET;
        waypoint3.position.z -= WAYPOINT_Z_OFFSET;
        waypoints.push_back(waypoint3);
        
        return waypoints;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Starting Arctos MoveIt test...");
    
    try
    {
        MoveitTester tester;
        tester.runTests();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Test failed: %s", e.what());
        return 1;
    }

    return 0;
}