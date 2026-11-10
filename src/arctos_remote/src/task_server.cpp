#include <functional>
#include <memory>
#include <thread>

#include "arctos_fndm_interface/action/alexa.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"

class TaskServer : public rclcpp::Node
{
public:
    using Alexa = arctos_fndm_interface::action::Alexa;
    using GoalHandleAlexa = rclcpp_action::ServerGoalHandle<Alexa>;

    //   arctos_remote_PUBLIC
    explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("task_server", options)
    {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<Alexa>(
            this,
            "task_server",
            std::bind(&TaskServer::handle_goal, this, _1, _2),
            std::bind(&TaskServer::handle_cancel, this, _1),
            std::bind(&TaskServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Task server started");
    }

private:
    rclcpp_action::Server<Alexa>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Alexa::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->task_number);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleAlexa> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        // auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(share_from_this(),'gripper');
        arm_move_group.stop();
        // gripper_move_group.stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleAlexa> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleAlexa> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        // auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(share_from_this('gripper'));
        std::vector<double> arm_joint_goal;
        std::vector<double> gripper_joint_goal;
        switch (goal_handle->get_goal()->task_number)
        {
        case 1:
            arm_joint_goal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // gripper_joint_goal = {0.0, 0.0};
            break;
        case 0:
            arm_joint_goal = {0.0, -1.37, 1.29, 0.0, -1.57, 0.0};
            // gripper_joint_goal = {1.57, 1.57};
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Invalid task number");
            return;
        }
        bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
        // gripper_move_group.setJointValueTarget(gripper_joint_goal);
        if (arm_within_bounds)
        {
            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
            // moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
            bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
            // bool gripper_plan_success = gripper_move_group.plan(gripper_plan);
            if (arm_plan_success)
            {
                arm_move_group.move();
                // gripper_move_group.execute(gripper_plan);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to plan arm movement");
                // goal_handle->abort();
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Goal successed");
            auto result = std::make_shared<Alexa::Result>();
            result->success = true;
            goal_handle->succeed(result);
        }
        else
        {
            // RCLCPP_WARN(rclcpp::get_logger('rclcpp'), "out of planning bounds");
            return;
            // goal_handle->abort();
        }
    }
}; // class TaskServer

// RCLCPP_COMPONENTS_REGISTER_NODE(arctos_remote::TaskServer)
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TaskServer>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}