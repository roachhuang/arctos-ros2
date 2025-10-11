#ifndef ROBOTARM_INTERFACE_H
#define ROBOTARM_INTERFACE_H

#include <cmath>
#include <string>
#include <memory>

#include "robotarm_controller/robotarm_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "hardware_interface/hardware_info.hpp"
#include <hardware_interface/system_interface.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// #include <pluginlib/class_list_macros.hpp>

#include <vector>

namespace hw = hardware_interface; // alias once
namespace robotarm_controller
{
    // using hw::CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::hw::CallbackReturn;

    class RobotArmInterface : public hw::SystemInterface
    {
    public:
        RobotArmInterface();
        ~RobotArmInterface();

        // SystemInterface overrides
        hw::CallbackReturn on_init(const hw::HardwareInfo &info) override;
        hw::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hw::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // Lifecycle node overrides
        hw::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hw::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hw::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        hw::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hw::StateInterface> export_state_interfaces() override;
        std::vector<hw::CommandInterface> export_command_interfaces() override;       

        hw::CallbackReturn disconnect_hardware();

    private:
        std::unique_ptr<RobotArmDriver> driver_;
        int baud_rate_ = 115200;
        rclcpp::Time last_command_time_;
        std::size_t num_joints_;

        // Joint states (position, velocity, effort)
        std::vector<double> position_commands_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        const double homePositions[6] = {0.0, -78.51, 73.90, 0.0, -90.0, 0.0};
        const double lower_limit[8] = {-114.0, -81.0, -180.0, -180.0, -139.0, -180.0, -0.57, -0.57};
        const double upper_limit[8] = {114.0, 77.0, 70.0, 180.0, 139.0, 180, 1.09, 1.09};
        std::vector<double> prev_position_commands_;

        // Objects for logging
        std::shared_ptr<rclcpp::Logger> logger_;
        rclcpp::Clock::SharedPtr clock_;
    };
}
#endif