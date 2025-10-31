#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
#include "arctos_hardware_interface/servo_can.hpp"
#include <algorithm>
#include <thread>
#include <cmath>

namespace arctos_hardware_interface
{

    CallbackReturn ArctosHardwareInterface::on_init(const hw::HardwareComponentInterfaceParams &params)
    {
        can_driver_ = std::make_unique<ServoCanSimple>();
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        initializeJointData();
        loadJointParameters();

        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                    "Initialized with %zu joints", num_joints_);
        return CallbackReturn::SUCCESS;
    }

    void ArctosHardwareInterface::initializeJointData()
    {
        num_joints_ = info_.joints.size();
        position_commands_.resize(num_joints_, 0.0);
        position_states_.resize(num_joints_, 0.0);
        velocity_states_.resize(num_joints_, 0.0);
        prev_position_commands_.resize(num_joints_, 0.0);
    }

    void ArctosHardwareInterface::loadJointParameters()
    {
        for (const auto &joint : info_.joints)
        {
            can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
            gear_ratios_.push_back(std::stod(joint.parameters.at("gear_ratio")));
        }
    }

    CallbackReturn ArctosHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Configuring hardware interface...");

        if (!loadHardwareParameters())
        {
            return CallbackReturn::ERROR;
        }

        if (!connectToCanInterface())
        {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    bool ArctosHardwareInterface::loadHardwareParameters()
    {
        try
        {
            can_interface_ = info_.hardware_parameters.at("can_interface");
            vel_ = std::stod(info_.hardware_parameters.at("vel"));
            accel_ = std::stod(info_.hardware_parameters.at("accel"));
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Failed to load parameters: %s", e.what());
            return false;
        }
    }

    bool ArctosHardwareInterface::connectToCanInterface()
    {
        if (can_driver_->connect(can_interface_))
        {
            RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                        "Connected to CAN interface: %s", can_interface_.c_str());
            return true;
        }

        RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                     "Failed to connect to CAN interface: %s", can_interface_.c_str());
        return false;
    }

    CallbackReturn ArctosHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Activating hardware...");

        readInitialPositions();
        return CallbackReturn::SUCCESS;
    }

    void ArctosHardwareInterface::readInitialPositions()
    {
        // TODO: Read all joints when all motors are installed
        const size_t active_joints = std::min(num_joints_, size_t(2));

        for (size_t i = 0; i < active_joints; ++i)
        {
            auto position_counts = can_driver_->readAbsolutePosition(can_ids_[i]);
            if (position_counts.has_value())
            {
                position_states_[i] = countsToRadians(position_counts.value(), gear_ratios_[i]);
            }
        }
    }

    CallbackReturn ArctosHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Deactivating hardware...");
        can_driver_->deactive();
        return CallbackReturn::SUCCESS;
    }

    std::vector<hw::StateInterface> ArctosHardwareInterface::export_state_interfaces()
    {
        std::vector<hw::StateInterface> state_interfaces;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name,
                hw::HW_IF_POSITION,
                &position_states_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name,
                hw::HW_IF_VELOCITY,
                &velocity_states_[i]);
        }

        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                    "Exported %zu state interfaces", state_interfaces.size());
        return state_interfaces;
    }

    std::vector<hw::CommandInterface> ArctosHardwareInterface::export_command_interfaces()
    {
        std::vector<hw::CommandInterface> cmds;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            cmds.emplace_back(
                info_.joints[i].name,
                hw::HW_IF_POSITION,
                &position_commands_[i]);
        }

        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                    "Exported %zu command interfaces", cmds.size());
        return cmds;
    }

    hw::return_type ArctosHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        double dt = period.seconds();

        for (size_t i = 0; i < num_joints_; ++i)
        {
            double prev_position = position_states_[i];

            if (!readJointPosition(i))
            {
                // Fallback to command position if read fails
                position_states_[i] = position_commands_[i];
            }

            updateJointVelocity(i, prev_position, dt);
        }

        return hw::return_type::OK;
    }

    bool ArctosHardwareInterface::readJointPosition(size_t joint_index)
    {
        auto position_counts = can_driver_->readAbsolutePosition(can_ids_[joint_index]);
        if (position_counts.has_value())
        {
            position_states_[joint_index] = countsToRadians(position_counts.value(), gear_ratios_[joint_index]);
            return true;
        }
        return false;
    }

    void ArctosHardwareInterface::updateJointVelocity(size_t joint_index, double prev_position, double dt)
    {
        velocity_states_[joint_index] = (dt > VELOCITY_EPSILON) ? 
            (position_states_[joint_index] - prev_position) / dt : 0.0;
    }

    hw::return_type ArctosHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!hasCommandsChanged())
        {
            return hw::return_type::OK;
        }

        try
        {
            sendPositionCommands();
            prev_position_commands_ = position_commands_;
            return hw::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Write command failed: %s", e.what());
            return hw::return_type::ERROR;
        }
    }

    bool ArctosHardwareInterface::hasCommandsChanged() const
    {
        for (size_t i = 0; i < position_commands_.size(); ++i)
        {
            if (std::abs(position_commands_[i] - prev_position_commands_[i]) > POSITION_CHANGE_THRESHOLD)
            {
                return true;
            }
        }
        return false;
    }

    void ArctosHardwareInterface::sendPositionCommands()
    {
        for (size_t i = 0; i < num_joints_; ++i)
        {
            int32_t target_counts = radiansToCount(position_commands_[i], gear_ratios_[i]);

            RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                        "Joint %zu: cmd=%.3f rad, counts=%d", i, position_commands_[i], target_counts);

            can_driver_->runPositionAbs(can_ids_[i], vel_, accel_, target_counts);
        }
    }

} // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hw::SystemInterface)