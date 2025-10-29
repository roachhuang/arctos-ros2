#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
#include <algorithm>
#include <thread>

namespace arctos_hardware_interface
{

    CallbackReturn ArctosHardwareInterface::on_init(const hw::HardwareComponentInterfaceParams &params)
    {
        can_driver_ = std::make_unique<ServoCanSimple>();
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // Initialize joint data
        num_joints_ = info_.joints.size();
        position_commands_.resize(num_joints_, 0.0);
        position_states_.resize(num_joints_, 0.0);
        velocity_states_.resize(num_joints_, 0.0);
        prev_position_commands_.resize(num_joints_, 0.0);

        // Load CAN interface and joint IDs from parameters
        can_iface_ = info_.hardware_parameters.at("can_interface");
        for (const auto &joint : info_.joints)
        {
            can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
            gear_ratios_.push_back(std::stod(joint.parameters.at("gear_ratio")));
        }
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                    "Initialized with %zu joints", num_joints_);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        std::string can_interface = info_.hardware_parameters["can_interface"];
        vel_ = std::stod(info_.hardware_parameters["vel"]);
        accel_ = std::stod(info_.hardware_parameters["accel"]);

        // Get CAN interface from parameters if available
        if (info_.hardware_parameters.find("can_interface") != info_.hardware_parameters.end())
        {
            can_interface = info_.hardware_parameters.at("can_interface");
        }
        can_driver_->connect(can_interface);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Activating hardware...");

        // Read initial positions
        for (size_t i = 0; i < num_joints_; ++i)
        {
            can_driver_->enableMotor(can_ids_[i], true);
            // auto enc = can_driver_->readEncoder(can_ids_[i]);
            // double counts = enc->first * 65536.0 + enc->second;
            // position_states_[i] = counts;
            // position_commands_[i] = counts;
            // prev_position_commands_[i] = counts;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Deactivating hardware...");
        can_driver_->deactive();
        return CallbackReturn::SUCCESS;
    }

    // CallbackReturn ArctosHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    // {
    //     (void)previous_state;
    //     RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Shutting down...");
    //     if (sock_ >= 0) close(sock_);
    //     return disconnect_hardware();
    // }

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

        position_states_= position_commands_;
        return hw::return_type::OK;


        std::vector<double> hw_positions;

        // Calculate velocity from position change
        double dt = period.seconds();
        // read encoder position from each servo
        for (size_t i = 0; i < num_joints_; ++i)
        {
            double prev_pos = position_states_[i];
            auto enc = can_driver_->readEncoder(can_ids_[i]);
            // convert encoder count to radians if you know counts_per_rev
            double counts = enc->first * 65536.0 + enc->second;
            position_states_[i] = (static_cast<double>(counts) / ENCODER_CPR) * 2 * M_PI / gear_ratios_[i];

            // position_states_[i] = counts * count_to_rad_;
            // }
            // auto spd = can_driver_->readSpeed(can_ids_[i]);
            // if (spd)
            //     velocity_states_[i] = (*spd) * rpm_to_radps_;
            velocity_states_[i] = (dt > 0) ? (position_states_[i] - prev_pos) / dt : 0.0;
        }

        return hw::return_type::OK;
    }

    hw::return_type ArctosHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Only send if commands changed significantly
        bool commands_changed = false;
        for (size_t i = 0; i < position_commands_.size(); ++i)
        {
            if (std::abs(position_commands_[i] - prev_position_commands_[i]) > 0.01)
            {
                commands_changed = true;
                break;
            }
        }

        if (!commands_changed)
        {
            return hw::return_type::OK;
        }

        try
        {
            // Send desired position to each servo
            for (size_t i = 0; i < num_joints_; ++i)
            {
                uint32_t target_counts = static_cast<uint32_t>((position_commands_[i] * gear_ratios_[i] / 2 * M_PI) * ENCODER_CPR);
                // int target_counts = static_cast<int>(position_commands_[i] * rad_to_count_);
                can_driver_->runPositionAbs(can_ids_[i], vel_, accel_, target_counts);
                prev_position_commands_ = position_commands_;
            }
            return hw::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Write command failed: %s", e.what());
            return hw::return_type::ERROR;
        }
    }

    // CallbackReturn ArctosHardwareInterface::disconnect_hardware()
    // {
    //     if (driver_ && driver_->is_connected())
    //     {
    //         try
    //         {
    //             driver_->deactivate();
    //             driver_->disconnect();
    //         }
    //         catch (const std::exception &e)
    //         {
    //             RCLCPP_WARN(rclcpp::get_logger("ArctosHardwareInterface"),
    //                         "Disconnect error: %s", e.what());
    //             return CallbackReturn::ERROR;
    //         }
    //     }
    //     return CallbackReturn::SUCCESS;
    //}

} // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hw::SystemInterface)