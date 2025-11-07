#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
#include <algorithm>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace arctos_hardware_interface
{

    hardware_interface::CallbackReturn ArctosHardwareInterface::on_init(const hw::HardwareComponentInterfaceParams &params)
    {
        // can_driver_ = std::make_unique<ServoCanSimple>();
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        initializeJointData();
        // loadJointParameters();

        RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"),
                    "Initialized with %zu joints", num_joints_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void ArctosHardwareInterface::initializeJointData()
    {
        num_joints_ = info_.joints.size();
        // This flag controls the logic in read()
        is_homing_.resize(num_joints_, false);
        in1_.resize(num_joints_, false);
        in2_.resize(num_joints_, false);
        position_commands_.resize(num_joints_, 0.0);
        position_states_.resize(num_joints_, 0.0);
        velocity_states_.resize(num_joints_, 0.0);
        prev_position_commands_.resize(num_joints_, 0.0);
    }

    // void ArctosHardwareInterface::loadJointParameters()
    // {
    //     for (const auto &joint : info_.joints)
    //     {
    //         can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
    //         gear_ratios_.push_back(std::stod(joint.parameters.at(param_prefix + "motor_id", "gear_ratio")));
    //     }
    // }

    CallbackReturn ArctosHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"), "Configuring hardware interface...");

        loadHardwareParameters();

        if (!connectToCanInterface())
        {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    void ArctosHardwareInterface::loadHardwareParameters()
    {
        can_interface_ = info_.hardware_parameters.at("can_interface");
        vel_ = std::stod(info_.hardware_parameters.at("vel"));
        accel_ = std::stod(info_.hardware_parameters.at("accel"));
        // require_homing = std::stoi(info_.hardware_parameters.at("require_homing"));

        // Collect joint names for service initialization
        // std::vector<std::string> joint_names;
        // joint_names.reserve(info_.joints.size());
        // Process joints and their interfaces
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            can_ids_.push_back(std::stoi(info_.joints[i].parameters.at("can_id")));
            gear_ratios_.push_back(std::stod(info_.joints[i].parameters.at("gear_ratio")));
            is_homing_[i] = false; // IMPORTANT: require homing on configure
        }
    }

    bool ArctosHardwareInterface::connectToCanInterface()
    {
        if (can_driver_.connect(can_interface_))
        {
            RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"),
                        "Connected to CAN interface: %s", can_interface_.c_str());
            return true;
        }

        RCLCPP_ERROR(rclcpp::get_logger("ArctosInterface"),
                     "Failed to connect to CAN interface: %s", can_interface_.c_str());
        return false;
    }

    CallbackReturn ArctosHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"), "Activating hardware...");
        // for (size_t i = 0; i < info_.joints.size(); i++)
        // for (size_t i = 1; i < 3; i++)
        // {

        //     // const auto &joint_name = info_.joints[i].name;
        //     if (!can_driver_.enableMotor(can_ids_[i], true))
        //     {
        //         RCLCPP_ERROR(rclcpp::get_logger("ArctosInterface"),
        //                      "Failed to enable motor %zu", i);
        //         return CallbackReturn::ERROR;
        //     }

        //     // }
        // }
        return CallbackReturn::SUCCESS;
    }

    // void ArctosHardwareInterface::readInitialPositions()
    // {
    //     // TODO: Read all joints when all motors are installed
    //     const size_t active_joints = std::min(num_joints_, size_t(2));

    //     for (size_t i = 0; i < active_joints; ++i)
    //     {
    //         auto position_counts = can_driver_.readAbsolutePosition(can_ids_[i]);
    //         if (position_counts.has_value())
    //         {
    //             double pos = countsToRadians(position_counts.value(), gear_ratios_[i]);
    //             pos = std::fmod(pos + M_PI, 2.0 * M_PI) - M_PI; // Normalize
    //             position_states_[i] = pos;
    //             position_commands_[i] = pos; // Initialize commands to current position
    //         }
    //         else
    //         {
    //             // Initialize to zero if CAN read fails
    //             position_states_[i] = 0.0;
    //             position_commands_[i] = 0.0;
    //         }
    //     }

    //     // Initialize inactive joints
    //     for (size_t i = active_joints; i < num_joints_; ++i)
    //     {
    //         position_states_[i] = 0.0;
    //         position_commands_[i] = 0.0;
    //     }
    // }

    CallbackReturn ArctosHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"), "Deactivating hardware...");
        for (auto can_id : can_ids_)
        {
            can_driver_.enableMotor(can_id, false);
        }
        can_driver_.deactive();
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

        RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"),
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

        RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"),
                    "Exported %zu command interfaces", cmds.size());
        return cmds;
    }

    hw::return_type ArctosHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        //     position_states_ = position_commands_;
        //     return hardware_interface::return_type::OK;

        double dt = period.seconds();
        auto positions = can_driver_.getPositions();
        // RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"),
        //             "Exported %zu postions", positions.size());

        // Use actual number of joints, not hardcoded 6
        for (size_t i = 0; i < num_joints_ && i < positions.size(); ++i)
        {
            if (is_homing_[i])
            {
                // If homing, we EXPECT to hit the switch.
                if (can_driver_.getIN1State(i))
                {
                    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
                                "Joint '%s' homing complete. Switch triggered.", info_.joints[i].name.c_str());

                    // This is the "zeroing" event.
                    // TODO: Tell the MKS driver to set its internal "0"
                    // mks_driver_->setEncoderZero(motor_id);

                    // Stop the homing flag
                    is_homing_[i] = false;
                }
            }
            else
            {
                if (can_driver_.getIN1State(i) || can_driver_.getIN2State(i))
                {
                    RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
                                 "HARDWARE LIMIT HIT on joint '%s' unexpectedly!", info_.joints[i].name.c_str());

                    // This is a critical safety stop.
                    can_driver_.EmergencyStop(can_ids_[i]);
                    // Tell ros2_control to stop everything.
                    return hardware_interface::return_type::ERROR;
                }
            }
            double prev = position_states_[i];
            double rad = countsToRadians(positions[i], gear_ratios_[i]);
            rad = std::fmod(rad + M_PI, 2.0 * M_PI) - M_PI;
            position_states_[i] = rad;

            // Calculate velocity with proper bounds checking
            if (dt > VELOCITY_EPSILON)
            {
                velocity_states_[i] = (rad - prev) / dt;
            }
            else
            {
                velocity_states_[i] = 0.0;
            }
        }
        // initialize inactive joints
        for (size_t i = positions.size(); i < num_joints_; ++i)
        {
            position_states_[i] = 0.0;
            velocity_states_[i] = 0.0;
        }
        return hardware_interface::return_type::OK;
    }

    void ArctosHardwareInterface::updateJointVelocity(size_t joint_index, double prev_position, double dt)
    {
        if (dt > VELOCITY_EPSILON)
        {
            double new_velocity = (position_states_[joint_index] - prev_position) / dt;
            // Smooth velocity with EMA (0.7 + 0.3 = 1.0) to prevent spikes
            if (std::isfinite(new_velocity))
            {
                velocity_states_[joint_index] = 0.7 * velocity_states_[joint_index] + 0.3 * new_velocity;
            } // Keep previous velocity if new_velocity is invalid
        }
        else
        {
            velocity_states_[joint_index] = 0.0;
        }
    }

    hw::return_type ArctosHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        // sendPositionCommands();
        // prev_position_commands_ = position_commands_;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            // Only send command if NOT homing (homing is a special "search" move)
            if (!is_homing_[i])
            {
                double cmd = position_commands_[i];
                double pos = position_states_[i];
                if (std::fabs(cmd - pos) < POSITION_CHANGE_THRESHOLD)
                {
                    cmd = pos;
                }
                int32_t target_pos = radiansToCounts(cmd, gear_ratios_[i]);
                can_driver_.runPositionAbs(can_ids_[i], vel_, accel_, target_pos);
            }
        }

        return hardware_interface::return_type::OK;
    }

    bool ArctosHardwareInterface::hasCommandsChanged() const
    {
        for (size_t i = 0; i < position_commands_.size(); ++i)
        {
            if (std::fabs(position_commands_[i] - position_states_[i]) > POSITION_CHANGE_THRESHOLD)
            {
                return true;
            }
        }
        return false;
    }

    // void ArctosHardwareInterface::sendPositionCommands()
    // {
    //     static size_t cmd_counter = 0;
    //     RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"),
    //                 "=== TRAJECTORY COMMAND %zu ===", ++cmd_counter);

    //     for (size_t i = 0; i < num_joints_; ++i)
    //     {
    //         int32_t target_counts = radiansToCounts(position_commands_[i], gear_ratios_[i]);

    //         RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"),
    //                     "Joint %zu: cmd=%.3f rad, counts=%d", i, position_commands_[i], target_counts);

    //         can_driver_.runPositionAbs(can_ids_[i], vel_, accel_, target_counts);
    //     }
    // }

} // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hardware_interface::SystemInterface)