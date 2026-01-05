#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
#include <algorithm>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
unified indexing scheme applied to both your MksServoDriver and ArctosHardwareInterface, so CAN IDs and internal vectors can never get out of sync again
Externally: always use CAN ID (1..N). Internally: always index vectors as idx = can_id_to_index(id).
*/
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ArctosInterface");

namespace arctos_hardware_interface
{
    hardware_interface::CallbackReturn ArctosHardwareInterface::on_init(const hw::HardwareComponentInterfaceParams &params)
    {
        // can_driver_ = std::make_unique<ServoCanSimple>();
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Use actual joint count from URDF/robot description instead of hardcoded value
        // num_joints_ = info_.joints.size();
        num_joints_ = DOF;
        // This flag controls the logic in read()
        is_homing_.resize(num_joints_, false);
        // in1_.resize(num_joints_, false);
        // in2_.resize(num_joints_, false);
        position_commands_.assign(num_joints_, 0.0);
        position_states_.assign(num_joints_, 0.0);
        velocity_states_.assign(num_joints_, 0.0);
        // effort_states_.assign(num_joints_, 0.0);
        // loadJointParameters();

        RCLCPP_INFO(LOGGER,
                    "Initialized with %zu joints", num_joints_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(LOGGER, "Configuring hardware interface...");

        loadHardwareParameters();

        if (!can_driver_.connect(can_interface_))
        {
            RCLCPP_FATAL(LOGGER,
                         "Failed to open CAN interface: %s", can_interface_.c_str());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    void ArctosHardwareInterface::loadHardwareParameters()
    {
        can_ids_.clear();
        gear_ratios_.clear();

        can_interface_ = info_.hardware_parameters.at("can_interface");
        vel_ = std::stod(info_.hardware_parameters.at("vel"));
        accel_ = std::stod(info_.hardware_parameters.at("accel"));
        // require_homing = std::stoi(info_.hardware_parameters.at("require_homing"));

        // Collect joint names for service initialization
        // std::vector<std::string> joint_names;
        // joint_names.reserve(info_.joints.size());
        // Process joints and their interface. gripper also has a can_id and gear_ratio
        can_ids_.reserve(info_.joints.size());
        gear_ratios_.reserve(info_.joints.size());
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            can_ids_.push_back(std::stoi(info_.joints[i].parameters.at("can_id")));
            gear_ratios_.push_back(std::stod(info_.joints[i].parameters.at("gear_ratio")));
        }
    }

    CallbackReturn ArctosHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(LOGGER, "Activating hardware and enabling motors...");
        // Enable all motors
        for (u_int8_t can_id : can_ids_)
        {
            if (!can_driver_.enableMotor(can_id, true))
            {
                RCLCPP_ERROR(LOGGER,
                             "Failed to enable CAN ID: %d)", can_id);
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(LOGGER, "Motor enabled for CAN ID: %d", can_id);
        }

        // Read current positions from hardware
        RCLCPP_INFO(LOGGER, "Reading initial positions from hardware...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Query all positions
        for (u_int8_t can_id : can_ids_)
        {
            can_driver_.queryPosition(can_id);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Get the position data
        auto positions = can_driver_.getPositions();

        if (positions.size() != num_joints_)
        {
            RCLCPP_WARN(LOGGER,
                        "Expected %zu positions, got %zu. Some joints may not report position.",
                        num_joints_, positions.size());
        }

        // ✅ Process positions into position_states_ and position_commands_
        for (size_t i = 0; i < num_joints_; i++)
        {
            if (i < positions.size())
            {
                // Convert encoder counts to radians
                double rad = countsToRadians(positions[i], gear_ratios_[i]);

                // Normalize continuous joints (X, A, C) to [-π, π]
                if (i == 0 || i == 3 || i == 5) // X_joint, A_joint, C_joint
                {
                    rad = std::fmod(rad + M_PI, 2.0 * M_PI) - M_PI; // Normalize
                }

                position_states_[i] = rad;
                position_commands_[i] = rad; // Init commands to match current state

                RCLCPP_INFO(LOGGER,
                            "Joint '%s' current position: %.4f rad (%.1f deg)",
                            info_.joints[i].name.c_str(), rad, rad * 180.0 / M_PI);
            }
            else
            {
                // Joint not reporting - initialize to zero
                position_states_[i] = 0.0;
                position_commands_[i] = 0.0;
                velocity_states_[i] = 0.0; // <-- ADD THIS LINE
                RCLCPP_WARN(LOGGER,
                            "Joint '%s' not reporting position - initializing to 0.0 rad",
                            info_.joints[i].name.c_str());
            }
        }
        // Start homing procedure for all joints
        // can_ids_={1};
        // for (u_int8_t can_id : can_ids_)
        // {
        //     can_driver_.home(can_id);
        // }
        // for (u_int8_t can_id : can_ids_)
        // {
        //     int wait_homing_cnt = 0;
        //     while (can_driver_.getHomingStatus(can_id) != 0x02) // Homing not complete
        //     {
        //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //         if (++wait_homing_cnt > 20)
        //         {
        //             RCLCPP_ERROR(LOGGER,
        //                          "Homing timeout for CAN ID: %d", can_id);
        //             return CallbackReturn::ERROR;
        //         }
        //     }
        //     can_driver_.setZero(can_id);
        // }
        RCLCPP_INFO(LOGGER, "Hardware activated. All joint positions synchronized with RViz.");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(LOGGER, "Deactivating hardware...");
        // for (auto can_id : can_ids_)
        // {
        //     can_driver_.enableMotor(can_id, false);
        // }
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
        // gripper joint
        state_interfaces.emplace_back(
            "Right_jaw_joint",
            hw::HW_IF_POSITION,
            &gripper_pos_);
        state_interfaces.emplace_back(
            "Right_jaw_joint",
            hw::HW_IF_VELOCITY,
            &gripper_vel_);
        RCLCPP_INFO(LOGGER,
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
        cmds.emplace_back(
           "Right_jaw_joint",
            hw::HW_IF_POSITION,
            &gripper_cmd_);

        RCLCPP_INFO(LOGGER,
                    "Exported %zu command interfaces", cmds.size());
        return cmds;
    }

    hw::return_type ArctosHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // position_states_ = position_commands_;
        // gripper_pos_ = gripper_cmd_;
        // gripper_vel_ = 0.0;
        // return hardware_interface::return_type::OK;

        double dt = period.seconds();
        // auto positions = can_driver_.getPositions();
        // RCLCPP_INFO(LOGGER,
        //             "Exported %zu postions", positions.size());

        // Use actual number of joints, not hardcoded 6
        for (size_t i = 0; i < num_joints_; ++i)
        {
            double prev = position_states_[i];

            u_int16_t can_id = can_ids_[i];
            int64_t pos_counts = can_driver_.getPosition(can_id);

            double rad = countsToRadians(pos_counts, gear_ratios_[i]);

            // Normalize continuous joints (X, A, C) to [-π, π]
            if (i == 0 || i == 3 || i == 5)
            {
                rad = std::fmod(rad + M_PI, 2.0 * M_PI) - M_PI; // Normalize
            }
            position_states_[i] = std::isfinite(rad) ? rad : 0.0;

            RCLCPP_DEBUG(LOGGER,
                         "Reading state from joint '%s': %.3f rad",
                         info_.joints[i].name.c_str(), position_states_[i]);

            // Calculate velocity with proper bounds checking
            updateJointVelocity(i, prev, dt);
        }
        gripper_pos_ = gripper_cmd_;
        gripper_vel_ = 0.0;

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

    hw::return_type ArctosHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            // Only send command if NOT homing (homing is a special "search" move)

            double cmd = position_commands_[i];
            if (std::fabs(cmd - position_states_[i]) > POSITION_CHANGE_THRESHOLD)
            {
                RCLCPP_INFO(LOGGER,
                            "Sending command to joint '%s': %.3f rad (current: %.3f)",
                            info_.joints[i].name.c_str(), cmd, position_states_[i]);
                int32_t target_pos = radiansToCounts(cmd, gear_ratios_[i]);
                can_driver_.runPositionAbs(can_ids_[i], vel_, accel_, target_pos);
            }
            else
            {
                RCLCPP_DEBUG(LOGGER,
                             "No significant command change for joint '%s'; skipping command.",
                             info_.joints[i].name.c_str());
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hardware_interface::SystemInterface)