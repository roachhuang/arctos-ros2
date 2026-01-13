#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
#include <algorithm>
#include <cmath>
#include <angles/angles.h>
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

        // Initialize tracking vectors
        last_sent_command_.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
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
        min_.clear();
        max_.clear();

        // require_homing = std::stoi(info_.hardware_parameters.at("require_homing"));

        // Collect joint names for service initialization
        // std::vector<std::string> joint_names;
        // joint_names.reserve(info_.joints.size());
        // Process joints and their interface. gripper also has a can_id and gear_ratio

        try
        {
            can_interface_ = info_.hardware_parameters.at("can_interface");
            // vel: 0~3000 in RPM (100), accel: 0~256. in 1000 RPM/s (10)
            vel_ = std::stod(info_.hardware_parameters.at("vel"));
            accel_ = std::stod(info_.hardware_parameters.at("accel"));
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(LOGGER, "Missing hardware parameter: %s", e.what());
            throw;
        }

        can_ids_.reserve(info_.joints.size());
        gear_ratios_.reserve(info_.joints.size());
        for (const auto &joint : info_.joints)
        {
            // joint_names.push_back(joint.name);
            can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
            gear_ratios_.push_back(std::stod(joint.parameters.at("gear_ratio")));
            for (const auto &cmd_interface : joint.command_interfaces)
            {
                if (cmd_interface.name == "position")
                    try
                    {
                        min_.push_back(std::stod(cmd_interface.parameters.at("min")));
                        max_.push_back(std::stod(cmd_interface.parameters.at("max")));
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_FATAL(LOGGER, "Missing min/max parameters for joint %s: %s", joint.name.c_str(), e.what());
                        throw;
                    }
            }
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
        // for (u_int8_t can_id : can_ids_)
        // {
        //     can_driver_.queryPosition(can_id);
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Get the position data
        auto initial_counts = can_driver_.getPositions();

        // Standard Joints (0-3)
        for (size_t i = 0; i < 4; i++)
        {
            double rad = countsToRadians(initial_counts[i], gear_ratios_[i]);
            position_states_[i] = rad;
            position_commands_[i] = rad; // Tells MoveIt "Stay where you are"
            last_sent_command_[i] = rad; // Tells the Driver "No movement needed yet"
        }

        // Differential Pair (4-5)
        double m5_rad = countsToRadians(initial_counts[4], gear_ratios_[4]);
        double m6_rad = countsToRadians(initial_counts[5], gear_ratios_[5]);

        // position_states_[4] = (m6_rad - m5_rad) / 2.0; // Pitch
        // position_states_[5] = (m5_rad + m6_rad) / 2.0; // Roll
        position_states_[4] = (m5_rad + m6_rad) / 2.0; // Pitch
        position_states_[5] = (m5_rad - m6_rad) / 2.0; // Roll
        position_commands_[4] = position_states_[4];
        position_commands_[5] = position_states_[5];

        // CRITICAL: initialize tracking with Actuator Space values
        last_sent_command_[4] = m5_rad;
        last_sent_command_[5] = m6_rad;

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
        auto cnts = can_driver_.getPositions();
        auto prev_pos = position_states_;

        // RCLCPP_INFO(LOGGER,
        //             "Exported %zu postions", positions.size());

        // Use actual number of joints, not hardcoded 6
        for (size_t i = 0; i < 4; ++i)
        {
            // double prev = position_states_[i];

            // int64_t pos_counts = can_driver_.getPosition(can_ids_[i]);

            double rad = countsToRadians(cnts[i], gear_ratios_[i]);

            // Normalize continuous joints (X, A, C) to [-π, π]
            if (i == 0 || i == 3 || i == 5)
            {
                rad = std::fmod(rad + M_PI, 2.0 * M_PI) - M_PI; // Normalize
            }
            position_states_[i] = std::isfinite(rad) ? rad : 0.0;

            // RCLCPP_INFO(LOGGER,
            //             "Reading state from joint '%s': %.3f rad",
            //             info_.joints[i].name.c_str(), position_states_[i]);

            // Calculate velocity with proper bounds checking
            updateJointVelocity(i, prev_pos[i], dt);
        }

        double m5_pos = countsToRadians(cnts[4], gear_ratios_[4]);
        double m6_pos = 0 - countsToRadians(cnts[5], gear_ratios_[5]);

        // 2. Inverse Differential Transformation
        // Joint B (Pitch) is the average
        // Joint C (Roll) is the half-difference
        position_states_[4] = (m5_pos + m6_pos) * 0.5;                      // Joint B (Pitch)
        position_states_[5] = (m5_pos - m6_pos) * 0.5;                      // Joint C (Roll)
        position_states_[5] = angles::normalize_angle(position_states_[5]); // Joint C (Roll)
        updateJointVelocity(4, prev_pos[4], dt);
        updateJointVelocity(5, prev_pos[5], dt);

        // gripper_pos_ = gripper_cmd_;
        // gripper_vel_ = 0.0;
        double err = gripper_cmd_ - gripper_pos_;
        double step = std::clamp(err, -0.02 * dt, 0.02 * dt);
        gripper_pos_ += step;
        gripper_vel_ = step / dt;
        return hardware_interface::return_type::OK;
    }

    void ArctosHardwareInterface::updateJointVelocity(size_t i, double prev_pos, double dt)
    {
        double new_velocity;
        if (dt > VELOCITY_EPSILON)
        {
            if (i == 0 || i == 3 || i == 5)
            {
                double delta = angles::shortest_angular_distance(prev_pos, position_states_[i]);
                new_velocity = delta / dt;
            }
            else
            {
                new_velocity = (position_states_[i] - prev_pos) / dt;
            }
            // Smooth velocity with EMA (0.7 + 0.3 = 1.0) to prevent spikes
            if (std::isfinite(new_velocity))
            {
                velocity_states_[i] = 0.7 * velocity_states_[i] + 0.3 * new_velocity;
            } // Keep previous velocity if new_velocity is invalid
        }
        else
        {
            velocity_states_[i] = 0.0;
        }
    }

    hw::return_type ArctosHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        // 1. Get commands from hardware interface memory (indices 4 and 5 for B and C)
        double joint_B_cmd = position_commands_[4]; // pitch
        double joint_C_cmd = position_commands_[5]; // roll

        // 2. Apply Differential Transformation (Actuator Space)
        // Note: You may need to flip signs based on your specific motor mounting
        // position_commands_[4] = joint_B_cmd + joint_C_cmd;
        // position_commands_[5] = joint_B_cmd - joint_C_cmd;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            double joint_target_rad;
            // --- DIFFERENTIAL MIXING LOGIC ---
            if (i == 4) // Motor 5 (ID 5)
            {
                joint_target_rad = joint_B_cmd + joint_C_cmd; // B + C
            }
            else if (i == 5) // Motor 6 (ID 6)
            {
                joint_target_rad = joint_B_cmd - joint_C_cmd; // B - C
                joint_target_rad = -joint_target_rad;         // Invert for correct direction
            }
            else // All other joints (0, 1, 2, 3, 6)
            {
                joint_target_rad = position_commands_[i];
            }
            // Only send command if NOT homing (homing is a special "search" move)

            double safe_cmd = std::clamp(joint_target_rad, min_[i], max_[i]);
            // 2. DATA INTEGRITY: Ensure MoveIt/MTC didn't send a NaN
            if (!std::isfinite(safe_cmd))
                continue;
            if (std::abs(safe_cmd - last_sent_command_[i]) > POSITION_CHANGE_THRESHOLD)
            {
                RCLCPP_INFO(LOGGER,
                            "Sending command to joint '%s': %.3f rad (current: %.3f)",
                            info_.joints[i].name.c_str(), safe_cmd, position_states_[i]);
                int32_t target_pos = radiansToCounts(safe_cmd, gear_ratios_[i]);

                can_driver_.runPositionAbs(can_ids_[i], vel_, accel_, target_pos);
                // Update tracking
                last_sent_command_[i] = safe_cmd;
            }
            // else
            // {
            //     RCLCPP_INFO(LOGGER,
            //                  "No significant command change for joint '%s'; skipping command.",
            //                  info_.joints[i].name.c_str());
            // }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hardware_interface::SystemInterface)