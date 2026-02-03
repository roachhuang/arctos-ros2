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
        last_sent_counts_.assign(num_joints_, INT32_MIN);
        // can_ids_.assign(info_.joints.size(), 0);
        // gear_ratios_.assign(info_.joints.size(), 0.0);
        // vel_.assign(info_.joints.size(), 0.0);
        // acc_.assign(info_.joints.size(), 0.0);

        // Initialize tracking vectors
        last_sent_command_.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
        send_accum_ = rclcpp::Duration(0, 0);

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
        vel_.clear();
        acc_.clear();
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
            //     // vel: 0~3000 in RPM (100), accel: 0~256. in 1000 RPM/s (10)
            //     vel_ = std::stod(info_.hardware_parameters.at("vel"));
            //     accel_ = std::stod(info_.hardware_parameters.at("accel"));
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(LOGGER, "Missing hardware parameter: %s", e.what());
            throw;
        }

        for (const auto &joint : info_.joints)
        {
            // joint_names.push_back(joint.name);
            can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
            gear_ratios_.push_back(std::stod(joint.parameters.at("gear_ratio")));
            // rad per second
            vel_.push_back(std::stod(joint.parameters.at("vel")));
            acc_.push_back(std::stod(joint.parameters.at("acc")));

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

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Get the position data
        // auto initial_counts = can_driver_.getPositions();

        // Standard Joints (0-3)
        for (size_t i = 0; i < 4; i++)
        {
            double rad = countsToRadians(can_driver_.getPosition(can_ids_[i]), gear_ratios_[i]);
            position_states_[i] = rad;
            position_commands_[i] = rad; // Tells MoveIt "Stay where you are"
            last_sent_command_[i] = rad; // Tells the Driver "No movement needed yet"
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto c5 = can_driver_.getPosition(can_ids_[4]);
        auto c6 = can_driver_.getPosition(can_ids_[5]);

        // unified motor space (motor6 sign applied here!)
        double m5_u = countsToRadians(c5, gear_ratios_[4]);
        double m6_u = countsToRadians(c6, gear_ratios_[5]) * M6_SIGN;
        m5_zero_ = m5_u;
        m6_zero_ = m6_u;
        wrist_zero_set_ = true;

        // Joint-space from unified motor space
        // position_states_[4] = 0.5 * (m5_u + m6_u); // Pitch
        // position_states_[5] = angles::normalize_angle(0.5 * (m5_u - m6_u));
        // position_commands_[4] = position_states_[4];
        // position_commands_[5] = position_states_[5];
        
        position_states_[4] = position_states_[5] = 0;
        position_commands_[4] = position_commands_[5] = 0;

        // tracking in physical counts for wrist (prevents initial jump)
        last_sent_counts_[4] = c5;
        last_sent_counts_[5] = c6;

        // CRITICAL: initialize tracking with Actuator Space values
        // Tracking: DO NOT store motor angles in last_sent_command_ if that vector is joint-space.
        // Prefer separate last_sent_counts_ for wrist, or store joint-space here:
        // last_sent_command_[4] = position_commands_[4];
        // last_sent_command_[5] = position_commands_[5];

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

    hw::return_type ArctosHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &period)
    {
        const double dt = period.seconds();
        auto prev = position_states_;

        // joints 1..4 as you already do
        for (size_t i = 0; i < 4; ++i)
        {
            // double prev = position_states_[i];

            int64_t encorder_cnt = can_driver_.getPosition(can_ids_[i]);

            double rad = countsToRadians(encorder_cnt, gear_ratios_[i]);

            // Normalize continuous joints (X, A, C) to [-π, π]
            rad = (i == 0 || i == 3) ? angles::normalize_angle(rad) : rad;
            position_states_[i] = std::isfinite(rad) ? rad : 0.0;

            // RCLCPP_INFO(LOGGER,
            //             "Reading state from joint '%s': %.3f rad",
            //             info_.joints[i].name.c_str(), position_states_[i]);

            // Calculate velocity with proper bounds checking
            updateJointVelocity(i, prev[i], dt);
        }

        const int64_t c5 = can_driver_.getPosition(can_ids_[4]);
        const int64_t c6 = can_driver_.getPosition(can_ids_[5]);
        double m5_u = countsToRadians(c5, gear_ratios_[4]);
        double m6_u = countsToRadians(c6, gear_ratios_[5]) * M6_SIGN;

        // zero-relative
        if (wrist_zero_set_)
        {
            m5_u -= m5_zero_;
            m6_u -= m6_zero_;
        }

        const double K = 0.5;
        const double B =0.5 * (m5_u + m6_u)/K;                          
        const double C = angles::normalize_angle(0.5 * (m5_u - m6_u)/K); 
        position_states_[4] = B;
        position_states_[5] = C;
        RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 500,
                                 "j5=%.2f deg, j6=%.2f deg, m5_u=%.2f deg, m6_u=%.2f deg, c5=%d c6=%d",
                                 angles::to_degrees(B), angles::to_degrees(C), angles::to_degrees(m5_u), angles::to_degrees(m6_u), c5, c6);
        // optional: no hard clamp here (MoveIt wants truth), but if noise causes bounds errors,
        // clamp ONLY tiny epsilon, not hard.
        updateJointVelocity(4, prev[4], dt);
        updateJointVelocity(5, prev[5], dt);

        return hw::return_type::OK;
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
    hw::return_type ArctosHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &period)
    {
        (void)period;
        u_int16_t rpm;                 // suppress unused variable warning
        constexpr double B_EPS = 1e-4; // ~0.0057 deg

        static rclcpp::Time last_send_time = this->get_clock()->now();
        if ((this->get_clock()->now() - last_send_time).seconds() < 0.1)
        {
            return hw::return_type::OK;
        }
        last_send_time = this->get_clock()->now();

        for (size_t i = 0; i < 4; ++i)
        {
            double joint_target_rad;
            joint_target_rad = position_commands_[i];
            // Only send command if NOT homing (homing is a special "search" move)
            double safe_cmd = std::clamp(joint_target_rad, min_[i], max_[i]);
            // 2. DATA INTEGRITY: Ensure MoveIt/MTC didn't send a NaN
            if (!std::isfinite(safe_cmd))
                continue;
            if (std::abs(safe_cmd - last_sent_command_[i]) > threshold_joint(gear_ratios_[i]))
            {
                // RCLCPP_DEBUG(LOGGER,
                //              "Sending command to joint '%s': %.3f rad (current: %.3f)",
                //              info_.joints[i].name.c_str(), safe_cmd, position_states_[i]);
                int32_t target_pos = radiansToCounts(safe_cmd, gear_ratios_[i]);
                rpm = vel_[i] * (60 / TWO_PI) * gear_ratios_[i];
                can_driver_.runPositionAbs(can_ids_[i], rpm, acc_[i], target_pos);
                // Update tracking
                last_sent_command_[i] = safe_cmd;
            }
        }

        // ----- joint limits -----
        constexpr double B_MIN = -1.55, B_MAX = 1.55;
        constexpr double C_MIN = -M_PI, C_MAX = M_PI;

        // ----- desired joints (relative) -----
        const double B = std::clamp(position_commands_[4], B_MIN + B_EPS, B_MAX - B_EPS);
        const double C = std::clamp(angles::normalize_angle(position_commands_[5]), C_MIN, C_MAX);

        if (!wrist_zero_set_)
            return hw::return_type::OK;
        const double K = 0.5;
        double m5_u_abs = m5_zero_ + K*(B + C);
        double m6_u_abs = m6_zero_ + K*(B - C);

        // ----- convert to counts (must llround, division before cast) -----
        const double effective_gear_ratio = gear_ratios_[5]; // both joints use same gear ratio        
        const int32_t c5 = radiansToCounts(m5_u_abs, effective_gear_ratio);
        const int32_t c6 = radiansToCounts(m6_u_abs * M6_SIGN, effective_gear_ratio); // back to physical

        // ----- deadband in COUNTS (prevents spamming) -----
        constexpr int32_t COUNT_EPS = 20; // tune: 5~20 counts
        rpm = vel_[4] * (60 / TWO_PI) * effective_gear_ratio;
        if (std::abs(c5 - last_sent_counts_[4]) > COUNT_EPS ||
            std::abs(c6 - last_sent_counts_[5]) > COUNT_EPS)
        {
            can_driver_.runPositionAbs(can_ids_[4], rpm, acc_[4], c5);
            can_driver_.runPositionAbs(can_ids_[5], rpm, acc_[5], c6);

            last_sent_counts_[4] = c5;
            last_sent_counts_[5] = c6;

            RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 500,
                                 "J5=%.2f deg, J6=%.2f deg, m5_abs=%.2f deg, m6_abs=%.2f deg, c5=%d c6=%d",
                                 angles::to_degrees(B), angles::to_degrees(C), angles::to_degrees(m5_u_abs), angles::to_degrees(m6_u_abs), c5, c6);
        }

        // ----- other joints (1-4) can stay as you already do -----
        // (but also use count-based eps similarly)

        return hw::return_type::OK;
    }

} // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hardware_interface::SystemInterface)