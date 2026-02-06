#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
#include <algorithm>
#include <angles/angles.h>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <optional>
#include <stdexcept>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
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
    int ArctosHardwareInterface::mapGripperPositionToRaw(double cmd, double close_pos, double open_pos)
    {
        double min_v = close_pos;
        double max_v = open_pos;
        if (min_v > max_v)
            std::swap(min_v, max_v);

        if (!std::isfinite(cmd))
            return -1;

        const double clamped = std::clamp(cmd, min_v, max_v);
        const double span = max_v - min_v;
        const int raw = (span > 1e-9) ? static_cast<int>(std::lround((clamped - min_v) * 255.0 / span)) : 0;
        return std::clamp(raw, 0, 255);
    }

    hardware_interface::CallbackReturn ArctosHardwareInterface::on_init(const hw::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        num_joints_ = DOF;
        if (info_.joints.size() < DOF)
        {
            RCLCPP_FATAL(LOGGER, "Hardware info has %zu joints; expected at least %zu arm joints.",
                         info_.joints.size(), DOF);
            return hardware_interface::CallbackReturn::ERROR;
        }
        is_homing_.resize(num_joints_, false);
        position_commands_.assign(num_joints_, 0.0);
        velocity_commands_.assign(num_joints_, 0.0);
        position_states_.assign(num_joints_, 0.0);
        velocity_states_.assign(num_joints_, 0.0);
        last_sent_counts_.assign(num_joints_, INT32_MIN);

        last_sent_command_.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());

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
        gripper_can_enabled_ = openGripperCanSocket();
        if (!gripper_can_enabled_)
        {
            RCLCPP_WARN(LOGGER, "Gripper CAN socket not available; gripper will be command-only (no CAN output).");
        }

        return CallbackReturn::SUCCESS;
    }

    void ArctosHardwareInterface::loadHardwareParameters()
    {
        can_ids_.assign(DOF, 0);
        gear_ratios_.assign(DOF, 0.0);
        vel_.assign(DOF, 0.0);
        acc_.assign(DOF, 0.0);
        min_.assign(DOF, 0.0);
        max_.assign(DOF, 0.0);
        std::vector<bool> arm_seen(DOF, false);

        try
        {
            can_interface_ = info_.hardware_parameters.at("can_interface");
            if (info_.hardware_parameters.count("gripper_can_id") > 0)
            {
                gripper_can_id_ = static_cast<uint16_t>(std::stoul(info_.hardware_parameters.at("gripper_can_id")));
            }
            if (info_.hardware_parameters.count("gripper_open_position") > 0)
            {
                gripper_open_pos_ = std::stod(info_.hardware_parameters.at("gripper_open_position"));
            }
            if (info_.hardware_parameters.count("gripper_close_position") > 0)
            {
                gripper_close_pos_ = std::stod(info_.hardware_parameters.at("gripper_close_position"));
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(LOGGER, "Missing hardware parameter: %s", e.what());
            throw;
        }

        auto arm_index = [this](const std::string &name) -> std::optional<size_t>
        {
            for (size_t i = 0; i < arm_joint_names_.size(); ++i)
            {
                if (arm_joint_names_[i] == name)
                    return i;
            }
            return std::nullopt;
        };

        for (const auto &joint : info_.joints)
        {
            if (joint.name == "Right_jaw_joint")
            {
                if (joint.parameters.count("can_id") > 0)
                {
                    gripper_can_id_ = static_cast<uint16_t>(std::stoul(joint.parameters.at("can_id")));
                }
                for (const auto &cmd_interface : joint.command_interfaces)
                {
                    if (cmd_interface.name == "position")
                    {
                        try
                        {
                            double min_v = std::stod(cmd_interface.parameters.at("min"));
                            double max_v = std::stod(cmd_interface.parameters.at("max"));
                            if (min_v > max_v)
                                std::swap(min_v, max_v);
                            gripper_close_pos_ = min_v;
                            gripper_open_pos_ = max_v;
                        }
                        catch (const std::exception &e)
                        {
                            RCLCPP_WARN(LOGGER, "Gripper min/max not set from joint params: %s", e.what());
                        }
                    }
                }
                continue;
            }

            auto idx_opt = arm_index(joint.name);
            if (!idx_opt)
            {
                RCLCPP_WARN(LOGGER, "Unknown joint in hardware info: %s (skipping)", joint.name.c_str());
                continue;
            }
            const size_t idx = *idx_opt;
            can_ids_[idx] = std::stoi(joint.parameters.at("can_id"));
            gear_ratios_[idx] = std::stod(joint.parameters.at("gear_ratio"));
            vel_[idx] = std::stod(joint.parameters.at("vel"));
            acc_[idx] = std::stod(joint.parameters.at("acc"));

            for (const auto &cmd_interface : joint.command_interfaces)
            {
                if (cmd_interface.name == "position")
                {
                    try
                    {
                        min_[idx] = std::stod(cmd_interface.parameters.at("min"));
                        max_[idx] = std::stod(cmd_interface.parameters.at("max"));
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_FATAL(LOGGER, "Missing min/max parameters for joint %s: %s", joint.name.c_str(), e.what());
                        throw;
                    }
                }
            }
            arm_seen[idx] = true;
        }

        for (size_t i = 0; i < arm_seen.size(); ++i)
        {
            if (!arm_seen[i])
            {
                RCLCPP_FATAL(LOGGER, "Missing arm joint parameters for %s", arm_joint_names_[i].c_str());
                throw std::runtime_error("Missing arm joint parameters");
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

        position_states_[4] = position_states_[5] = 0;
        position_commands_[4] = position_commands_[5] = 0;

        // tracking in physical counts for wrist (prevents initial jump)
        last_sent_counts_[4] = c5;
        last_sent_counts_[5] = c6;

        RCLCPP_INFO(LOGGER, "Hardware activated. All joint positions synchronized with RViz.");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(LOGGER, "Deactivating hardware...");
        can_driver_.deactive();
        closeGripperCanSocket();
        return CallbackReturn::SUCCESS;
    }

    std::vector<hw::StateInterface> ArctosHardwareInterface::export_state_interfaces()
    {
        std::vector<hw::StateInterface> state_interfaces;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            state_interfaces.emplace_back(
                arm_joint_names_[i],
                hw::HW_IF_POSITION,
                &position_states_[i]);

            state_interfaces.emplace_back(
                arm_joint_names_[i],
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
                arm_joint_names_[i],
                hw::HW_IF_POSITION,
                &position_commands_[i]);
            cmds.emplace_back(arm_joint_names_[i], hw::HW_IF_VELOCITY, &velocity_commands_[i]);
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

        for (size_t i = 0; i < 4; ++i)
        {
            int64_t encoder_counts = can_driver_.getPosition(can_ids_[i]);

            double rad = countsToRadians(encoder_counts, gear_ratios_[i]);

            // Normalize continuous joints (X, A, C) to [-π, π]
            rad = (i == 0 || i == 3) ? angles::normalize_angle(rad) : rad;
            position_states_[i] = std::isfinite(rad) ? rad : 0.0;

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

        // Wrist kinematics: convert motor space (m5/m6) into joint space (B/C).
        const double K = 0.5;
        const double B = 0.5 * (m5_u + m6_u) / K;
        const double C = angles::normalize_angle(0.5 * (m5_u - m6_u) / K);
        position_states_[4] = B;
        position_states_[5] = C;
        updateJointVelocity(4, prev[4], dt);
        updateJointVelocity(5, prev[5], dt);

        // Gripper has no encoder: keep soft state synced to command
        gripper_pos_ = gripper_cmd_;
        gripper_vel_ = 0.0;

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
        double planned_vel_rpm;        // suppress unused variable warning
        u_int16_t rpm = 0;             // suppress unused variable warning
        constexpr double B_EPS = 1e-4; // ~0.0057 deg

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
                int32_t target_pos = radiansToCounts(safe_cmd, gear_ratios_[i]);
                planned_vel_rpm = std::abs(velocity_commands_[i] * gear_ratios_[i] * (60.0 / TWO_PI));
                rpm = std::clamp(planned_vel_rpm, 50.0, 2000.0);
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
        // Wrist kinematics: convert desired joint space (B/C) into motor space (m5/m6).
        const double K = 0.5; // diff_gain
        double m5_u_abs = m5_zero_ + K * (B + C);
        double m6_u_abs = m6_zero_ + K * (B - C);

        // ----- convert to counts (must llround, division before cast) -----
        const double effective_gear_ratio = gear_ratios_[5]; // both joints use same gear ratio
        const int32_t c5 = radiansToCounts(m5_u_abs, effective_gear_ratio);
        const int32_t c6 = radiansToCounts(m6_u_abs * M6_SIGN, effective_gear_ratio); // back to physical

        // ----- deadband in COUNTS (prevents spamming) -----
        constexpr int32_t COUNT_EPS = 20; // tune: 5~20 counts
        rpm = vel_[4] * (60.0 / TWO_PI);  // * effective_gear_ratio;
        if (std::abs(c5 - last_sent_counts_[4]) > COUNT_EPS ||
            std::abs(c6 - last_sent_counts_[5]) > COUNT_EPS)
        {
            planned_vel_rpm = std::abs(velocity_commands_[4] * gear_ratios_[4] * (60.0 / TWO_PI));
            rpm = std::clamp(planned_vel_rpm, 50.0, 2000.0);
            can_driver_.runPositionAbs(can_ids_[4], rpm, acc_[4], c5);

            planned_vel_rpm = std::abs(velocity_commands_[5] * gear_ratios_[5] * (60.0 / TWO_PI));
            rpm = std::clamp(planned_vel_rpm, 50.0, 2000.0);
            can_driver_.runPositionAbs(can_ids_[5], rpm, acc_[5], c6);

            last_sent_counts_[4] = c5;
            last_sent_counts_[5] = c6;

            RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 500,
                                 "J5=%.2f deg, J6=%.2f deg, m5_abs=%.2f deg, m6_abs=%.2f deg, c5=%d c6=%d",
                                 angles::to_degrees(B), angles::to_degrees(C), angles::to_degrees(m5_u_abs), angles::to_degrees(m6_u_abs), c5, c6);
        }

        // ----- gripper (command-only CAN device, no encoder) -----
        if (gripper_can_enabled_)
        {
            const int raw_clamped = mapGripperPositionToRaw(gripper_cmd_, gripper_close_pos_, gripper_open_pos_);
            if (raw_clamped < 0)
                return hw::return_type::OK;

            if (raw_clamped != gripper_last_raw_)
            {
                const std::vector<uint8_t> data{static_cast<uint8_t>(raw_clamped)};
                if (sendGripperFrame(data))
                {
                    gripper_last_raw_ = raw_clamped;
                }
                else
                {
                    RCLCPP_WARN(LOGGER, "Failed to send gripper CAN command.");
                }
            }
        }

        return hw::return_type::OK;
    }

    bool ArctosHardwareInterface::openGripperCanSocket()
    {
        if (can_interface_.empty())
        {
            return false;
        }
        gripper_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (gripper_sock_ < 0)
        {
            return false;
        }
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
        if (ioctl(gripper_sock_, SIOCGIFINDEX, &ifr) < 0)
        {
            close(gripper_sock_);
            gripper_sock_ = -1;
            return false;
        }
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(gripper_sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            close(gripper_sock_);
            gripper_sock_ = -1;
            return false;
        }
        int flags = fcntl(gripper_sock_, F_GETFL, 0);
        fcntl(gripper_sock_, F_SETFL, flags | O_NONBLOCK);
        return true;
    }

    void ArctosHardwareInterface::closeGripperCanSocket()
    {
        if (gripper_sock_ >= 0)
        {
            close(gripper_sock_);
            gripper_sock_ = -1;
        }
    }

    bool ArctosHardwareInterface::sendGripperFrame(const std::vector<uint8_t> &data)
    {
        if (gripper_sock_ < 0)
        {
            return false;
        }
        if (data.size() > 8)
        {
            return false;
        }
        can_frame tx{};
        tx.can_id = gripper_can_id_ & CAN_SFF_MASK;
        tx.can_dlc = static_cast<__u8>(data.size());
        std::memcpy(tx.data, data.data(), tx.can_dlc);
        return ::write(gripper_sock_, &tx, sizeof(tx)) == sizeof(tx);
    }

} // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hardware_interface::SystemInterface)
