#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
#include <algorithm>
#include <angles/angles.h>
#include <cctype>
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
    namespace
    {
        constexpr size_t MAIN_JOINT_COUNT = 4;
        constexpr size_t B_IDX = 4;
        constexpr size_t C_IDX = 5;
        constexpr double DIFF_GAIN = ArctosHardwareInterface::WRIST_DIFF_GAIN;
        constexpr double B_EPS = 1e-4;
        // Smaller command deadband for the differential pair to reduce "step/hold"
        // behavior and keep B/C motion visually continuous.
        constexpr int32_t WRIST_COUNT_EPS = 5;

        struct WristJointState
        {
            double b{0.0};
            double c{0.0};
        };

        struct WristMotorState
        {
            double m5_abs{0.0};
            double m6_abs{0.0};
            int32_t c5{0};
            int32_t c6{0};
        };

        inline WristJointState motorsToWristJoints(double m5_u, double m6_u, double diff_gain)
        {
            WristJointState out;
            if (diff_gain == 0.0)
            {
                out.b = 0.0;
                out.c = 0.0;
                return out;
            }
            out.b = 0.5 * (m5_u + m6_u) / diff_gain;
            out.c = 0.5 * (m5_u - m6_u) / diff_gain;
            return out;
        }

        inline WristMotorState wristJointsToMotors(double b, double c,
                                                   double m5_zero, double m6_zero,
                                                   double diff_gain, double m6_sign,
                                                   const ArctosHardwareInterface &hw,
                                                   double ratio5, double ratio6)
        {
            WristMotorState out;
            out.m5_abs = m5_zero + diff_gain * (b + c);
            out.m6_abs = m6_zero + diff_gain * (b - c);
            out.c5 = hw.radiansToCounts(out.m5_abs, ratio5);
            out.c6 = hw.radiansToCounts(out.m6_abs * m6_sign, ratio6);
            return out;
        }
    } // namespace

    double ArctosHardwareInterface::clampRpm(double desired_joint_vel,
                                             const MotorConfig &motor,
                                             double default_min_rpm,
                                             double default_max_rpm)
    {
        const double abs_ratio = std::abs(motor.gear_ratio);
        double min_rpm = default_min_rpm;
        double max_rpm = default_max_rpm;

        if (motor.min_vel > 0.0 && abs_ratio > 0.0)
        {
            min_rpm = std::max(min_rpm, motor.min_vel * abs_ratio * (60.0 / TWO_PI));
        }
        if (motor.max_vel > 0.0 && abs_ratio > 0.0)
        {
            max_rpm = motor.max_vel * abs_ratio * (60.0 / TWO_PI);
        }

        const double desired_rpm = std::abs(desired_joint_vel) * abs_ratio * (60.0 / TWO_PI);
        return std::clamp(desired_rpm, min_rpm, max_rpm);
    }

    int ArctosHardwareInterface::mapGripperPositionToRaw(double cmd, double close_pos, double open_pos)
    {
        if (!std::isfinite(cmd))
            return -1;

        const double min_v = std::min(close_pos, open_pos);
        const double max_v = std::max(close_pos, open_pos);
        const double clamped = std::clamp(cmd, min_v, max_v);
        const double span = open_pos - close_pos;
        if (std::abs(span) <= 1e-9)
            return 0;

        int raw = static_cast<int>(std::lround((clamped - close_pos) * 255.0 / span));
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
        RCLCPP_INFO(LOGGER, "Hardware parameters loaded (can_interface=%s).", can_interface_.c_str());

        if (!can_driver_.connect(can_interface_))
        {
            RCLCPP_FATAL(LOGGER,
                         "Failed to open CAN interface: %s", can_interface_.c_str());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(LOGGER, "CAN driver connected on %s.", can_interface_.c_str());

        RCLCPP_INFO(LOGGER, "Applying CAN ID 6 startup config...");
        if (!configureCanId6Startup())
        {
            RCLCPP_FATAL(LOGGER, "Failed to send MKS startup config for CAN ID 6.");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(LOGGER, "CAN ID 6 startup config complete.");

        RCLCPP_INFO(LOGGER, "Opening gripper CAN socket...");
        gripper_can_enabled_ = openGripperCanSocket();
        if (!gripper_can_enabled_)
        {
            RCLCPP_WARN(LOGGER, "Gripper CAN socket not available; gripper will be command-only (no CAN output).");
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Gripper CAN socket opened.");
        }

        return CallbackReturn::SUCCESS;
    }

    void ArctosHardwareInterface::loadHardwareParameters()
    {
        motors_.fill({});
        std::vector<bool> arm_seen(DOF, false);
        bool gripper_limits_from_hw = false;

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
                gripper_limits_from_hw = true;
            }
            if (info_.hardware_parameters.count("gripper_close_position") > 0)
            {
                gripper_close_pos_ = std::stod(info_.hardware_parameters.at("gripper_close_position"));
                gripper_limits_from_hw = true;
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
                        if (!gripper_limits_from_hw)
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
            motors_[idx].name = joint.name;
            motors_[idx].can_id = static_cast<uint8_t>(std::stoul(joint.parameters.at("can_id")));
            motors_[idx].gear_ratio = std::stod(joint.parameters.at("gear_ratio"));
            motors_[idx].vel = std::stod(joint.parameters.at("vel"));
            motors_[idx].acc = std::stod(joint.parameters.at("acc"));
            motors_[idx].min_vel = 0.0;
            if (joint.parameters.count("min_vel") > 0)
            {
                motors_[idx].min_vel = std::stod(joint.parameters.at("min_vel"));
            }
            motors_[idx].max_vel = motors_[idx].vel;
            if (joint.parameters.count("max_vel") > 0)
            {
                motors_[idx].max_vel = std::stod(joint.parameters.at("max_vel"));
            }

            for (const auto &cmd_interface : joint.command_interfaces)
            {
                if (cmd_interface.name == "position")
                {
                    try
                    {
                        motors_[idx].min = std::stod(cmd_interface.parameters.at("min"));
                        motors_[idx].max = std::stod(cmd_interface.parameters.at("max"));
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

    bool ArctosHardwareInterface::sendCheckedCanCommand(
        uint16_t id, uint8_t cmd, const std::vector<uint8_t> &params, const char *label, int timeout_ms)
    {
        // Per MKS protocol used on this hardware: byte2==1 => success, 0 => fail.
        const uint8_t status = can_driver_.sendCmdWithStatusSync(id, cmd, params, timeout_ms);
        if (status != 0x01)
        {
            RCLCPP_ERROR(LOGGER, "CAN ID %u %s failed (cmd=0x%02X status=0x%02X).", id, label, cmd, status);
            return false;
        }
        return true;
    }

    bool ArctosHardwareInterface::configureCanId6Startup()
    {
        constexpr uint16_t kCanId = 6;
        constexpr int kDefaultMstep = 16;
        constexpr int kDefaultMode = static_cast<int>(mks_servo_driver::MotorMode::SR_vFOC);
        constexpr int kDefaultMa = 1600;
        constexpr int kCommandGapMs = 20;

        auto parseBool = [](const std::string &value, bool default_v) -> bool
        {
            std::string normalized = value;
            std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                           [](unsigned char c)
                           { return static_cast<char>(std::tolower(c)); });
            if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on")
                return true;
            if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off")
                return false;
            return default_v;
        };

        auto getInt = [this](const char *key, int default_v) -> int
        {
            const auto it = info_.hardware_parameters.find(key);
            if (it == info_.hardware_parameters.end())
                return default_v;
            try
            {
                return std::stoi(it->second);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(LOGGER, "Invalid integer for '%s' (%s). Using default=%d.",
                            key, e.what(), default_v);
                return default_v;
            }
        };

        auto getBool = [this, &parseBool](const char *key, bool default_v) -> bool
        {
            const auto it = info_.hardware_parameters.find(key);
            if (it == info_.hardware_parameters.end())
                return default_v;
            return parseBool(it->second, default_v);
        };

        const bool enable_startup_cfg = getBool("mks_cfg_canid6_enable", true);
        if (!enable_startup_cfg)
        {
            RCLCPP_INFO(LOGGER, "Skipping CAN ID 6 startup config (mks_cfg_canid6_enable=false).");
            return true;
        }

        const int mstep = kDefaultMstep;
        const int mode = std::clamp(getInt("mks_cfg_canid6_mode", kDefaultMode), 0, 5);
        const int ma = std::clamp(getInt("mks_cfg_canid6_ma", kDefaultMa), 0, 3000);
        const bool protect_enable = getBool("mks_cfg_canid6_protect_enable", false);

        if (!sendCheckedCanCommand(
                kCanId,
                mks_servo_driver::CANCommands::SET_SUBDIVISIONS,
                {0x10}, // 16
                "Mstep"))
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kCommandGapMs));

        if (!sendCheckedCanCommand(
                kCanId,
                mks_servo_driver::CANCommands::SET_WORKING_MODE,
                {static_cast<uint8_t>(mode)},
                "Mode"))
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kCommandGapMs));

        if (!sendCheckedCanCommand(
                kCanId,
                mks_servo_driver::CANCommands::SET_ENABLE_SETTINGS,
                {0x00},
                "EN pin active level (low)"))
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kCommandGapMs));

        if (!sendCheckedCanCommand(
                kCanId,
                mks_servo_driver::CANCommands::SET_CURRENT,
                {static_cast<uint8_t>((ma >> 8) & 0xFF), static_cast<uint8_t>(ma & 0xFF)},
                "Run current"))
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kCommandGapMs));

        if (!sendCheckedCanCommand(
                kCanId,
                mks_servo_driver::CANCommands::SET_SHAFT_PROTECTION,
                {0x00}, // disable
                "Protection disable"))
        {
            return false;
        }

        RCLCPP_INFO(LOGGER,
                    "CAN ID %u startup config requested: mstep=%d mode=%d en_pin=low ma=%d protect=%s",
                    kCanId, mstep, mode, ma,
                    protect_enable ? "on" : "off");
        return true;
    }

    CallbackReturn ArctosHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        constexpr int kStartupQueryTimeoutMs = 120;
        constexpr int kStartupFreshnessMs = 250;
        RCLCPP_INFO(LOGGER, "Activating hardware and enabling motors...");
        // Enable all motors
        for (const auto &motor : motors_)
        {
            if (!sendCheckedCanCommand(
                    motor.can_id,
                    mks_servo_driver::CANCommands::ENABLE_MOTOR,
                    {0x01},
                    "Enable motor"))
            {
                RCLCPP_ERROR(LOGGER,
                             "Failed to enable CAN ID: %d)", motor.can_id);
                return CallbackReturn::ERROR;
            }
        }
        
        RCLCPP_INFO(LOGGER, "All motors enabled.");

        for (const auto &motor : motors_)
        {
            if (!can_driver_.queryPositionSync(motor.can_id, kStartupQueryTimeoutMs))
            {
                RCLCPP_ERROR(LOGGER, "Failed to get fresh startup position from CAN ID: %d", motor.can_id);
                return CallbackReturn::ERROR;
            }
            if (!can_driver_.isPositionFresh(motor.can_id, kStartupFreshnessMs))
            {
                RCLCPP_ERROR(LOGGER, "Startup position for CAN ID %d is stale after sync query.", motor.can_id);
                return CallbackReturn::ERROR;
            }
        }

        // Standard Joints (0-3)
        for (size_t i = 0; i < MAIN_JOINT_COUNT; ++i)
        {
            double rad = countsToRadians(can_driver_.getPosition(motors_[i].can_id), motors_[i].gear_ratio);
            position_states_[i] = rad;
            position_commands_[i] = rad; // Tells MoveIt "Stay where you are"
            last_sent_command_[i] = rad; // Tells the Driver "No movement needed yet"
        }

        auto c5 = can_driver_.getPosition(motors_[B_IDX].can_id);
        auto c6 = can_driver_.getPosition(motors_[C_IDX].can_id);

        // unified motor space (motor6 sign applied here!)
        double m5_u = countsToRadians(c5, motors_[B_IDX].gear_ratio);
        double m6_u = countsToRadians(c6, motors_[C_IDX].gear_ratio) * M6_SIGN;
        m5_zero_ = m5_u;
        m6_zero_ = m6_u;
        wrist_zero_set_ = true;

        position_states_[B_IDX] = position_states_[C_IDX] = 0;
        position_commands_[B_IDX] = position_commands_[C_IDX] = 0;

        // tracking in physical counts for wrist (prevents initial jump)
        last_sent_counts_[B_IDX] = c5;
        last_sent_counts_[C_IDX] = c6;
        last_sent_command_[B_IDX] = 0.0;
        last_sent_command_[C_IDX] = 0.0;

        if (std::abs(motors_[B_IDX].gear_ratio - motors_[C_IDX].gear_ratio) > 1e-6)
        {
            RCLCPP_WARN(LOGGER,
                        "Differential motors have mismatched gear ratios: B=%.6f C=%.6f. "
                        "This can destabilize B/C tracking.",
                        motors_[B_IDX].gear_ratio, motors_[C_IDX].gear_ratio);
        }

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
        constexpr int kMaxPositionAgeMs = 200;
        const double dt = period.seconds();
        auto prev = position_states_;

        for (size_t i = 0; i < MAIN_JOINT_COUNT; ++i)
        {
            if (!can_driver_.isPositionFresh(motors_[i].can_id, kMaxPositionAgeMs))
            {
                RCLCPP_WARN_THROTTLE(
                    LOGGER, *this->get_clock(), 1000,
                    "Stale encoder data for CAN ID %d (joint %s); holding last state.",
                    motors_[i].can_id, motors_[i].name.c_str());
                velocity_states_[i] = 0.0;
                continue;
            }

            int64_t encoder_counts = can_driver_.getPosition(motors_[i].can_id);

            double rad = countsToRadians(encoder_counts, motors_[i].gear_ratio);

            // Normalize only continuous joints (X, A) to [-pi, pi].
            rad = (i == 0 || i == 3) ? angles::normalize_angle(rad) : rad;
            position_states_[i] = std::isfinite(rad) ? rad : 0.0;

            // Calculate velocity with proper bounds checking
            updateJointVelocity(i, prev[i], dt);
        }

        if (!can_driver_.isPositionFresh(motors_[B_IDX].can_id, kMaxPositionAgeMs) ||
            !can_driver_.isPositionFresh(motors_[C_IDX].can_id, kMaxPositionAgeMs))
        {
            RCLCPP_WARN_THROTTLE(
                LOGGER, *this->get_clock(), 1000,
                "Stale encoder data for wrist motors (CAN IDs %d/%d); holding last B/C state.",
                motors_[B_IDX].can_id, motors_[C_IDX].can_id);
            velocity_states_[B_IDX] = 0.0;
            velocity_states_[C_IDX] = 0.0;
        }
        else
        {
            const int64_t c5 = can_driver_.getPosition(motors_[B_IDX].can_id);
            const int64_t c6 = can_driver_.getPosition(motors_[C_IDX].can_id);
            (void)can_driver_.queryPosition(motors_[B_IDX].can_id);
            (void)can_driver_.queryPosition(motors_[C_IDX].can_id);
            double m5_u = countsToRadians(c5, motors_[B_IDX].gear_ratio);
            double m6_u = countsToRadians(c6, motors_[C_IDX].gear_ratio) * M6_SIGN;

            // zero-relative
            if (wrist_zero_set_)
            {
                m5_u -= m5_zero_;
                m6_u -= m6_zero_;
            }

            const WristJointState wrist_state = motorsToWristJoints(m5_u, m6_u, DIFF_GAIN);
            position_states_[B_IDX] = std::clamp(wrist_state.b, motors_[B_IDX].min, motors_[B_IDX].max);
            position_states_[C_IDX] = std::clamp(wrist_state.c, motors_[C_IDX].min, motors_[C_IDX].max);
            updateJointVelocity(B_IDX, prev[B_IDX], dt);
            updateJointVelocity(C_IDX, prev[C_IDX], dt);

            RCLCPP_INFO_THROTTLE(
                LOGGER, *this->get_clock(), 500,
                "FBK B=%.2f deg, C=%.2f deg, m5_u=%.2f deg, m6_u=%.2f deg, c5=%ld c6=%ld",
                angles::to_degrees(position_states_[B_IDX]),
                angles::to_degrees(position_states_[C_IDX]),
                angles::to_degrees(m5_u),
                angles::to_degrees(m6_u),
                static_cast<long>(c5),
                static_cast<long>(c6));
        }

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
            if (i == 0 || i == 3)
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
        uint16_t rpm = 0;

        for (size_t i = 0; i < MAIN_JOINT_COUNT; ++i)
        {
            const double safe_cmd = std::clamp(position_commands_[i], motors_[i].min, motors_[i].max);
            if (!std::isfinite(safe_cmd))
                continue;
            if (std::abs(safe_cmd - last_sent_command_[i]) > threshold_joint(motors_[i].gear_ratio))
            {
                int32_t target_pos = radiansToCounts(safe_cmd, motors_[i].gear_ratio);
                rpm = static_cast<uint16_t>(clampRpm(velocity_commands_[i], motors_[i], 50.0, 2000.0));
                can_driver_.runPositionAbs(motors_[i].can_id, rpm, motors_[i].acc, target_pos);
                last_sent_command_[i] = safe_cmd;
            }
        }

        // ----- desired joints (relative) -----
        if (!std::isfinite(position_commands_[B_IDX]) || !std::isfinite(position_commands_[C_IDX]))
            return hw::return_type::OK;
        const double B = std::clamp(position_commands_[B_IDX], motors_[B_IDX].min + B_EPS, motors_[B_IDX].max - B_EPS);
        const double C = std::clamp(position_commands_[C_IDX], motors_[C_IDX].min, motors_[C_IDX].max);

        if (!wrist_zero_set_)
            return hw::return_type::OK;
        const WristMotorState wrist_target = wristJointsToMotors(
            B, C, m5_zero_, m6_zero_, DIFF_GAIN, M6_SIGN,
            *this, motors_[B_IDX].gear_ratio, motors_[C_IDX].gear_ratio);

        if (std::abs(wrist_target.c5 - last_sent_counts_[B_IDX]) > WRIST_COUNT_EPS ||
            std::abs(wrist_target.c6 - last_sent_counts_[C_IDX]) > WRIST_COUNT_EPS)
        {
            const int32_t dc5 = wrist_target.c5 - last_sent_counts_[B_IDX];
            const int32_t dc6 = wrist_target.c6 - last_sent_counts_[C_IDX];
            double vb = velocity_commands_[B_IDX];
            double vc = velocity_commands_[C_IDX];
            if (!std::isfinite(vb))
                vb = 0.0;
            if (!std::isfinite(vc))
                vc = 0.0;

            // Differential kinematics in velocity form:
            // m5_dot = gain * (B_dot + C_dot), m6_dot = gain * (B_dot - C_dot).
            double m5_vel = std::abs(DIFF_GAIN * (vb + vc));
            double m6_vel = std::abs(DIFF_GAIN * (vb - vc));

            // Fallback when trajectory velocity is absent/near-zero.
            if (m5_vel < 1e-4 && m6_vel < 1e-4)
            {
                m5_vel = std::abs(motors_[B_IDX].vel);
                m6_vel = std::abs(motors_[C_IDX].vel);
            }

            const uint16_t rpm5 = static_cast<uint16_t>(clampRpm(m5_vel, motors_[B_IDX], 50.0, 2000.0));
            const uint16_t rpm6 = static_cast<uint16_t>(clampRpm(m6_vel, motors_[C_IDX], 50.0, 2000.0));
            const bool ok5 = can_driver_.runPositionAbs(motors_[B_IDX].can_id, rpm5, motors_[B_IDX].acc, wrist_target.c5);
            const bool ok6 = can_driver_.runPositionAbs(motors_[C_IDX].can_id, rpm6, motors_[C_IDX].acc, wrist_target.c6);

            if (ok5 && ok6)
            {
                last_sent_counts_[B_IDX] = wrist_target.c5;
                last_sent_command_[B_IDX] = B;
                last_sent_counts_[C_IDX] = wrist_target.c6;
                last_sent_command_[C_IDX] = C;
            }
            if (!ok5 || !ok6)
            {
                RCLCPP_WARN_THROTTLE(
                    LOGGER, *this->get_clock(), 1000,
                    "Wrist command send failed (ok5=%d ok6=%d, rpm5=%u rpm6=%u, c5=%d c6=%d)",
                    ok5 ? 1 : 0, ok6 ? 1 : 0, rpm5, rpm6, wrist_target.c5, wrist_target.c6);
            }

            RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 500,
                                 "CMD B=%.2f deg, C=%.2f deg, m5_abs=%.2f deg, m6_abs=%.2f deg, c5=%d c6=%d, dc5=%d dc6=%d",
                                 angles::to_degrees(B), angles::to_degrees(C),
                                 angles::to_degrees(wrist_target.m5_abs), angles::to_degrees(wrist_target.m6_abs),
                                 wrist_target.c5, wrist_target.c6, dc5, dc6);
        }

        // ----- gripper (command-only CAN device, no encoder) -----
        if (gripper_can_enabled_)
        {
            int raw_clamped = mapGripperPositionToRaw(gripper_cmd_, gripper_close_pos_, gripper_open_pos_);
            if (raw_clamped < 0)
                return hw::return_type::OK;
            raw_clamped = 255 - raw_clamped; // invert to match MoveIt goal state

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
