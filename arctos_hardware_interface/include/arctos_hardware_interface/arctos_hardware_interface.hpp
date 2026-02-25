/**
 * @file arctos_hardware_interface.hpp
 * @brief ROS2 hardware interface for Arctos arm and gripper.
 */
#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "arctos_hardware_interface/mks_servo_driver.hpp"

#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <cstdint>

namespace hw = hardware_interface;

namespace arctos_hardware_interface
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @class ArctosHardwareInterface
   * @brief ROS2 hardware interface for Arctos arm via MKS CAN servo drivers
   *
   * Manages communication with 6-DOF arm + 2-DOF gripper through CAN interface.
   * Implements ros2_control SystemInterface for trajectory execution.
   *
   * @note Requires CAN interface to be available at on_configure() time
   * @note All joint parameters (CAN ID, gear ratio) must be defined in URDF
   */
  class ArctosHardwareInterface : public hw::SystemInterface
  {
  public:
    static constexpr double M6_SIGN = -1.0;
    static constexpr double WRIST_DIFF_GAIN = 0.5;
    // (counts per revolution for the encoder), step size = 2pi/16384 radians ~ 0.00038 rads
    static constexpr int ENCODER_COUNTS_PER_REVOLUTION = 16384;

    // 1. Calculate the smallest physical step
    inline double min_step_joint(double gear_ratio)
    {
      return TWO_PI / (gear_ratio * ENCODER_COUNTS_PER_REVOLUTION);
    }
    inline double threshold_joint(double gear_ratio)
    {
      return 3.0 * min_step_joint(gear_ratio);
    }

    // static constexpr double POSITION_CHANGE_THRESHOLD = 0.001; // radians
    static constexpr double VELOCITY_EPSILON = 1e-9;
    static constexpr double TWO_PI = 2.0 * M_PI;

    // Conversion utilities
    inline double countsToRadians(int64_t counts, double gear_ratio) const
    {
      return (static_cast<double>(counts) * TWO_PI) / (gear_ratio * ENCODER_COUNTS_PER_REVOLUTION);
    }

    inline int32_t radiansToCounts(double radians, double gear_ratio) const
    {
      return (int32_t)llround(radians * gear_ratio * ENCODER_COUNTS_PER_REVOLUTION / TWO_PI);
    }

    // SystemInterface overrides
    /// @brief Initialize hardware interface with parameters from ROS2 control.
    CallbackReturn on_init(const hw::HardwareComponentInterfaceParams &params) override;
    /// @brief Read latest joint states from hardware.
    hw::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    /// @brief Write commanded joint targets to hardware.
    hw::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Lifecycle nodes overrides
    /// @brief Configure hardware (connect CAN, load parameters).
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    /// @brief Activate hardware and synchronize joint state.
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    /// @brief Deactivate hardware and release resources.
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    /// @brief Export state interfaces for joints and gripper.
    std::vector<hw::StateInterface> export_state_interfaces() override;
    /// @brief Export command interfaces for joints and gripper.
    std::vector<hw::CommandInterface> export_command_interfaces() override;

  private:
    // Core components
    mks_servo_driver::MksServoDriver can_driver_;
    static constexpr size_t DOF = 6;
    
    // Configuration
    std::size_t num_joints_;
    std::string can_interface_;
    // std::vector<double> last_joint_command_;  // size = DOF (B, C in joint space)
    std::vector<double> last_sent_command_; // size = DOF (only valid for motors)
    struct MotorConfig
    {
      std::string name;
      uint8_t can_id{0};
      double gear_ratio{0.0};
      double vel{0.0};
      double min_vel{0.0};
      double max_vel{0.0};
      double acc{0.0};
      double min{0.0};
      double max{0.0};
    };
    std::array<MotorConfig, DOF> motors_;
    double m5_zero_{0.0};
    double m6_zero_{0.0};
    bool wrist_zero_set_{false};

    std::vector<int32_t> last_sent_counts_;

    // Joint state data
    std::vector<std::string> arm_joint_names_{"X_joint", "Y_joint", "Z_joint", "A_joint", "B_joint", "C_joint"};
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> position_commands_;
    std::vector<double> velocity_commands_;
    double gripper_cmd_ = 0.0;
    double gripper_vel_ = 0.0;
    double gripper_pos_ = 0.0;
    uint16_t gripper_can_id_{7};
    double gripper_open_pos_{0.019};
    double gripper_close_pos_{-0.010};
    int gripper_sock_{-1};
    bool gripper_can_enabled_{false};
    int gripper_last_raw_{-1}; // -1 unknown, 0-255 last sent position


    // Helper methods
    void loadHardwareParameters();
    bool sendCheckedCanCommand(uint16_t id, uint8_t cmd, const std::vector<uint8_t> &params, const char *label, int timeout_ms = 150);
    bool configureCanId6Startup();

    void updateJointVelocity(size_t joint_index, double prev_position, double dt);

    static double clampRpm(double desired_joint_vel, const MotorConfig &motor, double default_min_rpm, double default_max_rpm);
    // Returns -1 when cmd is not finite; otherwise returns 0-255.
    static int mapGripperPositionToRaw(double cmd, double close_pos, double open_pos);

    bool openGripperCanSocket();
    void closeGripperCanSocket();
    bool sendGripperFrame(const std::vector<uint8_t> &data);
  };

} // namespace arctos_hardware_interface
