#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "arctos_hardware_interface/mks_servo_driver.hpp"

#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <cmath>

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
    CallbackReturn on_init(const hw::HardwareComponentInterfaceParams &params) override;
    hw::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hw::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Lifecycle nodes overrides
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hw::StateInterface> export_state_interfaces() override;
    std::vector<hw::CommandInterface> export_command_interfaces() override;

  private:
    // Core components
    // std::shared_ptr<ServoCanSimple> can_driver_;
    mks_servo_driver::MksServoDriver can_driver_;
    // ----- motor6 sign (apply ONCE everywhere) -----

    // Configuration
    std::size_t num_joints_;
    // note that vel_ is motor RPM, not joint rad/s
    uint16_t vel_;  // 0-3000 RPM
    uint8_t accel_; // 0-255
    std::string can_interface_;
    rclcpp::Duration send_accum_{0, 0};
    // std::vector<double> last_joint_command_;  // size = DOF (B, C in joint space)
    std::vector<double> last_sent_command_; // size = DOF (only valid for motors)
    // Joint parameters from URDF
    std::vector<u_int8_t> can_ids_;
    std::vector<double> gear_ratios_;
    std::vector<double> min_;
    std::vector<double> max_;
    double m5_zero_{0.0};
    double m6_zero_{0.0};
    bool wrist_zero_set_{false};

    rclcpp::Time last_send_time_;
    std::vector<int32_t> last_sent_counts_;

    // Joint state data
    static constexpr size_t DOF = 6;
    std::vector<std::string> arm_joint_names_{"X_joint", "Y_joint", "Z_joint", "A_joint", "B_joint", "C_joint"};
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    // std::vector<double> effort_states_;
    std::vector<double> position_commands_;
    double gripper_cmd_ = 0.0;
    double gripper_vel_ = 0.0;
    double gripper_pos_ = 0.0;

    // std::vector<double> prev_position_commands_;
    std::vector<bool> is_homing_;
    // IN_1 (home lmt)
    // std::vector<bool> in1_;
    // IN_2 (end lmt)
    // std::vector<bool> in2_;

    // Group related state together (single responsibility)
    struct JointState
    {
      double position;
      double velocity;
      double command;
      bool is_homing;
      bool limit_switch_in1;
      bool limit_switch_in2;
    };
    std::vector<JointState> _joint_states;

    // Helper methods
    void initializeJointData();
    // void loadJointParameters();
    void loadHardwareParameters();
    bool connectToCanInterface();
    void readInitialPositions();

    bool readJointPosition(size_t joint_index);
    void updateJointVelocity(size_t joint_index, double prev_position, double dt);
    // bool hasCommandsChanged() const;
    void sendPositionCommands();
  };

} // namespace arctos_hardware_interface
