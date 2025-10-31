#ifndef ARCTOS_HARDWARE_INTERFACE_HPP
#define ARCTOS_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "arctos_hardware_interface/servo_can.hpp"

#include <vector>
#include <memory>

namespace hw = hardware_interface;

namespace arctos_hardware_interface
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class ArctosHardwareInterface : public hw::SystemInterface
  {
  public:
    static constexpr int ENCODER_CPR = 16384;
    static constexpr double POSITION_CHANGE_THRESHOLD = 0.01;
    static constexpr double VELOCITY_EPSILON = 1e-9;
    static constexpr double TWO_PI = 2.0 * M_PI;

    // SystemInterface overrides
    CallbackReturn on_init(const hw::HardwareComponentInterfaceParams &params) override;
    hw::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hw::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Lifecycle overrides
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hw::StateInterface> export_state_interfaces() override;
    std::vector<hw::CommandInterface> export_command_interfaces() override;

  private:
    // Core components
    std::unique_ptr<ServoCanSimple> can_driver_;
    
    // Configuration
    std::size_t num_joints_;
    uint16_t vel_;
    uint8_t accel_;
    std::string can_interface_;

    // Joint parameters from URDF
    std::vector<int> can_ids_;
    std::vector<double> gear_ratios_;

    // Joint state data
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> position_commands_;
    std::vector<double> prev_position_commands_;

    // Helper methods
    void initializeJointData();
    void loadJointParameters();
    bool loadHardwareParameters();
    bool connectToCanInterface();
    void readInitialPositions();
    
    bool readJointPosition(size_t joint_index);
    void updateJointVelocity(size_t joint_index, double prev_position, double dt);
    bool hasCommandsChanged() const;
    void sendPositionCommands();
    
    // Conversion utilities
    inline double countsToRadians(int64_t counts, double gear_ratio) const
    {
        return static_cast<double>(counts) / ENCODER_CPR * TWO_PI / gear_ratio;
    }
    
    inline int32_t radiansToCount(double radians, double gear_ratio) const
    {
        return static_cast<int32_t>(radians * gear_ratio / TWO_PI * ENCODER_CPR);
    }
  };

} // namespace arctos_hardware_interface

#endif // ARCTOS_HARDWARE_INTERFACE_HPP