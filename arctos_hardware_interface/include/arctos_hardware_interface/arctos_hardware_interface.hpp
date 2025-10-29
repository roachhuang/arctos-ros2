#ifndef ARCTOS_HARDWARE_INTERFACE_HPP
#define ARCTOS_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
// #include "arctos_hardware_interface/can_driver.hpp"
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
    // ArctosHardwareInterface();
    // ~ArctosHardwareInterface();
    static constexpr int ENCODER_CPR = 16384; // counts per revolution    

    // SystemInterface overrides
    CallbackReturn on_init(const hw::HardwareComponentInterfaceParams &params) override;
    hw::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hw::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Lifecycle overrides
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    // CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hw::StateInterface> export_state_interfaces() override;
    std::vector<hw::CommandInterface> export_command_interfaces() override;

  private:
    std::unique_ptr<ServoCanSimple> can_driver_; 
    
    std::size_t num_joints_;
    uint16_t vel_;
    uint8_t accel_;

    // read from URDF
    std::vector<int> can_ids_;
    std::vector<double> gear_ratios_;
    std::string can_iface_;

    // std::shared_ptr<ServoCan> can_driver_;

    // Joint data
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> position_commands_;
    std::vector<double> prev_position_commands_;

    // Joint configuration is read from URDF via info_.joints
    // Conversion constants (adjust per your motor setup)
    // const double count_to_rad_ = (2.0 * M_PI) / ENCODER_CPR;
    // const double rad_to_count_ = 1.0 / count_to_rad_;
    // const double rpm_to_radps_ = 2.0 * M_PI / 60.0;

    // CallbackReturn disconnect_hardware();
  };

} // namespace arctos_hardware_interface

#endif // ARCTOS_HARDWARE_INTERFACE_HPP
