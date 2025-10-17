#ifndef ARCTOS_HARDWARE_INTERFACE_HPP
#define ARCTOS_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "arctos_hardware_interface/can_driver.hpp"

#include <vector>
#include <memory>

namespace hw = hardware_interface;

namespace arctos_hardware_interface
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class ArctosHardwareInterface : public hw::SystemInterface
  {
  public:
    ArctosHardwareInterface();
    ~ArctosHardwareInterface();

    // SystemInterface overrides
    CallbackReturn on_init(const hw::HardwareComponentInterfaceParams & params) override;
    hw::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hw::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Lifecycle overrides
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hw::StateInterface> export_state_interfaces() override;
    std::vector<hw::CommandInterface> export_command_interfaces() override;

  private:
    std::unique_ptr<CanDriver> driver_;
    std::size_t num_joints_;
    uint16_t vel_;
    uint8_t accel_;

    // Joint data
    std::vector<double> position_commands_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> prev_position_commands_;

    // Joint configuration is read from URDF via info_.joints

    CallbackReturn disconnect_hardware();
  };

} // namespace arctos_hardware_interface

#endif // ARCTOS_HARDWARE_INTERFACE_HPP
