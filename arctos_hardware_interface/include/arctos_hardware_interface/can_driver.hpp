#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <linux/can.h>
#include <rclcpp/rclcpp.hpp>
#include "arctos_hardware_interface/ServoManager.hpp"

namespace arctos_hardware_interface
{

class CanDriver
{
public:
    CanDriver();
    ~CanDriver();

    bool connect(const std::string& can_interface = "can0");
    void disconnect();
    bool is_connected() const;

    bool send_commands(const std::vector<double>& joint_positions);
    bool read_positions(std::vector<double>& joint_positions);
    bool activate();
    bool deactivate();

private:
    int can_socket_;
    bool connected_;
    std::mutex can_mutex_;
    std::vector<ServoManager> servos_;
    std::vector<uint8_t> can_ids_{1, 2, 3, 4, 5, 6};

    bool safe_send_can(const can_frame& frame);
    can_frame safe_receive_can();
    bool initialize_servos();
};

} // namespace arctos_hardware_interface