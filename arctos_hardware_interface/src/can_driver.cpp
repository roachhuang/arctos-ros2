#include "arctos_hardware_interface/can_driver.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>

namespace arctos_hardware_interface
{

CanDriver::CanDriver() : can_socket_(-1), connected_(false)
{
    // Initialize servos with CAN IDs 1-6
    for (size_t i = 0; i < can_ids_.size(); ++i) {
        servos_.emplace_back(can_ids_[i], 1.0); // CAN ID, gear ratio
    }
}

CanDriver::~CanDriver()
{
    disconnect();
}

bool CanDriver::connect(const std::string& can_interface)
{
    try {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Failed to create CAN socket");
            return false;
        }

        struct ifreq ifr{};
        std::strcpy(ifr.ifr_name, can_interface.c_str());
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Failed to get CAN interface index");
            close(can_socket_);
            return false;
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Failed to bind CAN socket");
            close(can_socket_);
            return false;
        }

        // Set timeout
        struct timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms
        setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        connected_ = true;
        RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "Connected to CAN interface: %s", can_interface.c_str());
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Connection failed: %s", e.what());
        return false;
    }
}

void CanDriver::disconnect()
{
    if (connected_ && can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
        connected_ = false;
        RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "Disconnected from CAN");
    }
}

bool CanDriver::is_connected() const
{
    return connected_ && can_socket_ >= 0;
}

bool CanDriver::send_commands(const std::vector<double>& joint_positions)
{
    if (!is_connected() || joint_positions.size() != servos_.size()) {
        return false;
    }

    try {
        for (size_t i = 0; i < servos_.size(); ++i) {
            can_frame cmd = servos_[i].absoluteMotionRad(joint_positions[i], 100, 10);
            safe_send_can(cmd);
        }
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Send commands failed: %s", e.what());
        return false;
    }
}

bool CanDriver::read_positions(std::vector<double>& joint_positions)
{
    if (!is_connected()) {
        return false;
    }
    
    joint_positions.resize(servos_.size());

    try {
        // Send read requests - cmd 31
        for (auto& servo : servos_) {
            can_frame req = servo.readEncoderAbsolute();
            safe_send_can(req);
        }

        // Collect responses
        size_t responses_received = 0;
        while (responses_received < servos_.size()) {
            can_frame resp = safe_receive_can();
            if (resp.can_id == 0) break; // timeout

            // Find servo by CAN ID
            for (size_t i = 0; i < servos_.size(); ++i) {
                if (can_ids_[i] == resp.can_id) {
                    servos_[i].parseResponse(resp);
                    joint_positions[i] = servos_[i].getAbsoluteAngle();
                    responses_received++;
                    break;
                }
            }
        }
        return responses_received == servos_.size();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Read positions failed: %s", e.what());
        return false;
    }
}

bool CanDriver::activate()
{
    if (!is_connected()) return false;

    try {
        for (auto& servo : servos_) {
            can_frame cmd = servo.enableServo(true);
            safe_send_can(cmd);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Activate failed: %s", e.what());
        return false;
    }
}

bool CanDriver::deactivate()
{
    if (!is_connected()) return false;

    try {
        for (auto& servo : servos_) {
            can_frame cmd = servo.enableServo(false);
            safe_send_can(cmd);
        }
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Deactivate failed: %s", e.what());
        return false;
    }
}

void CanDriver::safe_send_can(const can_frame& frame)
{
    std::lock_guard<std::mutex> lock(can_mutex_);
    ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        throw std::runtime_error("Failed to send CAN frame");
    }
}

can_frame CanDriver::safe_receive_can()
{
    can_frame frame{};
    ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));
    if (nbytes < 0) {
        frame.can_id = 0; // indicate timeout/error
    }
    return frame;
}

} // namespace arctos_hardware_interface