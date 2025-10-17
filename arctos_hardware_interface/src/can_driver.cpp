#include "arctos_hardware_interface/can_driver.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <thread>

namespace arctos_hardware_interface
{

    CanDriver::CanDriver() : can_socket_(-1), connected_(false)
    {
        servos_.reserve(can_ids_.size());
        for (uint8_t id : can_ids_)
        {
            servos_.emplace_back(id, 1.0);
        }
    }

    CanDriver::~CanDriver()
    {
        disconnect();
    }

    bool CanDriver::connect(const std::string &can_interface)
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Failed to create CAN socket");
            return false;
        }

        struct ifreq ifr{};
        std::strcpy(ifr.ifr_name, can_interface.c_str());
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Failed to get CAN interface index");
            close(can_socket_);
            return false;
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Failed to bind CAN socket");
            close(can_socket_);
            return false;
        }

        struct timeval timeout{0, 10000};
        setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        connected_ = true;
        RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "Connected to CAN: %s", can_interface.c_str());
        return true;
    }

    void CanDriver::disconnect()
    {
        if (connected_ && can_socket_ >= 0)
        {
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

    bool CanDriver::send_commands(const std::vector<double> &joint_positions)
    {
        if (!is_connected() || joint_positions.size() != servos_.size())
            return false;

        for (size_t i = 0; i < servos_.size(); ++i)
        {
            can_frame cmd = servos_[i].absoluteMotionRad(joint_positions[i], 100, 10);
            if (!safe_send_can(cmd))
                return false;
        }
        return true;
    }

    bool CanDriver::read_positions(std::vector<double> &joint_positions)
    {
        if (!is_connected())
            return false;

        joint_positions.resize(servos_.size());

        for (auto &servo : servos_)
        {
            can_frame req = servo.readEncoderAbsolute();
            if (!safe_send_can(req))
                return false;
        }

        size_t responses = 0;
        while (responses < servos_.size())
        {
            can_frame resp = safe_receive_can();
            if (resp.can_id == 0)
                break;

            for (size_t i = 0; i < servos_.size(); ++i)
            {
                if (can_ids_[i] == resp.can_id)
                {
                    servos_[i].parseResponse(resp);
                    joint_positions[i] = servos_[i].getAbsoluteAngle();
                    responses++;
                    break;
                }
            }
        }
        return responses == servos_.size();
    }

    bool CanDriver::activate()
    {
        if (!is_connected())
            return false;

        for (auto &servo : servos_)
        {
            can_frame cmd = servo.enableServo(true);
            if (!safe_send_can(cmd))
                return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return true;
    }

    bool CanDriver::deactivate()
    {
        if (!is_connected())
            return false;

        for (auto &servo : servos_)
        {
            can_frame cmd = servo.enableServo(false);
            if (!safe_send_can(cmd))
                return false;
        }
        return true;
    }

    bool CanDriver::safe_send_can(const can_frame &frame)
    {
        std::lock_guard<std::mutex> lock(can_mutex_);
        return ::write(can_socket_, &frame, sizeof(frame)) == sizeof(frame);
    }

    can_frame CanDriver::safe_receive_can()
    {
        can_frame frame{};
        ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));
        if (nbytes < 0)
        {
            frame.can_id = 0; // indicate timeout/error
        }
        return frame;
    }

} // namespace arctos_hardware_interface