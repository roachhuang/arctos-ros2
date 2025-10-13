#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
// #include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <sys/ioctl.h>
#include <cstring>

namespace arctos_hardware_interface
{
    // Lifecycle init
    CallbackReturn ArctosHardwareInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        try
        {
            // Clear old data
            servo_map_.clear();
            servo_order_.clear();

            size_t num_motors = info_.joints.size();

            // Initialize ServoManager objects and wrappers
            for (size_t i = 0; i < num_motors; ++i)
            {
                uint8_t can_id = static_cast<uint8_t>(i + 1); // CAN IDs 1..6
                double gear_ratio = 1.0;                      // could be taken from info.hardware_parameters
                auto manager = std::make_shared<ServoManager>(can_id, gear_ratio);

                ServoWrapper wrapper;
                wrapper.manager = manager;

                servo_map_[can_id] = wrapper;
                servo_order_.push_back(can_id);
            }

            // --- Initialize CAN socket on can0 ---
            can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (can_socket_ < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                             "Failed to create CAN socket");
                return CallbackReturn::ERROR;
            }

            struct ifreq ifr{};
            strcpy(ifr.ifr_name, "can0");
            if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                             "Failed to get CAN interface index for can0");
                return CallbackReturn::ERROR;
            }

            struct sockaddr_can addr{};
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                             "Failed to bind CAN socket to can0");
                return CallbackReturn::ERROR;
            }

            // Optional: set non-blocking read timeout (10 ms)
            struct timeval timeout{};
            timeout.tv_sec = 0;
            timeout.tv_usec = 10000;
            setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

            RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                        "CAN socket successfully initialized on can0. Initialized %zu servos.", servo_order_.size());

            // Log servo_order_ contents
            std::string servo_list = "Servo order: ";
            for (auto can_id : servo_order_)
            {
                servo_list += std::to_string(can_id) + " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "%s", servo_list.c_str());

            // Enable all servos
            for (auto can_id : servo_order_)
            {
                can_frame enable_frame;
                enable_frame.can_id = can_id;
                enable_frame.can_dlc = 8;
                enable_frame.data[0] = 0xF3;
                enable_frame.data[1] = 0x01;
                enable_frame.data[2] = 0x00;
                enable_frame.data[3] = 0x00;
                enable_frame.data[4] = 0x00;
                enable_frame.data[5] = 0x00;
                enable_frame.data[6] = 0x00;
                enable_frame.data[7] = 0xF4;

                RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Enabling servo CAN_ID=%d", can_id);
                sendCAN(enable_frame);

                // Small delay between commands
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Failed to initialize ServoManager: %s", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    // Export state interfaces
    std::vector<hardware_interface::StateInterface> ArctosHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        std::vector<std::string> joint_names = {"X_joint", "Y_joint", "Z_joint", "A_joint", "B_joint", "C_joint"};

        for (size_t idx = 0; idx < servo_order_.size(); ++idx)
        {
            uint8_t can_id = servo_order_[idx];
            auto it = servo_map_.find(can_id);
            if (it == servo_map_.end())
                continue;

            state_interfaces.emplace_back(joint_names[idx], "position", &it->second.current_angle);
            state_interfaces.emplace_back(joint_names[idx], "velocity", &it->second.current_velocity);
        }

        return state_interfaces;
    }

    // Export command interfaces
    std::vector<hardware_interface::CommandInterface> ArctosHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        std::vector<std::string> joint_names = {"X_joint", "Y_joint", "Z_joint", "A_joint", "B_joint", "C_joint"};

        for (size_t idx = 0; idx < servo_order_.size(); ++idx)
        {
            uint8_t can_id = servo_order_[idx];
            auto it = servo_map_.find(can_id);
            if (it == servo_map_.end())
                continue;

            command_interfaces.emplace_back(joint_names[idx], "position", &it->second.target_angle);
        }

        return command_interfaces;
    }

    // Read joint states from hardware
    return_type ArctosHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "on read...");
        // 1. Send read requests to all servos
        for (auto can_id : servo_order_)
        {
            auto &wrapper = servo_map_[can_id];
            can_frame req = wrapper.manager->readEncoderAbsolute();
            sendCAN(req);
        }

        // 2. Collect replies for all servos
        size_t replies_needed = servo_order_.size();
        size_t replies_got = 0;

        while (replies_got < replies_needed)
        {
            can_frame resp = receiveCAN();
            if (resp.can_id == 0)
                break; // timeout or no frame

            auto it = servo_map_.find(resp.can_id);
            if (it != servo_map_.end())
            {
                it->second.manager->parseResponse(resp);

                // Use wrapped angle for ROS
                it->second.current_angle = it->second.manager->getAbsoluteAngle();
                it->second.current_velocity = it->second.manager->getVelocityRadPerSec();
                ++replies_got;
            }
        }

        return return_type::OK;
    }

    // Write commands to hardware
    return_type ArctosHardwareInterface::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        if (servo_order_.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("ArctosHardwareInterface"), "servo_order_ is empty in write()");
            return return_type::OK;
        }

        for (auto can_id : servo_order_)
        {
            auto &wrapper = servo_map_[can_id];
            wrapper.manager->setTargetAngle(wrapper.target_angle);
            can_frame cmd = wrapper.manager->absoluteMotionRad(wrapper.target_angle, speed_, accel_);
            RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Write cmd: CAN_ID=%d, target=%.3f", can_id, wrapper.target_angle);
            sendCAN(cmd);
        }

        return return_type::OK;
    }

    // --- CAN read/write ---
    void ArctosHardwareInterface::sendCAN(const can_frame &frame)
    {
        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"), "CAN socket not initialized");
            return;
        }

        // Log CAN frame details
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                    "Sending CAN: ID=0x%03X, DLC=%d, Data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
                    frame.can_id, frame.can_dlc,
                    frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                    frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

        ssize_t nbytes = ::write(can_socket_, &frame, sizeof(frame));
        if (nbytes != static_cast<ssize_t>(sizeof(frame)))
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Failed to send CAN frame: %s", std::strerror(errno));
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "CAN frame sent successfully");
        }
    }

    can_frame ArctosHardwareInterface::receiveCAN()
    {
        can_frame frame{};
        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"), "CAN socket not initialized");
            return frame;
        }

        ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));
        if (nbytes < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                             "Error reading CAN frame: %s", strerror(errno));
            }
            return frame;
        }
        else if (nbytes < static_cast<ssize_t>(sizeof(frame)))
        {
            RCLCPP_WARN(rclcpp::get_logger("ArctosHardwareInterface"),
                        "Incomplete CAN frame received");
        }

        return frame;
    }

} // namespace arctos_hardware_interface
// Export plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hardware_interface::SystemInterface)
