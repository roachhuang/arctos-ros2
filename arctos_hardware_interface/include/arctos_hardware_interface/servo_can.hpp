// ServoCanSimple.hpp
// Clean SocketCAN driver for MKS SERVO42 & 57D_CAN (Manual v1.0.6)
// CRC = (CAN_ID + all data bytes before CRC) & 0xFF

#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <optional>
#include <cstring>
#include <stdexcept>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <fcntl.h>
#include <errno.h>
#include <rclcpp/rclcpp.hpp>

class ServoCanSimple
{
public:
    ServoCanSimple() : logger_(rclcpp::get_logger("ServoCanSimple")) {}
    struct ServoState
    {
        double position = 0.0; // Radians
        double velocity = 0.0; // Rad/s
        double effort = 0.0;   // Nm (if available)
        uint8_t status = 0;
        uint8_t error_code = 0;
        rclcpp::Time last_update;
        bool enabled = false;
    };

    static constexpr uint16_t MAX_RPM = 3000;
    static constexpr uint16_t MAX_CAN_ID = 0x7FF;
    static constexpr int DEFAULT_TIMEOUT_MS = 5;

    bool connect(const std::string &can_interface = "can0")
    {
        // 1. Create socket
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServoCanSimple"), "Failed to create CAN socket");
            return false;
        }
        // 2. Safe interface name copy
        struct ifreq ifr{};
        std::strcpy(ifr.ifr_name, can_interface.c_str());

        // 3. Get interface index
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServoCanSimple"), "CAN interface not found: %s", can_interface.c_str());
            close(sock_);
            sock_ = -1;
            return false;
        }
        // 4. Bind socket
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServoCanSimple"), "Failed to bind CAN socket");
            close(sock_);
            sock_ = -1;
            return false;
        }

        // ✅ Make the socket non-blocking
        int flags = fcntl(sock_, F_GETFL, 0);
        if (flags < 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("ServoCanSimple"), "Failed to get socket flags");
        }
        else if (fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("ServoCanSimple"), "Failed to set O_NONBLOCK");
        }

        // ✅ Set CAN filter right here
        struct can_filter filters[6];
        for (int i = 0; i < 6; ++i)
        {
            filters[i].can_id = i + 1; // servo ID 1–6
            filters[i].can_mask = CAN_SFF_MASK;
        }
        setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters));

        RCLCPP_INFO(rclcpp::get_logger("ServoCanSimple"),
                    "Connected to CAN: %s",
                    can_interface.c_str());

        return true;
    }

    bool sendCmd(uint16_t can_id, uint8_t code,
                 const std::vector<uint8_t> &params = {})
    // bool expect_resp = false,
    // std::vector<uint8_t> *out_resp = nullptr,
    //  int timeout_ms = DEFAULT_TIMEOUT_MS)
    {
        if (can_id > MAX_CAN_ID)
            throw std::invalid_argument("CAN ID exceeds maximum value");

        std::vector<uint8_t> data;
        data.push_back(code);
        data.insert(data.end(), params.begin(), params.end());
        data.push_back(calcCrc(can_id, data));

        if (data.size() > 8)
            return false;

        struct can_frame tx{};
        tx.can_id = can_id;
        tx.can_dlc = data.size();
        std::memcpy(tx.data, data.data(), tx.can_dlc);

        if (sock_ < 0)
            return false;

        if (write(sock_, &tx, sizeof(tx)) != sizeof(tx))
            return false;    
            
        return true;
    }

    bool enableMotor(uint16_t id, bool enable)
    {
        return sendSimple(id, 0xF3, {uint8_t(enable)});
    }

    bool queryStatus(uint16_t id, uint8_t &status)
    {
        if (!sendCmd(id, 0xF1)) {
            return false;
        }
        status = 0;  // Status not available in simplified version
        return true;
    }

    bool runSpeedMode(uint16_t id, bool dir_ccw, uint16_t rpm, uint8_t accel = 2)
    {
        rpm = std::min(rpm, MAX_RPM);
        uint8_t speed_high = (dir_ccw ? 0x80 : 0x00) | ((rpm >> 8) & 0x0F);
        uint8_t speed_low = rpm & 0xFF;
        return sendSimple(id, 0xF6, {speed_high, speed_low, accel});
    }

    bool runPositionAbs(uint16_t id, uint16_t speed, uint8_t accel, int32_t position)
    {
        std::vector<uint8_t> params{
            uint8_t(speed >> 8),
            uint8_t(speed & 0xFF),
            accel,
            uint8_t((position >> 16) & 0xFF),
            uint8_t((position >> 8) & 0xFF),
            uint8_t(position & 0xFF)};
        return sendSimple(id, 0xF5, params);
    }

    bool runPositionRel(uint16_t id, bool dir_ccw, uint16_t speed, uint8_t accel, int32_t relative_pos)
    {
        speed = std::min(speed, MAX_RPM);
        uint8_t speed_high = (dir_ccw ? 0x80 : 0x00) | ((speed >> 8) & 0x0F);
        uint8_t speed_low = speed & 0xFF;

        std::vector<uint8_t> params{
            speed_high, speed_low, accel,
            uint8_t((relative_pos >> 16) & 0xFF),
            uint8_t((relative_pos >> 8) & 0xFF),
            uint8_t(relative_pos & 0xFF)};
        return sendSimple(id, 0xFD, params);
    }

    bool setCanId(uint16_t current_id, uint16_t new_id)
    {
        return sendSimple(current_id, 0x8B,
                          {uint8_t(new_id & 0xFF), uint8_t((new_id >> 8) & 0x07)});
    }

    bool clearFault(uint16_t id)
    {
        return sendSimple(id, 0x40, {});
    }

    void deactive()
    {
        if (sock_ >= 0)
        {
            close(sock_);
            sock_ = -1;
        }
    }
    std::optional<int64_t> processAbsolutePosition(uint16_t servo_id, const can_frame &frame)
    {
        if (frame.can_dlc < 7)
        {
            RCLCPP_WARN(logger_, "Short position frame from ID %d: %d bytes",
                        servo_id, frame.can_dlc);
            return std::nullopt;
        }

        // Extract 48-bit absolute position (6 bytes)
        int64_t raw_position = toI48(&frame.data[1]);

        return raw_position;

        // RCLCPP_DEBUG(logger_, "Servo %d position: raw=%ld, angle=%.3f rad",
        //              servo_id, raw_position, output_angle_rad);
    }
   
    std::optional<int64_t> processCanFrame(const can_frame &frame)
    {
        // Validate basic frame structure
        if (frame.can_dlc < 2)
        { // Minimum: command + data
            RCLCPP_WARN(logger_, "Invalid CAN frame length: %d", frame.can_dlc);
            return std::nullopt;
        }

        uint16_t servo_id = frame.can_id;
        uint8_t command = frame.data[0];

        // Update last communication time
        // servo_states_[servo_id].last_update = clock_.now();
        if (command == 0x31)
        {
            return processAbsolutePosition(servo_id, frame);
        }

        return std::nullopt; // Unknown command
    }
  

    bool readAllAvailable(std::vector<can_frame> &frames, int max_frames = 100)
    {
        frames.clear();

        if (sock_ < 0)
        {
            RCLCPP_ERROR(logger_, "CAN socket not connected");
            return false;
        }

        int frames_read = 0;
        bool read_success = true;

        // Read until no more frames available or max_frames reached
        while (frames_read < max_frames)
        {
            can_frame frame;
            int result = readSingleFrame(frame);

            if (result == 1)
            {
                // Successfully read a frame
                frames.push_back(frame);
                frames_read++;
            }
            else if (result == 0)
            {
                // No more data available - normal exit
                break;
            }
            else
            {
                // Error occurred
                read_success = false;
                break;
            }
        }

        if (frames_read > 0)
        {
            RCLCPP_DEBUG(logger_, "Read %d CAN frames", frames_read);
        }

        return read_success;
    }

private:
    rclcpp::Logger logger_;
    int sock_{-1};

    bool sendSimple(uint16_t id, uint8_t code, const std::vector<uint8_t> &params)
    {
        std::vector<uint8_t> response;
        return sendCmd(id, code, params) &&
               response.size() >= 2 && response[1] == 1;
    }

    static uint8_t calcCrc(uint32_t id, const std::vector<uint8_t> &data)
    {
        uint32_t sum = id & 0x7FF;
        for (auto byte : data)
            sum += byte;
        return sum & 0xFF;
    }

    static int16_t toI16(const uint8_t *p)
    {
        return int16_t(p[0] | (p[1] << 8));
    }

    static int64_t toI48(const uint8_t *p)
    {
        uint64_t raw_value =
            (uint64_t)p[0] << 40 |
            (uint64_t)p[1] << 32 |
            (uint64_t)p[2] << 24 |
            (uint64_t)p[3] << 16 |
            (uint64_t)p[4] << 8 |
            (uint64_t)p[5];

        constexpr uint64_t SIGN_BIT_MASK = 0x0000800000000000ULL;
        constexpr uint64_t SIGN_EXTEND_MASK = 0xFFFF000000000000ULL;

        if (raw_value & SIGN_BIT_MASK)
            raw_value |= SIGN_EXTEND_MASK;

        return static_cast<int64_t>(raw_value);
    }

    int readSingleFrame(can_frame &frame)
    {
        ssize_t bytes_read = read(sock_, &frame, sizeof(frame));

        if (bytes_read == sizeof(frame))
        {
            return 1; // Success
        }
        else if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return 0; // No data available - normal for non-blocking
            }
            else
            {
                RCLCPP_ERROR(logger_, "CAN read error: %s", strerror(errno));
                return -1; // Actual error
            }
        }
        else if (bytes_read == 0)
        {
            RCLCPP_WARN(logger_, "CAN socket closed remotely");
            return -1;
        }
        else
        {
            RCLCPP_ERROR(logger_, "Partial CAN frame read: %zd/%zu bytes",
                         bytes_read, sizeof(frame));
            return -1;
        }
    }
};
