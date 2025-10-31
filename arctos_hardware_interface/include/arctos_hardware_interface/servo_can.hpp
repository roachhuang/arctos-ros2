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
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/select.h>
#include <rclcpp/rclcpp.hpp>

class ServoCanSimple
{
public:
    static constexpr uint16_t MAX_RPM = 3000;
    static constexpr uint16_t MAX_CAN_ID = 0x7FF;
    static constexpr int DEFAULT_TIMEOUT_MS = 10;
    static constexpr int SOCKET_TIMEOUT_US = 10000;
    
    ServoCanSimple() = default;

    bool connect(const std::string &can_interface = "can0")
    {
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServoCanSimple"), "Failed to create CAN socket");
            return false;
        }

        struct ifreq ifr{};
        std::strcpy(ifr.ifr_name, can_interface.c_str());
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServoCanSimple"), "CAN interface not found: %s", can_interface.c_str());
            close(sock_);
            sock_ = -1;
            return false;
        }

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

        struct timeval timeout{0, SOCKET_TIMEOUT_US};
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        RCLCPP_INFO(rclcpp::get_logger("ServoCanSimple"), "Connected to CAN: %s", can_interface.c_str());
        return true;
    }

    bool sendCmd(uint16_t can_id, uint8_t code,
                 const std::vector<uint8_t> &params = {},
                 bool expect_resp = false,
                 std::vector<uint8_t> *out_resp = nullptr,
                 int timeout_ms = DEFAULT_TIMEOUT_MS)
    {
        if (can_id > MAX_CAN_ID)
            throw std::invalid_argument("CAN ID exceeds maximum value");

        std::vector<uint8_t> data;
        data.push_back(code);
        data.insert(data.end(), params.begin(), params.end());
        data.push_back(calcCrc(can_id, data));

        struct can_frame tx{};
        tx.can_id = can_id;
        tx.can_dlc = data.size();
        std::memcpy(tx.data, data.data(), tx.can_dlc);

        if (write(sock_, &tx, sizeof(tx)) != sizeof(tx))
            return false;

        if (!expect_resp)
            return true;

        struct can_frame rx{};
        if (!readFrame(rx, timeout_ms) || rx.can_dlc < 2)
            return false;

        uint8_t crc_recv = rx.data[rx.can_dlc - 1];
        uint8_t crc_calc = calcCrc(rx.can_id, {rx.data, rx.data + rx.can_dlc - 1});
        if (crc_recv != crc_calc)
        {
            RCLCPP_WARN(rclcpp::get_logger("ServoCanSimple"), "CRC mismatch");
            return false;
        }
        
        if (out_resp)
            out_resp->assign(rx.data, rx.data + rx.can_dlc - 1);
        return true;
    }

    bool enableMotor(uint16_t id, bool enable)
    {
        return sendSimple(id, 0xF3, {uint8_t(enable)});
    }

    bool queryStatus(uint16_t id, uint8_t &status)
    {
        std::vector<uint8_t> response;
        if (!sendCmd(id, 0xF1, {}, true, &response) || response.size() < 2)
            return false;
        status = response[1];
        return true;
    }

    std::optional<int16_t> readSpeed(uint16_t id)
    {
        std::vector<uint8_t> response;
        if (!sendCmd(id, 0x32, {}, true, &response) || response.size() < 3)
            return std::nullopt;
        return toI16(&response[1]);
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
            uint8_t(position & 0xFF)
        };
        return sendSimple(id, 0xF5, params);
    }

    bool runPositionRel(uint16_t id, bool dir_ccw, uint16_t speed, uint8_t accel, int32_t relative_pos)
    {
        speed = std::min(speed, MAX_RPM);
        uint8_t speed_high = (dir_ccw ? 0x80 : 0x00) | ((speed >> 8) & 0x0F);
        uint8_t speed_low = speed & 0xFF;
        
        std::vector<uint8_t> params{
            speed_high, speed_low, accel,
            uint8_t(relative_pos & 0xFF),
            uint8_t((relative_pos >> 8) & 0xFF),
            uint8_t((relative_pos >> 16) & 0xFF)
        };
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

    std::optional<int64_t> readAbsolutePosition(uint16_t id)
    {
        std::vector<uint8_t> response;
        if (!sendCmd(id, 0x31, {}, true, &response) || response.size() < 7)
            return std::nullopt;

        return toI48(&response[1]);
    }

private:
    int sock_{-1};

    bool sendSimple(uint16_t id, uint8_t code, const std::vector<uint8_t> &params)
    {
        std::vector<uint8_t> response;
        return sendCmd(id, code, params, true, &response) && 
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
            (uint64_t)p[4] << 8  |
            (uint64_t)p[5];

        constexpr uint64_t SIGN_BIT_MASK = 0x0000800000000000ULL;
        constexpr uint64_t SIGN_EXTEND_MASK = 0xFFFF000000000000ULL;
        
        if (raw_value & SIGN_BIT_MASK)
            raw_value |= SIGN_EXTEND_MASK;

        return static_cast<int64_t>(raw_value);
    }

    bool readFrame(struct can_frame &rx_frame, int timeout_ms)
    {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sock_, &read_fds);
        
        struct timeval timeout{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
        
        if (select(sock_ + 1, &read_fds, nullptr, nullptr, &timeout) <= 0)
            return false;
        
        return read(sock_, &rx_frame, sizeof(rx_frame)) == sizeof(rx_frame);
    }
};