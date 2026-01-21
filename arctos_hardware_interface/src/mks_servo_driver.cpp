/**
 * @file servo_can.cpp
 * @brief Implementation of SocketCAN driver for MKS SERVO42 & 57D_CAN servos
 * @author Arctos Team
 */

#include "arctos_hardware_interface/mks_servo_driver.hpp"
// #include <cmath>
#include <algorithm>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mks_driver");
namespace mks_servo_driver
{

    MksServoDriver::MksServoDriver()
    {
        positions_.resize(6, 0);
        in1_states_.resize(6, true); // low activate
        in2_states_.resize(6, true);
        command_status_.resize(6, 0);
        homing_status_.resize(6, 0);
    }

    MksServoDriver::~MksServoDriver()
    {
        deactive();
    }

    bool MksServoDriver::connect(const std::string &can_interface)
    {
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0)
            return false;
        struct ifreq ifr{};
        std::strcpy(ifr.ifr_name, can_interface.c_str());
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        {
            close(sock_);
            return false;
        }
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            close(sock_);
            return false;
        }
        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
        struct can_filter filters[6];
        for (int i = 0; i < 6; ++i)
        {
            filters[i].can_id = i + 1;
            filters[i].can_mask = CAN_SFF_MASK;
        }
        setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters));
        running_ = true;
        try
        {
            poll_thread_ = std::thread(&MksServoDriver::pollLoop, this);
        }
        catch (const std::exception &)
        {
            running_ = false;
            close(sock_);
            sock_ = -1;
            return false;
        }
        return true;
    }

    void MksServoDriver::deactive()
    {
        running_ = false;
        for (uint16_t id = 1; id <= 6; ++id)
        {
            enableMotor(id, false);
        }

        if (poll_thread_.joinable())
        {
            poll_thread_.join();
        }
        if (sock_ >= 0)
        {
            close(sock_);
            sock_ = -1;
        }
    }

    bool MksServoDriver::sendCmd(uint16_t id, uint8_t code, const std::vector<uint8_t> &params)
    {
        if (sock_ < 0)
        {
            RCLCPP_ERROR(LOGGER, "Socket not connected!");
            return false;
        }
        std::vector<uint8_t> data{code};
        data.reserve(1 + params.size() + 1);
        data.insert(data.end(), params.begin(), params.end());
        data.push_back(calcCrc(id, data));

        if (data.size() > 8)
        {
            RCLCPP_ERROR(LOGGER, "invalid cmd size!: %ld", data.size());
            return false;
        }

        can_frame tx{};
        tx.can_id = id & CAN_SFF_MASK;
        tx.can_dlc = static_cast<__u8>(data.size());
        // tx.can_dlc = data.size();
        std::memcpy(tx.data, data.data(), tx.can_dlc);

        // auto now = std::chrono::steady_clock::now();
        // auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        // RCLCPP_INFO(LOGGER, "t_can_tx time: %ld.%09ld", ns / 1000000000, ns % 1000000000);

        std::lock_guard<std::mutex> lk(tx_mutex_);
        return ::write(sock_, &tx, sizeof(tx)) == sizeof(tx);
    }

    bool MksServoDriver::runPositionAbs(uint16_t id, uint16_t speed, uint8_t accel, int32_t position)
    {
        uint8_t acc_value = std::min(std::max(accel, uint8_t(0)), uint8_t(255));
        std::vector<uint8_t> params{
            uint8_t(speed >> 8), uint8_t(speed & 0xFF), acc_value,
            uint8_t((position >> 16) & 0xFF), uint8_t((position >> 8) & 0xFF), uint8_t(position & 0xFF)};
        return sendCmd(id, CANCommands::ABSOLUTE_POSITION, params);
    }

    bool MksServoDriver::queryPosition(uint16_t id)
    {
        return sendCmd(id, CANCommands::READ_ENCODER, {});
    }

    bool MksServoDriver::enableMotor(uint16_t id, bool flag)
    {
        uint8_t enable_val = flag ? 0x01 : 0x00;
        return sendCmd(id, CANCommands::ENABLE_MOTOR, {enable_val});
    }

    uint8_t MksServoDriver::enableMotorSync(uint16_t id, bool flag, int timeout_ms)
    {
        return sendCmdSync(id, CANCommands::ENABLE_MOTOR, {flag ? uint8_t(0x01) : uint8_t(0x00)}, timeout_ms);
    }

    uint8_t MksServoDriver::runPositionAbsSync(uint16_t id, uint16_t speed, uint8_t accel, int32_t position, int timeout_ms)
    {
        std::vector<uint8_t> params{
            uint8_t(speed >> 8), uint8_t(speed & 0xFF), accel,
            uint8_t((position >> 16) & 0xFF), uint8_t((position >> 8) & 0xFF), uint8_t(position & 0xFF)};
        return sendCmdSync(id, CANCommands::ABSOLUTE_POSITION, params, timeout_ms);
    }

    uint8_t MksServoDriver::homeSync(uint16_t id, int timeout_ms)
    {
        return sendCmdSync(id, CANCommands::GO_HOME, {}, timeout_ms, true); // Use homing status
    }

    uint8_t MksServoDriver::setZeroSync(uint16_t id, int timeout_ms)
    {
        return sendCmdSync(id, CANCommands::SET_ZERO_POSITION, {}, timeout_ms);
    }

    void MksServoDriver::setZero(uint16_t id)
    {
        sendCmd(id, CANCommands::SET_ZERO_POSITION, {});
    }

    void MksServoDriver::setHoldingCurrent(uint16_t id, uint8_t percentage)
    {
        uint8_t holding_val = (percentage - 10) / 10; // convert to 0-8 range.
        sendCmd(id, 0xf2, {holding_val});
    }

    std::vector<int64_t> MksServoDriver::getPositions()
    {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        return positions_;
    }

    void MksServoDriver::home(uint8_t can_id)
    {
        sendCmd(can_id, CANCommands::GO_HOME, {});
    }

    uint8_t MksServoDriver::sendCmdSync(uint16_t id, uint8_t cmd, const std::vector<uint8_t> &params, int timeout_ms, bool use_homing_status)
    {
        if (id < 1 || id > 6)
            return 0xFF;

        // Clear previous status
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            if (use_homing_status)
            {
                homing_status_[id - 1] = 0xFF;
            }
            else
            {
                command_status_[id - 1] = 0xFF;
            }
        }

        // Send command
        if (!sendCmd(id, cmd, params))
        {
            return 0xFF;
        }

        // Wait for response
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms))
        {
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                uint8_t status = use_homing_status ? homing_status_[id - 1] : command_status_[id - 1];
                if (status != 0xFF)
                {
                    return status;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        return 0xFF; // Timeout
    }

    void MksServoDriver::EmergencyStop(uint16_t id)
    {
        sendCmd(id, CANCommands::EMERGENCY_STOP, {});
    }

    bool MksServoDriver::queryIO(uint16_t id)
    {
        return sendCmd(id, CANCommands::READ_IO, {});
    }

    // std::vector<bool> MksServoDriver::getIN1States()
    // {
    //     std::lock_guard<std::mutex> lock(io_mutex_);
    //     return in1_states_;
    // }

    // std::vector<bool> MksServoDriver::getIN2States()
    // {
    //     std::lock_guard<std::mutex> lock(io_mutex_);
    //     return in2_states_;
    // }

    bool MksServoDriver::getIN1State(uint16_t id)
    {
        if (id < 1 || id > 6)
            return false;
        std::lock_guard<std::mutex> lock(io_mutex_);
        return in1_states_[can_index(id)];
    }

    bool MksServoDriver::getIN2State(uint16_t id)
    {
        if (id < 1 || id > 6)
            return false;
        std::lock_guard<std::mutex> lock(io_mutex_);
        return in2_states_[can_index(id)];
    }

    int64_t MksServoDriver::getPosition(uint16_t id)
    {
        if (id < 1 || id > 6)
            return 0;
        std::lock_guard<std::mutex> lock(pos_mutex_);
        return positions_[can_index(id)];
    }
    uint8_t MksServoDriver::getCommandStatus(uint16_t id)
    {
        if (id < 1 || id > 6)
            return 0xFF;
        std::lock_guard<std::mutex> lock(status_mutex_);
        return command_status_[can_index(id)];
    }
    // std::vector<uint8_t> MksServoDriver::getCommandStatus()
    // {
    //     std::lock_guard<std::mutex> lock(status_mutex_);
    //     return command_status_;
    // }

    uint8_t MksServoDriver::getHomingStatus(uint16_t id)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        return homing_status_[can_index(id)];
    }

    // runs every 5ms
    void MksServoDriver::pollLoop()
    {
        int tick = 0;
        while (running_)
        {
            // Every 10ms: query wrist motors (100Hz if sleep=5ms and tick%2==0)
            if ((tick % 2) == 0)
            {
                for (size_t id = 1; id <= 6; id++)
                {
                    queryPosition(id);
                }
            }           

            std::vector<can_frame> frames;
            if (readAllAvailable(frames))
            {
                for (const auto &f : frames)
                    processCanFrame(f);
            }

            ++tick;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    bool MksServoDriver::readAllAvailable(std::vector<can_frame> &frames)
    {
        frames.clear();
        if (sock_ < 0)
            return false;
        while (true)
        {
            can_frame frame;
            ssize_t bytes = read(sock_, &frame, sizeof(frame));
            if (bytes == sizeof(frame))
                frames.push_back(frame);
            else if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            else
                return false;
        }
        return true;
    }

    std::optional<int64_t> MksServoDriver::processCanFrame(const can_frame &frame)
    {
        const uint16_t can_id = static_cast<uint16_t>(frame.can_id & CAN_SFF_MASK);
        if (can_id < 1 || can_id > kNumMotors)
        {
            RCLCPP_DEBUG_THROTTLE(LOGGER, clock_, 2000,
                                  "Drop frame: raw_can_id=0x%X masked=%u dlc=%u",
                                  frame.can_id, can_id, frame.can_dlc);
            return std::nullopt;
        }

        const size_t idx = can_index(can_id);
        const uint8_t cmd = frame.data[0];

        switch (cmd)
        {
        case CANCommands::READ_ENCODER:
            if (frame.can_dlc == 8)
            {
                const int64_t pos = toI48(&frame.data[1]);
                std::lock_guard<std::mutex> lock(pos_mutex_);
                positions_[idx] = pos;
            }
            break;

        case CANCommands::READ_IO:
            if (frame.can_dlc == 3)
            {
                const uint8_t io_status = frame.data[1];
                std::lock_guard<std::mutex> lock(io_mutex_);
                in1_states_[idx] = (io_status & 0x01) == 0;
                in2_states_[idx] = (io_status & 0x02) == 0;
            }
            break;

        case CANCommands::GO_HOME:
        case CANCommands::CALIBRATE:
            if (frame.can_dlc >= 3)
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                homing_status_[idx] = frame.data[1];
            }
            break;

        case CANCommands::ABSOLUTE_POSITION:
        case CANCommands::ENABLE_MOTOR:
        case CANCommands::SET_ZERO_POSITION:
            if (frame.can_dlc >= 3)
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                command_status_[idx] = frame.data[1];
            }
            break;

        default:
            RCLCPP_DEBUG_THROTTLE(LOGGER, clock_, 2000,
                                  "Unhandled cmd 0x%02X from id=%u dlc=%u",
                                  cmd, can_id, frame.can_dlc);
            break;
        }

        return std::nullopt;
    }

    uint8_t MksServoDriver::calcCrc(uint32_t id, const std::vector<uint8_t> &data)
    {
        uint32_t sum = id & 0x7FF;
        for (auto byte : data)
            sum += byte;
        return sum & 0xFF;
    }

    int64_t MksServoDriver::toI48(const uint8_t *p)
    {
        uint64_t raw = ((uint64_t)p[0] << 40) | ((uint64_t)p[1] << 32) | ((uint64_t)p[2] << 24) |
                       ((uint64_t)p[3] << 16) | ((uint64_t)p[4] << 8) | p[5];
        if (raw & 0x0000800000000000ULL)
            raw |= 0xFFFF000000000000ULL;
        return static_cast<int64_t>(raw);
    }

} // end namespace