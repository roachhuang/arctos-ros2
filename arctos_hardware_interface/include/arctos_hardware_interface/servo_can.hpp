#pragma once
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <optional>
#include <chrono>
#include <cstring>
#include <errno.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

class ServoCanSimple
{
public:
    ServoCanSimple() { positions_.resize(6, 0); }
    ~ServoCanSimple() { deactive(); }

    bool connect(const std::string &can_interface = "can0")
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
        try {
            poll_thread_ = std::thread(&ServoCanSimple::pollLoop, this);
        } catch (const std::exception&) {
            running_ = false;
            close(sock_);
            sock_ = -1;
            return false;
        }
        return true;
    }

    void deactive()
    {
        running_ = false;
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }
        if (sock_ >= 0) {
            close(sock_);
            sock_ = -1;
        }
    }

    bool sendCmd(uint16_t id, uint8_t code, const std::vector<uint8_t> &params = {})
    {
        if (sock_ < 0) return false;
        std::vector<uint8_t> data{code};
        data.insert(data.end(), params.begin(), params.end());
        data.push_back(calcCrc(id, data));
        if (data.size() > 8)
            return false;
        can_frame tx{};
        tx.can_id = id;
        tx.can_dlc = data.size();
        std::memcpy(tx.data, data.data(), tx.can_dlc);
        return write(sock_, &tx, sizeof(tx)) == sizeof(tx);
    }

    bool runPositionAbs(uint16_t id, uint16_t speed, uint8_t accel, int32_t position)
    {
        std::vector<uint8_t> params{
            uint8_t(speed >> 8), uint8_t(speed & 0xFF), accel,
            uint8_t((position >> 16) & 0xFF), uint8_t((position >> 8) & 0xFF), uint8_t(position & 0xFF)};
        return sendCmd(id, 0xF5, params);
    }

    bool queryPosition(uint16_t id)
    {
        return sendCmd(id, 0x31, {});
    }

    std::vector<int64_t> getPositions()
    {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        return positions_;
    }

private:
    int sock_{-1};
    std::thread poll_thread_;
    std::mutex pos_mutex_;
    std::vector<int64_t> positions_;
    std::atomic<bool> running_{false};

    void pollLoop()
    {
        int query_counter = 0;
        while (running_)
        {
            // Query positions every 10ms (every 2nd cycle)
            if (query_counter % 2 == 0) {
                for (int id = 1; id <= 6; ++id) {
                    queryPosition(id);
                }
            }
            query_counter++;
            
            std::vector<can_frame> frames;
            readAllAvailable(frames);
            for (const auto &frame : frames)
            {
                auto encoder = processCanFrame(frame);
                if (encoder.has_value() && frame.can_id >= 1 && frame.can_id <= 6)
                {
                    size_t idx = frame.can_id - 1;
                    std::lock_guard<std::mutex> lock(pos_mutex_);
                    positions_[idx] = encoder.value();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    bool readAllAvailable(std::vector<can_frame> &frames)
    {
        frames.clear();
        if (sock_ < 0) return false;
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

    std::optional<int64_t> processCanFrame(const can_frame &frame)
    {
        if (frame.can_dlc < 2 || frame.data[0] != 0x31 || frame.can_dlc < 7)
            return std::nullopt;
        return toI48(&frame.data[1]);
    }

    static uint8_t calcCrc(uint32_t id, const std::vector<uint8_t> &data)
    {
        uint32_t sum = id & 0x7FF;
        for (auto byte : data)
            sum += byte;
        return sum & 0xFF;
    }

    static int64_t toI48(const uint8_t *p)
    {
        uint64_t raw = ((uint64_t)p[0] << 40) | ((uint64_t)p[1] << 32) | ((uint64_t)p[2] << 24) |
                       ((uint64_t)p[3] << 16) | ((uint64_t)p[4] << 8) | p[5];
        if (raw & 0x0000800000000000ULL)
            raw |= 0xFFFF000000000000ULL;
        return static_cast<int64_t>(raw);
    }
};