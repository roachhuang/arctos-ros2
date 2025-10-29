// ServoCanSimple.hpp
// Clean SocketCAN driver for MKS SERVO42 & 57D_CAN (Manual v1.0.6)
// CRC = (CAN_ID + all data bytes before CRC) & 0xFF
// Single unified sendCmd() + helpers for key function codes (0x30..0xFE)

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

class ServoCanSimple
{
public:
  // 1. Default Constructor (to allow object creation)
  ServoCanSimple() = default;

  bool connect(const std::string &can_interface = "can0")
  {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
    {
      std::cerr << "[ServoCAN] Error: fail to create socket." << std::endl;
      return false;
    }

    struct ifreq ifr{};
    std::strcpy(ifr.ifr_name, can_interface.c_str());
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
    {
      throw std::runtime_error("CAN iface not found: " + can_interface);
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      throw std::runtime_error("SocketCAN bind failed");
    }

    struct timeval timeout{0, 10000};
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    // RCLCPP_INFO(rclcpp::get_logger("CanDriver"), "Connected to CAN: %s", can_interface.c_str());
    return true;
  }

  // ------------------- CORE -------------------
  bool sendCmd(uint16_t can_id, uint8_t code,
               const std::vector<uint8_t> &params = {},
               bool expect_resp = false,
               std::vector<uint8_t> *out_resp = nullptr,
               int timeout_ms = 100)
  {
    if (can_id > 0x7FF)
      throw std::invalid_argument("CAN ID > 0x7FF");

    std::vector<uint8_t> data;
    data.push_back(code);
    data.insert(data.end(), params.begin(), params.end());
    data.push_back(calcCrc(can_id, data)); // append CRC

    struct can_frame tx{};
    tx.can_id = can_id;
    tx.can_dlc = data.size();
    std::memcpy(tx.data, data.data(), tx.can_dlc);

    if (write(sock_, &tx, sizeof(tx)) != sizeof(tx))
      perror("[ServoCAN] write");

    // debugTx(tx);

    if (!expect_resp)
      return true;

    struct can_frame rx{};
    if (!readFrame(rx, timeout_ms))
      return false;
    // debugRx(rx);

    // Validate CRC
    if (rx.can_dlc < 2)
      return false;
    uint8_t crc_recv = rx.data[rx.can_dlc - 1];
    uint8_t crc_calc = calcCrc(rx.can_id, {rx.data, rx.data + rx.can_dlc - 1});
    if (crc_recv != crc_calc)
    {
      std::cerr << "[ServoCAN] CRC mismatch\n";
      return false;
    }
    if (out_resp)
      out_resp->assign(rx.data, rx.data + rx.can_dlc - 1);
    return true;
  }

  // ------------------- BASIC COMMANDS -------------------
  bool enableMotor(uint16_t id, bool on)
  {
    return sendSimple(id, 0xF3, {uint8_t(on)});
  }

  bool queryStatus(uint16_t id, uint8_t &st)
  {
    std::vector<uint8_t> r;
    if (!sendCmd(id, 0xF1, {}, true, &r))
      return false;
    if (r.size() < 2)
      return false;
    st = r[1];
    return true;
  }

  std::optional<std::pair<int32_t, uint16_t>> readEncoder(uint16_t id)
  {
    std::vector<uint8_t> r;
    if (!sendCmd(id, 0x30, {}, true, &r))
      return std::nullopt;
    if (r.size() < 1 + 4 + 2)
      return std::nullopt;
    const uint8_t *p = &r[1];
    int32_t carry = toI32(p);
    uint16_t val = toU16(p + 4);
    return std::make_pair(carry, val);
  }

  std::optional<int16_t> readSpeed(uint16_t id)
  {
    std::vector<uint8_t> r;
    if (!sendCmd(id, 0x32, {}, true, &r))
      return std::nullopt;
    if (r.size() < 3)
      return std::nullopt;
    return toI16(&r[1]);
  }

  // ------------------- MOTION COMMANDS -------------------
  bool runSpeedMode(uint16_t id, bool dir_ccw, uint16_t rpm, uint8_t acc = 2)
  {
    if (rpm > 3000)
      rpm = 3000;
    uint8_t b2 = (dir_ccw ? 0x80 : 0x00) | ((rpm >> 8) & 0x0F);
    uint8_t b3 = rpm & 0xFF;
    return sendSimple(id, 0xF6, {b2, b3, acc});
  }

  // abs motion by axis counts (24-bit)
  bool runPositionAbs(uint16_t id, uint16_t speed, uint8_t accel, int32_t pos24)
  {
    if (speed > 3000)
      speed = 3000;
    // std::vector<uint8_t> p{
    //     uint8_t(speed & 0xFF),
    //     acc,
    //     uint8_t(pos24 & 0xFF),
    //     uint8_t((pos24 >> 8) & 0xFF),
    //     uint8_t((pos24 >> 16) & 0xFF)};
    std::vector<uint8_t> p{
        uint8_t(speed >> 8),
        uint8_t(speed & 0xff),
        accel,
        uint8_t((pos24 >> 16) & 0xFF),
        uint8_t((pos24 >> 8) & 0xFF),
        uint8_t(pos24 & 0xFF)};
    return sendSimple(id, 0xF5, p);
  }

  bool runPositionRel(uint16_t id, bool dir_ccw, uint16_t speed, uint8_t acc, int32_t rel24)
  {
    if (speed > 3000)
      speed = 3000;
    uint8_t b2 = (dir_ccw ? 0x80 : 0x00) | ((speed >> 8) & 0x0F);
    uint8_t b3 = speed & 0xFF;
    std::vector<uint8_t> p{b2, b3, acc,
                           uint8_t(rel24 & 0xFF), uint8_t((rel24 >> 8) & 0xFF), uint8_t((rel24 >> 16) & 0xFF)};
    return sendSimple(id, 0xFD, p);
  }

  bool setCanId(uint16_t cur_id, uint16_t new_id)
  {
    return sendSimple(cur_id, 0x8B,
                      {uint8_t(new_id & 0xFF), uint8_t((new_id >> 8) & 0x07)});
  }

  bool clearFault(uint16_t id) { return sendSimple(id, 0x40, {}); }

  void deactive()
  {
    close(sock_);
    sock_ = -1;
  }

private:
  int sock_{-1};

  // Helpers ------------------------------------------------
  bool sendSimple(uint16_t id, uint8_t code, const std::vector<uint8_t> &p)
  {
    std::vector<uint8_t> r;
    return sendCmd(id, code, p, true, &r) && r.size() >= 2 && r[1] == 1;
  }

  static uint8_t calcCrc(uint32_t id, const std::vector<uint8_t> &d)
  {
    uint32_t s = id & 0x7FF;
    for (auto b : d)
      s += b;
    return s & 0xFF;
  }

  static int16_t toI16(const uint8_t *p) { return int16_t(p[0] | (p[1] << 8)); }
  static uint16_t toU16(const uint8_t *p) { return uint16_t(p[0] | (p[1] << 8)); }
  static int32_t toI32(const uint8_t *p) { return int32_t(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24)); }

  bool readFrame(struct can_frame &rx, int timeout_ms)
  {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock_, &fds);
    struct timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    if (select(sock_ + 1, &fds, nullptr, nullptr, &tv) <= 0)
      return false;
    return read(sock_, &rx, sizeof(rx)) == sizeof(rx);
  }

  void debugTx(const struct can_frame &f)
  {
    std::cerr << "[TX] 0x" << std::hex << f.can_id << " : ";
    for (int i = 0; i < f.can_dlc; ++i)
      std::cerr << std::setw(2) << std::setfill('0') << (int)f.data[i] << " ";
    std::cerr << std::dec << std::endl;
  }
  void debugRx(const struct can_frame &f)
  {
    std::cerr << "[RX] 0x" << std::hex << f.can_id << " : ";
    for (int i = 0; i < f.can_dlc; ++i)
      std::cerr << std::setw(2) << std::setfill('0') << (int)f.data[i] << " ";
    std::cerr << std::dec << std::endl;
  }
};
