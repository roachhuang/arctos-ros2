#pragma once

#include <cstdint>
#include <string>
#include <sstream>
#include <mutex>
#include <condition_variable>

struct Motor
{
  // Identity
  uint16_t can_id;        // 1..N
  double   gear_ratio {1.0};  // motor revolutions per joint revolution

  // Motion configuration
  double   vel_cmd {0.0};     // device-specific velocity param
  double   accel_cmd {0.0};   // device-specific accel param

  // Raw feedback from servo
  int64_t  raw_position {0};  // encoder counts (48-bit)

  bool     in1 {true};        // limit/home switch (true = high; low = active)
  bool     in2 {true};

  // Status from servo
  uint8_t  homing_status {0xFF};
  bool     is_homing {false};

  // Derived state (filled/used by HW interface)
  double   position_rad {0.0};   // joint position in radians
  double   velocity_rad {0.0};   // joint velocity in rad/s

  // For synchronous commands
  uint8_t last_cmd_sent {0x00};
  uint8_t last_cmd_status {0xFF};
  bool    waiting_for_response {false};
  std::mutex cv_mutex;
  std::condition_variable cv;

  // For motion-complete detection
  double last_target_rad {0.0};       // last commanded target
  bool   target_active {false};       // whether a target is active
  double converge_tolerance {0.001};  // rad
  double converge_velocity_eps {0.001}; // rad/s

  // size_t index() const { return static_cast<size_t>(can_id - 1); }
};

inline std::string toString(const Motor &m)
{
  std::ostringstream ss;
  ss << "Motor{"
     << "can_id=" << m.can_id
     << ", gear_ratio=" << m.gear_ratio
     << ", vel_cmd=" << m.vel_cmd
     << ", accel_cmd=" << m.accel_cmd
     << ", raw_position=" << m.raw_position
     << ", position_rad=" << m.position_rad
     << ", velocity_rad=" << m.velocity_rad
     << ", in1=" << (m.in1 ? "HIGH" : "LOW")
     << ", in2=" << (m.in2 ? "HIGH" : "LOW")
     << ", homing_status=0x" << std::hex << static_cast<int>(m.homing_status) << std::dec
     << ", is_homing=" << (m.is_homing ? "true" : "false")
     << ", last_cmd_sent=0x" << std::hex << static_cast<int>(m.last_cmd_sent) << std::dec
     << ", last_cmd_status=0x" << std::hex << static_cast<int>(m.last_cmd_status) << std::dec
     << ", waiting_for_response=" << (m.waiting_for_response ? "true" : "false")
     << ", target_active=" << (m.target_active ? "true" : "false")
     << ", last_target_rad=" << m.last_target_rad
     << "}";
  return ss.str();
}
