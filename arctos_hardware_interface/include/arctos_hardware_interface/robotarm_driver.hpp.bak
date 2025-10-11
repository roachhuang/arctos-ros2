#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robotarm_controller
{
class RobotArmDriver
{
public:
  RobotArmDriver();
  ~RobotArmDriver();

  bool connect(const std::string& port, uint32_t baud_rate = 115200);
  void disconnect();
  bool is_connected() const;

  bool send_command(const std::vector<double>& joint_positions);
  bool activate();
  bool deactivate();
  bool send_home();

  // Feedback parsing for separation of concern
  bool parse_feedback(const std::string& msg, std::vector<double>& out_positions, bool& ack_received);

private:
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_;
  std::mutex serial_mutex_;
  bool connected_;

  void safe_write(const std::string& cmd);
  std::string format_command(const std::vector<double>& positions);
};

} // namespace robotarm_controller