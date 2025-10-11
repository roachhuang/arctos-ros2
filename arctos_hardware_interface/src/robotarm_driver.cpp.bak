#include "robotarm_controller/robotarm_driver.hpp"
#include <thread>
#include <sstream>
#include <iomanip>
#include <cmath>

/*In Boost, everything is inside boost::asio namespace.
You can't just write asio::xxx unless you: using namespace boost::asio;
*/

namespace robotarm_controller {
bool RobotArmDriver::parse_feedback(const std::string& msg, std::vector<double>& out_positions, bool& ack_received)
{
  ack_received = false;
  out_positions.clear();
  if (msg.empty()) return false;

  if (msg == "ack") {
    ack_received = true;
    RCLCPP_INFO(rclcpp::get_logger("RobotArmDriver"), "Arduino ACK received");
    return true;
  }
  if (msg[0] == 'f') {
    size_t star_pos = msg.find('*');
    if (star_pos == std::string::npos) {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmDriver"), "No checksum found in feedback: %s", msg.c_str());
      return false;
    }
    std::string data = msg.substr(0, star_pos);
    std::string checksum_str = msg.substr(star_pos + 1);
    uint8_t calc_checksum = 0;
    for (char c : data) {
      calc_checksum += static_cast<uint8_t>(c);
    }
    calc_checksum = calc_checksum % 256;
    int recv_checksum = -1;
    try {
      recv_checksum = std::stoi(checksum_str);
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmDriver"), "Invalid checksum in feedback: %s", msg.c_str());
      return false;
    }
    if (calc_checksum != recv_checksum) {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmDriver"), "Checksum mismatch! Calculated: %d, Received: %d, Data: %s", calc_checksum, recv_checksum, data.c_str());
      return false;
    }
    std::string numbers = data.substr(1); // skip 'f'
    std::stringstream ss(numbers);
    std::string item;
    while (std::getline(ss, item, ',')) {
      try {
        out_positions.push_back(std::stod(item));
      } catch (...) {
        RCLCPP_WARN(rclcpp::get_logger("RobotArmDriver"), "Failed to parse position: %s", item.c_str());
        return false;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotArmDriver"), "Parsed %zu positions from feedback", out_positions.size());
    return true;
  }
  return false;
}

RobotArmDriver::RobotArmDriver() 
  : serial_(io_service_), connected_(false)
{
}

RobotArmDriver::~RobotArmDriver()
{
  disconnect();
}

bool RobotArmDriver::connect(const std::string& port, uint32_t baud_rate)
{
  try
  {
    serial_.open(port);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    
    if (!serial_.is_open())
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmDriver"), "Failed to open port: %s", port.c_str());
      return false;
    }

    // Arduino initialization delay
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Wake up Arduino
    safe_write("\n");
    connected_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("RobotArmDriver"), "Connected to %s at %d baud", port.c_str(), baud_rate);
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmDriver"), "Connection failed: %s", e.what());
    return false;
  }
}

void RobotArmDriver::disconnect()
{
  if (connected_ && serial_.is_open())
  {
    try
    {
      deactivate();
      serial_.close();
      connected_ = false;
      RCLCPP_INFO(rclcpp::get_logger("RobotArmDriver"), "Disconnected from hardware");
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmDriver"), "Disconnect error: %s", e.what());
    }
  }
}

bool RobotArmDriver::is_connected() const
{
  return connected_ && serial_.is_open();
}

bool RobotArmDriver::send_command(const std::vector<double>& joint_positions)
{
  if (!is_connected()) return false;
  
  try
  {
    std::string cmd = format_command(joint_positions);
    safe_write(cmd);
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmDriver"), "Send command failed: %s", e.what());
    return false;
  }
}

bool RobotArmDriver::activate()
{
  if (!is_connected()) return false;
  
  try
  {
    safe_write("en\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    safe_write("g0.0,0.0,0.0,0.0,0.0,0.0\n");
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmDriver"), "Enable failed: %s", e.what());
    return false;
  }
}

bool RobotArmDriver::deactivate()
{
  if (!is_connected()) return false;
  
  try
  {
    safe_write("g0.0,-78.51,73.90,0.0,-90.0,0.0\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    safe_write("dis\n");
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmDriver"), "Disable failed: %s", e.what());
    return false;
  }
}

bool RobotArmDriver::send_home()
{
  if (!is_connected()) return false;
  
  try
  {
    safe_write("g28\n");
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmDriver"), "Home failed: %s", e.what());
    return false;
  }
}

void RobotArmDriver::safe_write(const std::string& cmd)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  boost::asio::write(serial_, boost::asio::buffer(cmd));
}

std::string RobotArmDriver::format_command(const std::vector<double>& positions)
{
  std::stringstream cmd;
  cmd << "g";
  
  for (size_t i = 0; i < positions.size(); ++i)
  {
    if (i > 0) cmd << ",";
    double deg = positions[i] * 180.0 / M_PI;
    cmd << std::fixed << std::setprecision(2) << deg;
  }
  cmd << "\n";
  
  return cmd.str();
}

} // namespace robotarm_controller