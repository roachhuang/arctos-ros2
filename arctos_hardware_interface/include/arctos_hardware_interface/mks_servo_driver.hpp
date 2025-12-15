/**
 * @file servo_can.hpp
 * @brief SocketCAN driver for MKS SERVO42 & 57D_CAN servos
 * @author roach + ai
 */

#pragma once
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <optional>
#include <chrono>
#include <cstring>
#include <string>
#include <errno.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <arctos_hardware_interface/motor_types.hpp>

/**
 * @class ServoCanSimple
 * @brief Thread-safe CAN driver for MKS servo motors
 *
 * Provides non-blocking CAN communication with automatic position polling
 * for real-time control applications. Supports up to 6 servos (CAN ID 1-6).
 */
namespace mks_servo_driver
{

    class MksServoDriver
    {
    public:
        MksServoDriver();
        ~MksServoDriver();

        /**
         * @brief Connect to CAN interface and start polling thread
         * @param can_interface CAN interface name (default: "can0")
         * @return true if connection successful
         */
        bool connect(const std::string &can_interface = "can0");

        /**
         * @brief Disconnect from CAN and stop polling thread
         */
        void deactive();

        bool sendCmd(uint16_t id, uint8_t code, const std::vector<uint8_t> &params = {});

        /**
         * @brief Send absolute position command to servo
         * @param id Servo CAN ID (1-6)
         * @param speed Speed in RPM
         * @param accel Acceleration (0-255)
         * @param position Target position in encoder counts
         * @return true if command sent successfully
         */
        bool runPositionAbs(uint16_t id, uint16_t speed, uint8_t accel, int32_t position);
        uint8_t runPositionAbsSync(uint16_t id, uint16_t speed, uint8_t accel, int32_t position, int timeout_ms = 100);

        bool queryPosition(uint16_t id);
        bool enableMotor(uint16_t id, bool flag);
        uint8_t enableMotorSync(uint16_t id, bool flag, int timeout_ms = 100);
        void setZero(uint16_t id);
        uint8_t setZeroSync(uint16_t id, int timeout_ms = 100);
        void setHoldingCurrent(uint16_t id, uint8_t percentage);
        void home(uint8_t);
        uint8_t homeSync(uint16_t id, int timeout_ms = 5000);
        void EmergencyStop(uint16_t id);
        bool queryIO(uint16_t id);

        /**
         * @brief Get current positions from all servos
         * @return Vector of 6 position values in encoder counts
         */
        std::vector<int64_t> getPositions();
        int64_t getPosition(uint16_t id);
        std::vector<bool> getIN1States();
        std::vector<bool> getIN2States();
        bool getIN1State(uint16_t id);
        bool getIN2State(uint16_t id);
        uint8_t getCommandStatus(u_int16_t);
        uint8_t getHomingStatus(u_int16_t id);

        inline size_t can_index(uint16_t can_id)
        {
            // Optional: add runtime check in debug builds
            // assert(can_id >= 1 && can_id <= 6);
            return static_cast<size_t>(can_id - 1);
        }

    private:
        int sock_{-1};
        std::thread poll_thread_;
        std::mutex pos_mutex_;
        std::mutex io_mutex_;
        std::mutex status_mutex_;
        std::vector<int64_t> positions_;
        std::vector<bool> in1_states_;
        std::vector<bool> in2_states_;
        std::vector<uint8_t> command_status_;
        std::vector<uint8_t> homing_status_;
        std::atomic<bool> running_{false};

        void pollLoop();
        bool readAllAvailable(std::vector<can_frame> &frames);
        std::optional<int64_t> processCanFrame(const can_frame &frame);
        uint8_t sendCmdSync(uint16_t id, uint8_t cmd, const std::vector<uint8_t> &params, int timeout_ms, bool use_homing_status = false);
        static uint8_t calcCrc(uint32_t id, const std::vector<uint8_t> &data);
        static int64_t toI48(const uint8_t *p);
    };
} // end namespace