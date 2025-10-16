#include "arctos_hardware_interface/ServoManager.hpp"
#include <cstring>
#include <rclcpp/rclcpp.hpp>

// Constructor
ServoManager::ServoManager(uint8_t id, double ratio)
    : canId(id), lastEncoder(0), accumulatedCounts(0), fullTurns(0),
      speedRadPerSec(0.0), accumulatedAngle(0.0), gearRatio(ratio), targetEncoder(0) {}

uint8_t ServoManager::checksum(const uint8_t *data, size_t len)
{
    uint16_t sum = canId;
    for (size_t i = 0; i < len; i++)
        sum += data[i];
    return sum & 0xFF;
}

can_frame ServoManager::pack(const uint8_t *data, size_t len)
{
    can_frame frame{};
    frame.can_id = canId;
    frame.can_dlc = len;
    std::memcpy(frame.data, data, len);
    return frame;
}


can_frame ServoManager::runMotorSpeedMode(uint8_t dir, uint16_t speed, uint8_t)
{
    uint8_t data[5] = {CMD::MOTOR::SPEED, dir, static_cast<uint8_t>(speed >> 8), static_cast<uint8_t>(speed & 0xFF), 0};
    data[4] = checksum(data, 4);
    return pack(data, 5);
}

can_frame ServoManager::absoluteMotion(uint16_t speed, uint8_t accel, uint32_t position)
{
    uint8_t data[8] = {
        CMD::MOTOR::ABSOLUTE_AXIS,
        static_cast<uint8_t>(speed >> 8),
        static_cast<uint8_t>(speed & 0xFF),
        accel,
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF),
        static_cast<uint8_t>((position >> 16) & 0xFF),
        0
    };
    data[7] = checksum(data, 7);
    return pack(data, 8);
}


can_frame ServoManager::absoluteMotionRad(double angleRad, uint16_t speed, uint8_t accel)
{
    uint32_t encoderValue = static_cast<uint32_t>((angleRad * gearRatio / TWO_PI) * ENCODER_CPR);
    uint8_t data[8] = {
        CMD::MOTOR::ABSOLUTE_AXIS,
        static_cast<uint8_t>(speed >> 8),
        static_cast<uint8_t>(speed & 0xFF),
        accel,
        static_cast<uint8_t>((encoderValue >> 16) & 0xFF),
        static_cast<uint8_t>((encoderValue >> 8) & 0xFF),
        static_cast<uint8_t>(encoderValue & 0xFF),
        0
    };
    data[7] = checksum(data, 7);
    return pack(data, 8);
}


can_frame ServoManager::enableServo(bool on)
{
    uint8_t data[3] = {CMD::CONTROL::ENABLE, static_cast<uint8_t>(on ? 1 : 0), 0};
    data[2] = checksum(data, 2);
    return pack(data, 3);
}

can_frame ServoManager::stopMotor()
{
    uint8_t data[2] = {CMD::MOTOR::STOP, 0};
    data[1] = checksum(data, 1);
    return pack(data, 2);
}


can_frame ServoManager::readEncoderAbsolute()
{
    uint8_t data[2] = {CMD::READ::ENCODER_ABSOLUTE, 0};
    data[1] = checksum(data, 1);
    return pack(data, 2);
}

can_frame ServoManager::readEncoderCarry()
{
    uint8_t data[2] = {CMD::READ::ENCODER_CARRY, 0};
    data[1] = checksum(data, 1);
    return pack(data, 2);
}

can_frame ServoManager::readSpeed()
{
    uint8_t data[2] = {CMD::READ::SPEED, 0};
    data[1] = checksum(data, 1);
    return pack(data, 2);
}


can_frame ServoManager::setWorkingCurrent(uint16_t current)
{
    uint8_t data[4] = {CMD::SET::WORKING_CURRENT, static_cast<uint8_t>(current >> 8), static_cast<uint8_t>(current & 0xFF), 0};
    data[3] = checksum(data, 3);
    return pack(data, 4);
}

can_frame ServoManager::setHoldingCurrent(uint8_t percent)
{
    uint8_t data[3] = {CMD::SET::HOLDING_CURRENT, percent, 0};
    data[2] = checksum(data, 2);
    return pack(data, 3);
}


ServoManager::ResponseData ServoManager::parseResponse(const struct can_frame &frame)
{
    ResponseData resp;

    if (frame.can_dlc < 4)
        return resp; // expect at least 4 bytes

    uint8_t cmd = frame.data[0];
    switch (cmd)
    {
    case CMD::READ::ENCODER_CARRY:
    {
        resp.type = TYPE_ENCODER;
        int32_t carry = static_cast<int8_t>(frame.data[1]); // signed 8-bit carry
        uint16_t value = (frame.data[2] << 8) | frame.data[3];
        uint32_t combined = (static_cast<int32_t>(carry) << 14) | value;
        resp.value = combined;
        updateEncoder(combined);
        break;
    }
    
    case CMD::READ::ENCODER_ABSOLUTE:
    {
        resp.type = TYPE_ENCODER;
        
        RCLCPP_INFO(rclcpp::get_logger("ServoManager"), "Frame data: [%02X %02X %02X %02X %02X %02X %02X %02X]", 
                   frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

        // Extract 6-byte encoder counter value from data[1] to data[6]
        uint64_t value = ((uint64_t)frame.data[1] << 40) | ((uint64_t)frame.data[2] << 32) | ((uint64_t)frame.data[3] << 24) | ((uint64_t)frame.data[4] << 16) | ((uint64_t)frame.data[5] << 8) | frame.data[6];
        
        RCLCPP_INFO(rclcpp::get_logger("ServoManager"), "Absolute encoder value: %lu", value);

        resp.value = (uint32_t)value;
        updateEncoder((uint32_t)value);
        break;
    }
    case CMD::READ::SPEED:
    {
        resp.type = TYPE_SPEED;
        int16_t rpm = static_cast<int16_t>((frame.data[1] << 8) | frame.data[2]);
        updateSpeedFromCAN(rpm);
        break;
    }
    default:
        resp.type = TYPE_NONE;
        break;
    }

    return resp;
}


void ServoManager::updateEncoder(uint32_t rawValue)
{
    int32_t delta = static_cast<int32_t>(rawValue - lastEncoder);
    lastEncoder = rawValue;
    accumulatedCounts += delta;
    accumulatedAngle = (static_cast<double>(accumulatedCounts) / ENCODER_CPR) * TWO_PI / gearRatio;
}

double ServoManager::getAbsoluteAngle() const
{
    return accumulatedAngle;
}

void ServoManager::setTargetAngle(double rad)
{
    targetEncoder = static_cast<uint32_t>((rad * gearRatio / TWO_PI) * ENCODER_CPR);
}

void ServoManager::updateSpeedFromCAN(int16_t rpm)
{
    speedRadPerSec = static_cast<double>(rpm) * TWO_PI / 60.0;
}