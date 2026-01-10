/*******************************************************************************
 * Copyright (c) 2025
 * Pololu Mini Maestro 18-Channel USB Servo Controller Protocol
 ******************************************************************************/

#pragma once

#include <fcntl.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// Pololu Protocol Commands (Compact Protocol - no device number)
constexpr uint8_t MAESTRO_CMD_SET_TARGET = 0x84;
constexpr uint8_t MAESTRO_CMD_SET_SPEED = 0x87;
constexpr uint8_t MAESTRO_CMD_SET_ACCELERATION = 0x89;
constexpr uint8_t MAESTRO_CMD_GET_POSITION = 0x90;
constexpr uint8_t MAESTRO_CMD_GET_MOVING_STATE = 0x93;
constexpr uint8_t MAESTRO_CMD_GET_ERRORS = 0xA1;
constexpr uint8_t MAESTRO_CMD_GO_HOME = 0xA2;

// Maestro limits (quarter-microseconds)
constexpr uint16_t MAESTRO_MIN_TARGET = 2000;   // 500µs
constexpr uint16_t MAESTRO_MAX_TARGET = 10000;  // 2500µs
constexpr uint16_t MAESTRO_CENTER = 6000;       // 1500µs = center
constexpr uint8_t MAESTRO_MAX_CHANNELS = 18;
constexpr uint8_t MAESTRO_DEVICE_NUMBER = 12;   // Default device number

class MaestroProtocol {
   public:
    MaestroProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& deviceName);
    virtual ~MaestroProtocol();

    bool triggerConnection();
    void closeConnection();
    bool isConnected() const;

    // Servo Control
    bool setTarget(uint8_t channel, uint16_t target);          // quarter-microseconds
    bool setTargetMicroseconds(uint8_t channel, uint16_t us);  // microseconds
    bool setSpeed(uint8_t channel, uint16_t speed);            // 0 = unlimited
    bool setAcceleration(uint8_t channel, uint8_t accel);      // 0 = unlimited

    // Status
    bool getPosition(uint8_t channel, uint16_t& position);
    bool getMovingState(bool& isMoving);
    bool getErrors(uint16_t& errors);
    bool goHome();

   private:
    bool writeBytes(const uint8_t* data, size_t length);
    bool readBytes(uint8_t* data, size_t length);

    std::shared_ptr<rclcpp::Node> node_;
    std::string deviceName_;
    int device_ = -1;
    bool isConnected_ = false;
    uint32_t interCommandDelayUs_ = 1000;
};
