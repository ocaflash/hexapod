/*******************************************************************************
 * Copyright (c) 2025
 * Pololu Mini Maestro 18-Channel USB Servo Controller Protocol
 ******************************************************************************/

#include "maestro_protocol.hpp"
#include <cstring>

MaestroProtocol::MaestroProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& deviceName)
    : node_(node), deviceName_(deviceName) {}

MaestroProtocol::~MaestroProtocol() {
    closeConnection();
}

bool MaestroProtocol::triggerConnection() {
    isConnected_ = false;

    device_ = open(deviceName_.c_str(), O_RDWR | O_NOCTTY);
    if (device_ == -1) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open device: %s", deviceName_.c_str());
        return false;
    }

    struct termios options;
    if (tcgetattr(device_, &options) < 0) {
        close(device_);
        return false;
    }

    // Maestro UART: 115200 baud
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // 8N1 configuration
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;

    // Raw mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    if (tcsetattr(device_, TCSANOW, &options) != 0) {
        close(device_);
        return false;
    }

    tcflush(device_, TCIOFLUSH);
    isConnected_ = true;
    
    RCLCPP_INFO(node_->get_logger(), "Maestro connected on %s", deviceName_.c_str());
    return true;
}

void MaestroProtocol::closeConnection() {
    if (device_ >= 0) {
        close(device_);
        device_ = -1;
    }
    isConnected_ = false;
}

bool MaestroProtocol::isConnected() const {
    return isConnected_;
}

bool MaestroProtocol::writeBytes(const uint8_t* data, size_t length) {
    if (!isConnected_) return false;
    return write(device_, data, length) == static_cast<ssize_t>(length);
}

bool MaestroProtocol::readBytes(uint8_t* data, size_t length) {
    if (!isConnected_) return false;
    return read(device_, data, length) == static_cast<ssize_t>(length);
}

bool MaestroProtocol::setTarget(uint8_t channel, uint16_t target) {
    if (channel >= MAESTRO_MAX_CHANNELS) return false;
    
    if (target < MAESTRO_MIN_TARGET) target = MAESTRO_MIN_TARGET;
    if (target > MAESTRO_MAX_TARGET) target = MAESTRO_MAX_TARGET;

    uint8_t cmd[4] = {
        MAESTRO_CMD_SET_TARGET,
        channel,
        static_cast<uint8_t>(target & 0x7F),
        static_cast<uint8_t>((target >> 7) & 0x7F)
    };
    
    return writeBytes(cmd, sizeof(cmd));
}

bool MaestroProtocol::setTargetMicroseconds(uint8_t channel, uint16_t us) {
    return setTarget(channel, us * 4);
}

bool MaestroProtocol::setSpeed(uint8_t channel, uint16_t speed) {
    if (channel >= MAESTRO_MAX_CHANNELS) return false;

    uint8_t cmd[4] = {
        MAESTRO_CMD_SET_SPEED,
        channel,
        static_cast<uint8_t>(speed & 0x7F),
        static_cast<uint8_t>((speed >> 7) & 0x7F)
    };
    
    return writeBytes(cmd, sizeof(cmd));
}

bool MaestroProtocol::setAcceleration(uint8_t channel, uint8_t accel) {
    if (channel >= MAESTRO_MAX_CHANNELS) return false;

    uint8_t cmd[4] = {
        MAESTRO_CMD_SET_ACCELERATION,
        channel,
        static_cast<uint8_t>(accel & 0x7F),
        0
    };
    
    return writeBytes(cmd, sizeof(cmd));
}

bool MaestroProtocol::getPosition(uint8_t channel, uint16_t& position) {
    if (channel >= MAESTRO_MAX_CHANNELS) return false;

    uint8_t cmd[2] = {MAESTRO_CMD_GET_POSITION, channel};
    if (!writeBytes(cmd, sizeof(cmd))) return false;

    uint8_t response[2];
    if (!readBytes(response, sizeof(response))) return false;

    position = response[0] | (response[1] << 8);
    return true;
}

bool MaestroProtocol::getMovingState(bool& isMoving) {
    uint8_t cmd = MAESTRO_CMD_GET_MOVING_STATE;
    if (!writeBytes(&cmd, 1)) return false;

    uint8_t response;
    if (!readBytes(&response, 1)) return false;

    isMoving = (response != 0);
    return true;
}

bool MaestroProtocol::getErrors(uint16_t& errors) {
    uint8_t cmd = MAESTRO_CMD_GET_ERRORS;
    if (!writeBytes(&cmd, 1)) return false;

    uint8_t response[2];
    if (!readBytes(response, sizeof(response))) return false;

    errors = response[0] | (response[1] << 8);
    return true;
}

bool MaestroProtocol::goHome() {
    uint8_t cmd = MAESTRO_CMD_GO_HOME;
    return writeBytes(&cmd, 1);
}
