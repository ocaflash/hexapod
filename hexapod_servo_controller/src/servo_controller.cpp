/*******************************************************************************
 * Copyright (c) 2025
 * Servo Controller for Pololu Mini Maestro 18-Channel with MG996R
 ******************************************************************************/

#include "servo_controller.hpp"
#include <chrono>
#include <cmath>

using namespace hexapod_interfaces::msg;
using namespace std::chrono_literals;

// MG996R servo parameters
constexpr uint16_t SERVO_MIN_US = 500;    // microseconds for -90°
constexpr uint16_t SERVO_MAX_US = 2500;   // microseconds for +90°
constexpr uint16_t SERVO_CENTER_US = 1500; // microseconds for 0°
constexpr double SERVO_RANGE_DEG = 180.0;  // total range in degrees

ServoController::ServoController(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "Initializing Maestro Servo Controller...");

    std::string serialPort = node_->declare_parameter<std::string>("SERIAL_PORT", "/dev/ttyACM0");

    std::vector<std::string> names =
        node_->declare_parameter<std::vector<std::string>>("SERVO_NAME", std::vector<std::string>());

    std::vector<double> adaptations =
        node_->declare_parameter<std::vector<double>>("SERVO_ADAPTATION_DEG", std::vector<double>());

    std::vector<double> offsets =
        node_->declare_parameter<std::vector<double>>("SERVO_OFFSET_DEG", std::vector<double>());

    std::vector<bool> clockwise =
        node_->declare_parameter<std::vector<bool>>("SERVO_ORIENTATION_CLOCKWISE", std::vector<bool>());

    std::vector<int64_t> channels =
        node_->declare_parameter<std::vector<int64_t>>("SERVO_CHANNEL", std::vector<int64_t>());

    for (size_t i = 0; i < names.size(); ++i) {
        servos_[i] = CServo{names[i], static_cast<uint8_t>(channels[i]), clockwise[i], offsets[i], adaptations[i]};
        nameToIdx_[names[i]] = i;
    }

    protocol_ = std::make_shared<MaestroProtocol>(node_, serialPort);

    if (!protocol_->triggerConnection()) {
        RCLCPP_ERROR(node_->get_logger(), "Maestro connection FAILED");
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "Maestro connection successful");

    // Clear any existing errors (turns off red LED)
    uint16_t errors = 0;
    if (protocol_->getErrors(errors)) {
        if (errors != 0) {
            RCLCPP_WARN(node_->get_logger(), "Cleared Maestro errors: 0x%04X", errors);
        }
    }

    // Send Go Home command to reset servos
    protocol_->goHome();

    // Set speed and acceleration for smooth movement
    // Speed: 0 = unlimited, higher = slower (units of 0.25μs/10ms)
    // Acceleration: 0 = unlimited, higher = slower (units of 0.25μs/10ms/80ms)
    for (auto& [idx, servo] : servos_) {
        protocol_->setSpeed(servo.getChannel(), 30);       // Smooth speed
        protocol_->setAcceleration(servo.getChannel(), 5); // Smooth acceleration
    }

    subServoRequest_ = node_->create_subscription<ServoRequest>(
        "servo_request", 10,
        std::bind(&ServoController::onServoRequestReceived, this, std::placeholders::_1));

    subSingleServoRequest_ = node_->create_subscription<ServoAngle>(
        "single_servo_request", 10,
        std::bind(&ServoController::onSingleServoRequestReceived, this, std::placeholders::_1));

    pubAngles_ = node_->create_publisher<ServoAngles>("servo_angles", 10);
    pubStatus_ = node_->create_publisher<ServoStatus>("servo_status", 10);

    timer_ = node_->create_wall_timer(500ms, std::bind(&ServoController::onTimerStatus, this));

    RCLCPP_INFO(node_->get_logger(), "Servo Controller initialized with %zu servos", servos_.size());
}

uint16_t ServoController::angleToMicroseconds(double angle, size_t idx) {
    angle = angle - servos_.at(idx).getOffsetDegree() + servos_.at(idx).getAdaptation();
    
    if (!servos_.at(idx).isOrientationClockwise()) {
        angle = -angle;
    }
    
    // Map angle (-90 to +90) to microseconds (500 to 2500)
    double us = SERVO_CENTER_US + (angle / 90.0) * (SERVO_MAX_US - SERVO_CENTER_US);
    
    // Clamp to valid range
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    
    return static_cast<uint16_t>(us);
}

double ServoController::microsecondsToAngle(uint16_t us, size_t idx) {
    double angle = (static_cast<double>(us) - SERVO_CENTER_US) / (SERVO_MAX_US - SERVO_CENTER_US) * 90.0;
    
    if (!servos_.at(idx).isOrientationClockwise()) {
        angle = -angle;
    }
    
    angle = angle - servos_.at(idx).getAdaptation() + servos_.at(idx).getOffsetDegree();
    return angle;
}

void ServoController::onTimerStatus() {
    auto msg_status = ServoStatus();
    msg_status.header.stamp = node_->get_clock()->now();
    
    // Maestro doesn't provide temperature/voltage feedback
    // Publish minimal status
    msg_status.max_temperature = 0.0;
    msg_status.max_voltage = 0.0;
    msg_status.min_voltage = 0.0;
    msg_status.error_code = ServoStatus::NO_ERROR;
    
    // Check for Maestro errors
    uint16_t errors = 0;
    if (protocol_->getErrors(errors) && errors != 0) {
        RCLCPP_WARN(node_->get_logger(), "Maestro error code: 0x%04X", errors);
    }
    
    pubStatus_->publish(msg_status);
    publishAngles();
}

void ServoController::publishAngles() {
    auto msg_angles = ServoAngles();
    msg_angles.header.stamp = node_->get_clock()->now();
    
    for (auto& [idx, servo] : servos_) {
        if (idx < 18) {
            msg_angles.current_angles[idx].angle_deg = servo.getAngle();
            msg_angles.current_angles[idx].name = servo.getName();
        }
    }
    
    pubAngles_->publish(msg_angles);
}

void ServoController::onServoRequestReceived(const ServoRequest& msg) {
    for (size_t idx = 0; idx < msg.target_angles.size() && idx < servos_.size(); ++idx) {
        double req_angle = msg.target_angles[idx].angle_deg;
        double diff = std::abs(servos_.at(idx).getAngle() - req_angle);
        
        if (diff < 0.5) continue;  // Skip if angle change is too small
        
        servos_.at(idx).setAngle(req_angle);
        uint16_t us = angleToMicroseconds(req_angle, idx);
        protocol_->setTargetMicroseconds(servos_.at(idx).getChannel(), us);
    }
}

void ServoController::onSingleServoRequestReceived(const ServoAngle& msg) {
    RCLCPP_INFO(node_->get_logger(), "single_servo_request: %s: %.1f", msg.name.c_str(), msg.angle_deg);

    auto it = nameToIdx_.find(msg.name);
    if (it == nameToIdx_.end()) {
        RCLCPP_WARN(node_->get_logger(), "Unknown servo: %s", msg.name.c_str());
        return;
    }
    
    size_t idx = it->second;
    servos_.at(idx).setAngle(msg.angle_deg);
    uint16_t us = angleToMicroseconds(msg.angle_deg, idx);
    protocol_->setTargetMicroseconds(servos_.at(idx).getChannel(), us);
}
