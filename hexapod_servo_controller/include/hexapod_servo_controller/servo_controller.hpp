/*******************************************************************************
 * Copyright (c) 2025
 * Servo Controller for Pololu Mini Maestro 18-Channel with MG996R
 ******************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "maestro_protocol.hpp"
#include "hexapod_interfaces/msg/servo_angle.hpp"
#include "hexapod_interfaces/msg/servo_angles.hpp"
#include "hexapod_interfaces/msg/servo_request.hpp"
#include "hexapod_interfaces/msg/servo_status.hpp"
#include "rclcpp/rclcpp.hpp"

class CServo {
   public:
    CServo(const std::string& name, uint8_t channel, bool orientation_clockwise, 
           double offset_degree, double adaptation)
        : name_(name),
          channel_(channel),
          orientation_clockwise_(orientation_clockwise),
          offset_degree_(offset_degree),
          adaptation_(adaptation) {}

    CServo() = default;
    virtual ~CServo() = default;

    std::string getName() const { return name_; }
    uint8_t getChannel() const { return channel_; }
    bool isOrientationClockwise() const { return orientation_clockwise_; }
    double getOffsetDegree() const { return offset_degree_; }
    double getAdaptation() const { return adaptation_; }
    double getAngle() const { return angle_; }

    void setAngle(double angle) { angle_ = angle; }
    void setChannel(uint8_t channel) { channel_ = channel; }

   private:
    std::string name_;
    uint8_t channel_ = 0;
    bool orientation_clockwise_ = true;
    double offset_degree_ = 0.0;
    double adaptation_ = 0.0;
    double angle_ = 0.0;
};

class ServoController {
   public:
    ServoController(std::shared_ptr<rclcpp::Node> node);
    virtual ~ServoController() = default;

    void onServoRequestReceived(const hexapod_interfaces::msg::ServoRequest& msg);
    void onSingleServoRequestReceived(const hexapod_interfaces::msg::ServoAngle& msg);

   private:
    void onTimerStatus();
    void publishAngles();

    // MG996R: ~180° range, 500-2500µs pulse width
    uint16_t angleToMicroseconds(double angle, size_t idx);
    double microsecondsToAngle(uint16_t us, size_t idx);

    std::map<size_t, CServo> servos_;
    std::map<std::string, size_t> nameToIdx_;
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<hexapod_interfaces::msg::ServoRequest>::SharedPtr subServoRequest_;
    rclcpp::Subscription<hexapod_interfaces::msg::ServoAngle>::SharedPtr subSingleServoRequest_;
    rclcpp::Publisher<hexapod_interfaces::msg::ServoStatus>::SharedPtr pubStatus_;
    rclcpp::Publisher<hexapod_interfaces::msg::ServoAngles>::SharedPtr pubAngles_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<MaestroProtocol> protocol_;
};
