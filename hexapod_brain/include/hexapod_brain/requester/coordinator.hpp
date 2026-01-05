/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
//
#include "hexapod_interfaces/msg/joystick_request.hpp"
#include "hexapod_interfaces/msg/movement_request.hpp"
#include "hexapod_interfaces/msg/servo_status.hpp"
//
#include "action/action_planner.hpp"
#include "handler/callback_timer.hpp"
#include "irequester.hpp"
#include "requester/error_management.hpp"
#include "requester/simpletimer.hpp"
#include "requester/text_interpreter.hpp"
#include "requester/utility.hpp"

namespace brain {

class CCoordinator : public IRequester {
   public:
    CCoordinator(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionPlanner> jobHandler);
    virtual ~CCoordinator() = default;

    void update() override;

    void joystickRequestReceived(const hexapod_interfaces::msg::JoystickRequest& msg);
    void speechRecognized(std::string text);
    void supplyVoltageReceived(float voltage);
    void servoStatusReceived(const hexapod_interfaces::msg::ServoStatus& msg);

   private:
    template <typename RequestT, typename... Args>
    void submitSingleRequest(Prio prio, Args&&... args);

    void submitRequestMove(uint32_t movementType, uint32_t duration_ms = 0, std::string comment = "",
                           Prio prio = Prio::Normal,
                           hexapod_interfaces::msg::Pose body = hexapod_interfaces::msg::Pose());

    void requestShutdown(Prio prio);
    void requestReactionOnError(std::string text, bool isShutdownRequested, Prio prio = Prio::Normal);
    void requestNotFound(std::string textRecognized, Prio prio = Prio::Normal);
    void requestDefault(Prio prio = Prio::Normal);
    void requestStopMoveBody(Prio prio = Prio::Normal);
    void requestTellSupplyVoltage(Prio prio = Prio::Normal);
    void requestTellServoVoltage(Prio prio = Prio::Normal);
    void requestTellServoTemperature(Prio prio = Prio::Normal);
    void requestMusikOn(Prio prio = Prio::Normal);
    void requestMusikOff(Prio prio = Prio::Normal);
    void requestTalking(std::string text, Prio prio = Prio::Normal);
    void requestChat(std::string text, Prio prio = Prio::Normal);
    void requestWaiting(Prio prio = Prio::Normal);
    void requestTestBody();
    void requestTestLegs();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPlanner> actionPlanner_;
    std::shared_ptr<CErrorManagement> errorManagement_;
    std::shared_ptr<CTextInterpreter> textInterpreter_;
    std::shared_ptr<CCallbackTimer> timerErrorRequest_;
    std::shared_ptr<CSimpleTimer> timerMovementRequest_;
    std::shared_ptr<CSimpleTimer> timerNoRequest_;

    std::atomic<bool> isNewMoveRequestLocked_{false};

    // Button state tracking for edge detection
    bool prevButtonStart_ = false;
    bool prevButtonSelect_ = false;
    bool prevDpadUp_ = false;
    bool prevDpadDown_ = false;
    bool prevButtonA_ = false;
    bool prevButtonB_ = false;
    bool prevButtonX_ = false;
    bool prevButtonY_ = false;
    bool prevButtonL1_ = false;
    bool prevButtonR1_ = false;

    geometry_msgs::msg::Twist actualVelocity_ = geometry_msgs::msg::Twist();
    uint32_t actualMovementType_ = hexapod_interfaces::msg::MovementRequest::NO_REQUEST;
    bool isStanding_ = false;
    bool isRelayOn_ = false;
    bool sticksWereNeutral_ = true;  // Track if sticks were in neutral before movement
    bool joystickTimedOut_ = false;

    rclcpp::Time lastJoystickMsgTime_{0, 0, RCL_ROS_TIME};
    rclcpp::Time lastNonNeutralTime_{0, 0, RCL_ROS_TIME};
    std::chrono::milliseconds joystickTimeout_{300};
    std::chrono::milliseconds joystickStopDelay_{200};
    uint32_t moveToStandDurationMs_{600};

    float param_velocity_factor_linear_ = 0.0;
    float param_velocity_factor_rotation_ = 0.0;
    float param_joystick_deadzone_ = 0.0;
    float param_activate_movement_waiting_ = false;

    std::map<uint32_t, std::string> movementTypeName_ = {
        {hexapod_interfaces::msg::MovementRequest::NO_REQUEST, "NO_REQUEST"},
        {hexapod_interfaces::msg::MovementRequest::LAYDOWN, "LAYDOWN"},
        {hexapod_interfaces::msg::MovementRequest::STAND_UP, "STAND_UP"},
        {hexapod_interfaces::msg::MovementRequest::WAITING, "WAITING"},
        {hexapod_interfaces::msg::MovementRequest::MOVE, "MOVE"},
        {hexapod_interfaces::msg::MovementRequest::MOVE_TO_STAND, "MOVE_TO_STAND"},
        {hexapod_interfaces::msg::MovementRequest::WATCH, "WATCH"},
        {hexapod_interfaces::msg::MovementRequest::LOOK_LEFT, "LOOK_LEFT"},
        {hexapod_interfaces::msg::MovementRequest::LOOK_RIGHT, "LOOK_RIGHT"},
        {hexapod_interfaces::msg::MovementRequest::DANCE, "DANCE"},
        {hexapod_interfaces::msg::MovementRequest::HIGH_FIVE, "HIGH_FIVE"},
        {hexapod_interfaces::msg::MovementRequest::LEGS_WAVE, "LEGS_WAVE"},
        {hexapod_interfaces::msg::MovementRequest::BODY_ROLL, "BODY_ROLL"},
        {hexapod_interfaces::msg::MovementRequest::BITE, "BITE"},
        {hexapod_interfaces::msg::MovementRequest::STOMP, "STOMP"},
        {hexapod_interfaces::msg::MovementRequest::CLAP, "CLAP"},
        {hexapod_interfaces::msg::MovementRequest::TRANSPORT, "TRANSPORT"},
        {hexapod_interfaces::msg::MovementRequest::TESTBODY, "TESTBODY"},
        {hexapod_interfaces::msg::MovementRequest::TESTLEGS, "TESTLEGS"},
        {hexapod_interfaces::msg::MovementRequest::NEUTRAL, "NEUTRAL"},
        {hexapod_interfaces::msg::MovementRequest::CALIBRATE, "CALIBRATE"},
    };
};

}  // namespace brain