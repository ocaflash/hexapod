/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
//
#include "oca_interfaces/msg/joystick_request.hpp"
#include "oca_interfaces/msg/movement_request.hpp"
#include "oca_interfaces/msg/servo_status.hpp"
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

    void joystickRequestReceived(const oca_interfaces::msg::JoystickRequest& msg);
    void speechRecognized(std::string text);
    void supplyVoltageReceived(float voltage);
    void servoStatusReceived(const oca_interfaces::msg::ServoStatus& msg);

   private:
    template <typename RequestT, typename... Args>
    void submitSingleRequest(Prio prio, Args&&... args);

    void submitRequestMove(uint32_t movementType, uint32_t duration_ms = 0, std::string comment = "",
                           Prio prio = Prio::Normal,
                           oca_interfaces::msg::Pose body = oca_interfaces::msg::Pose());

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

    geometry_msgs::msg::Twist actualVelocity_ = geometry_msgs::msg::Twist();
    uint32_t actualMovementType_ = oca_interfaces::msg::MovementRequest::NO_REQUEST;
    bool isStanding_ = false;
    bool isRelayOn_ = false;

    float param_velocity_factor_linear_ = 0.0;
    float param_velocity_factor_rotation_ = 0.0;
    float param_joystick_deadzone_ = 0.0;
    float param_activate_movement_waiting_ = false;

    std::map<uint32_t, std::string> movementTypeName_ = {
        {oca_interfaces::msg::MovementRequest::NO_REQUEST, "NO_REQUEST"},
        {oca_interfaces::msg::MovementRequest::LAYDOWN, "LAYDOWN"},
        {oca_interfaces::msg::MovementRequest::STAND_UP, "STAND_UP"},
        {oca_interfaces::msg::MovementRequest::WAITING, "WAITING"},
        {oca_interfaces::msg::MovementRequest::MOVE, "MOVE"},
        {oca_interfaces::msg::MovementRequest::MOVE_TO_STAND, "MOVE_TO_STAND"},
        {oca_interfaces::msg::MovementRequest::WATCH, "WATCH"},
        {oca_interfaces::msg::MovementRequest::LOOK_LEFT, "LOOK_LEFT"},
        {oca_interfaces::msg::MovementRequest::LOOK_RIGHT, "LOOK_RIGHT"},
        {oca_interfaces::msg::MovementRequest::DANCE, "DANCE"},
        {oca_interfaces::msg::MovementRequest::HIGH_FIVE, "HIGH_FIVE"},
        {oca_interfaces::msg::MovementRequest::LEGS_WAVE, "LEGS_WAVE"},
        {oca_interfaces::msg::MovementRequest::BODY_ROLL, "BODY_ROLL"},
        {oca_interfaces::msg::MovementRequest::BITE, "BITE"},
        {oca_interfaces::msg::MovementRequest::STOMP, "STOMP"},
        {oca_interfaces::msg::MovementRequest::CLAP, "CLAP"},
        {oca_interfaces::msg::MovementRequest::TRANSPORT, "TRANSPORT"},
        {oca_interfaces::msg::MovementRequest::TESTBODY, "TESTBODY"},
        {oca_interfaces::msg::MovementRequest::TESTLEGS, "TESTLEGS"},
        {oca_interfaces::msg::MovementRequest::NEUTRAL, "NEUTRAL"},
        {oca_interfaces::msg::MovementRequest::CALIBRATE, "CALIBRATE"},
    };
};

}  // namespace brain