/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 * Modified: Simplified for Maestro controller without voice/voltage feedback
 ******************************************************************************/

#include "requester/coordinator.hpp"

#include <sstream>

using namespace hexapod_interfaces::msg;
using namespace std::chrono_literals;

namespace brain {

CCoordinator::CCoordinator(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionPlanner> actionPlanner)
    : node_(node), actionPlanner_(actionPlanner) {
    textInterpreter_ = std::make_shared<CTextInterpreter>(node_);
    errorManagement_ = std::make_shared<CErrorManagement>(node_);
    timerErrorRequest_ = std::make_shared<CCallbackTimer>();
    timerMovementRequest_ = std::make_shared<CSimpleTimer>(false);
    timerNoRequest_ = std::make_shared<CSimpleTimer>(false);

    node->declare_parameter("velocity_factor_linear", 0.01);
    param_velocity_factor_linear_ =
        node->get_parameter("velocity_factor_linear").get_parameter_value().get<double>();

    node->declare_parameter("velocity_factor_rotation", 0.01);
    param_velocity_factor_rotation_ =
        node->get_parameter("velocity_factor_rotation").get_parameter_value().get<double>();

    // Deadzone to filter stick drift
    node->declare_parameter("joystick_deadzone", 0.25);
    param_joystick_deadzone_ =
        node->get_parameter("joystick_deadzone").get_parameter_value().get<double>();

    node->declare_parameter("autostart_listening", false);
    node->declare_parameter("activate_movement_waiting", false);
    param_activate_movement_waiting_ = false;

    isStanding_ = false;
    node->declare_parameter("joystick_timeout_ms", 300);
    joystickTimeout_ =
        std::chrono::milliseconds(node->get_parameter("joystick_timeout_ms").get_parameter_value().get<int>());

    node->declare_parameter("joystick_stop_delay_ms", 200);
    joystickStopDelay_ =
        std::chrono::milliseconds(node->get_parameter("joystick_stop_delay_ms").get_parameter_value().get<int>());

    RCLCPP_INFO(node_->get_logger(), "Robot starting in LAYDOWN. Press OPTIONS or DPAD UP to stand.");
}

void CCoordinator::joystickRequestReceived(const JoystickRequest& msg) {
    lastJoystickMsgTime_ = node_->get_clock()->now();
    joystickTimedOut_ = false;

    // Edge detection for buttons
    bool buttonStartPressed = msg.button_start && !prevButtonStart_;
    bool buttonSelectPressed = msg.button_select && !prevButtonSelect_;
    bool dpadUpPressed = (msg.dpad_vertical == 1) && !prevDpadUp_;
    bool dpadDownPressed = (msg.dpad_vertical == -1) && !prevDpadDown_;
    bool buttonAPressed = msg.button_a && !prevButtonA_;
    bool buttonBPressed = msg.button_b && !prevButtonB_;
    bool buttonXPressed = msg.button_x && !prevButtonX_;
    bool buttonYPressed = msg.button_y && !prevButtonY_;
    bool buttonL1Pressed = msg.button_l1 && !prevButtonL1_;
    bool buttonR1Pressed = msg.button_r1 && !prevButtonR1_;

    // Update previous states
    prevButtonStart_ = msg.button_start;
    prevButtonSelect_ = msg.button_select;
    prevDpadUp_ = (msg.dpad_vertical == 1);
    prevDpadDown_ = (msg.dpad_vertical == -1);
    prevButtonA_ = msg.button_a;
    prevButtonB_ = msg.button_b;
    prevButtonX_ = msg.button_x;
    prevButtonY_ = msg.button_y;
    prevButtonL1_ = msg.button_l1;
    prevButtonR1_ = msg.button_r1;

    // Ignore input while locked
    if (isNewMoveRequestLocked_) {
        return;
    }

    // SHARE - shutdown
    if (buttonSelectPressed) {
        requestShutdown(Prio::High);
        return;
    }

    // OPTIONS - toggle stand/laydown
    if (buttonStartPressed) {
        if (!isStanding_) {
            RCLCPP_INFO(node_->get_logger(), "OPTIONS: Standing up...");
            isStanding_ = true;
            submitRequestMove(MovementRequest::STAND_UP, 2000, "", Prio::High);
        } else {
            RCLCPP_INFO(node_->get_logger(), "OPTIONS: Laying down...");
            isStanding_ = false;
            submitRequestMove(MovementRequest::LAYDOWN, 2000, "", Prio::High);
        }
        return;
    }

    // Not standing - only allow stand up
    if (!isStanding_) {
        if (dpadUpPressed) {
            RCLCPP_INFO(node_->get_logger(), "DPAD UP: Standing up...");
            isStanding_ = true;
            submitRequestMove(MovementRequest::STAND_UP, 2000, "", Prio::High);
        }
        return;
    }

    // === STANDING ===

    // DPAD DOWN - lay down
    if (dpadDownPressed) {
        RCLCPP_INFO(node_->get_logger(), "DPAD DOWN: Laying down...");
        isStanding_ = false;
        submitRequestMove(MovementRequest::LAYDOWN, 2000, "", Prio::High);
        return;
    }

    // Action buttons
    if (buttonAPressed) {
        RCLCPP_INFO(node_->get_logger(), "X: Watch");
        submitRequestMove(MovementRequest::WATCH, 3000, "", Prio::High);
        return;
    }
    if (buttonBPressed) {
        RCLCPP_INFO(node_->get_logger(), "Circle: High Five");
        submitRequestMove(MovementRequest::HIGH_FIVE, 3000, "", Prio::High);
        return;
    }
    if (buttonXPressed) {
        RCLCPP_INFO(node_->get_logger(), "Square: Clap");
        submitRequestMove(MovementRequest::CLAP, 3000, "", Prio::High);
        return;
    }
    if (buttonYPressed) {
        RCLCPP_INFO(node_->get_logger(), "Triangle: Transport");
        isStanding_ = false;
        submitRequestMove(MovementRequest::TRANSPORT, 2000, "", Prio::High);
        return;
    }
    if (buttonL1Pressed) {
        RCLCPP_INFO(node_->get_logger(), "L1: Look Left");
        submitRequestMove(MovementRequest::LOOK_LEFT, 2000, "", Prio::High);
        return;
    }
    if (buttonR1Pressed) {
        RCLCPP_INFO(node_->get_logger(), "R1: Look Right");
        submitRequestMove(MovementRequest::LOOK_RIGHT, 2000, "", Prio::High);
        return;
    }

    // === ANALOG STICKS ===
    
    // Left stick - movement
    bool hasInput = false;
    if (std::abs(msg.left_stick_vertical) > param_joystick_deadzone_) {
        actualVelocity_.linear.x = -msg.left_stick_vertical * param_velocity_factor_linear_;
        hasInput = true;
    } else {
        actualVelocity_.linear.x = 0.0;
    }

    if (std::abs(msg.left_stick_horizontal) > param_joystick_deadzone_) {
        actualVelocity_.linear.y = msg.left_stick_horizontal * param_velocity_factor_linear_;
        hasInput = true;
    } else {
        actualVelocity_.linear.y = 0.0;
    }

    // Right stick horizontal - rotation
    if (std::abs(msg.right_stick_horizontal) > param_joystick_deadzone_) {
        actualVelocity_.angular.z = msg.right_stick_horizontal * param_velocity_factor_rotation_;
        hasInput = true;
    } else {
        actualVelocity_.angular.z = 0.0;
    }

    // Right stick vertical - body height
    hexapod_interfaces::msg::Pose body;
    if (std::abs(msg.right_stick_vertical) > param_joystick_deadzone_) {
        body.position.z = msg.right_stick_vertical * 0.04;
    }

    // Move if stick input
    if (hasInput) {
        lastNonNeutralTime_ = lastJoystickMsgTime_;
        sticksWereNeutral_ = false;
        // MOVE is a continuous stream; treat it as Background to avoid ActionPlanner cancelling on every update.
        submitRequestMove(MovementRequest::MOVE, 0, "", Prio::Background, body);
        return;
    }

    // Stop only after sticks have been neutral for a short period (debounce/hysteresis).
    // Without this, small BT/SDL jitter can cause rapid MOVE <-> MOVE_TO_STAND toggling ("random leg motion").
    if (actualMovementType_ == MovementRequest::MOVE && !sticksWereNeutral_) {
        if (lastNonNeutralTime_.nanoseconds() == 0) {
            lastNonNeutralTime_ = lastJoystickMsgTime_;
            return;
        }

        const auto age = lastJoystickMsgTime_ - lastNonNeutralTime_;
        const auto age_ms = std::chrono::milliseconds(age.nanoseconds() / 1000000);
        if (age_ms > joystickStopDelay_) {
            sticksWereNeutral_ = true;
            submitRequestMove(MovementRequest::MOVE_TO_STAND, 300, "", Prio::High);
        }
    }
}

void CCoordinator::speechRecognized(std::string text) { (void)text; }
void CCoordinator::supplyVoltageReceived(float voltage) { (void)voltage; }
void CCoordinator::servoStatusReceived(const ServoStatus& msg) { (void)msg; }

template <typename RequestT, typename... Args>
void CCoordinator::submitSingleRequest(Prio prio, Args&&... args) {
    static_assert(std::is_base_of<RequestBase, RequestT>::value, "RequestT must derive from RequestBase");
    std::vector<std::shared_ptr<RequestBase>> request_v;
    request_v.push_back(std::make_shared<RequestT>(std::forward<Args>(args)...));
    actionPlanner_->request(request_v, prio);
}

void CCoordinator::requestNotFound(std::string textRecognized, Prio prio) { (void)textRecognized; (void)prio; }

void CCoordinator::requestShutdown(Prio prio) {
    RCLCPP_INFO(node_->get_logger(), "Shutdown requested");
    if (isStanding_) {
        submitRequestMove(MovementRequest::LAYDOWN, 2000, "", prio);
    }
    submitSingleRequest<RequestSystem>(prio, true, true);
}

void CCoordinator::requestReactionOnError(std::string text, bool isShutdownRequested, Prio prio) {
    (void)text; (void)isShutdownRequested; (void)prio;
}

void CCoordinator::requestMusikOn(Prio prio) { (void)prio; }
void CCoordinator::requestMusikOff(Prio prio) { (void)prio; }
void CCoordinator::requestTalking(std::string text, Prio prio) { (void)text; (void)prio; }
void CCoordinator::requestChat(std::string text, Prio prio) { (void)text; (void)prio; }

void CCoordinator::requestTestBody() {
    hexapod_interfaces::msg::Pose body;
    body.position.x = 0.05;
    submitRequestMove(MovementRequest::TESTBODY, 2000, "", Prio::High, body);
}

void CCoordinator::requestTestLegs() {
    submitRequestMove(MovementRequest::TESTLEGS, 2000, "", Prio::High);
}

void CCoordinator::requestWaiting(Prio prio) { (void)prio; }

void CCoordinator::submitRequestMove(uint32_t movementType, uint32_t duration_ms, std::string comment,
                                     Prio prio, hexapod_interfaces::msg::Pose body) {
    (void)comment;

    // If we are about to execute a non-MOVE request, clear any background MOVE so it cannot
    // automatically resume after the higher-priority request finishes.
    // This is crucial because Prio::Background is "sticky" by design in CActionPlanner.
    if (movementType != MovementRequest::MOVE) {
        actionPlanner_->request({}, Prio::Background);
    }
    
    auto request = MovementRequest();
    request.type = movementType;
    request.body = body;
    request.velocity = actualVelocity_;
    request.duration_ms = duration_ms;
    request.name = movementTypeName_.at(request.type);
    
    std::vector<std::shared_ptr<RequestBase>> request_v;
    request_v.push_back(std::make_shared<CRequestMove>(request));
    actionPlanner_->request(request_v, prio);

    actualMovementType_ = movementType;

    // Don't lock for continuous MOVE
    if (movementType == MovementRequest::MOVE) {
        return;
    }
    
    isNewMoveRequestLocked_ = true;
    timerMovementRequest_->waitMsNonBlocking(duration_ms, [this]() {
        isNewMoveRequestLocked_ = false;
        actualMovementType_ = MovementRequest::NO_REQUEST;
    });
}

void CCoordinator::requestTellSupplyVoltage(Prio prio) { (void)prio; }
void CCoordinator::requestTellServoVoltage(Prio prio) { (void)prio; }
void CCoordinator::requestTellServoTemperature(Prio prio) { (void)prio; }

void CCoordinator::update() {
    // Safety: if joystick messages stop (teleop crash/disconnect), stop MOVE.
    // Without this, hexapod_movement stays in MOVE forever with last velocity.
    if (!isStanding_) return;
    if (actualMovementType_ != MovementRequest::MOVE) return;
    if (joystickTimedOut_) return;
    if (lastJoystickMsgTime_.nanoseconds() == 0) return;

    const auto now = node_->get_clock()->now();
    const auto age = now - lastJoystickMsgTime_;
    const auto age_ms = std::chrono::milliseconds(age.nanoseconds() / 1000000);

    if (age_ms > joystickTimeout_) {
        joystickTimedOut_ = true;
        actualVelocity_ = geometry_msgs::msg::Twist();  // zero
        RCLCPP_WARN(node_->get_logger(), "Joystick timeout (%ld ms). Stopping movement.",
                    static_cast<long>(age_ms.count()));
        submitRequestMove(MovementRequest::MOVE_TO_STAND, 300, "", Prio::Highest);
    }
}

}  // namespace brain
