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

    node->declare_parameter("joystick_deadzone", 0.1);
    param_joystick_deadzone_ =
        node->get_parameter("joystick_deadzone").get_parameter_value().get<double>();

    node->declare_parameter("autostart_listening", false);
    node->declare_parameter("activate_movement_waiting", false);
    param_activate_movement_waiting_ = false;  // Always disabled

    // Start in laydown position, wait for stand up command
    isStanding_ = false;
    RCLCPP_INFO(node_->get_logger(), "Robot starting in LAYDOWN position. Press DPAD UP to stand up.");
}

void CCoordinator::joystickRequestReceived(const JoystickRequest& msg) {
    // Edge detection for buttons (react only on press, not hold)
    bool buttonStartPressed = msg.button_start && !prevButtonStart_;
    bool buttonSelectPressed = msg.button_select && !prevButtonSelect_;
    bool dpadUpPressed = (msg.dpad_vertical == 1) && !prevDpadUp_;
    bool dpadDownPressed = (msg.dpad_vertical == -1) && !prevDpadDown_;

    // Update previous button states
    prevButtonStart_ = msg.button_start;
    prevButtonSelect_ = msg.button_select;
    prevDpadUp_ = (msg.dpad_vertical == 1);
    prevDpadDown_ = (msg.dpad_vertical == -1);

    // Ignore all input while locked (during stand up/lay down transitions)
    if (isNewMoveRequestLocked_) {
        return;
    }

    uint32_t duration_ms = 0;
    uint32_t newMovementType = MovementRequest::NO_REQUEST;
    hexapod_interfaces::msg::Pose body;

    // SHARE button - shutdown (edge detection)
    if (buttonSelectPressed) {
        requestShutdown(Prio::High);
        return;
    }

    // OPTIONS button (button_start) - Stand up / Lay down toggle (edge detection)
    if (buttonStartPressed) {
        if (!isStanding_) {
            newMovementType = MovementRequest::STAND_UP;
            duration_ms = 3000;
            RCLCPP_INFO(node_->get_logger(), "OPTIONS pressed: Standing up...");
        } else {
            newMovementType = MovementRequest::LAYDOWN;
            duration_ms = 3000;
            RCLCPP_INFO(node_->get_logger(), "OPTIONS pressed: Laying down...");
        }
        submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        return;
    }
    
    // DPAD UP - Stand up (edge detection)
    if (dpadUpPressed && !isStanding_) {
        newMovementType = MovementRequest::STAND_UP;
        duration_ms = 3000;
        RCLCPP_INFO(node_->get_logger(), "DPAD UP: Standing up...");
        submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        return;
    }
    // DPAD DOWN - Lay down (edge detection)
    if (dpadDownPressed && isStanding_) {
        newMovementType = MovementRequest::LAYDOWN;
        duration_ms = 3000;
        RCLCPP_INFO(node_->get_logger(), "DPAD DOWN: Laying down...");
        submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        return;
    }

    // Not standing - ignore all other input
    if (!isStanding_) {
        return;
    }

    // BUTTONS (only when standing)
    if (msg.button_a) {
        newMovementType = MovementRequest::WATCH;
        duration_ms = 5000;
        submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        return;
    }
    if (msg.button_b) {
        newMovementType = MovementRequest::HIGH_FIVE;
        duration_ms = 5000;
        submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        return;
    }

    // LEFT STICK - linear movement
    bool hasLinearInput = false;
    if (std::abs(msg.left_stick_vertical) > param_joystick_deadzone_) {
        actualVelocity_.linear.x = -msg.left_stick_vertical * param_velocity_factor_linear_;
        hasLinearInput = true;
    } else {
        actualVelocity_.linear.x = 0.0;
    }

    if (std::abs(msg.left_stick_horizontal) > param_joystick_deadzone_) {
        actualVelocity_.linear.y = msg.left_stick_horizontal * param_velocity_factor_linear_;
        hasLinearInput = true;
    } else {
        actualVelocity_.linear.y = 0.0;
    }

    // RIGHT STICK - rotation
    bool hasRotationInput = false;
    if (std::abs(msg.right_stick_horizontal) > param_joystick_deadzone_) {
        actualVelocity_.angular.z = msg.right_stick_horizontal * param_velocity_factor_rotation_;
        hasRotationInput = true;
    } else {
        actualVelocity_.angular.z = 0.0;
    }

    // Body height control
    if (std::abs(msg.right_stick_vertical) > param_joystick_deadzone_) {
        body.position.z = msg.right_stick_vertical * 0.04;
        hasLinearInput = true;
    }

    // Set MOVE only if there is actual stick input
    if (hasLinearInput || hasRotationInput) {
        newMovementType = MovementRequest::MOVE;
        submitRequestMove(newMovementType, duration_ms, "", Prio::High, body);
        return;
    }

    // Stop moving when sticks released (transition from MOVE to standing)
    if (actualMovementType_ == MovementRequest::MOVE) {
        newMovementType = MovementRequest::MOVE_TO_STAND;
        duration_ms = 500;
        submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        return;
    }
}

void CCoordinator::speechRecognized(std::string text) {
    // Speech recognition disabled
    (void)text;
}

void CCoordinator::supplyVoltageReceived(float voltage) {
    // Voltage monitoring disabled for Maestro
    (void)voltage;
}

void CCoordinator::servoStatusReceived(const ServoStatus& msg) {
    // Servo status monitoring disabled for Maestro
    (void)msg;
}

// ----------------------------------------------------------------------------
//  Requests
// ---------------------------------------------------------------------------
template <typename RequestT, typename... Args>
void CCoordinator::submitSingleRequest(Prio prio, Args&&... args) {
    static_assert(std::is_base_of<RequestBase, RequestT>::value, "RequestT must derive from RequestBase");
    std::vector<std::shared_ptr<RequestBase>> request_v;
    request_v.push_back(std::make_shared<RequestT>(std::forward<Args>(args)...));
    actionPlanner_->request(request_v, prio);
}

void CCoordinator::requestNotFound(std::string textRecognized, Prio prio) {
    (void)textRecognized;
    (void)prio;
}

void CCoordinator::requestShutdown(Prio prio) {
    RCLCPP_INFO(node_->get_logger(), "Shutdown requested");
    if (isStanding_) {
        submitRequestMove(MovementRequest::LAYDOWN, 2000, "", prio);
    }
    submitSingleRequest<RequestSystem>(prio, true, true);
}

void CCoordinator::requestReactionOnError(std::string text, bool isShutdownRequested, Prio prio) {
    (void)text;
    (void)isShutdownRequested;
    (void)prio;
}

void CCoordinator::requestMusikOn(Prio prio) { (void)prio; }
void CCoordinator::requestMusikOff(Prio prio) { (void)prio; }
void CCoordinator::requestTalking(std::string text, Prio prio) { (void)text; (void)prio; }
void CCoordinator::requestChat(std::string text, Prio prio) { (void)text; (void)prio; }

void CCoordinator::requestTestBody() {
    hexapod_interfaces::msg::Pose body;
    int duration_ms = 2000;

    body.position.x = 0.05;
    submitRequestMove(MovementRequest::TESTBODY, duration_ms, "", Prio::High, body);
    body.position.x = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_ms, "", Prio::High, body);

    body.position.y = 0.05;
    submitRequestMove(MovementRequest::TESTBODY, duration_ms, "", Prio::High, body);
    body.position.y = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_ms, "", Prio::High, body);

    body.position.z = 0.03;
    submitRequestMove(MovementRequest::TESTBODY, duration_ms, "", Prio::High, body);
    body.position.z = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_ms, "", Prio::High, body);
}

void CCoordinator::requestTestLegs() {
    submitRequestMove(MovementRequest::TESTLEGS, 2000, "", Prio::High);
}

void CCoordinator::requestWaiting(Prio prio) {
    (void)prio;
}

void CCoordinator::submitRequestMove(uint32_t movementType, uint32_t duration_ms, std::string comment,
                                     Prio prio, hexapod_interfaces::msg::Pose body) {
    std::vector<std::shared_ptr<RequestBase>> request_v;
    (void)comment;  // No speech output

    auto request = MovementRequest();
    request.type = movementType;
    request.body = body;
    request.velocity = actualVelocity_;
    request.duration_ms = duration_ms;
    request.name = movementTypeName_.at(request.type);
    request_v.push_back(std::make_shared<CRequestMove>(request));
    actionPlanner_->request(request_v, prio);

    actualMovementType_ = movementType;

    // Lock requests except for continuous MOVE
    if (MovementRequest::MOVE == movementType) {
        return;
    }
    isNewMoveRequestLocked_ = true;
    
    // Capture movementType for the lambda
    uint32_t capturedType = movementType;
    timerMovementRequest_->waitMsNonBlocking(duration_ms, [this, capturedType]() {
        isNewMoveRequestLocked_ = false;
        actualMovementType_ = MovementRequest::NO_REQUEST;
        
        // Update standing state AFTER movement completes
        if (capturedType == MovementRequest::LAYDOWN || capturedType == MovementRequest::TRANSPORT) {
            isStanding_ = false;
            RCLCPP_INFO(node_->get_logger(), "Robot is now laying down");
        } else if (capturedType == MovementRequest::STAND_UP) {
            isStanding_ = true;
            RCLCPP_INFO(node_->get_logger(), "Robot is now standing");
        }
    });
}

void CCoordinator::requestTellSupplyVoltage(Prio prio) { (void)prio; }
void CCoordinator::requestTellServoVoltage(Prio prio) { (void)prio; }
void CCoordinator::requestTellServoTemperature(Prio prio) { (void)prio; }

void CCoordinator::update() {
    // No automatic movement when idle
}

}  // namespace brain
