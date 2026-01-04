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

    node->declare_parameter("joystick_deadzone", 0.15);  // Increased to prevent drift
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
    bool buttonAPressed = msg.button_a && !prevButtonA_;
    bool buttonBPressed = msg.button_b && !prevButtonB_;
    bool buttonXPressed = msg.button_x && !prevButtonX_;
    bool buttonYPressed = msg.button_y && !prevButtonY_;
    bool buttonL1Pressed = msg.button_l1 && !prevButtonL1_;
    bool buttonR1Pressed = msg.button_r1 && !prevButtonR1_;

    // Update previous button states
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

    // Ignore all input while locked (during transitions)
    if (isNewMoveRequestLocked_) {
        return;
    }

    uint32_t duration_ms = 0;
    uint32_t newMovementType = MovementRequest::NO_REQUEST;
    hexapod_interfaces::msg::Pose body;

    // === SYSTEM BUTTONS (always active) ===
    
    // SHARE button - shutdown
    if (buttonSelectPressed) {
        requestShutdown(Prio::High);
        return;
    }

    // OPTIONS button - Stand up / Lay down toggle
    if (buttonStartPressed) {
        if (!isStanding_) {
            newMovementType = MovementRequest::STAND_UP;
            duration_ms = 2000;
            RCLCPP_INFO(node_->get_logger(), "OPTIONS: Standing up...");
            isStanding_ = true;  // Set immediately to prevent re-trigger
            sticksWereNeutral_ = false;  // Require sticks to go neutral before walking
        } else {
            newMovementType = MovementRequest::LAYDOWN;
            duration_ms = 2000;
            RCLCPP_INFO(node_->get_logger(), "OPTIONS: Laying down...");
            isStanding_ = false;  // Set immediately to prevent re-trigger
        }
        submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        return;
    }

    // === NOT STANDING - only allow stand up ===
    if (!isStanding_) {
        // DPAD UP - Stand up (alternative)
        if (dpadUpPressed) {
            newMovementType = MovementRequest::STAND_UP;
            duration_ms = 2000;
            RCLCPP_INFO(node_->get_logger(), "DPAD UP: Standing up...");
            isStanding_ = true;
            sticksWereNeutral_ = false;  // Require sticks to go neutral before walking
            submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        }
        return;
    }

    // === STANDING - process all inputs ===

    // DPAD DOWN - Lay down
    if (dpadDownPressed) {
        newMovementType = MovementRequest::LAYDOWN;
        duration_ms = 2000;
        RCLCPP_INFO(node_->get_logger(), "DPAD DOWN: Laying down...");
        isStanding_ = false;
        submitRequestMove(newMovementType, duration_ms, "", Prio::High);
        return;
    }

    // === ACTION BUTTONS (DualShock mapping) ===
    // X (button_a) - Watch/Look around
    if (buttonAPressed) {
        RCLCPP_INFO(node_->get_logger(), "X button: Watch");
        submitRequestMove(MovementRequest::WATCH, 3000, "", Prio::High);
        return;
    }

    // Circle (button_b) - High Five
    if (buttonBPressed) {
        RCLCPP_INFO(node_->get_logger(), "Circle button: High Five");
        submitRequestMove(MovementRequest::HIGH_FIVE, 3000, "", Prio::High);
        return;
    }

    // Square (button_x) - Clap
    if (buttonXPressed) {
        RCLCPP_INFO(node_->get_logger(), "Square button: Clap");
        submitRequestMove(MovementRequest::CLAP, 3000, "", Prio::High);
        return;
    }

    // Triangle (button_y) - Transport mode (compact)
    if (buttonYPressed) {
        RCLCPP_INFO(node_->get_logger(), "Triangle button: Transport");
        submitRequestMove(MovementRequest::TRANSPORT, 2000, "", Prio::High);
        isStanding_ = false;
        return;
    }

    // L1 - Test body movement
    if (buttonL1Pressed) {
        RCLCPP_INFO(node_->get_logger(), "L1 button: Test Body");
        submitRequestMove(MovementRequest::TESTBODY, 2000, "", Prio::High);
        return;
    }

    // R1 - Test legs
    if (buttonR1Pressed) {
        RCLCPP_INFO(node_->get_logger(), "R1 button: Test Legs");
        submitRequestMove(MovementRequest::TESTLEGS, 2000, "", Prio::High);
        return;
    }

    // DPAD LEFT/RIGHT - could be used for side stepping or rotation
    if (msg.dpad_horizontal == -1) {
        // DPAD LEFT - Stomp left
        // Could add action here
    }
    if (msg.dpad_horizontal == 1) {
        // DPAD RIGHT - Stomp right
        // Could add action here
    }

    // === ANALOG STICKS - Movement ===
    
    // Check if all sticks are within deadzone (neutral position)
    bool leftStickNeutral = std::abs(msg.left_stick_vertical) <= param_joystick_deadzone_ &&
                            std::abs(msg.left_stick_horizontal) <= param_joystick_deadzone_;
    bool rightStickNeutral = std::abs(msg.right_stick_horizontal) <= param_joystick_deadzone_ &&
                             std::abs(msg.right_stick_vertical) <= param_joystick_deadzone_;
    bool allSticksNeutral = leftStickNeutral && rightStickNeutral;
    
    // Track neutral state - must go through neutral before starting movement
    if (allSticksNeutral) {
        sticksWereNeutral_ = true;
    }
    
    // LEFT STICK - linear movement (forward/back, strafe)
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

    // RIGHT STICK horizontal - rotation (turn left/right)
    bool hasRotationInput = false;
    if (std::abs(msg.right_stick_horizontal) > param_joystick_deadzone_) {
        actualVelocity_.angular.z = msg.right_stick_horizontal * param_velocity_factor_rotation_;
        hasRotationInput = true;
    } else {
        actualVelocity_.angular.z = 0.0;
    }

    // RIGHT STICK vertical - body height
    hexapod_interfaces::msg::Pose body;
    if (std::abs(msg.right_stick_vertical) > param_joystick_deadzone_) {
        body.position.z = msg.right_stick_vertical * 0.04;
    }

    // MOVE only if:
    // 1. There is actual stick input
    // 2. Sticks were in neutral position before (prevents auto-walk after stand up)
    // 3. We're not in the middle of another action
    if ((hasLinearInput || hasRotationInput) && sticksWereNeutral_) {
        sticksWereNeutral_ = false;  // Reset flag
        if (actualMovementType_ == MovementRequest::NO_REQUEST || 
            actualMovementType_ == MovementRequest::MOVE) {
            submitRequestMove(MovementRequest::MOVE, 0, "", Prio::High, body);
        }
        return;
    }

    // Stop moving when sticks released (transition from MOVE to standing)
    if (actualMovementType_ == MovementRequest::MOVE && allSticksNeutral) {
        submitRequestMove(MovementRequest::MOVE_TO_STAND, 500, "", Prio::High);
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
    
    // Lock input during action
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
    // No automatic movement when idle
}

}  // namespace brain
