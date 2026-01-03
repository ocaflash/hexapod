/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "handler/servohandler.hpp"

#include <rclcpp/create_timer.hpp>

using namespace std::chrono_literals;
using namespace hexapod_interfaces::msg;

// Servo indices for 18 servos (no head)
// Layout: RightFront(0-2), RightMid(3-5), RightBack(6-8), LeftFront(9-11), LeftMid(12-14), LeftBack(15-17)
enum ServoIdx {
    LEG_RIGHT_FRONT_COXA = 0, LEG_RIGHT_FRONT_FEMUR, LEG_RIGHT_FRONT_TIBIA,
    LEG_RIGHT_MID_COXA, LEG_RIGHT_MID_FEMUR, LEG_RIGHT_MID_TIBIA,
    LEG_RIGHT_BACK_COXA, LEG_RIGHT_BACK_FEMUR, LEG_RIGHT_BACK_TIBIA,
    LEG_LEFT_FRONT_COXA, LEG_LEFT_FRONT_FEMUR, LEG_LEFT_FRONT_TIBIA,
    LEG_LEFT_MID_COXA, LEG_LEFT_MID_FEMUR, LEG_LEFT_MID_TIBIA,
    LEG_LEFT_BACK_COXA, LEG_LEFT_BACK_FEMUR, LEG_LEFT_BACK_TIBIA
};

CServoHandler::CServoHandler(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    // Initialize servo names
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_COXA).name = "LEG_RIGHT_FRONT_COXA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_FEMUR).name = "LEG_RIGHT_FRONT_FEMUR";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_TIBIA).name = "LEG_RIGHT_FRONT_TIBIA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_COXA).name = "LEG_RIGHT_MID_COXA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_FEMUR).name = "LEG_RIGHT_MID_FEMUR";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_TIBIA).name = "LEG_RIGHT_MID_TIBIA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_COXA).name = "LEG_RIGHT_BACK_COXA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_FEMUR).name = "LEG_RIGHT_BACK_FEMUR";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_TIBIA).name = "LEG_RIGHT_BACK_TIBIA";

    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_COXA).name = "LEG_LEFT_FRONT_COXA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_FEMUR).name = "LEG_LEFT_FRONT_FEMUR";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_TIBIA).name = "LEG_LEFT_FRONT_TIBIA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_COXA).name = "LEG_LEFT_MID_COXA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_FEMUR).name = "LEG_LEFT_MID_FEMUR";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_TIBIA).name = "LEG_LEFT_MID_TIBIA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_COXA).name = "LEG_LEFT_BACK_COXA";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_FEMUR).name = "LEG_LEFT_BACK_FEMUR";
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_TIBIA).name = "LEG_LEFT_BACK_TIBIA";

    callbackTimer_ = std::make_unique<CCallbackTimer>();

    pubServoRequest_ = node_->create_publisher<hexapod_interfaces::msg::ServoRequest>("servo_request", 10);
}

void CServoHandler::setOnDoneCallback(std::function<void()> cb) {
    doneCallback_ = cb;
}

void CServoHandler::run(std::shared_ptr<CRequestSendDuration> request) {
    msgServoRequest_.header.stamp = node_->now();
    msgServoRequest_.time_to_reach_target_angles_ms = request->durationMs();
    pubServoRequest_->publish(msgServoRequest_);

    if (request->blocking()) {
        callbackTimer_->start(request->durationMs(), std::bind(&CServoHandler::timerCallback, this), true);
    }
}

void CServoHandler::run(std::shared_ptr<CRequestLegs> request) {
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_COXA).angle_deg =
        request->angles().at(ELegIndex::RightFront).degCoxa;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_FEMUR).angle_deg =
        request->angles().at(ELegIndex::RightFront).degFemur;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_TIBIA).angle_deg =
        request->angles().at(ELegIndex::RightFront).degTibia;

    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_COXA).angle_deg =
        request->angles().at(ELegIndex::RightMid).degCoxa;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_FEMUR).angle_deg =
        request->angles().at(ELegIndex::RightMid).degFemur;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_TIBIA).angle_deg =
        request->angles().at(ELegIndex::RightMid).degTibia;

    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_COXA).angle_deg =
        request->angles().at(ELegIndex::RightBack).degCoxa;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_FEMUR).angle_deg =
        request->angles().at(ELegIndex::RightBack).degFemur;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_TIBIA).angle_deg =
        request->angles().at(ELegIndex::RightBack).degTibia;

    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_COXA).angle_deg =
        request->angles().at(ELegIndex::LeftFront).degCoxa;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_FEMUR).angle_deg =
        request->angles().at(ELegIndex::LeftFront).degFemur;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_TIBIA).angle_deg =
        request->angles().at(ELegIndex::LeftFront).degTibia;

    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_COXA).angle_deg =
        request->angles().at(ELegIndex::LeftMid).degCoxa;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_FEMUR).angle_deg =
        request->angles().at(ELegIndex::LeftMid).degFemur;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_TIBIA).angle_deg =
        request->angles().at(ELegIndex::LeftMid).degTibia;

    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_COXA).angle_deg =
        request->angles().at(ELegIndex::LeftBack).degCoxa;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_FEMUR).angle_deg =
        request->angles().at(ELegIndex::LeftBack).degFemur;
    msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_TIBIA).angle_deg =
        request->angles().at(ELegIndex::LeftBack).degTibia;
}

void CServoHandler::run(std::shared_ptr<CRequestLeg> request) {
    switch (request->legIndex()) {
        case ELegIndex::RightFront:
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_COXA).angle_deg = request->angles().degCoxa;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_FEMUR).angle_deg = request->angles().degFemur;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_FRONT_TIBIA).angle_deg = request->angles().degTibia;
            break;
        case ELegIndex::RightMid:
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_COXA).angle_deg = request->angles().degCoxa;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_FEMUR).angle_deg = request->angles().degFemur;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_MID_TIBIA).angle_deg = request->angles().degTibia;
            break;
        case ELegIndex::RightBack:
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_COXA).angle_deg = request->angles().degCoxa;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_FEMUR).angle_deg = request->angles().degFemur;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_RIGHT_BACK_TIBIA).angle_deg = request->angles().degTibia;
            break;
        case ELegIndex::LeftFront:
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_COXA).angle_deg = request->angles().degCoxa;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_FEMUR).angle_deg = request->angles().degFemur;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_FRONT_TIBIA).angle_deg = request->angles().degTibia;
            break;
        case ELegIndex::LeftMid:
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_COXA).angle_deg = request->angles().degCoxa;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_FEMUR).angle_deg = request->angles().degFemur;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_MID_TIBIA).angle_deg = request->angles().degTibia;
            break;
        case ELegIndex::LeftBack:
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_COXA).angle_deg = request->angles().degCoxa;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_FEMUR).angle_deg = request->angles().degFemur;
            msgServoRequest_.target_angles.at(ServoIdx::LEG_LEFT_BACK_TIBIA).angle_deg = request->angles().degTibia;
            break;
        default:
            break;
    }
}

void CServoHandler::run(std::shared_ptr<CRequestHead> request) {
    // Head removed - no operation
    (void)request;
}

void CServoHandler::timerCallback() {
    if (doneCallback_) {
        doneCallback_();
    }
}

void CServoHandler::cancel() {
    callbackTimer_->stop();
}
