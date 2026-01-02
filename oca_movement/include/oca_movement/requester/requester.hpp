/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
//
#include "geometry_msgs/msg/twist.hpp"
#include "oca_interfaces/msg/movement_request.hpp"
#include "oca_interfaces/msg/servo_index.hpp"
#include "oca_interfaces/msg/servo_request.hpp"
//
#include "action/action_executor.hpp"
#include "actionpackagesparser.hpp"
#include "gaitcontroller.hpp"
#include "kinematics.hpp"
#include "requests.hpp"

class CRequester {
   public:
    CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionExecutor> actionExecutor);
    virtual ~CRequester() = default;

    void update(std::chrono::milliseconds timeslice);

    // void onServoStatus(const oca_interfaces::msg::ServoStatus& msg);
    void onMovementRequest(const oca_interfaces::msg::MovementRequest& msg);

   private:
    rclcpp::Subscription<oca_interfaces::msg::MovementRequest>::SharedPtr m_subMovementRequest;

    void initializeRequestHandlers();
    void requestLayDown(const oca_interfaces::msg::MovementRequest& msg);
    void requestStandUp(const oca_interfaces::msg::MovementRequest& msg);
    void requestWaiting(const oca_interfaces::msg::MovementRequest& msg);
    void requestMove(const oca_interfaces::msg::MovementRequest& msg);
    void requestMoveToStand(const oca_interfaces::msg::MovementRequest& msg);
    void requestWatch(const oca_interfaces::msg::MovementRequest& msg);
    void requestLookSideways(const oca_interfaces::msg::MovementRequest& msg);
    void requestDance(const oca_interfaces::msg::MovementRequest& msg);
    void requestHighFive(const oca_interfaces::msg::MovementRequest& msg);
    void requestLegsWave(const oca_interfaces::msg::MovementRequest& msg);
    void requestBodyRoll(const oca_interfaces::msg::MovementRequest& msg);
    void requestBite(const oca_interfaces::msg::MovementRequest& msg);
    void requestStomp(const oca_interfaces::msg::MovementRequest& msg);
    void requestClap(const oca_interfaces::msg::MovementRequest& msg);
    void requestTransport(const oca_interfaces::msg::MovementRequest& msg);
    // the following requests are used for calibration and testing and are not used for normal operation
    void requestTestBody(const oca_interfaces::msg::MovementRequest& msg);
    void requestTestLegs(const oca_interfaces::msg::MovementRequest& msg);

    void requestNeutral(const oca_interfaces::msg::MovementRequest& msg);
    void requestCalibrate(const oca_interfaces::msg::MovementRequest& msg);
    void requestSequence(const oca_interfaces::msg::MovementRequest& msg);

    void liftLegsFirstTripodGroup();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionExecutor> actionExecutor_;
    std::shared_ptr<CKinematics> kinematics_;
    std::shared_ptr<CGaitController> gaitController_;
    std::shared_ptr<CActionPackagesParser> actionPackagesParser_;

    rclcpp::Subscription<oca_interfaces::msg::MovementRequest>::SharedPtr subMovementRequest_;
    // rclcpp::Subscription<oca_interfaces::msg::ServoStatus>::SharedPtr subServoStatus_;
    uint8_t activeRequest_ = oca_interfaces::msg::MovementRequest::NO_REQUEST;
    std::unordered_map<uint8_t, std::function<void(const oca_interfaces::msg::MovementRequest&)>>
        requestHandlers_;
    geometry_msgs::msg::Twist velocity_;
    oca_interfaces::msg::Pose poseBody_;
    bool transitionToMoveActive_ = false;
    bool transitionFromMoveActive_ = false;
};
