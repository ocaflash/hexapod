/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
//
#include "action/action_executor.hpp"
#include "requester/requester.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_movement");

    // Higher rate = smoother gait updates (default was 10Hz, which is visibly "steppy").
    node->declare_parameter<double>("refresh_rate_hz", 30.0);
    const double refresh_rate_hz = node->get_parameter("refresh_rate_hz").get_parameter_value().get<double>();
    const double safe_rate_hz = (refresh_rate_hz > 1.0) ? refresh_rate_hz : 1.0;
    auto actionExecutor = std::make_shared<CActionExecutor>(node);

    auto requester = std::make_shared<CRequester>(node, actionExecutor);

    rclcpp::Rate loop_rate(safe_rate_hz);
    auto timeslice_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / safe_rate_hz));

    while (rclcpp::ok()) {
        requester->update(timeslice_ms);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    return 0;
}
