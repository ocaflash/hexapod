/*******************************************************************************
 * Copyright (c) 2025
 * Main node for Pololu Mini Maestro Servo Controller
 ******************************************************************************/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "servo_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("node_servo_controller");
    auto controller = std::make_shared<ServoController>(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
