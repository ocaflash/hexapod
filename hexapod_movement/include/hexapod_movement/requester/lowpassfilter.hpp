#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "hexapod_interfaces/msg/pose.hpp"

// Helper for low-pass filtering a Vector3
geometry_msgs::msg::Vector3 lowPassFilterVector3(const geometry_msgs::msg::Vector3& lastValue,
                                                 const geometry_msgs::msg::Vector3& targetValue,
                                                 double alpha);

// Helper for low-pass filtering orientation (Euler angles)
auto lowPassFilterOrientation(const decltype(hexapod_interfaces::msg::Pose::orientation)& lastValue,
                              const decltype(hexapod_interfaces::msg::Pose::orientation)& targetValue,
                              double alpha);

// Helper for low-pass filtering a Twist
geometry_msgs::msg::Twist lowPassFilterTwist(const geometry_msgs::msg::Twist& lastValue,
                                             const geometry_msgs::msg::Twist& targetValue, double alpha);

// Helper for low-pass filtering a Pose
hexapod_interfaces::msg::Pose lowPassFilterPose(const hexapod_interfaces::msg::Pose& lastValue,
                                                  const hexapod_interfaces::msg::Pose& targetValue,
                                                  double alpha);