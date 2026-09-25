#pragma once
#include "control_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  void tick();
  robot::ControlCore core_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  double path_received_ = 0, odom_received_ = 0, path_timeout_, odom_timeout_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
