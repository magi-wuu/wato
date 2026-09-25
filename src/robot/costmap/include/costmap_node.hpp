#pragma once
#include "costmap_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  robot::CostmapCore core_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};
