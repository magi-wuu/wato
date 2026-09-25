#pragma once
#include "map_memory_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <memory>
class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  void update();
  robot::MapMemoryCore core_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr local_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Point last_position_;
  double last_yaw_ = 0, last_update_ = 0, distance_threshold_, angle_threshold_,
         max_interval_;
  bool initialized_ = false;
};
