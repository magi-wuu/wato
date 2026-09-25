#pragma once
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "planner_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  void tick();
  void plan();
  void stop();
  enum class State { WaitingForGoal, Navigating };
  State state_ = State::WaitingForGoal;
  robot::PlannerCore core_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  geometry_msgs::msg::PointStamped goal_;
  double started_ = 0, last_plan_ = 0, tolerance_, timeout_, replan_interval_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
