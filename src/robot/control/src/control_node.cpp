#include "control_node.hpp"
ControlNode::ControlNode()
    : Node("control"), core_(declare_parameter("lookahead_distance", 0.6),
                             declare_parameter("goal_tolerance", 0.3),
                             declare_parameter("linear_speed", 0.5),
                             declare_parameter("max_angular_speed", 0.6)) {
  path_timeout_ = declare_parameter("path_timeout", 2.5);
  odom_timeout_ = declare_parameter("odom_timeout", 0.5);
  pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::ConstSharedPtr m) {
        path_ = m;
        path_received_ = robot::steadySeconds();
      });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_ = m;
        odom_received_ = robot::steadySeconds();
      });
  timer_ = create_wall_timer(std::chrono::milliseconds(50),
                             std::bind(&ControlNode::tick, this));
}
void ControlNode::tick() {
  geometry_msgs::msg::Twist cmd;
  const double t = robot::steadySeconds();
  if (path_ && odom_ && t - path_received_ < path_timeout_ &&
      t - odom_received_ < odom_timeout_ &&
      std::abs((now() - rclcpp::Time(odom_->header.stamp)).seconds()) <
          odom_timeout_ &&
      path_->header.frame_id == odom_->header.frame_id)
    cmd = core_.command(*path_, odom_->pose.pose);
  pub_->publish(cmd);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
