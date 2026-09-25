#include "planner_node.hpp"
PlannerNode::PlannerNode()
    : Node("planner"), core_(declare_parameter("lethal_cost", 40),
                             declare_parameter("cost_weight", 5.0),
                             declare_parameter("unknown_cost", 0.2)) {
  tolerance_ = declare_parameter("goal_tolerance", 0.3);
  timeout_ = declare_parameter("goal_timeout", 180.0);
  replan_interval_ = declare_parameter("replan_interval", 1.0);
  pub_ = create_publisher<nav_msgs::msg::Path>(
      "/path", rclcpp::QoS(1).transient_local());
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
        map_ = m;
        if (state_ == State::Navigating)
          plan();
      });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; });
  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {
        stop();
        goal_ = *m;
        started_ = now().seconds();
        state_ = State::Navigating;
        plan();
      });
  timer_ = create_wall_timer(std::chrono::milliseconds(200),
                             std::bind(&PlannerNode::tick, this));
}
void PlannerNode::stop() {
  nav_msgs::msg::Path p;
  p.header.stamp = now();
  if (map_)
    p.header.frame_id = map_->header.frame_id;
  pub_->publish(p);
  state_ = State::WaitingForGoal;
}
void PlannerNode::tick() {
  if (state_ != State::Navigating)
    return;
  if (now().seconds() - started_ > timeout_) {
    RCLCPP_WARN(get_logger(), "Goal timed out");
    stop();
    return;
  }
  if (odom_ && odom_->header.frame_id == goal_.header.frame_id &&
      robot::distance(odom_->pose.pose.position, goal_.point) < tolerance_) {
    RCLCPP_INFO(get_logger(), "Goal reached");
    stop();
    return;
  }
  if (now().seconds() - last_plan_ >= replan_interval_)
    plan();
}
void PlannerNode::plan() {
  last_plan_ = now().seconds();
  nav_msgs::msg::Path p;
  p.header.stamp = now();
  if (map_)
    p.header.frame_id = map_->header.frame_id;
  if (map_ && odom_ && map_->header.frame_id == odom_->header.frame_id &&
      goal_.header.frame_id == map_->header.frame_id &&
      std::abs((now() - rclcpp::Time(odom_->header.stamp)).seconds()) < 1.0) {
    p = core_.plan(*map_, odom_->pose.pose.position, goal_.point);
    p.header.stamp = now();
    for (auto &pose : p.poses)
      pose.header = p.header;
  }
  if (p.poses.empty())
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "No valid path (check goal frame, bounds, obstacles and odometry)");
  pub_->publish(p);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
