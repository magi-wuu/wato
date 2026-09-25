#include "map_memory_node.hpp"
MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      core_(declare_parameter("resolution", 0.2),
            declare_parameter("size", 60.0),
            declare_parameter<std::string>("global_frame", "sim_world")) {
  distance_threshold_ = declare_parameter("update_distance", 0.3);
  angle_threshold_ = declare_parameter("update_angle", 0.2);
  max_interval_ = declare_parameter("max_update_interval", 1.0);
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());
  cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { local_ = m; });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; });
  core_.map().header.stamp = now();
  pub_->publish(core_.map());
  timer_ = create_wall_timer(std::chrono::milliseconds(200),
                             std::bind(&MapMemoryNode::update, this));
}
void MapMemoryNode::update() {
  if (!local_ || !odom_ || !robot::valid(*local_))
    return;
  if (odom_->header.frame_id != core_.map().header.frame_id)
    return;
  geometry_msgs::msg::Pose pose;
  // Use the transform at scan acquisition time, not a newer robot position.
  try {
    auto transform = tf_->lookupTransform(core_.map().header.frame_id,
                                          local_->header.frame_id,
                                          rclcpp::Time(local_->header.stamp));
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.orientation = transform.transform.rotation;
  } catch (const tf2::TransformException &) {
    return;
  }
  const double a = robot::yaw(pose.orientation);
  const double turn =
      std::abs(std::atan2(std::sin(a - last_yaw_), std::cos(a - last_yaw_)));
  if (initialized_ &&
      robot::distance(pose.position, last_position_) < distance_threshold_ &&
      turn < angle_threshold_ && now().seconds() - last_update_ < max_interval_)
    return;
  core_.integrate(*local_, pose);
  core_.map().header.stamp = local_->header.stamp;
  pub_->publish(core_.map());
  initialized_ = true;
  last_position_ = pose.position;
  last_yaw_ = a;
  last_update_ = now().seconds();
  local_.reset();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
