#include "costmap_node.hpp"
CostmapNode::CostmapNode()
    : Node("costmap"), core_(declare_parameter("resolution", 0.1),
                             declare_parameter("size", 42.0),
                             declare_parameter("inflation_radius", 3.0)) {
  pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        if (!scan->header.frame_id.empty())
          pub_->publish(core_.build(*scan));
      });
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
