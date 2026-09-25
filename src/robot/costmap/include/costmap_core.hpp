#pragma once
#include "navigation_common/grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace robot {
class CostmapCore {
public:
  CostmapCore(double resolution, double size, double radius)
      : resolution_(resolution), size_(size), radius_(radius) {
    if (!(radius > 0))
      throw std::invalid_argument("inflation_radius must be positive");
    grid(resolution, size, "");
  }
  nav_msgs::msg::OccupancyGrid
  build(const sensor_msgs::msg::LaserScan &scan) const;

private:
  double resolution_, size_, radius_;
};
} // namespace robot
