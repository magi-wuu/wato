#pragma once
#include "navigation_common/grid.hpp"
namespace robot {
class MapMemoryCore {
public:
  MapMemoryCore(double resolution, double size, const std::string &frame)
      : map_(grid(resolution, size, frame)) {}
  void integrate(const nav_msgs::msg::OccupancyGrid &local,
                 const geometry_msgs::msg::Pose &pose);
  nav_msgs::msg::OccupancyGrid &map() { return map_; }

private:
  nav_msgs::msg::OccupancyGrid map_;
};
} // namespace robot
