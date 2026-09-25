#pragma once
#include "nav_msgs/msg/path.hpp"
#include "navigation_common/grid.hpp"
namespace robot {
class PlannerCore {
public:
  PlannerCore(int lethal, double cost_weight, double unknown_cost)
      : lethal_(lethal), weight_(cost_weight), unknown_(unknown_cost) {
    if (lethal < 1 || lethal > 100 || cost_weight < 0 || unknown_cost < 0)
      throw std::invalid_argument("Invalid planner costs");
  }
  nav_msgs::msg::Path plan(const nav_msgs::msg::OccupancyGrid &map,
                           const geometry_msgs::msg::Point &start,
                           const geometry_msgs::msg::Point &goal) const;

private:
  int lethal_;
  double weight_, unknown_;
};
} // namespace robot
