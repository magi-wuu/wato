#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation_common/grid.hpp"
namespace robot {
class ControlCore {
public:
  ControlCore(double lookahead, double tolerance, double speed, double angular)
      : lookahead_(lookahead), tolerance_(tolerance), speed_(speed),
        angular_(angular) {
    if (!(lookahead > 0 && tolerance > 0 && speed > 0 && angular > 0))
      throw std::invalid_argument("Invalid controller limits");
  }
  geometry_msgs::msg::Twist command(const nav_msgs::msg::Path &path,
                                    const geometry_msgs::msg::Pose &pose) const;

private:
  double lookahead_, tolerance_, speed_, angular_;
};
} // namespace robot
