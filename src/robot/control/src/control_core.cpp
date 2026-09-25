#include "control_core.hpp"
#include <limits>
namespace robot {
geometry_msgs::msg::Twist
ControlCore::command(const nav_msgs::msg::Path &path,
                     const geometry_msgs::msg::Pose &pose) const {
  geometry_msgs::msg::Twist cmd;
  if (path.poses.empty())
    return cmd;
  const double remaining =
      distance(pose.position, path.poses.back().pose.position);
  if (remaining < tolerance_)
    return cmd;
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double d = distance(pose.position, path.poses[i].pose.position);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  auto target = path.poses.back().pose.position;
  double accumulated = 0;
  for (size_t i = nearest + 1; i < path.poses.size(); ++i) {
    auto a = path.poses[i - 1].pose.position, b = path.poses[i].pose.position;
    const double segment = distance(a, b);
    if (segment > 0 && accumulated + segment >= lookahead_) {
      const double t = (lookahead_ - accumulated) / segment;
      target.x = a.x + t * (b.x - a.x);
      target.y = a.y + t * (b.y - a.y);
      break;
    }
    accumulated += segment;
  }
  const double dx = target.x - pose.position.x, dy = target.y - pose.position.y,
               theta = yaw(pose.orientation);
  const double local_x = std::cos(theta) * dx + std::sin(theta) * dy;
  const double local_y = -std::sin(theta) * dx + std::cos(theta) * dy;
  const double angle = std::atan2(local_y, local_x);
  if (std::abs(angle) > 0.7) {
    cmd.angular.z = std::clamp(1.5 * angle, -angular_, angular_);
    return cmd;
  }
  const double curvature = 2 * local_y / std::max(dx * dx + dy * dy, 0.01);
  cmd.linear.x = std::min(speed_, remaining) * std::max(0.2, std::cos(angle));
  cmd.linear.x =
      std::min(cmd.linear.x, angular_ / std::max(std::abs(curvature), 0.001));
  cmd.angular.z = std::clamp(cmd.linear.x * curvature, -angular_, angular_);
  return cmd;
}
} // namespace robot
