#pragma once
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
namespace robot {
inline double steadySeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
inline double yaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z));
}
inline bool valid(const nav_msgs::msg::OccupancyGrid &g) {
  return g.info.resolution > 0 && std::isfinite(g.info.resolution) &&
         g.info.width > 0 && g.info.height > 0 &&
         g.data.size() == static_cast<size_t>(g.info.width) * g.info.height;
}
inline bool cell(const nav_msgs::msg::OccupancyGrid &g, double x, double y,
                 int &cx, int &cy) {
  if (!valid(g) || !std::isfinite(x) || !std::isfinite(y))
    return false;
  x -= g.info.origin.position.x;
  y -= g.info.origin.position.y;
  const double a = yaw(g.info.origin.orientation), c = std::cos(a),
               s = std::sin(a);
  const double gx = std::floor((c * x + s * y) / g.info.resolution);
  const double gy = std::floor((-s * x + c * y) / g.info.resolution);
  if (gx < 0 || gy < 0 || gx >= g.info.width || gy >= g.info.height)
    return false;
  cx = static_cast<int>(gx);
  cy = static_cast<int>(gy);
  return true;
}
inline geometry_msgs::msg::Point center(const nav_msgs::msg::OccupancyGrid &g,
                                        int x, int y) {
  const double a = yaw(g.info.origin.orientation),
               lx = (x + 0.5) * g.info.resolution,
               ly = (y + 0.5) * g.info.resolution;
  geometry_msgs::msg::Point p;
  p.x = g.info.origin.position.x + std::cos(a) * lx - std::sin(a) * ly;
  p.y = g.info.origin.position.y + std::sin(a) * lx + std::cos(a) * ly;
  return p;
}
inline nav_msgs::msg::OccupancyGrid grid(double resolution, double size,
                                         const std::string &frame) {
  if (!(resolution > 0 && size > 0) || !std::isfinite(size / resolution) ||
      size / resolution > 2000)
    throw std::invalid_argument("Invalid grid dimensions");
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = frame;
  g.info.resolution = resolution;
  g.info.width = g.info.height = std::ceil(size / resolution);
  g.info.origin.position.x = g.info.origin.position.y =
      -0.5 * g.info.width * resolution;
  g.info.origin.orientation.w = 1;
  g.data.assign(g.info.width * g.info.height, -1);
  return g;
}
inline double distance(const geometry_msgs::msg::Point &a,
                       const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
} // namespace robot
