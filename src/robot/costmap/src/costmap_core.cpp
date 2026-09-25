#include "costmap_core.hpp"
#include <vector>
namespace robot {
nav_msgs::msg::OccupancyGrid
CostmapCore::build(const sensor_msgs::msg::LaserScan &scan) const {
  auto g = grid(resolution_, size_, scan.header.frame_id);
  g.header = scan.header;
  std::vector<std::pair<int, int>> obstacles;
  if (!(scan.range_max > scan.range_min) ||
      !std::isfinite(scan.angle_increment))
    return g;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i],
                 a = scan.angle_min + i * scan.angle_increment;
    if (std::isnan(r) || r < scan.range_min || !std::isfinite(a))
      continue;
    const bool hit = std::isfinite(r) && r < scan.range_max;
    const double reach = std::min(static_cast<double>(scan.range_max), r);
    // Only observed rays become free. Unseen/occluded space stays unknown.
    for (double d = 0; d < reach; d += resolution_ * 0.5) {
      int x, y;
      if (!cell(g, d * std::cos(a), d * std::sin(a), x, y))
        break;
      g.data[y * g.info.width + x] = 0;
    }
    int x, y;
    if (hit && cell(g, r * std::cos(a), r * std::sin(a), x, y))
      obstacles.emplace_back(x, y);
  }
  const int n = std::ceil(radius_ / resolution_);
  for (const auto &[ox, oy] : obstacles) {
    for (int dy = -n; dy <= n; ++dy)
      for (int dx = -n; dx <= n; ++dx) {
        const int x = ox + dx, y = oy + dy;
        const double d = std::hypot(dx, dy) * resolution_;
        if (x < 0 || y < 0 || x >= static_cast<int>(g.info.width) ||
            y >= static_cast<int>(g.info.height) || d > radius_)
          continue;
        const int cost = std::lround(100 * (1 - d / radius_));
        auto &v = g.data[y * g.info.width + x];
        v = std::max(static_cast<int>(v), cost);
      }
  }
  return g;
}
} // namespace robot
