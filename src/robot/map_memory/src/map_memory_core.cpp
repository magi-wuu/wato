#include "map_memory_core.hpp"
namespace robot {
void MapMemoryCore::integrate(const nav_msgs::msg::OccupancyGrid &local,
                              const geometry_msgs::msg::Pose &pose) {
  if (!valid(local))
    return;
  const double a = yaw(pose.orientation), c = std::cos(a), s = std::sin(a);
  // Multiple fine cells can land in one coarse cell. Keep the highest NEW cost,
  // then overwrite old data once; a later free sample must not erase an
  // obstacle.
  std::vector<int8_t> fresh(map_.data.size(), -1);
  for (unsigned y = 0; y < local.info.height; ++y)
    for (unsigned x = 0; x < local.info.width; ++x) {
      const auto v = local.data[y * local.info.width + x];
      if (v < 0)
        continue;
      auto p = center(local, x, y);
      int mx, my;
      if (cell(map_, pose.position.x + c * p.x - s * p.y,
               pose.position.y + s * p.x + c * p.y, mx, my)) {
        auto &dest = fresh[my * map_.info.width + mx];
        dest = std::max(dest, v);
      }
    }
  for (size_t i = 0; i < fresh.size(); ++i)
    if (fresh[i] >= 0)
      map_.data[i] = fresh[i];
}
} // namespace robot
