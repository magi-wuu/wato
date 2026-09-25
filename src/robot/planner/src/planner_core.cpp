#include "planner_core.hpp"
#include <limits>
#include <queue>
namespace robot {
nav_msgs::msg::Path
PlannerCore::plan(const nav_msgs::msg::OccupancyGrid &m,
                  const geometry_msgs::msg::Point &start,
                  const geometry_msgs::msg::Point &goal) const {
  nav_msgs::msg::Path path;
  path.header = m.header;
  int sx, sy, gx, gy;
  if (!cell(m, start.x, start.y, sx, sy) || !cell(m, goal.x, goal.y, gx, gy))
    return path;
  const int w = m.info.width, h = m.info.height, source = sy * w + sx,
            target = gy * w + gx;
  auto free = [&](int x, int y) {
    return x >= 0 && y >= 0 && x < w && y < h && m.data[y * w + x] < lethal_;
  };
  // The spawn pose can lie inside the conservative inflation margin.
  // Permit escape toward equal/lower costs, but never from an occupied cell.
  if (m.data[source] >= 100 || !free(gx, gy))
    return path;
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  std::vector<double> scores(m.data.size(),
                             std::numeric_limits<double>::infinity());
  std::vector<int> parent(m.data.size(), -1);
  std::vector<bool> closed(m.data.size(), false);
  scores[source] = 0;
  open.emplace(std::hypot(gx - sx, gy - sy), source);
  while (!open.empty()) {
    const int u = open.top().second;
    open.pop();
    if (closed[u])
      continue;
    closed[u] = true;
    if (u == target)
      break;
    const int x = u % w, y = u / w;
    auto reachable = [&](int nx, int ny) {
      if (nx < 0 || ny < 0 || nx >= w || ny >= h)
        return false;
      const int next = m.data[ny * w + nx];
      return free(nx, ny) ||
             (m.data[u] >= lethal_ && next < 100 && next <= m.data[u]);
    };
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        if ((!dx && !dy) || !reachable(x + dx, y + dy))
          continue;
        // Diagonal moves cannot cut between occupied cells.
        if (dx && dy && (!reachable(x + dx, y) || !reachable(x, y + dy)))
          continue;
        int v = (y + dy) * w + x + dx;
        double cost = m.data[v] < 0 ? unknown_ : m.data[v] / 100.0;
        double candidate =
            scores[u] + std::hypot(dx, dy) * (1 + weight_ * cost);
        if (candidate < scores[v]) {
          scores[v] = candidate;
          parent[v] = u;
          open.emplace(candidate + std::hypot(gx - x - dx, gy - y - dy), v);
        }
      }
  }
  if (!closed[target])
    return path;
  std::vector<int> cells;
  for (int u = target; u != -1; u = parent[u])
    cells.push_back(u);
  std::reverse(cells.begin(), cells.end());
  for (int u : cells) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position = center(m, u % w, u / w);
    p.pose.orientation.w = 1;
    path.poses.push_back(p);
  }
  path.poses.front().pose.position = start;
  // Retain exact requested endpoint, including when start and goal share a
  // cell.
  geometry_msgs::msg::PoseStamped p = path.poses.back();
  p.pose.position = goal;
  path.poses.push_back(p);
  return path;
}
} // namespace robot
