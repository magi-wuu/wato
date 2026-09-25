#include "map_memory_core.hpp"
#include <gtest/gtest.h>
TEST(MapMemory, RotationTranslationUnknownRetentionAndOverwrite) {
  robot::MapMemoryCore core(0.2, 10, "world");
  auto local = robot::grid(0.1, 2, "lidar");
  int x, y;
  ASSERT_TRUE(robot::cell(local, 0.45, 0.05, x, y));
  local.data[y * local.info.width + x] = 100;
  geometry_msgs::msg::Pose p;
  p.position.x = 1;
  p.position.y = 1;
  p.orientation.z = std::sin(M_PI / 4);
  p.orientation.w = std::cos(M_PI / 4);
  core.integrate(local, p);
  int mx, my;
  ASSERT_TRUE(robot::cell(core.map(), 0.95, 1.45, mx, my));
  EXPECT_EQ(core.map().data[my * core.map().info.width + mx], 100);
  local.data.assign(local.data.size(), -1);
  core.integrate(local, p);
  EXPECT_EQ(core.map().data[my * core.map().info.width + mx], 100);
  local.data.assign(local.data.size(), 0);
  core.integrate(local, p);
  EXPECT_EQ(core.map().data[my * core.map().info.width + mx], 0);
}
TEST(MapMemory, FineCellFreeDoesNotEraseNewObstacle) {
  robot::MapMemoryCore core(0.2, 10, "world");
  auto local = robot::grid(0.1, 2, "lidar");
  local.data.assign(local.data.size(), 0);
  local.data[0] = 100;
  geometry_msgs::msg::Pose p;
  p.orientation.w = 1;
  core.integrate(local, p);
  EXPECT_EQ(std::count(core.map().data.begin(), core.map().data.end(), 100), 1);
}
