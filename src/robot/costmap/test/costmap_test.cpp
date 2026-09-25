#include "costmap_core.hpp"
#include <gtest/gtest.h>
#include <limits>
TEST(Costmap, RaysInflationAndInvalidReadings) {
  robot::CostmapCore core(0.1, 10, 1);
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id = "lidar";
  s.range_min = 0.1;
  s.range_max = 4;
  s.angle_increment = 1;
  s.ranges = {2, std::numeric_limits<float>::quiet_NaN(), -1};
  auto g = core.build(s);
  int x, y;
  ASSERT_TRUE(robot::cell(g, 2, 0, x, y));
  EXPECT_EQ(g.data[y * g.info.width + x], 100);
  ASSERT_TRUE(robot::cell(g, 1.5, 0, x, y));
  EXPECT_NEAR(g.data[y * g.info.width + x], 50, 11);
  ASSERT_TRUE(robot::cell(g, 0.5, 0, x, y));
  EXPECT_EQ(g.data[y * g.info.width + x], 0);
  ASSERT_TRUE(robot::cell(g, -2, -2, x, y));
  EXPECT_EQ(g.data[y * g.info.width + x], -1);
}
TEST(Costmap, InfinityClearsRayWithoutObstacle) {
  robot::CostmapCore core(0.1, 10, 1);
  sensor_msgs::msg::LaserScan s;
  s.range_min = 0.1;
  s.range_max = 4;
  s.angle_increment = 1;
  s.ranges = {std::numeric_limits<float>::infinity()};
  auto g = core.build(s);
  EXPECT_EQ(std::count(g.data.begin(), g.data.end(), 100), 0);
  int x, y;
  ASSERT_TRUE(robot::cell(g, 3, 0, x, y));
  EXPECT_EQ(g.data[y * g.info.width + x], 0);
}
