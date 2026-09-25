#include "planner_core.hpp"
#include <gtest/gtest.h>
TEST(Planner, DetoursAndUnreachableGoals) {
  robot::PlannerCore core(40, 5, 0.2);
  auto m = robot::grid(1, 10, "world");
  m.data.assign(100, 0);
  for (int y = 0; y < 9; ++y)
    m.data[y * 10 + 5] = 100;
  auto s = robot::center(m, 2, 5), g = robot::center(m, 8, 5);
  auto path = core.plan(m, s, g);
  ASSERT_FALSE(path.poses.empty());
  for (const auto &p : path.poses) {
    int x, y;
    ASSERT_TRUE(robot::cell(m, p.pose.position.x, p.pose.position.y, x, y));
    EXPECT_LT(m.data[y * 10 + x], 40);
  }
  m.data[95] = 100;
  EXPECT_TRUE(core.plan(m, s, g).poses.empty());
  g.x = 100;
  EXPECT_TRUE(core.plan(m, s, g).poses.empty());
}
TEST(Planner, NoCornerCuttingAndSameCellEndpoint) {
  robot::PlannerCore core(40, 5, 0.2);
  auto m = robot::grid(1, 3, "world");
  m.data.assign(9, 100);
  m.data[0] = m.data[4] = 0;
  EXPECT_TRUE(core.plan(m, robot::center(m, 0, 0), robot::center(m, 1, 1))
                  .poses.empty());
  auto s = robot::center(m, 0, 0), g = s;
  g.x += 0.1;
  auto path = core.plan(m, s, g);
  ASSERT_EQ(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, g.x);
}
TEST(Planner, UnknownExplorationAndMalformedGrid) {
  robot::PlannerCore core(40, 5, 0.2);
  auto m = robot::grid(1, 3, "world");
  EXPECT_FALSE(core.plan(m, robot::center(m, 0, 0), robot::center(m, 2, 2))
                   .poses.empty());
  m.data.clear();
  EXPECT_TRUE(
      core.plan(m, geometry_msgs::msg::Point(), geometry_msgs::msg::Point())
          .poses.empty());
}

TEST(Planner, EscapesInflatedStartWithoutCrossingHigherCosts) {
  robot::PlannerCore core(40, 5, 0.2);
  auto m = robot::grid(1, 5, "world");
  m.data.assign(25, 100);
  m.data[10] = 47;
  m.data[11] = 43;
  m.data[12] = 39;
  m.data[13] = 0;
  auto start = robot::center(m, 0, 2), goal = robot::center(m, 3, 2);
  EXPECT_FALSE(core.plan(m, start, goal).poses.empty());
  m.data[11] = 48;
  EXPECT_TRUE(core.plan(m, start, goal).poses.empty());
  m.data[10] = 100;
  EXPECT_TRUE(core.plan(m, start, goal).poses.empty());
}
