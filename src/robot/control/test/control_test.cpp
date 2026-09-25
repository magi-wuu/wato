#include "control_core.hpp"
#include <gtest/gtest.h>
TEST(Control, EmptyGoalAndTurnBehind) {
  robot::ControlCore core(0.6, 0.3, 0.5, 0.6);
  nav_msgs::msg::Path path;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  EXPECT_EQ(core.command(path, pose).linear.x, 0);
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 2;
  path.poses.push_back(p);
  auto cmd = core.command(path, pose);
  EXPECT_GT(cmd.linear.x, 0);
  EXPECT_EQ(cmd.angular.z, 0);
  pose.position.x = 2;
  EXPECT_EQ(core.command(path, pose).linear.x, 0);
  pose.position.x = 3;
  cmd = core.command(path, pose);
  EXPECT_EQ(cmd.linear.x, 0);
  EXPECT_LE(std::abs(cmd.angular.z), 0.6);
  EXPECT_GT(std::abs(cmd.angular.z), 0);
}
TEST(Control, CurvatureTurnsTowardPathAndRespectsLimits) {
  robot::ControlCore core(0.6, 0.3, 0.5, 0.6);
  nav_msgs::msg::Path path;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 2;
  p.pose.position.y = 1;
  path.poses.push_back(p);
  auto cmd = core.command(path, pose);
  EXPECT_GT(cmd.angular.z, 0);
  EXPECT_LE(cmd.linear.x, 0.5);
  EXPECT_LE(std::abs(cmd.angular.z), 0.6);
}
