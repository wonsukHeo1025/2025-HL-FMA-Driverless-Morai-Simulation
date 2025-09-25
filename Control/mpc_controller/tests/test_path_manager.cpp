#include <gtest/gtest.h>

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mpc_controller/path_manager.hpp"

namespace mpc_controller::test {

static nav_msgs::Path BuildStraightPath(double length, double step)
{
  nav_msgs::Path path;
  const int points = static_cast<int>(length / step) + 1;
  for (int i = 0; i < points; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i) * step;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }
  return path;
}

static nav_msgs::Path BuildCircularPath(double radius, double step, int points)
{
  nav_msgs::Path path;
  for (int i = 0; i < points; ++i) {
    const double theta = step * static_cast<double>(i);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = radius * std::cos(theta);
    pose.pose.position.y = radius * std::sin(theta);
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta + M_PI_2);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }
  return path;
}

TEST(PathManagerTest, ClosestPointStraightLine)
{
  PathManager manager;
  manager.Update(BuildStraightPath(10.0, 1.0));
  VehicleState state;
  state.x = 4.6;
  state.y = 0.1;

  const int closest = manager.ClosestIndex(state);
  EXPECT_EQ(closest, 5);
}

TEST(PathManagerTest, CurvatureMetric)
{
  PathManager manager;
  manager.Update(BuildCircularPath(5.0, 0.05, 200));

  const double max_abs = manager.CurvatureMetric(0, 99, "max_abs");
  const double rms = manager.CurvatureMetric(0, 99, "rms");

  EXPECT_GT(max_abs, 0.0);
  EXPECT_GT(rms, 0.0);
  EXPECT_GE(max_abs, rms);
}

}  // namespace mpc_controller::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
