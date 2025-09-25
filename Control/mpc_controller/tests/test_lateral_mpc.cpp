#include <gtest/gtest.h>

#include <cmath>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mpc_controller/lateral_mpc.hpp"
#include "mpc_controller/path_manager.hpp"

namespace mpc_controller::test {

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

TEST(LateralMpcTest, DynamicHorizonRespondsToCurvature)
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/lateral_mpc_test");

  private_nh.setParam("lateral_mpc/sample_time", 0.02);
  private_nh.setParam("lateral_mpc/prediction_horizon", 60);
  private_nh.setParam("lateral_mpc/control_horizon", 10);
  private_nh.setParam("lateral_mpc/wheelbase", 3.0);
  private_nh.setParam("lateral_mpc/r_weight", 1.0);
  private_nh.setParam("lateral_mpc/r_delta_weight", 1.0);
  private_nh.setParam("lateral_mpc/delta_min", -0.5);
  private_nh.setParam("lateral_mpc/delta_max", 0.5);
  private_nh.setParam("lateral_mpc/delta_rate_limit", 0.1);
  private_nh.setParam("lateral_mpc/dynamic_horizon/enable", true);
  private_nh.setParam("lateral_mpc/dynamic_horizon/min_steps", 20);
  private_nh.setParam("lateral_mpc/dynamic_horizon/max_steps", 60);
  private_nh.setParam("lateral_mpc/dynamic_horizon/quantize_steps", 5);
  private_nh.setParam("lateral_mpc/dynamic_horizon/change_threshold", 1);
  private_nh.setParam("lateral_mpc/dynamic_horizon/cooldown_sec", 0.0);
  private_nh.setParam("lateral_mpc/dynamic_horizon/roi_steps", 30);
  private_nh.setParam("lateral_mpc/dynamic_horizon/kappa_agg", "rms");
  private_nh.setParam("lateral_mpc/dynamic_horizon/kappa_low", 0.001);
  private_nh.setParam("lateral_mpc/dynamic_horizon/kappa_high", 0.1);

  LateralMpc mpc;
  ASSERT_TRUE(mpc.Configure(nh, private_nh));

  PathManager manager;
  manager.Update(BuildCircularPath(5.0, 0.05, 200));

  VehicleState state;
  state.x = 5.0;
  state.y = 0.0;
  state.heading = M_PI_2;
  state.vx = 10.0;

  LateralSolution tight_solution = mpc.Solve(state, manager, 0, nullptr);
  EXPECT_LE(mpc.active_prediction_horizon(), 40);
  EXPECT_NEAR(tight_solution.horizon_steps, mpc.active_prediction_horizon(), 1e-6);
  EXPECT_GT(tight_solution.roi_curvature_metric, 0.0);

  manager.Update(BuildStraightPath(200.0, 1.0));
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  LateralSolution straight_solution = mpc.Solve(state, manager, 0, nullptr);
  EXPECT_GE(mpc.active_prediction_horizon(), 55);
  EXPECT_NEAR(straight_solution.horizon_steps, mpc.active_prediction_horizon(), 1e-6);
  EXPECT_GE(straight_solution.roi_curvature_metric, 0.0);
}

}  // namespace mpc_controller::test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_controller_lateral_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
