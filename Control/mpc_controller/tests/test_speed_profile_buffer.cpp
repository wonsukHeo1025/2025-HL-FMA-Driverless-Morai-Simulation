#include <gtest/gtest.h>

#include "mpc_controller/speed_profile_buffer.hpp"

#include <limits>

namespace mpc_controller::test {

TEST(SpeedProfileBufferTest, UpsampleAndTrajectory)
{
  SpeedProfileBuffer buffer;
  buffer.Configure(0.5, 10);

  std::vector<double> profile = {0.0, 5.0, 10.0};
  buffer.Update(profile);

  EXPECT_FALSE(buffer.Empty());
  EXPECT_EQ(buffer.Size(), (profile.size() - 1) * 10 + 1);

  const double control_period = 0.02;  // 50 Hz
  const double velocity = 5.0;
  const std::size_t horizon = 5;
  auto trajectory = buffer.BuildTrajectory(0, velocity, control_period, horizon);

  ASSERT_EQ(trajectory.size(), horizon);
  EXPECT_GE(trajectory.front(), 0.0);
  EXPECT_LE(trajectory.front(), trajectory.back());
}

TEST(SpeedProfileBufferTest, DistanceToStop)
{
  SpeedProfileBuffer buffer;
  buffer.Configure(0.5, 10);

  std::vector<double> profile(200, 5.0);
  profile[50] = 0.0;
  buffer.Update(profile);

  const double distance = buffer.DistanceToNextStop(0, 200);
  EXPECT_LT(distance, std::numeric_limits<double>::infinity());
  EXPECT_NEAR(distance, 50 * 0.5, 1e-6);
}

TEST(SpeedProfileBufferTest, DistanceToStopRespectsUpsampleFactor)
{
  SpeedProfileBuffer buffer;
  buffer.Configure(0.5, 10);

  std::vector<double> profile(40, 4.0);
  profile[4] = 0.0;
  buffer.Update(profile);

  const double distance = buffer.DistanceToNextStop(0, 5);
  EXPECT_NEAR(distance, 4 * 0.5, 1e-6);
}

TEST(SpeedProfileBufferTest, DistanceToStopReturnsInfinityWhenNoStop)
{
  SpeedProfileBuffer buffer;
  buffer.Configure(0.5, 6);

  std::vector<double> profile(60, 3.0);
  buffer.Update(profile);

  const double distance = buffer.DistanceToNextStop(0, 20);
  EXPECT_EQ(distance, std::numeric_limits<double>::infinity());
}

TEST(SpeedProfileBufferTest, ValueClampsPastEnd)
{
  SpeedProfileBuffer buffer;
  buffer.Configure(1.0, 4);

  std::vector<double> profile = {2.0, 4.0, 6.0};
  buffer.Update(profile);

  EXPECT_DOUBLE_EQ(buffer.Value(0), 2.0);
  EXPECT_DOUBLE_EQ(buffer.Value(10), 6.0);
}

TEST(SpeedProfileBufferTest, BuildTrajectoryClampsToLastSample)
{
  SpeedProfileBuffer buffer;
  buffer.Configure(0.5, 5);

  std::vector<double> profile = {0.0, 5.0};
  buffer.Update(profile);

  const double control_period = 0.1;
  const double speed = 20.0;
  const std::size_t horizon = 6;
  auto trajectory = buffer.BuildTrajectory(0, speed, control_period, horizon);

  ASSERT_EQ(trajectory.size(), horizon);
  EXPECT_DOUBLE_EQ(trajectory.back(), 5.0);
}

TEST(SpeedProfileBufferTest, UpdateFromRosMessageMatchesVector)
{
  SpeedProfileBuffer buffer;
  buffer.Configure(0.4, 8);

  std::vector<double> profile = {1.0, 2.0, 3.0};
  buffer.Update(profile);
  const auto size_from_vector = buffer.Size();

  std_msgs::Float32MultiArray msg;
  msg.data.assign(profile.begin(), profile.end());
  buffer.Update(msg);

  EXPECT_EQ(buffer.Size(), size_from_vector);
  EXPECT_DOUBLE_EQ(buffer.Value(0), 1.0);
  EXPECT_DOUBLE_EQ(buffer.Value(buffer.Size() - 1), 3.0);
}

}  // namespace mpc_controller::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
