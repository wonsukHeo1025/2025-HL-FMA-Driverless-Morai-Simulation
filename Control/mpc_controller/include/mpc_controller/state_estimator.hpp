#pragma once

#include <deque>
#include <mutex>
#include <string>

#include <morai_msgs/EgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/buffer.h>

#include "mpc_controller/types.hpp"

namespace mpc_controller {

class StateEstimator {
public:
  explicit StateEstimator(std::size_t history_size = 10);

  bool UpdateFromTf(const tf2_ros::Buffer& buffer,
                    const std::string& reference_frame,
                    const std::string& base_frame);

  void UpdateFromEgoStatus(const morai_msgs::EgoVehicleStatus& msg);
  void UpdateFromImu(const sensor_msgs::Imu& msg);
  void UpdateFromOdometry(const nav_msgs::Odometry& msg);

  VehicleState CurrentState() const;
  bool IsValid() const;

private:
  void PushVelocity(double vx, double vy, double time_sec);

  VehicleState state_;
  VehicleState last_tf_state_;
  ros::Time last_tf_time_;
  bool valid_{false};
  bool have_last_tf_{false};
  std::deque<double> vx_history_;
  std::deque<double> vy_history_;
  std::deque<double> time_history_;
  std::size_t history_size_;
  mutable std::mutex mutex_;
};

}  // namespace mpc_controller

