#include "mpc_controller/state_estimator.hpp"

#include <algorithm>
#include <utility>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "mpc_controller/utils.hpp"

namespace mpc_controller {

StateEstimator::StateEstimator(std::size_t history_size)
  : history_size_(history_size)
{
}

bool StateEstimator::UpdateFromTf(const tf2_ros::Buffer& buffer,
                                  const std::string& reference_frame,
                                  const std::string& base_frame)
{
  geometry_msgs::TransformStamped transform;
  try {
    if (!buffer.canTransform(reference_frame, base_frame, ros::Time(0), ros::Duration(0.0))) {
      return false;
    }
    transform = buffer.lookupTransform(reference_frame, base_frame, ros::Time(0), ros::Duration(0.0));
  } catch (const tf2::TransformException& ex) {
    ROS_DEBUG_STREAM_THROTTLE(1.0, "StateEstimator TF lookup failed: " << ex.what());
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  VehicleState new_state = state_;
  new_state.x = transform.transform.translation.x;
  new_state.y = transform.transform.translation.y;
  new_state.z = transform.transform.translation.z;
  new_state.heading = tf2::getYaw(transform.transform.rotation);

  const ros::Time stamp = transform.header.stamp.isZero() ? ros::Time::now() : transform.header.stamp;
  if (have_last_tf_) {
    const double dt = (stamp - last_tf_time_).toSec();
    if (dt > 1e-3) {
      new_state.vx = (new_state.x - last_tf_state_.x) / dt;
      new_state.vy = (new_state.y - last_tf_state_.y) / dt;
      new_state.yaw_rate = NormalizeAngle(new_state.heading - last_tf_state_.heading) / dt;

      const double dvx = new_state.vx - last_tf_state_.vx;
      const double dvy = new_state.vy - last_tf_state_.vy;
      new_state.ax = dvx / dt;
      new_state.ay = dvy / dt;
      PushVelocity(new_state.vx, new_state.vy, stamp.toSec());
    }
  }

  state_ = new_state;
  last_tf_state_ = new_state;
  last_tf_time_ = stamp;
  valid_ = true;
  have_last_tf_ = true;
  return true;
}

void StateEstimator::UpdateFromEgoStatus(const morai_msgs::EgoVehicleStatus& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.vx = msg.velocity.x;
  state_.vy = msg.velocity.y;
  state_.vz = msg.velocity.z;
  state_.heading = msg.heading;
  state_.yaw_rate = state_.yaw_rate;  // not provided directly, maintain previous estimate
  state_.ax = msg.acceleration.x;
  state_.ay = msg.acceleration.y;
  valid_ = true;
}

void StateEstimator::UpdateFromImu(const sensor_msgs::Imu& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.yaw_rate = msg.angular_velocity.z;
  const double heading = state_.heading;
  const double cos_h = std::cos(heading);
  const double sin_h = std::sin(heading);
  const double ax_body = msg.linear_acceleration.x;
  const double ay_body = msg.linear_acceleration.y;
  state_.ax = cos_h * ax_body - sin_h * ay_body;
  state_.ay = sin_h * ax_body + cos_h * ay_body;
  valid_ = true;
}

void StateEstimator::UpdateFromOdometry(const nav_msgs::Odometry& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.x = msg.pose.pose.position.x;
  state_.y = msg.pose.pose.position.y;
  state_.z = msg.pose.pose.position.z;
  state_.heading = tf2::getYaw(msg.pose.pose.orientation);
  state_.vx = msg.twist.twist.linear.x;
  state_.vy = msg.twist.twist.linear.y;
  state_.vz = msg.twist.twist.linear.z;
  state_.yaw_rate = msg.twist.twist.angular.z;
  valid_ = true;
}

VehicleState StateEstimator::CurrentState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool StateEstimator::IsValid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return valid_;
}

void StateEstimator::PushVelocity(double vx, double vy, double time_sec)
{
  vx_history_.push_back(vx);
  vy_history_.push_back(vy);
  time_history_.push_back(time_sec);

  while (vx_history_.size() > history_size_) {
    vx_history_.pop_front();
    vy_history_.pop_front();
    time_history_.pop_front();
  }
}

}  // namespace mpc_controller
