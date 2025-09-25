#include "mpc_controller/debug_publisher.hpp"

#include <algorithm>
#include <cmath>

#include <custom_interface/ControlInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mpc_controller/path_manager.hpp"

namespace mpc_controller {

void DebugPublisher::Initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
  private_nh.param("publish_control_info", publish_control_info_, true);
  if (!private_nh.getParam("vis_namespace", visualization_ns_)) {
    private_nh.param("visualization_namespace", visualization_ns_, visualization_ns_);
  }
  private_nh.param("reference_frame", reference_frame_, reference_frame_);
  private_nh.param("lateral_mpc/sample_time", lateral_sample_time_, lateral_sample_time_);
  private_nh.param("lateral_mpc/wheelbase", wheelbase_, wheelbase_);
  private_nh.param("lateral_mpc/vis/speed_pillar_base_xy", speed_marker_base_xy_, speed_marker_base_xy_);
  private_nh.param("lateral_mpc/vis/speed_pillar_height_per_mps", speed_marker_height_per_mps_, speed_marker_height_per_mps_);
  private_nh.param("lateral_mpc/vis/steer_scale", steer_marker_scale_, steer_marker_scale_);
  private_nh.param("predicted_velocity_topic", predicted_velocity_topic_, predicted_velocity_topic_);

  if (publish_control_info_) {
    control_info_pub_ = nh.advertise<custom_interface::ControlInfo>("/lateral_mpc/control_info", 10);
  }

  std::string vis_ns = visualization_ns_;
  if (!vis_ns.empty() && vis_ns.back() == '/') {
    vis_ns.pop_back();
  }
  if (vis_ns.empty()) {
    vis_ns = "/vis";
  }
  const std::string base_topic = vis_ns + "/control/lateral_mpc";
  predicted_path_pub_ = nh.advertise<nav_msgs::Path>(base_topic + "/predicted_trajectory", 1);
  predicted_speed_pub_ = nh.advertise<visualization_msgs::MarkerArray>(base_topic + "/predicted_speed_pillars", 1);
  predicted_steer_pub_ = nh.advertise<visualization_msgs::MarkerArray>(base_topic + "/predicted_steer_pillars", 1);
  current_idx_marker_pub_ = nh.advertise<visualization_msgs::Marker>(base_topic + "/current_idx_marker", 1);
  current_idx_window_pub_ = nh.advertise<visualization_msgs::MarkerArray>(base_topic + "/current_idx_window", 1);
  reinit_disk_pub_ = nh.advertise<visualization_msgs::Marker>(base_topic + "/reinit_disk", 1);
  predicted_velocity_pub_ = nh.advertise<std_msgs::Float32MultiArray>(predicted_velocity_topic_, 1);
}

void DebugPublisher::PublishLateralInfo(const LateralSolution& solution,
                                        const PathManager& path_manager,
                                        int reference_index,
                                        const VehicleState& state)
{
  if (publish_control_info_ && control_info_pub_) {
    custom_interface::ControlInfo msg;
    msg.steering = solution.command.steering;
    msg.current_idx = reference_index;
    msg.horizon_steps = solution.horizon_steps;
    msg.roi_curvature = solution.roi_curvature_metric;
    msg.roi_curvature_mode = solution.roi_curvature_mode;
    control_info_pub_.publish(msg);
  }

  const ros::Time stamp = ros::Time::now();

  const std::vector<double>& steering_seq = solution.steering_sequence;
  const std::size_t steps = static_cast<std::size_t>(std::max(1, solution.horizon_steps));

  std::vector<double> velocity_seq = last_velocity_prediction_;
  if (velocity_seq.empty()) {
    velocity_seq.assign(steps + 1, state.vx);
  } else if (velocity_seq.size() < steps + 1) {
    velocity_seq.resize(steps + 1, velocity_seq.back());
  }

  nav_msgs::Path path_msg;
  path_msg.header.stamp = stamp;
  path_msg.header.frame_id = reference_frame_;
  path_msg.poses.reserve(steps + 1);

  std::vector<std::pair<double, double>> positions;
  std::vector<double> headings;
  positions.reserve(steps + 1);
  headings.reserve(steps + 1);

  double x = state.x;
  double y = state.y;
  double yaw = state.heading;
  positions.emplace_back(x, y);
  headings.push_back(yaw);

  for (std::size_t i = 0; i < steps; ++i) {
    const double delta = !steering_seq.empty()
                             ? steering_seq[std::min(i, steering_seq.size() - 1)]
                             : solution.command.steering;
    const double v = velocity_seq[std::min(i, velocity_seq.size() - 1)];
    const double yaw_rate = (v / std::max(1e-3, wheelbase_)) * std::tan(delta);
    yaw += yaw_rate * lateral_sample_time_;
    x += v * std::cos(yaw) * lateral_sample_time_;
    y += v * std::sin(yaw) * lateral_sample_time_;
    positions.emplace_back(x, y);
    headings.push_back(yaw);
  }

  for (std::size_t i = 0; i < positions.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = positions[i].first;
    pose.pose.position.y = positions[i].second;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, headings[i]);
    pose.pose.orientation = tf2::toMsg(q);
    path_msg.poses.push_back(pose);
  }
  if (predicted_path_pub_) {
    predicted_path_pub_.publish(path_msg);
  }

  if (current_idx_marker_pub_ && path_manager.HasPath()) {
    const std::size_t path_size = path_manager.Size();
    const std::size_t clamped_index = path_size > 0
                                        ? static_cast<std::size_t>(std::clamp(reference_index, 0,
                                                                               static_cast<int>(path_size - 1)))
                                        : 0;
    PathPoint point = path_manager.Point(clamped_index);
    visualization_msgs::Marker marker;
    marker.header = path_msg.header;
    marker.ns = "current_idx";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z + 0.1;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.55;
    marker.color.b = 0.0;
    marker.color.a = 0.55;
    current_idx_marker_pub_.publish(marker);
  }

  if (current_idx_window_pub_) {
    visualization_msgs::MarkerArray window_msg;
    visualization_msgs::Marker clear;
    clear.header = path_msg.header;
    clear.ns = "current_idx_window";
    clear.id = 0;
    clear.action = visualization_msgs::Marker::DELETEALL;
    window_msg.markers.push_back(clear);
    current_idx_window_pub_.publish(window_msg);
  }

  if (reinit_disk_pub_) {
    visualization_msgs::Marker marker;
    marker.header = path_msg.header;
    marker.ns = "reinit_disk";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::DELETE;
    reinit_disk_pub_.publish(marker);
  }

  if (predicted_speed_pub_) {
    visualization_msgs::MarkerArray speed_markers;
    visualization_msgs::Marker clear;
    clear.header = path_msg.header;
    clear.ns = "predicted_speed_pillars";
    clear.id = 0;
    clear.action = visualization_msgs::Marker::DELETEALL;
    speed_markers.markers.push_back(clear);

    for (std::size_t i = 0; i < positions.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header = path_msg.header;
      marker.ns = "predicted_speed_pillars";
      marker.id = static_cast<int>(i + 1);
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = positions[i].first;
      marker.pose.position.y = positions[i].second;
      const double height = std::max(0.05, velocity_seq[std::min(i, velocity_seq.size() - 1)] * speed_marker_height_per_mps_);
      marker.scale.x = speed_marker_base_xy_;
      marker.scale.y = speed_marker_base_xy_;
      marker.scale.z = height;
      marker.pose.position.z = 0.5 * height;
      marker.pose.orientation.w = 1.0;
      marker.color.r = 0.1;
      marker.color.g = 0.6;
      marker.color.b = 1.0;
      marker.color.a = 0.6;
      speed_markers.markers.push_back(marker);
    }

    predicted_speed_pub_.publish(speed_markers);
  }

  if (predicted_steer_pub_) {
    visualization_msgs::MarkerArray steer_markers;
    visualization_msgs::Marker clear;
    clear.header = path_msg.header;
    clear.ns = "predicted_steer_pillars";
    clear.id = 0;
    clear.action = visualization_msgs::Marker::DELETEALL;
    steer_markers.markers.push_back(clear);

    for (std::size_t i = 0; i < steps; ++i) {
      const double seq_delta = !steering_seq.empty()
                                   ? steering_seq[std::min(i, steering_seq.size() - 1)]
                                   : solution.command.steering;
      visualization_msgs::Marker marker;
      marker.header = path_msg.header;
      marker.ns = "predicted_steer_pillars";
      marker.id = static_cast<int>(i + 1);
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      const auto& pos = positions[std::min(i, positions.size() - 1)];
      marker.pose.position.x = pos.first;
      marker.pose.position.y = pos.second;
      const double height = std::max(0.05, std::abs(seq_delta) * steer_marker_scale_);
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = height;
      marker.pose.position.z = 0.5 * height;
      marker.pose.orientation.w = 1.0;
      if (seq_delta >= 0.0) {
        marker.color.r = 0.1;
        marker.color.g = 1.0;
        marker.color.b = 0.3;
      } else {
        marker.color.r = 1.0;
        marker.color.g = 0.3;
        marker.color.b = 0.1;
      }
      marker.color.a = 0.6;
      steer_markers.markers.push_back(marker);
    }

    predicted_steer_pub_.publish(steer_markers);
  }
}

void DebugPublisher::PublishLongitudinalInfo(const LongitudinalSolution& solution,
                                             double current_velocity_mps)
{
  last_velocity_prediction_ = solution.velocity_prediction;
  if (last_velocity_prediction_.empty()) {
    last_velocity_prediction_.push_back(current_velocity_mps);
  }

  if (predicted_velocity_pub_) {
    std_msgs::Float32MultiArray msg;
    msg.data.reserve(last_velocity_prediction_.size());
    for (double v : last_velocity_prediction_) {
      msg.data.push_back(static_cast<float>(v));
    }
    predicted_velocity_pub_.publish(msg);
  }
}

}  // namespace mpc_controller
