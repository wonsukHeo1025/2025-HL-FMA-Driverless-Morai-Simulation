#include "mpc_controller/controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <ros/ros.h>

#include "mpc_controller/utils.hpp"

namespace mpc_controller {

MpcControllerNode::MpcControllerNode()
  : nh_(), pnh_("~"), state_estimator_(10)
{
  next_low_speed_pub_time_ = ros::Time::now();
  shadow_last_change_ = ros::Time::now();
}

bool MpcControllerNode::Initialize()
{
  pnh_.param("control_rate", control_rate_hz_, control_rate_hz_);
  pnh_.param("publish_debug", publish_debug_, publish_debug_);
  pnh_.param("reference_frame", reference_frame_, reference_frame_);
  pnh_.param("ref_frame_id", reference_frame_, reference_frame_);
  pnh_.param("base_frame", base_frame_, base_frame_);
  pnh_.param("base_frame_id", base_frame_, base_frame_);
  pnh_.param("state_source", state_source_, state_source_);
  pnh_.param("low_speed/freeze_enable", low_speed_freeze_enable_, low_speed_freeze_enable_);
  pnh_.param("low_speed/freeze_below_kmph", low_speed_freeze_kmph_, low_speed_freeze_kmph_);
  pnh_.param("low_speed/release_above_kmph", low_speed_release_kmph_, low_speed_release_kmph_);
  pnh_.param("low_speed/publish_rate_hz", low_speed_publish_rate_, low_speed_publish_rate_);
  pnh_.param("shadow_topic", shadow_topic_, shadow_topic_);
  pnh_.param("shadow_freeze_steering", shadow_freeze_steering_, shadow_freeze_steering_);
  pnh_.param("shadow_end_idx_margin", shadow_end_idx_margin_, shadow_end_idx_margin_);
  pnh_.param("path_resolution_m", path_resolution_m_, path_resolution_m_);
  int upsample_factor_param = static_cast<int>(upsample_factor_);
  pnh_.param("profile_upsample_factor", upsample_factor_param, upsample_factor_param);
  upsample_factor_ = static_cast<std::size_t>(std::max(1, upsample_factor_param));

  std::string path_topic = "/planning/global/path";
  std::string speed_profile_topic = "/planning/speed_profile/global";
  std::string ego_topic = "/Competition_topic";
  std::string imu_topic = "/imu";
  std::string cmd_topic = "/ctrl_cmd";

  pnh_.param("path_topic", path_topic, path_topic);
  pnh_.param("speed_profile_topic", speed_profile_topic, speed_profile_topic);
  pnh_.param("ego_topic", ego_topic, ego_topic);
  pnh_.param("imu_topic", imu_topic, imu_topic);
  pnh_.param("cmd_topic", cmd_topic, cmd_topic);

  path_sub_ = nh_.subscribe(path_topic, 1, &MpcControllerNode::PathCallback, this);
  speed_profile_sub_ = nh_.subscribe(speed_profile_topic, 1, &MpcControllerNode::SpeedProfileCallback, this);
  ego_sub_ = nh_.subscribe(ego_topic, 10, &MpcControllerNode::EgoStatusCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic, 10, &MpcControllerNode::ImuCallback, this);
  if (!shadow_topic_.empty()) {
    shadow_sub_ = nh_.subscribe(shadow_topic_, 1, &MpcControllerNode::ShadowCallback, this);
  }

  ctrl_cmd_pub_ = nh_.advertise<morai_msgs::CtrlCmd>(cmd_topic, 1);
  lateral_debug_pub_ = nh_.advertise<custom_interface::LateralMpcDebug>("/lateral_mpc/debug", 10);
  lateral_status_pub_ = nh_.advertise<custom_interface::LateralMpcStatus>("/lateral_mpc/status", 10);
  lateral_timing_pub_ = nh_.advertise<custom_interface::LateralMpcTiming>("/lateral_mpc/status/verbose", 10);
  lateral_metrics_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/lateral_mpc/metrics", 10);

  std::string monitoring_ns = "/mpc_longitudinal";
  pnh_.param("monitoring_namespace", monitoring_ns, monitoring_ns);
  if (monitoring_ns.empty()) {
    monitoring_ns = "/mpc_longitudinal";
  } else {
    if (monitoring_ns.front() != '/') {
      monitoring_ns.insert(monitoring_ns.begin(), '/');
    }
    while (monitoring_ns.size() > 1 && monitoring_ns.back() == '/') {
      monitoring_ns.pop_back();
    }
  }

  longitudinal_velocity_pub_ = nh_.advertise<std_msgs::Float64>(monitoring_ns + "/current_velocity_kmph", 1);
  longitudinal_target_pub_ = nh_.advertise<std_msgs::Float64>(monitoring_ns + "/target_velocity_kmph", 1);
  longitudinal_solver_status_pub_ = nh_.advertise<std_msgs::String>(monitoring_ns + "/solver_status", 1);
  longitudinal_timing_pub_ = nh_.advertise<custom_interface::LateralMpcTiming>(monitoring_ns + "/status/verbose", 1);

  speed_profile_.Configure(path_resolution_m_, upsample_factor_);

  if (!lateral_mpc_.Configure(nh_, pnh_)) {
    ROS_ERROR("Failed to configure lateral MPC");
    return false;
  }
  if (!longitudinal_mpc_.Configure(nh_, pnh_)) {
    ROS_ERROR("Failed to configure longitudinal MPC");
    return false;
  }

  if (state_source_ == "tf") {
    tf_buffer_.setUsingDedicatedThread(true);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  }

  debug_publisher_.Initialize(nh_, pnh_);

  const double period = control_rate_hz_ > 0.0 ? 1.0 / control_rate_hz_ : 0.02;
  control_timer_ = nh_.createTimer(ros::Duration(period), &MpcControllerNode::ControlTimerCallback, this);
  status_timer_ = nh_.createTimer(ros::Duration(1.0), &MpcControllerNode::StatusTimerCallback, this);

  initialized_ = true;
  ROS_INFO_STREAM("mpc_controller initialized at " << control_rate_hz_ << " Hz");
  return true;
}

void MpcControllerNode::Spin()
{
  ros::spin();
}

void MpcControllerNode::PathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  if (!msg) {
    return;
  }
  path_manager_.Update(*msg);
}

void MpcControllerNode::SpeedProfileCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if (!msg) {
    return;
  }
  speed_profile_.Update(*msg);
}

void MpcControllerNode::EgoStatusCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
{
  if (!msg) {
    return;
  }
  state_estimator_.UpdateFromEgoStatus(*msg);
}

void MpcControllerNode::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (!msg) {
    return;
  }
  state_estimator_.UpdateFromImu(*msg);
}

void MpcControllerNode::ControlTimerCallback(const ros::TimerEvent& event)
{
  if (!initialized_) {
    return;
  }

  const ros::WallTime loop_start = ros::WallTime::now();
  const ros::Time now = ros::Time::now();
  double t_state_ms = 0.0;

  if (state_source_ == "tf") {
    const ros::WallTime tf_start = ros::WallTime::now();
    const bool state_ok = state_estimator_.UpdateFromTf(tf_buffer_, reference_frame_, base_frame_);
    t_state_ms = (ros::WallTime::now() - tf_start).toSec() * 1000.0;
    if (!state_ok) {
      PublishTimingOnly(event, now, "no_tf", t_state_ms);
      return;
    }
  }

  latest_state_ = state_estimator_.CurrentState();
  path_available_ = path_manager_.HasPath();
  if (!path_available_) {
    PublishTimingOnly(event, now, "inactive", t_state_ms);
    return;
  }

  if (shadow_active_ && shadow_freeze_active_) {
    morai_msgs::CtrlCmd shadow_cmd;
    shadow_cmd.longlCmdType = 1;
    shadow_cmd.steering = shadow_freeze_steering_;
    shadow_cmd.accel = 0.0;
    shadow_cmd.brake = 0.0;
    ctrl_cmd_pub_.publish(shadow_cmd);
    PublishTimingOnly(event, now, "gps_shadow", t_state_ms);
    return;
  }

  const double current_speed_kmph = std::abs(latest_state_.vx) * 3.6;
  if (low_speed_freeze_enable_) {
    if (is_low_speed_hold_) {
      if (current_speed_kmph >= low_speed_release_kmph_) {
        is_low_speed_hold_ = false;
      } else {
        if (now >= next_low_speed_pub_time_) {
          morai_msgs::CtrlCmd hold_cmd;
          hold_cmd.longlCmdType = 1;
          hold_cmd.steering = last_steering_command_;
          hold_cmd.accel = 0.0;
          hold_cmd.brake = 0.0;
          ctrl_cmd_pub_.publish(hold_cmd);
          next_low_speed_pub_time_ = now + ros::Duration(1.0 / std::max(1e-3, low_speed_publish_rate_));
        }
        PublishTimingOnly(event, now, "low_speed_hold", t_state_ms);
        return;
      }
    }

    if (!is_low_speed_hold_ && current_speed_kmph <= low_speed_freeze_kmph_) {
      is_low_speed_hold_ = true;
      next_low_speed_pub_time_ = now;
      morai_msgs::CtrlCmd hold_cmd;
      hold_cmd.longlCmdType = 1;
      hold_cmd.steering = last_steering_command_;
      hold_cmd.accel = 0.0;
      hold_cmd.brake = 0.0;
      ctrl_cmd_pub_.publish(hold_cmd);
      PublishTimingOnly(event, now, "low_speed_hold", t_state_ms);
      return;
    }
  }

  const int reference_index = path_manager_.ClosestIndex(latest_state_);

  const ros::WallTime lateral_start = ros::WallTime::now();
  LateralSolution lateral_solution = lateral_mpc_.Solve(latest_state_,
                                                        path_manager_,
                                                        reference_index,
                                                        publish_debug_ ? &speed_profile_ : nullptr);
  const double lateral_solve_ms = (ros::WallTime::now() - lateral_start).toSec() * 1000.0;

  std::vector<double> reference_velocity;
  double target_velocity = 0.0;
  if (!speed_profile_.Empty()) {
    reference_velocity = speed_profile_.BuildTrajectory(static_cast<std::size_t>(std::max(0, reference_index)),
                                                        latest_state_.vx,
                                                        1.0 / control_rate_hz_,
                                                        static_cast<std::size_t>(longitudinal_mpc_.config().prediction_horizon));
    if (!reference_velocity.empty()) {
      target_velocity = reference_velocity.front();
    }
  }

  const ros::WallTime longitudinal_start = ros::WallTime::now();
  LongitudinalSolution longitudinal_solution = longitudinal_mpc_.Solve(latest_state_.vx,
                                                                       previous_longitudinal_input_,
                                                                       reference_velocity);
  const double longitudinal_solve_ms = (ros::WallTime::now() - longitudinal_start).toSec() * 1000.0;

  previous_longitudinal_input_ = longitudinal_solution.command.accel - longitudinal_solution.command.brake;

  morai_msgs::CtrlCmd cmd;
  cmd.longlCmdType = 1;
  cmd.steering = lateral_solution.command.steering;
  cmd.accel = longitudinal_solution.command.accel;
  cmd.brake = longitudinal_solution.command.brake;
  ctrl_cmd_pub_.publish(cmd);

  // Compute tracking errors for diagnostics.
  double lateral_error = 0.0;
  double heading_error = 0.0;
  double path_curvature = 0.0;
  if (path_available_ && reference_index >= 0) {
    const auto reference_point = path_manager_.Point(static_cast<std::size_t>(reference_index));
    const double dx = latest_state_.x - reference_point.x;
    const double dy = latest_state_.y - reference_point.y;
    const double sin_h = std::sin(reference_point.heading);
    const double cos_h = std::cos(reference_point.heading);
    lateral_error = -sin_h * dx + cos_h * dy;
    heading_error = NormalizeAngle(latest_state_.heading - reference_point.heading);
    path_curvature = reference_point.curvature;
  }

  double lateral_error_rate = 0.0;
  double heading_error_rate = 0.0;
  if (!last_error_stamp_.isZero()) {
    const double dt = (now - last_error_stamp_).toSec();
    if (dt > 1e-3) {
      lateral_error_rate = (lateral_error - last_lateral_error_) / dt;
      heading_error_rate = (heading_error - last_heading_error_) / dt;
    }
  }
  last_error_stamp_ = now;
  last_lateral_error_ = lateral_error;
  last_heading_error_ = heading_error;

  std::string solver_status = lateral_solution.report.status;
  if (solver_status == "solved") {
    solver_status = "optimal";
  }
  if (solver_status == "solved_inaccurate") {
    solver_status = "suboptimal";
  }
  if (lateral_solution.report.success) {
    solver_status = "optimal";
    consecutive_failures_ = 0;
  } else {
    if (solver_status.empty()) {
      solver_status = "failed";
    }
    consecutive_failures_ = std::min(consecutive_failures_ + 1, 1000000);
  }
  last_solver_status_ = solver_status;
  last_steering_command_ = lateral_solution.command.steering;

  const auto buffer_limit = static_cast<std::size_t>(std::max(1.0, control_rate_hz_));
  auto push_with_limit = [buffer_limit](std::vector<double>& buf, double value) {
    buf.push_back(value);
    if (buf.size() > buffer_limit) {
      buf.erase(buf.begin());
    }
  };
  push_with_limit(lateral_error_history_, lateral_error);
  push_with_limit(heading_error_history_, heading_error);
  push_with_limit(steering_history_, lateral_solution.command.steering);
  push_with_limit(solver_time_history_, lateral_solve_ms);

  // Always update longitudinal prediction topic.
  debug_publisher_.PublishLongitudinalInfo(longitudinal_solution, latest_state_.vx);

  PublishLongitudinalDebug(latest_state_.vx,
                           target_velocity,
                           longitudinal_solution);

  if (publish_debug_) {
    PublishLateralDebug(lateral_solution,
                        lateral_error,
                        lateral_error_rate,
                        heading_error,
                        heading_error_rate,
                        path_curvature,
                        lateral_solve_ms);
  }
  debug_publisher_.PublishLateralInfo(lateral_solution, path_manager_, reference_index, latest_state_);

  const ros::WallTime loop_end = ros::WallTime::now();
  custom_interface::LateralMpcTiming timing_msg;
  timing_msg.header.stamp = now;
  double loop_period_ms = (event.current_real - event.last_real).toSec() * 1000.0;
  if (!std::isfinite(loop_period_ms) || loop_period_ms <= 0.0) {
    loop_period_ms = (loop_end - loop_start).toSec() * 1000.0;
  }
  timing_msg.loop_period_ms = loop_period_ms;
  timing_msg.loop_frequency_hz = loop_period_ms > 1e-6 ? 1000.0 / loop_period_ms : 0.0;
  timing_msg.t_state_estimation_ms = t_state_ms;
  timing_msg.t_error_computation_ms = 0.0;
  timing_msg.t_reference_build_ms = 0.0;
  timing_msg.t_mpc_setup_ms = 0.0;
  timing_msg.t_mpc_solve_ms = lateral_solve_ms;
  const double publish_ms = std::max(0.0, (loop_end - loop_start).toSec() * 1000.0 - lateral_solve_ms - longitudinal_solve_ms);
  timing_msg.t_publish_ms = publish_ms;
  timing_msg.solver_status = solver_status;
  timing_msg.solver_cost = lateral_solution.report.cost;
  timing_msg.solver_iterations = lateral_solution.report.iterations;
  lateral_timing_pub_.publish(timing_msg);

  custom_interface::LateralMpcTiming longitudinal_timing = timing_msg;
  longitudinal_timing.t_mpc_solve_ms = longitudinal_solve_ms;
  longitudinal_timing.solver_status = longitudinal_solution.report.status;
  longitudinal_timing.solver_cost = longitudinal_solution.report.cost;
  longitudinal_timing.solver_iterations = longitudinal_solution.report.iterations;
  longitudinal_timing_pub_.publish(longitudinal_timing);

  last_status_string_ = solver_status;
}

void MpcControllerNode::PublishLateralDebug(const LateralSolution& solution,
                                            double lateral_error,
                                            double lateral_error_rate,
                                            double heading_error,
                                            double heading_error_rate,
                                            double path_curvature,
                                            double solver_time_ms)
{
  if (!lateral_debug_pub_) {
    return;
  }

  custom_interface::LateralMpcDebug msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = base_frame_;
  msg.lateral_error = lateral_error;
  msg.lateral_error_rate = lateral_error_rate;
  msg.heading_error = heading_error;
  msg.heading_error_rate = heading_error_rate;
  msg.steering_command = solution.command.steering;
  double steering_rate = 0.0;
  if (steering_history_.size() >= 2) {
    steering_rate = (steering_history_.back() - *(steering_history_.end() - 2)) * control_rate_hz_;
  }
  msg.steering_rate = steering_rate;
  msg.feedforward_steering = 0.0;
  msg.feedback_steering = solution.command.steering;
  msg.cost_value = solution.report.cost;
  msg.solver_time = solver_time_ms;
  msg.solver_status = solution.report.status;
  msg.solver_iterations = solution.report.iterations;
  msg.predicted_steering.assign(solution.steering_sequence.begin(), solution.steering_sequence.end());
  msg.predicted_lateral_errors.assign(solution.predicted_lateral_errors.begin(),
                                      solution.predicted_lateral_errors.end());
  msg.predicted_heading_errors.assign(solution.predicted_heading_errors.begin(),
                                      solution.predicted_heading_errors.end());
  msg.path_curvature = path_curvature;
  msg.lookahead_curvature = path_curvature;
  msg.preview_distance = lateral_mpc_.config().preview_distance;
  msg.current_speed = latest_state_.vx;
  msg.control_mode = "kinematic";
  msg.understeer_gradient = 0.0;

  lateral_debug_pub_.publish(msg);
}

void MpcControllerNode::PublishLongitudinalDebug(double current_velocity_mps,
                                                 double target_velocity_mps,
                                                 const LongitudinalSolution& solution)
{
  const double current_kmph = current_velocity_mps * 3.6;
  const double target_kmph = target_velocity_mps * 3.6;

  if (longitudinal_velocity_pub_) {
    std_msgs::Float64 msg;
    msg.data = current_kmph;
    longitudinal_velocity_pub_.publish(msg);
  }

  if (longitudinal_target_pub_) {
    std_msgs::Float64 msg;
    msg.data = target_kmph;
    longitudinal_target_pub_.publish(msg);
  }

  if (longitudinal_solver_status_pub_) {
    std_msgs::String msg;
    msg.data = solution.report.status;
    longitudinal_solver_status_pub_.publish(msg);
  }
}

void MpcControllerNode::ShadowCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg) {
    return;
  }
  const bool new_state = msg->data;
  if (new_state != shadow_active_) {
    shadow_active_ = new_state;
    shadow_last_change_ = ros::Time::now();
    if (!shadow_active_) {
      shadow_freeze_active_ = false;
    } else {
      shadow_freeze_active_ = std::abs(shadow_freeze_steering_) > 1e-6;
    }
  } else {
    shadow_active_ = new_state;
  }
}

void MpcControllerNode::PublishTimingOnly(const ros::TimerEvent& event,
                                          const ros::Time& stamp,
                                          const std::string& status,
                                          double t_state_ms)
{
  double loop_period_ms = (event.current_real - event.last_real).toSec() * 1000.0;
  if (!std::isfinite(loop_period_ms) || loop_period_ms <= 0.0) {
    loop_period_ms = 1000.0 / std::max(1e-3, control_rate_hz_);
  }

  custom_interface::LateralMpcTiming timing_msg;
  timing_msg.header.stamp = stamp;
  timing_msg.loop_period_ms = loop_period_ms;
  timing_msg.loop_frequency_hz = loop_period_ms > 1e-6 ? 1000.0 / loop_period_ms : 0.0;
  timing_msg.t_state_estimation_ms = t_state_ms;
  timing_msg.t_error_computation_ms = 0.0;
  timing_msg.t_reference_build_ms = 0.0;
  timing_msg.t_mpc_setup_ms = 0.0;
  timing_msg.t_mpc_solve_ms = 0.0;
  timing_msg.t_publish_ms = 0.0;
  timing_msg.solver_status = status;
  timing_msg.solver_cost = 0.0;
  timing_msg.solver_iterations = 0;
  lateral_timing_pub_.publish(timing_msg);

  custom_interface::LateralMpcTiming longitudinal = timing_msg;
  longitudinal_timing_pub_.publish(longitudinal);
  last_status_string_ = status;
}

void MpcControllerNode::StatusTimerCallback(const ros::TimerEvent& /*event*/)
{
  if (!initialized_) {
    return;
  }

  custom_interface::LateralMpcStatus status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.is_active = path_available_;
  status_msg.is_initialized = initialized_;
  status_msg.control_mode = "kinematic";

  if (!lateral_error_history_.empty()) {
    double sum_sq = 0.0;
    double max_abs = 0.0;
    for (double e : lateral_error_history_) {
      sum_sq += e * e;
      max_abs = std::max(max_abs, std::abs(e));
    }
    status_msg.lateral_rmse = std::sqrt(sum_sq / lateral_error_history_.size());
    status_msg.max_lateral_error = max_abs;
  }

  if (!heading_error_history_.empty()) {
    double sum_sq = 0.0;
    for (double e : heading_error_history_) {
      sum_sq += e * e;
    }
    status_msg.heading_rmse = std::sqrt(sum_sq / heading_error_history_.size());
  }

  if (!solver_time_history_.empty()) {
    const double avg = std::accumulate(solver_time_history_.begin(), solver_time_history_.end(), 0.0) /
                       static_cast<double>(solver_time_history_.size());
    status_msg.avg_solver_time = avg;
  }

  status_msg.path_available = path_available_;
  const int closest_index = path_manager_.ClosestIndex(latest_state_);
  status_msg.closest_waypoint_idx = closest_index;
  const std::size_t path_size = path_manager_.Size();
  if (path_size > 1 && closest_index >= 0) {
    const int remaining = std::max(0, static_cast<int>(path_size) - 1 - closest_index);
    status_msg.distance_to_goal = remaining * path_resolution_m_;
    status_msg.path_completion = 100.0 * static_cast<double>(closest_index) /
                                 static_cast<double>(path_size - 1);
  } else {
    status_msg.distance_to_goal = 0.0;
    status_msg.path_completion = 0.0;
  }

  status_msg.solver_healthy = consecutive_failures_ < 3;
  status_msg.state_estimation_ok = state_estimator_.IsValid();
  status_msg.consecutive_failures = consecutive_failures_;
  status_msg.error_message = last_status_string_;
  if (!status_msg.solver_healthy) {
    status_msg.error_message = last_solver_status_;
  }

  if (lateral_status_pub_) {
    lateral_status_pub_.publish(status_msg);
  }

  if (!lateral_error_history_.empty() && lateral_metrics_pub_) {
    std_msgs::Float32MultiArray metrics;
    const double lat_rmse = status_msg.lateral_rmse;
    const double max_err = status_msg.max_lateral_error;
    double heading_rmse = 0.0;
    if (!heading_error_history_.empty()) {
      double sum_sq = 0.0;
      for (double e : heading_error_history_) {
        sum_sq += e * e;
      }
      heading_rmse = std::sqrt(sum_sq / heading_error_history_.size());
    }
    double steering_smoothness = 0.0;
    if (steering_history_.size() >= 2) {
      std::vector<double> diffs;
      diffs.reserve(steering_history_.size() - 1);
      for (std::size_t i = 1; i < steering_history_.size(); ++i) {
        diffs.push_back(steering_history_[i] - steering_history_[i - 1]);
      }
      double mean = std::accumulate(diffs.begin(), diffs.end(), 0.0) / diffs.size();
      double var = 0.0;
      for (double d : diffs) {
        const double delta = d - mean;
        var += delta * delta;
      }
      var /= diffs.size();
      steering_smoothness = std::sqrt(var);
    }
    metrics.data = {
      static_cast<float>(lat_rmse),
      static_cast<float>(max_err),
      static_cast<float>(heading_rmse),
      static_cast<float>(steering_smoothness)
    };
    lateral_metrics_pub_.publish(metrics);
  }
}

}  // namespace mpc_controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_controller");
  mpc_controller::MpcControllerNode node;
  if (!node.Initialize()) {
    ROS_ERROR("Failed to initialize mpc_controller node");
    return 1;
  }
  node.Spin();
  return 0;
}
