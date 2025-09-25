#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <custom_interface/LateralMpcDebug.h>
#include <custom_interface/LateralMpcStatus.h>
#include <custom_interface/LateralMpcTiming.h>
#include <custom_interface/ControlInfo.h>
#include <tf2_ros/transform_listener.h>

#include "mpc_controller/debug_publisher.hpp"
#include "mpc_controller/lateral_mpc.hpp"
#include "mpc_controller/longitudinal_mpc.hpp"
#include "mpc_controller/path_manager.hpp"
#include "mpc_controller/speed_profile_buffer.hpp"
#include "mpc_controller/state_estimator.hpp"
#include "mpc_controller/types.hpp"

namespace mpc_controller {

class MpcControllerNode {
public:
  MpcControllerNode();

  bool Initialize();
  void Spin();

private:
  void PathCallback(const nav_msgs::Path::ConstPtr& msg);
  void SpeedProfileCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void EgoStatusCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg);
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  void ControlTimerCallback(const ros::TimerEvent& event);
  void StatusTimerCallback(const ros::TimerEvent& event);

  void PublishLateralDebug(const LateralSolution& solution,
                           double lateral_error,
                           double lateral_error_rate,
                           double heading_error,
                           double heading_error_rate,
                           double path_curvature,
                           double solver_time_ms);

  void PublishLongitudinalDebug(double current_velocity_mps,
                                double target_velocity_mps,
                                const LongitudinalSolution& solution);

  void ShadowCallback(const std_msgs::Bool::ConstPtr& msg);

  void PublishTimingOnly(const ros::TimerEvent& event,
                         const ros::Time& stamp,
                         const std::string& status,
                         double t_state_ms = 0.0);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber path_sub_;
  ros::Subscriber speed_profile_sub_;
  ros::Subscriber ego_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber shadow_sub_;

  ros::Publisher ctrl_cmd_pub_;
  ros::Publisher lateral_debug_pub_;
  ros::Publisher lateral_status_pub_;
  ros::Publisher lateral_timing_pub_;
  ros::Publisher lateral_metrics_pub_;
  ros::Publisher longitudinal_velocity_pub_;
  ros::Publisher longitudinal_target_pub_;
  ros::Publisher longitudinal_solver_status_pub_;
  ros::Publisher longitudinal_timing_pub_;

  ros::Timer control_timer_;
  ros::Timer status_timer_;

  PathManager path_manager_;
  SpeedProfileBuffer speed_profile_;
  StateEstimator state_estimator_;
  DebugPublisher debug_publisher_;

  LateralMpc lateral_mpc_;
  LongitudinalMpc longitudinal_mpc_;

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  VehicleState latest_state_;
  double previous_longitudinal_input_{0.0};
  bool initialized_{false};
  double control_rate_hz_{50.0};
  double path_resolution_m_{0.5};
  std::size_t upsample_factor_{10};

  std::string reference_frame_{"reference"};
  std::string base_frame_{"base"};
  std::string state_source_{"tf"};

  bool low_speed_freeze_enable_{true};
  double low_speed_freeze_kmph_{1.0};
  double low_speed_release_kmph_{2.0};
  double low_speed_publish_rate_{10.0};
  bool is_low_speed_hold_{false};
  ros::Time next_low_speed_pub_time_;

  std::string shadow_topic_{"/localization/gps_shadow"};
  double shadow_freeze_steering_{0.0};
  int shadow_end_idx_margin_{0};
  bool shadow_active_{false};
  bool shadow_freeze_active_{false};
  ros::Time shadow_last_change_;

  bool publish_debug_{true};
  bool path_available_{false};
  int consecutive_failures_{0};
  double last_lateral_error_{0.0};
  double last_heading_error_{0.0};
  ros::Time last_error_stamp_;
  double last_steering_command_{0.0};
  std::string last_solver_status_{"not_run"};
  std::string last_status_string_{"inactive"};

  std::vector<double> lateral_error_history_;
  std::vector<double> heading_error_history_;
  std::vector<double> steering_history_;
  std::vector<double> solver_time_history_;
};

}  // namespace mpc_controller
