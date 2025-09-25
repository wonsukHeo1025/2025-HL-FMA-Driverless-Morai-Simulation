#pragma once

#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "mpc_controller/path_manager.hpp"
#include "mpc_controller/types.hpp"

namespace mpc_controller {

class DebugPublisher {
public:
  DebugPublisher() = default;

  void Initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

  void PublishLateralInfo(const LateralSolution& solution,
                          const PathManager& path_manager,
                          int reference_index,
                          const VehicleState& state);

  void PublishLongitudinalInfo(const LongitudinalSolution& solution,
                               double current_velocity_mps);

private:
  ros::Publisher control_info_pub_;
  ros::Publisher predicted_path_pub_;
  ros::Publisher predicted_speed_pub_;
  ros::Publisher predicted_steer_pub_;
  ros::Publisher current_idx_marker_pub_;
  ros::Publisher current_idx_window_pub_;
  ros::Publisher reinit_disk_pub_;
  ros::Publisher predicted_velocity_pub_;

  bool publish_control_info_{true};
  std::string visualization_ns_{"/vis"};
  std::string reference_frame_{"reference"};
  double lateral_sample_time_{0.02};
  double wheelbase_{3.0};
  double speed_marker_base_xy_{0.2};
  double speed_marker_height_per_mps_{0.5};
  double steer_marker_scale_{1.0};
  std::string predicted_velocity_topic_{"/mpc_longitudinal/predicted_velocity"};
  std::vector<double> last_velocity_prediction_;
};

}  // namespace mpc_controller
