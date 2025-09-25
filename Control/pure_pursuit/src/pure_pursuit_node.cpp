#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <morai_msgs/CtrlCmd.h>   // MORAI
#include <morai_msgs/EgoVehicleStatus.h>  // For Competition_topic
#include <custom_interface/ControlInfo.h>  // For lateral-only mode

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PurePursuitNode
{
public:
  PurePursuitNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_)
  {
    // ---- Parameters ----
    pnh_.param<std::string>("path_topic", path_topic_, std::string("/global_waypoints"));
    pnh_.param<std::string>("speed_profile_topic", speed_profile_topic_, std::string("/desired_speed_profile"));
    // pnh_.param<std::string>("current_speed_topic", current_speed_topic_, std::string("/current_speed"));
    pnh_.param<std::string>("ego_status_topic", ego_status_topic_, std::string("/Competition_topic"));

    pnh_.param<std::string>("ref_frame", ref_frame_, std::string("reference"));
    pnh_.param<std::string>("base_frame", base_frame_, std::string("base")); // 후륜축 중심

    pnh_.param<double>("wheelbase", wheelbase_, 3.0);
    pnh_.param<double>("control_rate_hz", ctrl_rate_hz_, 30.0);

    // Lookahead: Ld = clamp(k_v * v + L0, Ld_min, Ld_max)
    pnh_.param<double>("Ld_min", Ld_min_, 2.0);
    pnh_.param<double>("Ld_max", Ld_max_, 15.0);
    pnh_.param<double>("Ld0",    Ld0_,    3.0);
    pnh_.param<double>("k_v",    k_v_,    0.5);

    // 명령 상한
    pnh_.param<double>("max_speed_cmd_mps", max_speed_cmd_mps_, 11.0);
    pnh_.param<double>("max_steer_rad",     max_steer_rad_,      0.6);
    pnh_.param<double>("default_speed_mps", default_speed_mps_,  3.0);

    // MORAI 제어 타입
    pnh_.param<int>("longlCmdType", longlCmdType_, 2);  // 1: throttle/brake, 2: velocity(km/h), 3: acceleration(m/s^2)
    pnh_.param<std::string>("ctrl_topic", ctrl_topic_, std::string("/ctrl_cmd"));

    // longlCmdType=1 사용 시 간단 P제어
    pnh_.param<double>("long_kp",        long_kp_,        0.5);  // 속도 오차 → accel/brake [0..1]
    pnh_.param<double>("accel_max_cmd",  accel_max_cmd_,  2.0);
    pnh_.param<double>("brake_max_cmd",  brake_max_cmd_,  2.0);

    pnh_.param<bool>("publish_markers",  publish_markers_, true);
    pnh_.param<bool>("lateral_only_mode", lateral_only_mode_, false);

    // ---- ROS I/O ----
    path_sub_  = nh_.subscribe(path_topic_, 1, &PurePursuitNode::pathCb, this);
    sp_sub_    = nh_.subscribe(speed_profile_topic_, 1, &PurePursuitNode::speedProfileCb, this);
    // v_sub_     = nh_.subscribe(current_speed_topic_, 10, &PurePursuitNode::speedCb, this);
    ego_sub_   = nh_.subscribe(ego_status_topic_, 10, &PurePursuitNode::egoCb, this);

    ctrl_pub_   = nh_.advertise<morai_msgs::CtrlCmd>(ctrl_topic_, 10);
    speed_pub_  = nh_.advertise<std_msgs::Float64>("/cmd_speed", 10);
    steer_pub_  = nh_.advertise<std_msgs::Float64>("/cmd_steering", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/lookahead_marker", 1);
    control_info_pub_ = nh_.advertise<custom_interface::ControlInfo>("/pure_pursuit/control_info", 10);

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, ctrl_rate_hz_)),
                             &PurePursuitNode::onTimer, this);

    ROS_INFO_STREAM("[pure_pursuit] ref=" << ref_frame_ << ", base=" << base_frame_
                    << ", longlCmdType=" << longlCmdType_ << " (2=velocity km/h), ctrl_topic=" << ctrl_topic_
                    << ", lateral_only_mode=" << (lateral_only_mode_ ? "true" : "false")
                    << ", ego_status_topic=" << ego_status_topic_);
  }

private:
  // ---- Callbacks ----
  void pathCb(const nav_msgs::Path::ConstPtr& msg)
  {
    path_ = *msg;
    buildCumulativeS();
  }

  void speedProfileCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    speed_profile_.assign(msg->data.begin(), msg->data.end());
  }

  // OLD: From /current_speed
  // void speedCb(const std_msgs::Float64::ConstPtr& msg)
  // {
  //   current_speed_mps_ = std::max(0.0, msg->data);
  //   has_current_speed_ = true;
  // }
  
  // NEW: From /Competition_topic (MORAI ego status)
  void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
  {
    // velocity.x is in m/s from MORAI
    current_speed_mps_ = std::max(0.0, static_cast<double>(msg->velocity.x));
    has_current_speed_ = true;
  }

  // ---- Control loop ----
  void onTimer(const ros::TimerEvent&)
  {
    if (path_.poses.size() < 2) return;

    // 1) 현재 pose (reference->base)
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(ref_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "TF %s->%s unavailable: %s", ref_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }
    const double xb = tf.transform.translation.x;
    const double yb = tf.transform.translation.y;
    const double yaw = yawFromQuat(tf.transform.rotation);

    // 2) 최근접 인덱스
    const size_t nearest_idx = nearestIndex(xb, yb);
    if (nearest_idx >= path_.poses.size()) return;

    // 3) Lookahead 거리 계산
    double v_for_ld = has_current_speed_ ? current_speed_mps_
                                         : (use_speed_profile() ? speedAtIndex(nearest_idx) : default_speed_mps_);
    double Ld = std::clamp(k_v_ * v_for_ld + Ld0_, Ld_min_, Ld_max_);

    // 4) Lookahead 목표 인덱스
    const size_t target_idx = indexAtArcAhead(nearest_idx, Ld);
    const auto& pT = path_.poses[target_idx].pose.position;

    // 5) 목표점을 차량좌표계로 변환
    const double dx = pT.x - xb;
       const double dy = pT.y - yb;
    const double cos_y = std::cos(yaw), sin_y = std::sin(yaw);
    const double x_r =  cos_y * dx + sin_y * dy;      // 전방(+x)
    const double y_r = -sin_y * dx + cos_y * dy;      // 좌측(+y)
    const double Ld_real = std::hypot(x_r, y_r);
    if (Ld_real < 1e-6) return;

    // 6) Pure Pursuit 조향
    const double alpha = std::atan2(y_r, x_r);
    double steer = std::atan2(2.0 * wheelbase_ * std::sin(alpha), Ld_real);
    steer = std::clamp(steer, -max_steer_rad_, max_steer_rad_);

    // 7) 속도 목표(m/s)
    double v_cmd_mps = use_speed_profile() ? speedAtIndex(target_idx) : default_speed_mps_;
    v_cmd_mps = std::clamp(v_cmd_mps, 0.0, max_speed_cmd_mps_);

    // 8) MORAI CtrlCmd 생성
    morai_msgs::CtrlCmd ctrl;
    ctrl.longlCmdType = longlCmdType_;      // 1: throttle/brake, 2: velocity(km/h), 3: acceleration(m/s^2)
    ctrl.steering     = steer;              // [rad]

    if (longlCmdType_ == 2) {
      ctrl.velocity = v_cmd_mps * 3.6;     // [km/h]
      ctrl.accel = 0.0; ctrl.brake = 0.0;  // 미사용
      ctrl.acceleration = 0.0;
    } else if (longlCmdType_ == 1) {
      // 간단 P제어: 속도 오차 → accel/brake [0..1]
      const double v_err = v_cmd_mps - (has_current_speed_ ? current_speed_mps_ : 0.0);
      double u = long_kp_ * v_err;         // >0 → accel, <0 → brake
      if (u >= 0.0) { ctrl.accel = std::clamp(u, 0.0, accel_max_cmd_); ctrl.brake = 0.0; }
      else          { ctrl.accel = 0.0; ctrl.brake = std::clamp(-u, 0.0, brake_max_cmd_); }
      ctrl.velocity = 0.0; ctrl.acceleration = 0.0;
    } else { // longlCmdType_ == 3 (가속 명령)
      const double v_err = v_cmd_mps - (has_current_speed_ ? current_speed_mps_ : 0.0);
      ctrl.acceleration = std::clamp(long_kp_ * v_err, -3.0, 3.0); // [-3,3] m/s^2 예시
      ctrl.accel = 0.0; ctrl.brake = 0.0; ctrl.velocity = 0.0;
    }

    // 9) Publish
    // Publish control info (simplified for MPC integration)
    custom_interface::ControlInfo info;
    info.steering = steer;
    info.current_idx = nearest_idx;
    // Legacy fields removed: target_idx, lookahead_distance
    // info.target_idx = target_idx;  // REMOVED - no longer needed
    // info.lookahead_distance = Ld;  // REMOVED - no longer needed
    control_info_pub_.publish(info);
    
    if (lateral_only_mode_) {
      // In lateral-only mode, only publish control info
      ROS_DEBUG_THROTTLE(1.0, "[lateral_only] steer=%.3f, current_idx=%zu", 
                         steer, nearest_idx);
    } else {
      // Normal mode: also publish full control command
      ctrl_pub_.publish(ctrl);
    }

    // 스칼라 디버그 토픽
    std_msgs::Float64 vs, ds;
    vs.data = v_cmd_mps; ds.data = steer;
    speed_pub_.publish(vs);
    steer_pub_.publish(ds);

    if (publish_markers_) publishTargetMarker(pT, path_.header);
  }

  // ---- Helpers ----
  static double yawFromQuat(const geometry_msgs::Quaternion& q)
  {
    tf2::Quaternion qq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(qq).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void buildCumulativeS()
  {
    const size_t N = path_.poses.size();
    s_.assign(N, 0.0);
    for (size_t i = 1; i < N; ++i) {
      const auto& p0 = path_.poses[i-1].pose.position;
      const auto& p1 = path_.poses[i  ].pose.position;
      s_[i] = s_[i-1] + std::hypot(p1.x - p0.x, p1.y - p0.y);
    }
  }

  size_t nearestIndex(double x, double y) const
  {
    const size_t N = path_.poses.size();
    size_t best = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < N; ++i) {
      const auto& p = path_.poses[i].pose.position;
      const double d2 = (p.x - x)*(p.x - x) + (p.y - y)*(p.y - y);
      if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return best;
  }

  size_t indexAtArcAhead(size_t from_idx, double ds_ahead) const
  {
    const size_t N = s_.size();
    if (N == 0) return 0;
    const double s_target = s_[from_idx] + ds_ahead;
    size_t j = from_idx;
    while (j + 1 < N && s_[j + 1] < s_target) ++j;
    return std::min(j + 1, N - 1);
  }

  double speedAtIndex(size_t i) const
  {
    if (speed_profile_.empty()) return default_speed_mps_;
    if (i >= speed_profile_.size()) return std::max(0.0f, speed_profile_.back());
    return std::max(0.0f, speed_profile_[i]);
  }

  void publishTargetMarker(const geometry_msgs::Point& p, const std_msgs::Header& hdr)
  {
    visualization_msgs::Marker m;
    m.header = hdr;
    m.ns = "pure_pursuit_target";
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position = p;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.6;
    m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.0; m.color.a = 0.9;
    m.lifetime = ros::Duration(0.0);
    marker_pub_.publish(m);
  }

  bool use_speed_profile() const { return !speed_profile_.empty(); }

  // ---- Members ----
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber path_sub_, sp_sub_; //, v_sub_;
  ros::Subscriber ego_sub_;  // NEW: from Competition_topic
  ros::Publisher ctrl_pub_, speed_pub_, steer_pub_, marker_pub_, control_info_pub_;
  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Params
  std::string path_topic_, speed_profile_topic_; //, current_speed_topic_;
  std::string ego_status_topic_;
  std::string ref_frame_, base_frame_;
  std::string ctrl_topic_;
  int    longlCmdType_{2};
  double wheelbase_{3.0};
  double ctrl_rate_hz_{30.0};
  double Ld_min_{1.5}, Ld_max_{15.0}, Ld0_{3.0}, k_v_{0.5};
  double max_speed_cmd_mps_{11.0}, max_steer_rad_{0.6}, default_speed_mps_{3.0};
  double long_kp_{0.5}, accel_max_cmd_{1.0}, brake_max_cmd_{1.0};
  bool publish_markers_{true};
  bool lateral_only_mode_{false};

  // State
  nav_msgs::Path path_;
  std::vector<double> s_;
  std::vector<float> speed_profile_;
  double current_speed_mps_{0.0};
  bool has_current_speed_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pure_pursuit_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  PurePursuitNode node(nh, pnh);
  ros::spin();
  return 0;
}
