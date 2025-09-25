/*  collision_detector_node.cpp
 *  클러스터 마커(/vis/perception/lidar/cluster_markers) + 속도(/fix_velocity)로
 *  충돌 플래그(/perception/collision/flag)·디버그 마커(/vis/perception/collision/debug_markers)를 퍼블리시
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h> 
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>

#include <array>
#include <vector>
#include <cmath>
#include <utility>

/* ───────────────── Zone 구조 ───────────────── */
struct ZoneParam
{
  float front_ext, side_ext, rear_ext, front_radius;
  float color[3];
};

/* ───────────────── 노드 ───────────────── */
class CollisionDetectorNode
{
public:
  CollisionDetectorNode()
  : nh_("~")
  {
    /* Sub/Pub ------------------------------------------------------- */
    nh_.param<std::string>("cluster_markers_topic", cluster_markers_topic_, std::string("/vis/perception/lidar/cluster_markers"));
    nh_.param<std::string>("twist_topic", twist_topic_, std::string("/ublox_gps_node/fix_velocity"));
    nh_.param<std::string>("odometry_topic", odometry_topic_, std::string("/odometry"));
    nh_.param<std::string>("flag_topic", flag_topic_, std::string("/perception/collision/flag"));
    nh_.param<std::string>("debug_topic", debug_topic_, std::string("/vis/perception/collision/debug_markers"));
    // legacy topics removed

    sub_markers_ = nh_.subscribe(cluster_markers_topic_, 10, &CollisionDetectorNode::cbMarkers, this);
    sub_vel_ = nh_.subscribe(twist_topic_, 10, &CollisionDetectorNode::cbVelocity, this);
    sub_odom_ = nh_.subscribe(odometry_topic_, 20, &CollisionDetectorNode::cbOdometry, this);
    pub_flag_ = nh_.advertise<std_msgs::UInt8>(flag_topic_, 10);
    pub_dbg_  = nh_.advertise<visualization_msgs::MarkerArray>(debug_topic_, 10);
    

    /* 초기화 --------------------------------------------------------- */
    initStaticZones();
    updateDangerZone(); // speed=0
    prev_dbg_time_ = ros::Time::now();

    ROS_INFO("Collision-only node started (no green zone)");
    ROS_INFO("\033[33m[ZONES]\033[0m Caution: %.1fm front, %.1fm side | Danger: dynamic by speed", 
                caution_.front_ext, caution_.side_ext);
  }

private:
  /* ───────── 속도 콜백 ───────── */
  void cbVelocity(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
  {
    const auto &v = msg->twist.twist.linear;
    updateSpeedAndZone(std::hypot(v.x, v.y), "SPEED");
  }

  /* ───────── 오도메트리 콜백 ───────── */
  void cbOdometry(const nav_msgs::Odometry::ConstPtr& msg)
  {
    const auto &v = msg->twist.twist.linear;
    updateSpeedAndZone(std::hypot(v.x, v.y), "ODOM SPEED");
  }

  /* ───────── 마커 콜백 ───────── */
  void cbMarkers(const visualization_msgs::Marker::ConstPtr& msg)
  {
    uint8_t flag = evaluate(msg->points);

    std_msgs::UInt8 f;  f.data = flag;
    pub_flag_.publish(f);
    

    static uint8_t last_flag = 255;
    if (flag != last_flag) {
      const char* RESET = "\033[0m";
      const char* RED = "\033[31m";
      const char* YELLOW = "\033[33m";
      const char* GREEN = "\033[32m";
      const char* BOLD = "\033[1m";
      
      switch(flag) {
        case 0:
          ROS_INFO("%s%s[COLLISION] Clear - No obstacles detected%s", GREEN, BOLD, RESET);
          break;
        case 1:
          ROS_WARN("%s%s[COLLISION] CAUTION - Obstacle in yellow zone! (flag=1)%s", YELLOW, BOLD, RESET);
          break;
        case 2:
          ROS_ERROR("%s%s[COLLISION] DANGER - EMERGENCY STOP! (flag=2)%s", RED, BOLD, RESET);
          break;
      }
      last_flag = flag;
    }

    auto now = ros::Time::now();
    if ((now - prev_dbg_time_).toSec() < 0.001) return;
    prev_dbg_time_ = now;
    auto arr = buildDebugArray(msg->header, flag);
    pub_dbg_.publish(arr);
    
  }

  /* ───────── 충돌 판정 ───────── */
  uint8_t evaluate(const std::vector<geometry_msgs::Point>& pts)
  {
    bool danger=false, caution=false;
    for (const auto &p : pts)
    {
      if (inZone(p.x, p.y, danger_)) { danger=true; break; }
      else if (inZone(p.x, p.y, caution_)) caution=true;
    }
    if (danger)  return 2;
    if (caution) return 1;
    return 0;
  }

  /* ───────── Zone 포함 검사 ───────── */
  bool inZone(float x, float y, const ZoneParam& z) const
  {
    float rear  = LIDAR_REAR_OFFSET_ - z.rear_ext;
    float front = LIDAR_REAR_OFFSET_ + VEH_L_ + z.front_ext;
    float half  = VEH_W_/2 + z.side_ext;

    if (rear <= x && x <= front && -half <= y && y <= half) return true;
    if (x >= front)
    {
      float dx = x - front;
      return dx*dx + y*y <= z.front_radius*z.front_radius;
    }
    return false;
  }

  /* ───────── Danger 동적 갱신 ───────── */
  void updateDangerZone()
  {
    size_t idx = SPEED_RANGE_.size() - 1;
    for (size_t i=0;i<SPEED_RANGE_.size();++i)
      if (SPEED_RANGE_[i].first <= current_speed_
          && current_speed_ < SPEED_RANGE_[i].second) { idx=i; break; }
    danger_ = { DANGER_FRONT_[idx], 0.0f, 0.0f,
                DANGER_RADIUS_[idx], {1,0,0}};
  }

  /* ───────── 공통 속도 처리 ───────── */
  inline size_t speedIndexFor(float s) const
  {
    for (size_t i=0;i<SPEED_RANGE_.size();++i)
      if (SPEED_RANGE_[i].first <= s && s < SPEED_RANGE_[i].second) return i;
    return SPEED_RANGE_.size() - 1;
  }

  void updateSpeedAndZone(float new_speed, const char* tag)
  {
    float prev_speed = current_speed_;
    current_speed_ = new_speed;
    size_t prev_idx = speedIndexFor(prev_speed);
    size_t curr_idx = speedIndexFor(current_speed_);
    if (prev_idx != curr_idx)
    {
      ROS_INFO("[%s] %.2f m/s - Danger zone updated: front=%.1fm, radius=%.1fm",
               tag, current_speed_, DANGER_FRONT_[curr_idx], DANGER_RADIUS_[curr_idx]);
    }
    updateDangerZone();
  }

  /* ───────── 디버그 마커 ───────── */
  visualization_msgs::MarkerArray
  buildDebugArray(const std_msgs::Header& hdr, uint8_t flag)
  {
    visualization_msgs::MarkerArray arr;
    float half = VEH_W_/2;

    const std::array<float,4> car_rgba{1,1,1,1};
    arr.markers.push_back(makeRect(hdr,"car",0,
      LIDAR_REAR_OFFSET_, LIDAR_REAR_OFFSET_+VEH_L_, -half, half, car_rgba));

    arr.markers.push_back(makeZone(hdr,"caution",1,caution_,flag==1));
    arr.markers.push_back(makeZone(hdr,"danger",2,danger_, flag==2));

    visualization_msgs::Marker txt;
    txt.header=hdr; txt.ns="speed"; txt.id=3;
    txt.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
    txt.action=visualization_msgs::Marker::ADD;
    txt.pose.position.x=LIDAR_REAR_OFFSET_+VEH_L_/2;
    txt.pose.position.y=0; txt.pose.position.z=1;
    txt.pose.orientation.w=1;
    txt.scale.z=0.3;
    txt.color.r=txt.color.g=txt.color.b=txt.color.a=1.0;
    txt.text = "v=" + std::to_string(static_cast<int>(current_speed_*10)/10.0) + " m/s";
    arr.markers.push_back(std::move(txt));
    return arr;
  }

  /* ───────── 헬퍼(사각형) ───────── */
  visualization_msgs::Marker makeRect(
      const std_msgs::Header& hdr,const std::string& ns,int id,
      float x0,float x1,float y0,float y1,const std::array<float,4>& rgba)
  {
    visualization_msgs::Marker mk;
    mk.header=hdr; mk.ns=ns; mk.id=id;
    mk.type=visualization_msgs::Marker::LINE_STRIP;
    mk.action=visualization_msgs::Marker::ADD;
    mk.scale.x=0.06; mk.pose.orientation.w=1;
    mk.color.r=rgba[0]; mk.color.g=rgba[1];
    mk.color.b=rgba[2]; mk.color.a=rgba[3];

    std::vector<geometry_msgs::Point> p(5);
    p[0].x=x0; p[0].y=y0;
    p[1].x=x1; p[1].y=y0;
    p[2].x=x1; p[2].y=y1;
    p[3].x=x0; p[3].y=y1;
    p[4]=p[0];
    mk.points=p;
    return mk;
  }

  /* ───────── 헬퍼(Zone) ───────── */
  visualization_msgs::Marker makeZone(
      const std_msgs::Header& hdr,const std::string& ns,int id,
      const ZoneParam& z,bool active)
  {
    visualization_msgs::Marker mk;
    mk.header=hdr; mk.ns=ns; mk.id=id;
    mk.type=visualization_msgs::Marker::LINE_STRIP;
    mk.action=visualization_msgs::Marker::ADD;
    mk.scale.x=0.08; mk.pose.orientation.w=1;
    mk.color.r=z.color[0]; mk.color.g=z.color[1];
    mk.color.b=z.color[2]; mk.color.a=active?0.8f:0.25f;

    float rear  = LIDAR_REAR_OFFSET_ - z.rear_ext;
    float front = LIDAR_REAR_OFFSET_ + VEH_L_ + z.front_ext;
    float half  = VEH_W_/2 + z.side_ext;

    std::vector<geometry_msgs::Point> pts;
    pts.reserve(22);
    geometry_msgs::Point p; p.z=0;
    p.x=rear;  p.y=-half; pts.push_back(p);
    p.x=front; p.y=-half; pts.push_back(p);

    const int n=16;
    for(int i=0;i<=n;++i){
      double th=M_PI/2 - i*M_PI/n;
      p.x = front + z.front_radius*std::cos(th);
      p.y = z.front_radius*std::sin(th);
      pts.push_back(p);
    }
    p.x=front; p.y=half; pts.push_back(p);
    p.x=rear;  p.y=half; pts.push_back(p);
    pts.push_back(pts.front());

    mk.points=pts;
    return mk;
  }

  /* ───────── 고정 파라미터 ───────── */
  void initStaticZones()
  {
    nh_.param("VEH_L", VEH_L_, 1.1f);
    nh_.param("VEH_W", VEH_W_, 0.7f);
    nh_.param("LIDAR_REAR_OFFSET", LIDAR_REAR_OFFSET_, -0.1f);

    caution_ = {
      2.0f, 0.5f, -1.0f, 0.8f, {1,0.65f,0}
    };

    SPEED_RANGE_={{0,0.3f},{0.3f,1},{1,2},{2,1e9f}};
    DANGER_FRONT_ ={0.1f,0.6f,1.2f,1.5f};
    DANGER_RADIUS_={0.5f,0.6f,0.7f,0.8f};
  }

  /* ───────── 멤버 변수 ───────── */
  ros::NodeHandle nh_;
  ros::Subscriber sub_markers_;
  ros::Subscriber sub_vel_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_flag_;
  ros::Publisher pub_dbg_;
  

  float VEH_L_, VEH_W_, LIDAR_REAR_OFFSET_;
  ZoneParam caution_, danger_;
  std::vector<std::pair<float,float>> SPEED_RANGE_;
  std::vector<float> DANGER_FRONT_, DANGER_RADIUS_;

  float current_speed_{0.0f};
  ros::Time prev_dbg_time_;

  // topics/params
  std::string cluster_markers_topic_, twist_topic_, odometry_topic_;
  std::string flag_topic_, debug_topic_;
  
};

/* ───────── main ───────── */
int main(int argc,char** argv)
{
  ros::init(argc,argv,"collision_detector_node");
  CollisionDetectorNode cdn;
  ros::spin();
  return 0;
}
