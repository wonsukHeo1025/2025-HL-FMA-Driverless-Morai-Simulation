/*  point_cloud_processor.cpp
 *  Robust LiDAR point cloud pre-processor (ROS1 Noetic)
 *
 *  - 입력 : /velodyne_points   (sensor_msgs/PointCloud2)
 *  - 출력 : /perception/lidar/processed_points  (sensor_msgs/PointCloud2)
 *
 *  설계 요약
 *    1) VoxelGrid 다운샘플로 연산량/노이즈 저감 (기본 ON)
 *    2) ROI 필터 (각도/거리/측면폭/축 제한)
 *    3) 지면 제거: RANSAC 수평면(기본) 또는 2.5D grid (옵션)
 *    4) 2D 투영은 옵션화(기본 OFF) — 높이 정보 유지가 원칙
 *    5) 단계별 디버그 토픽 퍼블리시(옵션)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <string>
#include <unordered_map>
#include <cmath>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class PointCloudProcessor
{
public:
  PointCloudProcessor()
  : nh_("~")
  {
    /* ───────── 파라미터 초기값 ───────── */
    nh_.param("min_angle_deg", min_angle_deg_, -100.0);
    nh_.param("max_angle_deg", max_angle_deg_,  100.0);
    nh_.param("min_range",     min_range_,       0.3);
    nh_.param("max_range",     max_range_,      50.0);
    nh_.param("min_y",         min_y_,          -5.0);
    nh_.param("max_y",         max_y_,           5.0);
    nh_.param("min_z",         min_z_,          -2.0);
    nh_.param("max_z",         max_z_,           2.0);
    nh_.param("use_width_ratio", use_width_ratio_, false);
    nh_.param("width_ratio",     width_ratio_,     0.5);

    nh_.param("use_voxelgrid",     use_voxel_,         true);
    nh_.param("voxel_leaf_size",   voxel_leaf_size_,   0.15);

    nh_.param("ground_method",     ground_method_,     std::string("ransac")); // ransac | grid | none
    nh_.param("plane_distance_threshold", plane_dist_thr_, 0.15);
    nh_.param("ransac_max_iterations",    ransac_max_iter_, 100);
    nh_.param("ransac_eps_angle_deg",      ransac_eps_angle_deg_, 10.0); // 수평면 허용 각도(도)

    nh_.param("grid_cell_size", grid_cell_size_, 1.0);
    nh_.param("grid_z_thresh",  grid_z_thresh_,  0.30);

    nh_.param("project_to_2d",  project_to_2d_,  false);
    nh_.param("publish_debug",  publish_debug_,  true);

    /* ───────── 통신 설정 ───────── */
    nh_.param<std::string>("input_topic",  input_topic_,  std::string("/velodyne_points"));
    nh_.param<std::string>("output_topic", output_topic_, std::string("/perception/lidar/processed_points"));

    sub_ = nh_.subscribe(input_topic_, 10, &PointCloudProcessor::callback, this);
    pub_processed_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    pub_roi_       = nh_.advertise<sensor_msgs::PointCloud2>("/perception/lidar/roi_points", 5);
    pub_nonground_ = nh_.advertise<sensor_msgs::PointCloud2>("/perception/lidar/nonground_points", 5);

    ROS_INFO("[PCP] ROI Angle(deg): %.1f ~ %.1f, Range: %.1f~%.1f m, Y: %.1f~%.1f m, Z: %.1f~%.1f m",
             min_angle_deg_, max_angle_deg_, min_range_, max_range_, min_y_, max_y_, min_z_, max_z_);
    ROS_INFO("[PCP] VoxelGrid: %s (leaf=%.2f)", use_voxel_?"ON":"OFF", voxel_leaf_size_);
    ROS_INFO("[PCP] Ground: %s (method=%s, dist=%.2f, max_iter=%d, eps=%.1fdeg)",
             (ground_method_=="none")?"OFF":"ON", ground_method_.c_str(),
             plane_dist_thr_, ransac_max_iter_, ransac_eps_angle_deg_);
    ROS_INFO("[PCP] Project2D: %s, Debug: %s", project_to_2d_?"ON":"OFF", publish_debug_?"ON":"OFF");
  }

private:
  /* ----------- 콜백 ----------- */
  void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    /* ① ROS → PCL */
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    const size_t original_cnt = cloud->size();
    if (original_cnt == 0) {
      ROS_WARN_THROTTLE(1.0, "[PCP] Empty cloud received");
      return;
    }

    /* ② VoxelGrid (옵션) */
    CloudT::Ptr cloud_ds = use_voxel_ ? voxel_downsample(cloud) : cloud;

    /* ③ ROI 필터 */
    CloudT::Ptr roi_cloud = apply_roi_filter(cloud_ds);
    const size_t roi_cnt = roi_cloud->size();
    if (roi_cnt == 0) {
      ROS_WARN_THROTTLE(1.0, "[PCP] No points in ROI");
      return;
    }
    if (publish_debug_ && pub_roi_.getNumSubscribers() > 0) publish_cloud(pub_roi_, roi_cloud, msg->header);

    /* ④ 지면 제거 */
    CloudT::Ptr nonground_cloud;
    if (ground_method_ == "ransac")      nonground_cloud = remove_ground_ransac(roi_cloud);
    else if (ground_method_ == "grid")   nonground_cloud = remove_ground_grid(roi_cloud);
    else                                  nonground_cloud = roi_cloud;

    const size_t nonground_cnt = nonground_cloud->size();
    if (nonground_cnt == 0) {
      ROS_WARN_THROTTLE(1.0, "[PCP] All points removed by ground filtering");
      return;
    }
    if (publish_debug_ && pub_nonground_.getNumSubscribers() > 0) publish_cloud(pub_nonground_, nonground_cloud, msg->header);

    /* ⑤ 2D 투영 (옵션) */
    CloudT::Ptr processed_cloud = nonground_cloud;
    if (project_to_2d_) {
      for (auto & pt : *processed_cloud)  pt.z = 0.0f;
    }

    /* ⑥ PCL → ROS */
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*processed_cloud, out_msg);
    out_msg.header = msg->header;
    pub_processed_.publish(out_msg);

    /* 로그 */
    ROS_INFO_THROTTLE(0.5,
      "[PCP] Published %zu pts (orig:%zu → voxel:%zu → ROI:%zu → nonground:%zu)",
      processed_cloud->size(), original_cnt, cloud_ds->size(), roi_cnt, nonground_cnt);
  }

  /* ----------- 퍼블리시 유틸 ----------- */
  void publish_cloud(ros::Publisher& pub, const CloudT::Ptr& cloud, const std_msgs::Header& header)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    pub.publish(msg);
  }

  /* ----------- VoxelGrid ----------- */
  CloudT::Ptr voxel_downsample(const CloudT::Ptr& cloud)
  {
    CloudT::Ptr out(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    vg.filter(*out);
    return out->empty() ? cloud : out;
  }

  /* ----------- ROI 필터 ----------- */
  CloudT::Ptr apply_roi_filter(const CloudT::Ptr& cloud)
  {
    CloudT::Ptr out(new CloudT);
    out->reserve(cloud->size());

    const double min_ang = min_angle_deg_ * M_PI / 180.0;
    const double max_ang = max_angle_deg_ * M_PI / 180.0;
    const double min_r2 = min_range_ * min_range_;
    const double max_r2 = max_range_ * max_range_;

    for (const auto & p : *cloud)
    {
      const double r2_xy = static_cast<double>(p.x)*p.x + static_cast<double>(p.y)*p.y;
      if (r2_xy < min_r2 || r2_xy > max_r2) continue;

      const double ang = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
      if (ang < min_ang || ang > max_ang) continue;

      if (p.z < min_z_ || p.z > max_z_) continue;

      bool y_ok = false;
      if (use_width_ratio_) {
        const double y_limit = std::abs(static_cast<double>(p.x)) * width_ratio_;
        y_ok = std::abs(static_cast<double>(p.y)) <= y_limit;
      } else {
        y_ok = (p.y >= min_y_) && (p.y <= max_y_);
      }
      if (!y_ok) continue;

      out->push_back(p);
    }
    return out;
  }

  /* ----------- 지면 제거: RANSAC 수평면 ----------- */
  CloudT::Ptr remove_ground_ransac(const CloudT::Ptr& cloud)
  {
    if (cloud->size() < 50) return cloud; // too small to segment

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // 수평면(축 Z에 수직) 선호
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(static_cast<float>(ransac_eps_angle_deg_ * M_PI / 180.0));
    seg.setMaxIterations(ransac_max_iter_);
    seg.setDistanceThreshold(plane_dist_thr_);
    seg.setInputCloud(cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coeffs);

    if (!inliers || inliers->indices.empty()) {
      ROS_WARN_THROTTLE(1.0, "[PCP] RANSAC found no ground plane; skipping ground removal");
      return cloud;
    }

    CloudT::Ptr out(new CloudT);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // remove ground
    extract.filter(*out);

    ROS_DEBUG("[PCP] Ground inliers: %zu → remain %zu", inliers->indices.size(), out->size());
    return out->empty() ? cloud : out;
  }

  /* ----------- 지면 제거: 2.5D grid ----------- */
  CloudT::Ptr remove_ground_grid(const CloudT::Ptr& cloud)
  {
    struct PairHash {
      std::size_t operator()(const std::pair<int,int>& p) const noexcept
      { return (static_cast<std::size_t>(p.first) << 32) ^ (static_cast<std::size_t>(p.second) & 0xffffffffu); }
    };

    std::unordered_map<std::pair<int,int>, float, PairHash> cell_min_z;
    const double GRID = grid_cell_size_;

    for (const auto & pt : *cloud) {
      int gx = static_cast<int>(std::floor(pt.x / GRID));
      int gy = static_cast<int>(std::floor(pt.y / GRID));
      auto key = std::make_pair(gx, gy);
      auto it  = cell_min_z.find(key);
      if (it == cell_min_z.end() || pt.z < it->second)
        cell_min_z[key] = pt.z;
    }

    CloudT::Ptr out(new CloudT);
    out->reserve(cloud->size());
    size_t ground_cnt = 0;

    for (const auto & pt : *cloud) {
      int gx = static_cast<int>(std::floor(pt.x / GRID));
      int gy = static_cast<int>(std::floor(pt.y / GRID));
      float min_neighbor_z = std::numeric_limits<float>::infinity();
      for (int dx=-1; dx<=1; ++dx)
        for (int dy=-1; dy<=1; ++dy) {
          auto it = cell_min_z.find(std::make_pair(gx+dx, gy+dy));
          if (it != cell_min_z.end() && it->second < min_neighbor_z)
            min_neighbor_z = it->second;
        }
      if (!std::isfinite(min_neighbor_z) || (pt.z - min_neighbor_z) >= grid_z_thresh_) {
        out->push_back(pt); // keep
      } else {
        ++ground_cnt;
      }
    }
    ROS_INFO_THROTTLE(1.0, "[PCP] Grid ground removed: %zu", ground_cnt);
    return out->empty() ? cloud : out;
  }

  /* ----------- 멤버 ----------- */
  ros::NodeHandle nh_;

  /* 파라미터 */
  double min_angle_deg_, max_angle_deg_;
  double min_range_, max_range_;
  double min_y_, max_y_, min_z_, max_z_, width_ratio_;
  bool   use_width_ratio_;

  bool   use_voxel_;
  double voxel_leaf_size_;

  std::string ground_method_;
  double plane_dist_thr_;
  int    ransac_max_iter_;
  double ransac_eps_angle_deg_;

  double grid_cell_size_;
  double grid_z_thresh_;

  bool   project_to_2d_;
  bool   publish_debug_;

  /* 통신 */
  ros::Subscriber sub_;
  ros::Publisher  pub_processed_;
  ros::Publisher  pub_roi_;
  ros::Publisher  pub_nonground_;

  std::string input_topic_, output_topic_;
};

/* ----------- main ----------- */
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "point_cloud_processor");
  PointCloudProcessor pcp;
  ros::spin();
  return 0;
}

