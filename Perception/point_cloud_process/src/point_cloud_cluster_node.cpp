/*  point_cloud_cluster_node.cpp
 *
 *  - 입력  : /perception/lidar/processed_points   (sensor_msgs/PointCloud2)
 *  - 출력  : /vis/perception/lidar/cluster_markers (visualization_msgs/Marker – 클러스터 중심점)
 *
 *  설계 요약
 *    1) 선택적 VoxelGrid 다운샘플로 성능 향상
 *    2) Euclidean Clustering으로 3D 클러스터 추출
 *    3) 클러스터 기초 shape 필터(너비/높이)로 노이즈 제거(옵션)
 *    4) 중심점 병합(거리 임계)으로 중복 축약
 *    5) Marker(POINTS) 퍼블리시
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <string>

#include <Eigen/Dense>

#include <vector>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <cfloat>

using PointT  = pcl::PointXYZ;
using CloudT  = pcl::PointCloud<PointT>;
using Vec3f   = Eigen::Vector3f;

class PointCloudClusterNode
{
public:
  PointCloudClusterNode()
  : nh_("~")
  {
    /* ── 파라미터 선언 (launch/CLI 로 조정 가능) ─────────────────────── */
    nh_.param("use_voxel_downsample", use_voxel_downsample_, false);
    nh_.param("voxel_leaf_size",      voxel_leaf_size_,      0.15);

    nh_.param("cluster_tolerance", cluster_tolerance_, 0.40);
    nh_.param("min_cluster_size", min_cluster_size_, 5);
    nh_.param("max_cluster_size", max_cluster_size_, 100000);
    nh_.param("merge_threshold",  merge_thresh_xy_,  0.30); // XY 병합 거리
    nh_.param("marker_scale",     marker_scale_,     0.30);

    nh_.param("use_shape_filter",    use_shape_filter_,    false);
    nh_.param("min_cluster_height",  min_cluster_h_,       0.01);
    nh_.param("max_cluster_height",  max_cluster_h_,       3.00);
    nh_.param("min_cluster_width",   min_cluster_w_,       0.03);
    nh_.param("max_cluster_width",   max_cluster_w_,       3.00);

    /* ── 통신 설정 ──────────────────────────────────────────────────── */
    nh_.param<std::string>("input_topic", input_topic_, std::string("/perception/lidar/processed_points"));
    nh_.param<std::string>("marker_topic", marker_topic_, std::string("/vis/perception/lidar/cluster_markers"));

    sub_ = nh_.subscribe(input_topic_, 10, &PointCloudClusterNode::cbCloud, this);
    pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);

    ROS_INFO("[PCC] VoxelGrid=%s (leaf=%.2f)  tol=%.2f  size=[%d,%d]  merge_xy=%.2f  shape_filter=%s",
             use_voxel_downsample_?"ON":"OFF", voxel_leaf_size_, cluster_tolerance_,
             min_cluster_size_, max_cluster_size_, merge_thresh_xy_, use_shape_filter_?"ON":"OFF");
  }

private:
  /* ========= 중심점 계산 ========= */
  static Vec3f centroid(const CloudT::Ptr & cloud, const pcl::PointIndices& indices)
  {
    Vec3f c(0,0,0);
    for (int i : indices.indices) {
      c += cloud->points[i].getVector3fMap();
    }
    c /= static_cast<float>(indices.indices.size());
    return c;
  }

  struct Extents { float min_x, max_x, min_y, max_y, min_z, max_z; };

  static Extents compute_extents(const CloudT::Ptr & cloud, const pcl::PointIndices& indices)
  {
    Extents e {+FLT_MAX, -FLT_MAX, +FLT_MAX, -FLT_MAX, +FLT_MAX, -FLT_MAX};
    for (int i : indices.indices) {
      const auto &p = cloud->points[i];
      e.min_x = std::min(e.min_x, p.x); e.max_x = std::max(e.max_x, p.x);
      e.min_y = std::min(e.min_y, p.y); e.max_y = std::max(e.max_y, p.y);
      e.min_z = std::min(e.min_z, p.z); e.max_z = std::max(e.max_z, p.z);
    }
    return e;
  }

  /* ========= 콜백 ========= */
  void cbCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    /* ① ROS → PCL */
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      ROS_WARN_THROTTLE(1.0, "[PCC] Empty cloud received");
      return;
    }

    /* ② VoxelGrid (옵션) */
    CloudT::Ptr cloud_in = cloud;
    CloudT::Ptr cloud_ds(new CloudT);
    if (use_voxel_downsample_) {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      vg.filter(*cloud_ds);
      if (!cloud_ds->empty()) cloud_in = cloud_ds;
    }

    /* ③ Euclidean Clustering */
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
      ROS_INFO_THROTTLE(1.0, "[PCC] No clusters found");
      return;
    }

    /* ④ 중심점 및 extents 계산 + 필터 */
    std::vector<Vec3f> centers;
    centers.reserve(cluster_indices.size());
    for (const auto & indices : cluster_indices) {
      if (use_shape_filter_) {
        Extents e = compute_extents(cloud_in, indices);
        const float w = std::max(e.max_x - e.min_x, e.max_y - e.min_y);
        const float h = e.max_z - e.min_z;
        if (w < min_cluster_w_ || w > max_cluster_w_ || h < min_cluster_h_ || h > max_cluster_h_) {
          continue; // drop noisy/invalid clusters
        }
      }
      centers.emplace_back(centroid(cloud_in, indices));
    }

    if (centers.empty()) {
      ROS_INFO_THROTTLE(1.0, "[PCC] All clusters filtered by shape limits");
      return;
    }

    /* ⑤ 클러스터 병합 (XY 거리) */
    const size_t M = centers.size();
    std::vector<bool> merged(M, false);
    std::vector<Vec3f> final_centers;

    for (size_t i=0; i<M; ++i)
    {
      if (merged[i]) continue;

      Vec3f mean = centers[i];
      int count = 1;
      merged[i] = true;

      for (size_t j=i+1; j<M; ++j)
      {
        if (merged[j]) continue;
        const float dx = centers[i].x() - centers[j].x();
        const float dy = centers[i].y() - centers[j].y();
        const float dist_xy = std::sqrt(dx*dx + dy*dy);
        if (dist_xy <= static_cast<float>(merge_thresh_xy_))
        {
          mean += centers[j];
          count++;
          merged[j] = true;
        }
      }
      mean /= static_cast<float>(count);
      final_centers.push_back(mean);
    }

    /* ⑥ Marker 작성 */
    visualization_msgs::Marker marker;
    marker.header   = msg->header;
    marker.ns       = "cluster_centers";
    marker.id       = 0;
    marker.type     = visualization_msgs::Marker::POINTS;
    marker.action   = visualization_msgs::Marker::ADD;
    marker.scale.x  = marker_scale_;
    marker.scale.y  = marker_scale_;
    marker.color.a  = 1.0f;

    for (const auto & c : final_centers)
    {
      geometry_msgs::Point p;
      p.x = c.x(); p.y = c.y(); p.z = c.z();
      marker.points.push_back(p);

      std_msgs::ColorRGBA col;
      col.r = 1.0f; col.g = 0.0f; col.b = 0.0f; col.a = 1.0f;
      marker.colors.push_back(col);
    }

    pub_.publish(marker);
    ROS_INFO_THROTTLE(0.5, "[PCC] Found %zu clusters → published %zu merged centers", 
                cluster_indices.size(), final_centers.size());
  }

  /* ── 멤버 ───────────────────────────────────────────────────────── */
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  bool   use_voxel_downsample_;
  double voxel_leaf_size_;

  double cluster_tolerance_, merge_thresh_xy_, marker_scale_;
  int    min_cluster_size_, max_cluster_size_;

  bool   use_shape_filter_;
  double min_cluster_h_, max_cluster_h_;
  double min_cluster_w_, max_cluster_w_;

  std::string input_topic_, marker_topic_;
};

/* ===== main ===== */
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "point_cloud_cluster_node");
  PointCloudClusterNode pccn;
  ros::spin();
  return 0;
}
