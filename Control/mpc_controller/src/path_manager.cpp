#include "mpc_controller/path_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "mpc_controller/utils.hpp"

namespace mpc_controller {

namespace {

double QuaternionToYaw(const geometry_msgs::Quaternion& q)
{
  return tf2::getYaw(q);
}

}  // namespace

PathManager::PathManager() = default;

void PathManager::Update(const nav_msgs::Path& path_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  path_points_.clear();
  path_points_.reserve(path_msg.poses.size());

  for (const auto& pose_stamped : path_msg.poses) {
    PathPoint point;
    point.x = pose_stamped.pose.position.x;
    point.y = pose_stamped.pose.position.y;
    point.z = pose_stamped.pose.position.z;
    point.heading = QuaternionToYaw(pose_stamped.pose.orientation);
    path_points_.push_back(point);
  }

  if (path_points_.size() < 3) {
    for (auto& point : path_points_) {
      point.curvature = 0.0;
    }
    return;
  }

  for (std::size_t i = 1; i + 1 < path_points_.size(); ++i) {
    const auto& prev = path_points_[i - 1];
    const auto& current = path_points_[i];
    const auto& next = path_points_[i + 1];
    path_points_[i].curvature = ComputeCurvature(prev, current, next);
  }
  path_points_.front().curvature = path_points_[1].curvature;
  path_points_.back().curvature = path_points_[path_points_.size() - 2].curvature;
}

bool PathManager::HasPath() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !path_points_.empty();
}

std::size_t PathManager::Size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return path_points_.size();
}

PathPoint PathManager::Point(std::size_t index) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= path_points_.size()) {
    return PathPoint{};
  }
  return path_points_[index];
}

int PathManager::ClosestIndex(const VehicleState& state) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (path_points_.empty()) {
    return -1;
  }

  double min_dist = std::numeric_limits<double>::infinity();
  int best_index = 0;

  for (std::size_t i = 0; i < path_points_.size(); ++i) {
    const auto& point = path_points_[i];
    const double dx = point.x - state.x;
    const double dy = point.y - state.y;
    const double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_index = static_cast<int>(i);
    }
  }

  return best_index;
}

double PathManager::CurvatureMetric(std::size_t start_index,
                                     std::size_t end_index,
                                     const std::string& mode) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (path_points_.empty() || start_index >= path_points_.size()) {
    return 0.0;
  }
  end_index = std::min(end_index, path_points_.size() - 1);
  if (end_index <= start_index) {
    return std::abs(path_points_[start_index].curvature);
  }

  if (mode == "rms") {
    double sum_sq = 0.0;
    std::size_t count = 0;
    for (std::size_t i = start_index; i <= end_index; ++i) {
      const double kappa = path_points_[i].curvature;
      sum_sq += kappa * kappa;
      ++count;
    }
    return count > 0 ? std::sqrt(sum_sq / static_cast<double>(count)) : 0.0;
  }

  // Default: max absolute curvature.
  double max_abs = 0.0;
  for (std::size_t i = start_index; i <= end_index; ++i) {
    max_abs = std::max(max_abs, std::abs(path_points_[i].curvature));
  }
  return max_abs;
}

double PathManager::ComputeCurvature(const PathPoint& prev,
                                      const PathPoint& current,
                                      const PathPoint& next) const
{
  const double x1 = prev.x;
  const double y1 = prev.y;
  const double x2 = current.x;
  const double y2 = current.y;
  const double x3 = next.x;
  const double y3 = next.y;

  const double d12 = std::hypot(x2 - x1, y2 - y1);
  const double d23 = std::hypot(x3 - x2, y3 - y2);
  const double d31 = std::hypot(x1 - x3, y1 - y3);

  if (d12 < 1e-6 || d23 < 1e-6 || d31 < 1e-6) {
    return 0.0;
  }

  const double area = 0.5 * std::abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1));
  if (area < 1e-6) {
    return 0.0;
  }

  double curvature = 4.0 * area / (d12 * d23 * d31);
  const double cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
  if (cross < 0.0) {
    curvature = -curvature;
  }
  return curvature;
}

}  // namespace mpc_controller
