#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include "mpc_controller/types.hpp"

namespace mpc_controller {

class PathManager {
public:
  PathManager();

  void Update(const nav_msgs::Path& path_msg);

  bool HasPath() const;
  std::size_t Size() const;

  PathPoint Point(std::size_t index) const;

  int ClosestIndex(const VehicleState& state) const;

  double CurvatureMetric(std::size_t start_index,
                         std::size_t end_index,
                         const std::string& mode) const;

private:
  double ComputeCurvature(const PathPoint& prev,
                          const PathPoint& current,
                          const PathPoint& next) const;

  std::vector<PathPoint> path_points_;
  mutable std::mutex mutex_;
};

}  // namespace mpc_controller

