#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

#include <std_msgs/Float32MultiArray.h>

namespace mpc_controller {

class SpeedProfileBuffer {
public:
  SpeedProfileBuffer();

  void Configure(double base_resolution_m, std::size_t upsample_factor);

  void Update(const std::vector<double>& values);
  void Update(const std_msgs::Float32MultiArray& msg);

  bool Empty() const;
  std::size_t Size() const;

  double Value(std::size_t index) const;

  std::vector<double> BuildTrajectory(std::size_t start_index,
                                      double current_speed_mps,
                                      double control_period_s,
                                      std::size_t horizon_steps) const;

  double DistanceToNextStop(std::size_t start_index,
                            std::size_t max_lookahead_points = 2000,
                            double stop_threshold = 1e-3) const;

  double Resolution() const;

private:
  std::vector<double> Upsample(const std::vector<double>& raw) const;

  std::vector<double> profile_;
  double base_resolution_m_{0.5};
  std::size_t upsample_factor_{10};
  double resolution_m_{0.05};
  mutable std::mutex mutex_;
};

}  // namespace mpc_controller

