#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>
#include <ros/time.h>

#include "mpc_controller/path_manager.hpp"
#include "mpc_controller/speed_profile_buffer.hpp"
#include "mpc_controller/types.hpp"

namespace mpc_controller {

struct DynamicHorizonConfig {
  bool enable{false};
  int min_steps{60};
  int max_steps{110};
  int quantize_steps{10};
  int change_threshold{15};
  double cooldown_sec{0.3};
  int roi_steps{35};
  std::string kappa_agg{"max_abs"};
  double kappa_low{0.01};
  double kappa_high{0.1};
};

struct LateralConfig {
  double sample_time{0.02};
  int prediction_horizon{60};
  int control_horizon{10};
  double wheelbase{3.0};
  double preview_distance{5.0};
  std::vector<double> q_weights{200.0, 250.0};
  std::vector<double> p_weights{2000.0, 2500.0};
  double r_weight{1.0};
  double r_delta_weight{40.0};
  double delta_min{-0.7};
  double delta_max{0.7};
  double delta_rate_limit{0.015};
  bool enable_band{true};
  DynamicHorizonConfig dynamic_horizon;
};

class LateralMpc {
public:
  LateralMpc();

  bool Configure(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

  LateralSolution Solve(const VehicleState& state,
                        const PathManager& path_manager,
                        int reference_index,
                        const SpeedProfileBuffer* speed_profile = nullptr);

  const LateralConfig& config() const { return config_; }
  int active_prediction_horizon() const { return active_prediction_horizon_; }
  int active_control_horizon() const { return active_control_horizon_; }
  double last_roi_metric() const { return last_roi_metric_; }
  const std::string& last_roi_mode() const { return last_roi_mode_; }
  int last_roi_start_index() const { return last_roi_start_index_; }
  int last_roi_end_index() const { return last_roi_end_index_; }

private:
  void BuildProblem();
  void ResetSolver();

  Eigen::Vector2d ComputeError(const VehicleState& state,
                               const PathPoint& reference_point) const;

  Eigen::Matrix2d ComputeAd(double vx) const;
  Eigen::Vector2d ComputeBd(double vx) const;
  Eigen::Vector2d ComputeEd(double vx) const;

  inline int StateIndex(int timestep, int state) const
  {
    return timestep * static_cast<int>(nx_) + state;
  }

  inline int ControlIndex(int timestep) const
  {
    return static_cast<int>(nx_ * (active_prediction_horizon_ + 1)) + timestep;
  }

  bool configured_{false};
  LateralConfig config_;

  OsqpEigen::Solver solver_;

  std::size_t nx_{2};
  std::size_t nu_{1};

  double last_control_{0.0};
  int active_prediction_horizon_{0};
  int active_control_horizon_{0};
  DynamicHorizonConfig dynamic_config_;
  double last_roi_metric_{0.0};
  std::string last_roi_mode_{""};
  int last_roi_start_index_{0};
  int last_roi_end_index_{0};
  ros::Time last_horizon_update_;
};

}  // namespace mpc_controller
