#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>

#include "mpc_controller/types.hpp"

namespace mpc_controller {

struct LongitudinalConfig {
  double sample_time{0.02};
  int prediction_horizon{77};
  int control_horizon{20};
  double A{0.9956};
  double B{0.0779};
  double d{0.0336};
  double q_weight{1.0};
  double r_weight{1.0};
  double r_delta_weight{0.25};
  double u_min{-1.0};
  double u_max{1.0};
  double delta_u_max{0.1};
  double v_min{0.0};
  double v_max{49.5 / 3.6};
};

class LongitudinalMpc {
public:
  LongitudinalMpc();

  bool Configure(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

  LongitudinalSolution Solve(double current_velocity_mps,
                             double previous_input,
                             const std::vector<double>& reference_velocity);

  void SetLowSpeedMode(bool enabled);
  void SetDeltaUMax(double value);

  const LongitudinalConfig& config() const { return config_; }

private:
  void BuildProblem();
  void ResetSolver();

  inline int StateIndex(int timestep) const
  {
    return timestep;
  }

  inline int ControlIndex(int timestep) const
  {
    return (config_.prediction_horizon + 1) + timestep;
  }

  bool configured_{false};
  bool low_speed_active_{false};

  LongitudinalConfig config_;

  OsqpEigen::Solver solver_;

  double last_control_{0.0};
};

}  // namespace mpc_controller

