#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace mpc_controller {

struct VehicleState {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double heading{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double yaw_rate{0.0};
  double ax{0.0};
  double ay{0.0};
};

struct PathPoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double heading{0.0};
  double curvature{0.0};
};

struct ControlCommand {
  double steering{0.0};
  double accel{0.0};
  double brake{0.0};
};

struct SolverReport {
  bool success{false};
  std::string status{"not_run"};
  double cost{0.0};
  int iterations{0};
  double solve_time_ms{0.0};
};

struct LateralSolution {
  ControlCommand command;
  std::vector<double> steering_sequence;
  std::vector<double> predicted_lateral_errors;
  std::vector<double> predicted_heading_errors;
  SolverReport report;
  int horizon_steps{0};
  double roi_curvature_metric{0.0};
  std::string roi_curvature_mode;
  int roi_start_index{0};
  int roi_end_index{0};
};

struct LongitudinalSolution {
  ControlCommand command;
  std::vector<double> velocity_prediction;
  SolverReport report;
};

}  // namespace mpc_controller
