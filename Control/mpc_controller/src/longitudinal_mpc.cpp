#include "mpc_controller/longitudinal_mpc.hpp"

#include <algorithm>
#include <limits>

#include <ros/ros.h>

#include "mpc_controller/utils.hpp"

namespace mpc_controller {
namespace {

std::string StatusToString(OsqpEigen::Status status)
{
  switch (status) {
    case OsqpEigen::Status::Solved:
      return "solved";
    case OsqpEigen::Status::SolvedInaccurate:
      return "solved_inaccurate";
    case OsqpEigen::Status::PrimalInfeasible:
      return "primal_infeasible";
    case OsqpEigen::Status::DualInfeasible:
      return "dual_infeasible";
    case OsqpEigen::Status::MaxIterReached:
      return "max_iter";
    default:
      return "unknown";
  }
}

}  // namespace

LongitudinalMpc::LongitudinalMpc() = default;

bool LongitudinalMpc::Configure(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
{
  (void)nh;
  ros::NodeHandle config_nh(private_nh, "longitudinal_mpc");
  config_nh.param("sample_time", config_.sample_time, config_.sample_time);
  config_nh.param("prediction_horizon", config_.prediction_horizon, config_.prediction_horizon);
  config_nh.param("control_horizon", config_.control_horizon, config_.control_horizon);
  config_nh.param("A", config_.A, config_.A);
  config_nh.param("B", config_.B, config_.B);
  config_nh.param("d", config_.d, config_.d);
  config_nh.param("q_weight", config_.q_weight, config_.q_weight);
  config_nh.param("r_weight", config_.r_weight, config_.r_weight);
  config_nh.param("r_delta_weight", config_.r_delta_weight, config_.r_delta_weight);
  config_nh.param("u_min", config_.u_min, config_.u_min);
  config_nh.param("u_max", config_.u_max, config_.u_max);
  config_nh.param("delta_u_max", config_.delta_u_max, config_.delta_u_max);
  config_nh.param("v_min", config_.v_min, config_.v_min);
  config_nh.param("v_max", config_.v_max, config_.v_max);

  config_.prediction_horizon = std::max(1, config_.prediction_horizon);
  config_.control_horizon = std::min(std::max(1, config_.control_horizon), config_.prediction_horizon);

  configured_ = true;
  return true;
}

LongitudinalSolution LongitudinalMpc::Solve(double current_velocity_mps,
                                            double previous_input,
                                            const std::vector<double>& reference_velocity)
{
  LongitudinalSolution solution;
  solution.report.success = false;
  solution.report.status = configured_ ? "solve_failed" : "not_configured";
  solution.command.steering = 0.0;
  solution.command.accel = 0.0;
  solution.command.brake = 0.0;

  if (!configured_) {
    return solution;
  }

  const int Np = config_.prediction_horizon;
  const int Nc = config_.control_horizon;
  const int n_state_vars = Np + 1;
  const int n_control_vars = Nc;
  const int n_vars = n_state_vars + n_control_vars;

  last_control_ = previous_input;

  std::vector<Eigen::Triplet<double>> h_triplets;
  h_triplets.reserve(n_vars * 3);

  for (int k = 1; k <= Np; ++k) {
    const int idx_state = StateIndex(k);
    h_triplets.emplace_back(idx_state, idx_state, 2.0 * config_.q_weight);
  }

  for (int k = 0; k < Nc; ++k) {
    const int idx_u = ControlIndex(k);
    h_triplets.emplace_back(idx_u, idx_u, 2.0 * config_.r_weight);
  }

  const double rd = config_.r_delta_weight;
  if (rd > 1e-6) {
    if (Nc > 0) {
      const int idx0 = ControlIndex(0);
      h_triplets.emplace_back(idx0, idx0, 2.0 * rd);
    }
    for (int k = 1; k < Nc; ++k) {
      const int idx_u = ControlIndex(k);
      const int idx_prev = ControlIndex(k - 1);
      h_triplets.emplace_back(idx_u, idx_u, 2.0 * rd);
      h_triplets.emplace_back(idx_prev, idx_prev, 2.0 * rd);
      h_triplets.emplace_back(idx_u, idx_prev, -2.0 * rd);
      h_triplets.emplace_back(idx_prev, idx_u, -2.0 * rd);
    }
  }

  Eigen::SparseMatrix<double> hessian(n_vars, n_vars);
  hessian.setFromTriplets(h_triplets.begin(), h_triplets.end());

  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_vars);
  if (rd > 1e-6 && Nc > 0) {
    gradient[ControlIndex(0)] += -2.0 * rd * last_control_;
  }

  if (!reference_velocity.empty()) {
    const int ref_size = std::min(static_cast<int>(reference_velocity.size()), Np);
    for (int k = 1; k <= ref_size; ++k) {
      gradient[StateIndex(k)] += -2.0 * config_.q_weight * reference_velocity[k - 1];
    }
  }

  const int eq_rows = Np + 1;
  const int control_bound_rows = 2 * Nc;
  const int delta_rows = Nc;
  const int state_bound_rows = Np;
  const int total_constraints = eq_rows + control_bound_rows + delta_rows + state_bound_rows;

  Eigen::SparseMatrix<double> constraint(total_constraints, n_vars);
  std::vector<Eigen::Triplet<double>> a_triplets;
  a_triplets.reserve(total_constraints * 3);
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(total_constraints);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(total_constraints);

  int row = 0;
  a_triplets.emplace_back(row, StateIndex(0), 1.0);
  lower[row] = current_velocity_mps;
  upper[row] = current_velocity_mps;
  ++row;

  for (int k = 0; k < Np; ++k) {
    const int row_index = row++;
    a_triplets.emplace_back(row_index, StateIndex(k + 1), 1.0);
    a_triplets.emplace_back(row_index, StateIndex(k), -config_.A);
    const int u_index = std::min(k, Nc - 1);
    a_triplets.emplace_back(row_index, ControlIndex(u_index), -config_.B);
    lower[row_index] = config_.d;
    upper[row_index] = config_.d;
  }

  for (int k = 0; k < Nc; ++k) {
    const int row_min = row++;
    a_triplets.emplace_back(row_min, ControlIndex(k), 1.0);
    lower[row_min] = config_.u_min;
    upper[row_min] = config_.u_max;

    const int row_delta = row++;
    if (k == 0) {
      a_triplets.emplace_back(row_delta, ControlIndex(0), 1.0);
      lower[row_delta] = last_control_ - config_.delta_u_max;
      upper[row_delta] = last_control_ + config_.delta_u_max;
    } else {
      a_triplets.emplace_back(row_delta, ControlIndex(k), 1.0);
      a_triplets.emplace_back(row_delta, ControlIndex(k - 1), -1.0);
      lower[row_delta] = -config_.delta_u_max;
      upper[row_delta] = config_.delta_u_max;
    }
  }

  for (int k = 1; k <= Np; ++k) {
    const int row_state = row++;
    a_triplets.emplace_back(row_state, StateIndex(k), 1.0);
    lower[row_state] = config_.v_min;
    upper[row_state] = config_.v_max;
  }

  constraint.setFromTriplets(a_triplets.begin(), a_triplets.end());

  solver_.clearSolver();
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setVerbosity(false);

  solver_.data()->setNumberOfVariables(n_vars);
  solver_.data()->setNumberOfConstraints(total_constraints);

  solver_.data()->clearHessianMatrix();
  solver_.data()->clearLinearConstraintsMatrix();

  bool data_ok = solver_.data()->setHessianMatrix(hessian);
  data_ok = data_ok && solver_.data()->setGradient(gradient);
  data_ok = data_ok && solver_.data()->setLinearConstraintsMatrix(constraint);
  data_ok = data_ok && solver_.data()->setLowerBound(lower);
  data_ok = data_ok && solver_.data()->setUpperBound(upper);

  if (!data_ok) {
    solution.report.status = "data_error";
    return solution;
  }

  if (!solver_.initSolver()) {
    solution.report.status = "init_failed";
    return solution;
  }

  const auto exit_flag = solver_.solveProblem();
  solution.report.status = StatusToString(solver_.getStatus());
  solution.report.cost = solver_.getObjValue();
  solution.report.iterations = 0;
  solution.report.solve_time_ms = 0.0;
#ifdef OSQP_EIGEN_OSQP_IS_V1
  const auto& solver_ptr = solver_.solver();
  if (solver_ptr && solver_ptr->info) {
    solution.report.iterations = static_cast<int>(solver_ptr->info->iter);
    solution.report.solve_time_ms = static_cast<double>(solver_ptr->info->run_time) * 1000.0;
  }
#else
  const auto& workspace_ptr = solver_.workspace();
  if (workspace_ptr && workspace_ptr->info) {
    solution.report.iterations = static_cast<int>(workspace_ptr->info->iter);
    solution.report.solve_time_ms = static_cast<double>(workspace_ptr->info->run_time) * 1000.0;
  }
#endif
  solution.report.success = (exit_flag == OsqpEigen::ErrorExitFlag::NoError) &&
                            (solver_.getStatus() == OsqpEigen::Status::Solved ||
                             solver_.getStatus() == OsqpEigen::Status::SolvedInaccurate);

  if (!solution.report.success) {
    return solution;
  }

  const Eigen::VectorXd& qp_solution = solver_.getSolution();
  if (qp_solution.size() >= ControlIndex(0) + 1) {
    const double u0 = Clamp(qp_solution[ControlIndex(0)], config_.u_min, config_.u_max);
    solution.command.accel = Clamp(u0, 0.0, 1.0);
    solution.command.brake = Clamp(-u0, 0.0, 1.0);
    last_control_ = u0;
  }

  solution.velocity_prediction.clear();
  solution.velocity_prediction.reserve(Np + 1);
  for (int k = 0; k <= Np; ++k) {
    solution.velocity_prediction.push_back(qp_solution[StateIndex(k)]);
  }

  return solution;
}

void LongitudinalMpc::SetLowSpeedMode(bool enabled)
{
  low_speed_active_ = enabled;
}

void LongitudinalMpc::SetDeltaUMax(double value)
{
  config_.delta_u_max = value;
}

void LongitudinalMpc::BuildProblem()
{
  // Placeholder for future incremental updates.
}

void LongitudinalMpc::ResetSolver()
{
  solver_.clearSolver();
}

}  // namespace mpc_controller
