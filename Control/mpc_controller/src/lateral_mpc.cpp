#include "mpc_controller/lateral_mpc.hpp"

#include <algorithm>
#include <cctype>
#include <limits>

#include <ros/ros.h>

#include "mpc_controller/utils.hpp"

namespace mpc_controller {
namespace {

bool LoadVectorParam(const ros::NodeHandle& nh, const std::string& key, std::vector<double>& value)
{
  if (!nh.hasParam(key)) {
    return false;
  }
  std::vector<double> tmp;
  if (nh.getParam(key, tmp) && !tmp.empty()) {
    value = tmp;
    return true;
  }
  return false;
}

template <typename T>
bool GetParamWithFallback(const ros::NodeHandle& primary,
                          const ros::NodeHandle& fallback,
                          const std::string& primary_key,
                          const std::string& fallback_key,
                          T& value)
{
  if (primary.getParam(primary_key, value)) {
    return true;
  }
  return fallback.getParam(fallback_key, value);
}

template <typename T>
bool GetParamWithFallback(const ros::NodeHandle& primary,
                          const ros::NodeHandle& fallback,
                          const std::string& key,
                          T& value)
{
  return GetParamWithFallback(primary, fallback, key, key, value);
}

bool LoadVectorParamWithFallback(const ros::NodeHandle& primary,
                                 const ros::NodeHandle& fallback,
                                 const std::string& primary_key,
                                 const std::string& fallback_key,
                                 std::vector<double>& value)
{
  if (LoadVectorParam(primary, primary_key, value)) {
    return true;
  }
  return LoadVectorParam(fallback, fallback_key, value);
}

bool LoadVectorParamWithFallback(const ros::NodeHandle& primary,
                                 const ros::NodeHandle& fallback,
                                 const std::string& key,
                                 std::vector<double>& value)
{
  return LoadVectorParamWithFallback(primary, fallback, key, key, value);
}

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

LateralMpc::LateralMpc()
{
  last_horizon_update_ = ros::Time::now();
}

bool LateralMpc::Configure(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
{
  (void)nh;
  ros::NodeHandle config_nh(private_nh, "lateral_mpc");
  ros::NodeHandle fallback_nh(private_nh, "mpc");
  ros::NodeHandle model_nh(private_nh, "model");

  GetParamWithFallback(config_nh, private_nh, "sample_time", config_.sample_time);
  GetParamWithFallback(config_nh, fallback_nh, "prediction_horizon", config_.prediction_horizon);
  GetParamWithFallback(config_nh, fallback_nh, "control_horizon", config_.control_horizon);
  GetParamWithFallback(config_nh, fallback_nh, "preview_distance", config_.preview_distance);
  GetParamWithFallback(config_nh, model_nh, "wheelbase", config_.wheelbase);
  if (!GetParamWithFallback(config_nh, fallback_nh, "r_weight", config_.r_weight)) {
    GetParamWithFallback(config_nh, fallback_nh, "R", config_.r_weight);
  }
  if (!GetParamWithFallback(config_nh, fallback_nh, "r_delta_weight", config_.r_delta_weight)) {
    GetParamWithFallback(config_nh, fallback_nh, "R_delta", config_.r_delta_weight);
  }
  std::vector<double> delta_limits;
  if (LoadVectorParamWithFallback(config_nh, fallback_nh, "delta_limits", delta_limits) && delta_limits.size() >= 2) {
    config_.delta_min = delta_limits.front();
    config_.delta_max = delta_limits.back();
  }
  GetParamWithFallback(config_nh, fallback_nh, "delta_min", config_.delta_min);
  GetParamWithFallback(config_nh, fallback_nh, "delta_max", config_.delta_max);
  if (!GetParamWithFallback(config_nh, fallback_nh, "delta_rate_limit", config_.delta_rate_limit)) {
    GetParamWithFallback(config_nh, fallback_nh, "delta_rate_max", config_.delta_rate_limit);
  }
  if (!GetParamWithFallback(config_nh, fallback_nh, "enable_band", config_.enable_band)) {
    GetParamWithFallback(config_nh, fallback_nh, "band_enable", config_.enable_band);
  }

  std::vector<double> qp_weights;
  if (!LoadVectorParamWithFallback(config_nh, fallback_nh, "q_weights", qp_weights)) {
    LoadVectorParamWithFallback(config_nh, fallback_nh, "Q_kinematic", qp_weights);
  }
  if (!qp_weights.empty()) {
    config_.q_weights = qp_weights;
  }
  std::vector<double> pp_weights;
  if (!LoadVectorParamWithFallback(config_nh, fallback_nh, "p_weights", pp_weights)) {
    LoadVectorParamWithFallback(config_nh, fallback_nh, "P_kinematic", pp_weights);
  }
  if (!pp_weights.empty()) {
    config_.p_weights = pp_weights;
  }

  ros::NodeHandle dyn_nh(config_nh, "dynamic_horizon");
  GetParamWithFallback(dyn_nh, fallback_nh, "enable", "horizon_dynamic_enable", config_.dynamic_horizon.enable);
  GetParamWithFallback(dyn_nh, fallback_nh, "min_steps", "horizon_min_steps", config_.dynamic_horizon.min_steps);
  GetParamWithFallback(dyn_nh, fallback_nh, "max_steps", "horizon_max_steps", config_.dynamic_horizon.max_steps);
  GetParamWithFallback(dyn_nh, fallback_nh, "quantize_steps", "horizon_quantize_steps", config_.dynamic_horizon.quantize_steps);
  if (!GetParamWithFallback(dyn_nh, fallback_nh, "change_threshold", config_.dynamic_horizon.change_threshold)) {
    GetParamWithFallback(config_nh, fallback_nh, "horizon_change_threshold_steps", config_.dynamic_horizon.change_threshold);
  }
  if (!GetParamWithFallback(dyn_nh, fallback_nh, "cooldown_sec", config_.dynamic_horizon.cooldown_sec)) {
    GetParamWithFallback(config_nh, fallback_nh, "horizon_cooldown_sec", config_.dynamic_horizon.cooldown_sec);
  }
  if (!GetParamWithFallback(dyn_nh, fallback_nh, "roi_steps", config_.dynamic_horizon.roi_steps)) {
    GetParamWithFallback(config_nh, fallback_nh, "horizon_roi_steps", config_.dynamic_horizon.roi_steps);
  }
  if (!GetParamWithFallback(dyn_nh, fallback_nh, "kappa_agg", config_.dynamic_horizon.kappa_agg)) {
    GetParamWithFallback(config_nh, fallback_nh, "horizon_kappa_agg", config_.dynamic_horizon.kappa_agg);
  }
  if (!GetParamWithFallback(dyn_nh, fallback_nh, "kappa_low", config_.dynamic_horizon.kappa_low)) {
    GetParamWithFallback(config_nh, fallback_nh, "horizon_kappa_low", config_.dynamic_horizon.kappa_low);
  }
  if (!GetParamWithFallback(dyn_nh, fallback_nh, "kappa_high", config_.dynamic_horizon.kappa_high)) {
    GetParamWithFallback(config_nh, fallback_nh, "horizon_kappa_high", config_.dynamic_horizon.kappa_high);
  }
  std::transform(config_.dynamic_horizon.kappa_agg.begin(),
                 config_.dynamic_horizon.kappa_agg.end(),
                 config_.dynamic_horizon.kappa_agg.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  config_.prediction_horizon = std::max(1, config_.prediction_horizon);
  config_.control_horizon = std::min(std::max(1, config_.control_horizon), config_.prediction_horizon);

  dynamic_config_ = config_.dynamic_horizon;
  active_prediction_horizon_ = config_.prediction_horizon;
  active_control_horizon_ = config_.control_horizon;

  configured_ = true;
  return true;
}

Eigen::Vector2d LateralMpc::ComputeError(const VehicleState& state,
                                         const PathPoint& reference_point) const
{
  const double dx = state.x - reference_point.x;
  const double dy = state.y - reference_point.y;
  const double ref_heading = reference_point.heading;
  const double sin_h = std::sin(ref_heading);
  const double cos_h = std::cos(ref_heading);

  Eigen::Vector2d error;
  error[0] = -sin_h * dx + cos_h * dy;  // lateral error
  error[1] = NormalizeAngle(state.heading - ref_heading);
  return error;
}

Eigen::Matrix2d LateralMpc::ComputeAd(double vx) const
{
  const double Ts = config_.sample_time;
  Eigen::Matrix2d Ad = Eigen::Matrix2d::Identity();
  Ad(0, 1) = Ts * vx;
  return Ad;
}

Eigen::Vector2d LateralMpc::ComputeBd(double vx) const
{
  const double Ts = config_.sample_time;
  const double L = std::max(1e-3, config_.wheelbase);
  Eigen::Vector2d Bd;
  Bd[0] = 0.0;
  Bd[1] = Ts * vx / L;
  return Bd;
}

Eigen::Vector2d LateralMpc::ComputeEd(double vx) const
{
  const double Ts = config_.sample_time;
  Eigen::Vector2d Ed;
  Ed[0] = 0.0;
  Ed[1] = -Ts * vx;
  return Ed;
}

LateralSolution LateralMpc::Solve(const VehicleState& state,
                                  const PathManager& path_manager,
                                  int reference_index,
                                  const SpeedProfileBuffer* /*speed_profile*/)
{
  LateralSolution solution;
  solution.command.steering = last_control_;
  solution.command.accel = 0.0;
  solution.command.brake = 0.0;
  solution.report.success = false;
  solution.report.status = "not_configured";

  if (!configured_) {
    return solution;
  }
  if (!path_manager.HasPath()) {
    solution.report.status = "no_path";
    return solution;
  }

  const std::size_t path_size = path_manager.Size();
  const int max_index = path_size > 0 ? static_cast<int>(path_size - 1) : 0;
  const int clamped_index = std::clamp(reference_index, 0, max_index);

  const double vx = std::copysign(std::max(0.1, std::abs(state.vx)), state.vx);

  // Dynamic horizon selection
  if (dynamic_config_.enable && path_size > 0) {
    const int roi_start = clamped_index;
    const int roi_extent = std::max(1, dynamic_config_.roi_steps);
    const int roi_end = std::clamp(roi_start + roi_extent, 0, max_index);
    const double metric = path_manager.CurvatureMetric(static_cast<std::size_t>(roi_start),
                                                       static_cast<std::size_t>(roi_end),
                                                       dynamic_config_.kappa_agg);

    const double k_low = dynamic_config_.kappa_low;
    const double k_high = std::max(dynamic_config_.kappa_high, k_low + 1e-6);
    double blend = 0.0;
    if (metric <= k_low) {
      blend = 0.0;
    } else if (metric >= k_high) {
      blend = 1.0;
    } else {
      blend = (metric - k_low) / (k_high - k_low);
    }

    const int span = std::max(0, dynamic_config_.max_steps - dynamic_config_.min_steps);
    int raw_target = dynamic_config_.max_steps - static_cast<int>(std::round(blend * span));
    raw_target = std::clamp(raw_target, dynamic_config_.min_steps, dynamic_config_.max_steps);
    const int q = std::max(1, dynamic_config_.quantize_steps);
    int quantized_target = ((raw_target + q / 2) / q) * q;
    quantized_target = std::clamp(quantized_target, dynamic_config_.min_steps, dynamic_config_.max_steps);

    if (active_prediction_horizon_ <= 0) {
      active_prediction_horizon_ = config_.prediction_horizon;
    }

    const ros::Time now = ros::Time::now();
    const double elapsed = (now - last_horizon_update_).toSec();
    if (elapsed >= dynamic_config_.cooldown_sec &&
        std::abs(quantized_target - active_prediction_horizon_) >= dynamic_config_.change_threshold) {
      active_prediction_horizon_ = quantized_target;
      last_horizon_update_ = now;
    }

    last_roi_metric_ = metric;
    last_roi_mode_ = dynamic_config_.kappa_agg;
    last_roi_start_index_ = roi_start;
    last_roi_end_index_ = roi_end;
  } else {
    active_prediction_horizon_ = config_.prediction_horizon;
    last_roi_metric_ = 0.0;
    last_roi_mode_ = "disabled";
    last_roi_start_index_ = clamped_index;
    last_roi_end_index_ = clamped_index;
  }

  active_prediction_horizon_ = std::max(1, active_prediction_horizon_);
  active_control_horizon_ = std::min(config_.control_horizon, active_prediction_horizon_);
  active_control_horizon_ = std::max(1, active_control_horizon_);

  const int Np = active_prediction_horizon_;
  const int Nc = active_control_horizon_;
  solution.horizon_steps = active_prediction_horizon_;
  solution.roi_curvature_metric = last_roi_metric_;
  solution.roi_curvature_mode = last_roi_mode_;
  solution.roi_start_index = last_roi_start_index_;
  solution.roi_end_index = last_roi_end_index_;
  const int n_state_vars = static_cast<int>(nx_) * (Np + 1);
  const int n_control_vars = Nc;
  const int n_vars = n_state_vars + n_control_vars;

  const Eigen::Vector2d x0 =
      ComputeError(state, path_manager.Point(static_cast<std::size_t>(clamped_index)));

  std::vector<double> curvature_seq(Np, 0.0);
  for (int k = 0; k < Np; ++k) {
    const std::size_t idx = static_cast<std::size_t>(std::clamp(clamped_index + k, 0, max_index));
    curvature_seq[k] = path_manager.Point(idx).curvature;
  }

  const Eigen::Matrix2d Ad = ComputeAd(vx);
  const Eigen::Vector2d Bd = ComputeBd(vx);
  const Eigen::Vector2d Ed = ComputeEd(vx);

  const double q_lat = config_.q_weights.size() > 0 ? config_.q_weights[0] : 200.0;
  const double q_heading = config_.q_weights.size() > 1 ? config_.q_weights[1] : 250.0;
  const double p_lat = config_.p_weights.size() > 0 ? config_.p_weights[0] : q_lat * 10.0;
  const double p_heading = config_.p_weights.size() > 1 ? config_.p_weights[1] : q_heading * 10.0;

  std::vector<Eigen::Triplet<double>> h_triplets;
  h_triplets.reserve(n_vars * 3);

  for (int k = 0; k < Np; ++k) {
    const int idx_lat = StateIndex(k, 0);
    const int idx_heading = StateIndex(k, 1);
    h_triplets.emplace_back(idx_lat, idx_lat, 2.0 * q_lat);
    h_triplets.emplace_back(idx_heading, idx_heading, 2.0 * q_heading);
  }

  const int terminal_lat = StateIndex(Np, 0);
  const int terminal_heading = StateIndex(Np, 1);
  h_triplets.emplace_back(terminal_lat, terminal_lat, 2.0 * p_lat);
  h_triplets.emplace_back(terminal_heading, terminal_heading, 2.0 * p_heading);

  for (int k = 0; k < Nc; ++k) {
    const int idx_u = ControlIndex(k);
    h_triplets.emplace_back(idx_u, idx_u, 2.0 * config_.r_weight);
  }

  // Input smoothness cost (finite difference)
  const double rd = config_.r_delta_weight;
  if (rd > 1e-6) {
    for (int k = 1; k < Nc; ++k) {
      const int idx_u = ControlIndex(k);
      const int idx_prev = ControlIndex(k - 1);
      h_triplets.emplace_back(idx_u, idx_u, 2.0 * rd);
      h_triplets.emplace_back(idx_prev, idx_prev, 2.0 * rd);
      h_triplets.emplace_back(idx_u, idx_prev, -2.0 * rd);
      h_triplets.emplace_back(idx_prev, idx_u, -2.0 * rd);
    }
    if (Nc > 0) {
      const int idx0 = ControlIndex(0);
      h_triplets.emplace_back(idx0, idx0, 2.0 * rd);
    }
  }

  Eigen::SparseMatrix<double> hessian(n_vars, n_vars);
  hessian.setFromTriplets(h_triplets.begin(), h_triplets.end());

  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_vars);
  if (rd > 1e-6 && Nc > 0) {
    gradient[ControlIndex(0)] += -2.0 * rd * last_control_;
  }

  const int eq_rows = static_cast<int>(nx_) + Np * static_cast<int>(nx_);
  const int ineq_rows = 2 * Nc;
  const int total_constraints = eq_rows + ineq_rows;

  Eigen::SparseMatrix<double> constraint(total_constraints, n_vars);
  std::vector<Eigen::Triplet<double>> a_triplets;
  a_triplets.reserve(total_constraints * 5);
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(total_constraints);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(total_constraints);

  int row = 0;
  for (int i = 0; i < static_cast<int>(nx_); ++i) {
    a_triplets.emplace_back(row, StateIndex(0, i), 1.0);
    lower[row] = x0[i];
    upper[row] = x0[i];
    ++row;
  }

  for (int k = 0; k < Np; ++k) {
    const int u_index = std::min(k, Nc - 1);
    const double curvature = curvature_seq[k];
    const Eigen::Vector2d rhs = Ed * curvature;

    for (int i = 0; i < static_cast<int>(nx_); ++i) {
      const int row_index = row + i;
      a_triplets.emplace_back(row_index, StateIndex(k + 1, i), 1.0);
    }

    for (int i = 0; i < static_cast<int>(nx_); ++i) {
      for (int j = 0; j < static_cast<int>(nx_); ++j) {
        const double value = -Ad(i, j);
        if (std::abs(value) > 1e-12) {
          a_triplets.emplace_back(row + i, StateIndex(k, j), value);
        }
      }
    }

    const double value_u_lat = -Bd[0];
    const double value_u_heading = -Bd[1];
    if (std::abs(value_u_lat) > 1e-12) {
      a_triplets.emplace_back(row + 0, ControlIndex(u_index), value_u_lat);
    }
    if (std::abs(value_u_heading) > 1e-12) {
      a_triplets.emplace_back(row + 1, ControlIndex(u_index), value_u_heading);
    }

    lower.segment(row, static_cast<int>(nx_)) = rhs;
    upper.segment(row, static_cast<int>(nx_)) = rhs;
    row += static_cast<int>(nx_);
  }

  for (int k = 0; k < Nc; ++k) {
    const int row_lower = row++;
    a_triplets.emplace_back(row_lower, ControlIndex(k), 1.0);
    lower[row_lower] = config_.delta_min;
    upper[row_lower] = config_.delta_max;

    const int row_rate = row++;
    if (k == 0) {
      a_triplets.emplace_back(row_rate, ControlIndex(k), 1.0);
      lower[row_rate] = last_control_ - config_.delta_rate_limit;
      upper[row_rate] = last_control_ + config_.delta_rate_limit;
    } else {
      a_triplets.emplace_back(row_rate, ControlIndex(k), 1.0);
      a_triplets.emplace_back(row_rate, ControlIndex(k - 1), -1.0);
      lower[row_rate] = -config_.delta_rate_limit;
      upper[row_rate] = config_.delta_rate_limit;
    }
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
    const double u0 = qp_solution[ControlIndex(0)];
    const double steering = Clamp(u0, config_.delta_min, config_.delta_max);
    solution.command.steering = steering;
    last_control_ = steering;
    solution.steering_sequence.clear();
    for (int k = 0; k < Nc; ++k) {
      solution.steering_sequence.push_back(qp_solution[ControlIndex(k)]);
    }
  }

  solution.predicted_lateral_errors.clear();
  solution.predicted_heading_errors.clear();
  solution.predicted_lateral_errors.reserve(Np + 1);
  solution.predicted_heading_errors.reserve(Np + 1);
  for (int k = 0; k <= Np; ++k) {
    solution.predicted_lateral_errors.push_back(qp_solution[StateIndex(k, 0)]);
    solution.predicted_heading_errors.push_back(qp_solution[StateIndex(k, 1)]);
  }

  return solution;
}

void LateralMpc::BuildProblem()
{
  // Placeholder for future incremental updates.
}

void LateralMpc::ResetSolver()
{
  solver_.clearSolver();
}

}  // namespace mpc_controller
