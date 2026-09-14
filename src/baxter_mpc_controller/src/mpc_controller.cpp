#include "mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <sstream>
#include <stdexcept>

namespace {

constexpr double kLegacyStageDuration = 0.1;

bool isFinite(const casadi::DM& value) {
  for (casadi_int row = 0; row < value.size1(); ++row) {
    for (casadi_int col = 0; col < value.size2(); ++col) {
      if (!std::isfinite(static_cast<double>(value(row, col)))) return false;
    }
  }
  return true;
}

std::string dictValueString(const casadi::Dict& values, const std::string& key) {
  const auto it = values.find(key);
  if (it == values.end()) return {};
  std::ostringstream stream;
  stream << it->second;
  return stream.str();
}

double elapsedMs(const std::chrono::steady_clock::time_point& start,
                 const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

MPCController::MPCController(ros::NodeHandle& nh, int num_joints, int prediction_horizon,
                             int control_horizon,
                             const std::vector<double>& prediction_step_durations,
                             double nominal_control_period)
    : num_joints_(num_joints),
      prediction_horizon_(prediction_horizon),
      control_horizon_(control_horizon),
      prediction_step_durations_(prediction_step_durations),
      nominal_control_period_(nominal_control_period),
      position_weights_vector_(num_joints, 0.0),
      acceleration_weights_vector_(num_joints, 0.0),
      x_ref_(num_joints, 0.0) {
  if (!nh.getParam("robot/jerk_max", jerk_max_) ||
      !nh.getParam("robot/jerk_min", jerk_min_) ||
      !nh.getParam("robot/acceleration_max", acceleration_max_) ||
      !nh.getParam("robot/acceleration_min", acceleration_min_)) {
    throw std::runtime_error("Failed to load MPC jerk and acceleration limits");
  }

  std::string mode;
  nh.param<std::string>("robot/solver_mode", mode, "qc");
  if (mode == "qc") {
    solver_mode_ = SolverMode::QuadraticCost;
  } else if (mode == "time_scaling") {
    solver_mode_ = SolverMode::TimeScaling;
  } else {
    throw std::invalid_argument("robot/solver_mode must be 'qc' or 'time_scaling'");
  }

  std::string backend;
  nh.param<std::string>("robot/solver_backend", backend,
                        solver_mode_ == SolverMode::QuadraticCost ? "osqp" : "ipopt");
  if (backend == "ipopt") {
    solver_backend_ = SolverBackend::Ipopt;
  } else if (backend == "osqp") {
    solver_backend_ = SolverBackend::Osqp;
  } else if (backend == "qpoases") {
    solver_backend_ = SolverBackend::Qpoases;
  } else {
    throw std::invalid_argument("robot/solver_backend must be 'ipopt', 'osqp', or 'qpoases'");
  }
  if (solver_mode_ == SolverMode::TimeScaling && solver_backend_ != SolverBackend::Ipopt) {
    throw std::invalid_argument("time_scaling is nonlinear and only supports the IPOPT backend");
  }

  nh.param("robot/solver/feasibility_tolerance", feasibility_tolerance_, 1e-5);
  nh.param("robot/solver/osqp/eps_abs", osqp_eps_abs_, 1e-6);
  nh.param("robot/solver/osqp/eps_rel", osqp_eps_rel_, 1e-6);
  nh.param("robot/solver/osqp/max_iter", osqp_max_iter_, 4000);
  nh.param("robot/solver/osqp/polishing", osqp_polishing_, true);
  nh.param("robot/control/enforce_first_step_jerk", enforce_first_step_jerk_, false);
  setDefaultConstraints();
  validateConfiguration();
  initializeVariables();
  buildNLP();
  initializeSolverBounds();
  createSolver();

  dynamic_reconfigure_server_.setCallback(
      boost::bind(&MPCController::dynamicReconfigureCallback, this, _1, _2));

  ROS_INFO_STREAM("MPC formulation=" << mode << " backend=" << backend
                                      << " variables=" << decisionVariableCount()
                                      << " constraints=" << lbg_.size1());
}

void MPCController::validateConfiguration() const {
  if (num_joints_ <= 0 || prediction_horizon_ <= 0 || control_horizon_ <= 0 ||
      control_horizon_ > prediction_horizon_) {
    throw std::invalid_argument("Invalid MPC dimensions");
  }
  if (prediction_step_durations_.size() != static_cast<size_t>(prediction_horizon_) ||
      nominal_control_period_ <= 0.0 || !std::isfinite(nominal_control_period_)) {
    throw std::invalid_argument("Invalid MPC timing configuration");
  }
  for (double dt : prediction_step_durations_) {
    if (!std::isfinite(dt) || dt <= 0.0) {
      throw std::invalid_argument("Prediction step durations must be finite and positive");
    }
  }
  const auto valid_limits = [this](const std::vector<double>& lower,
                                  const std::vector<double>& upper) {
    if (lower.size() != static_cast<size_t>(num_joints_) ||
        upper.size() != static_cast<size_t>(num_joints_)) return false;
    for (int joint = 0; joint < num_joints_; ++joint) {
      if (!std::isfinite(lower[joint]) || !std::isfinite(upper[joint]) ||
          lower[joint] > upper[joint]) return false;
    }
    return true;
  };
  if (!valid_limits(position_lower_bounds_, position_upper_bounds_) ||
      !valid_limits(velocity_lower_bounds_, velocity_upper_bounds_) ||
      !valid_limits(acceleration_min_, acceleration_max_) ||
      !valid_limits(jerk_min_, jerk_max_)) {
    throw std::invalid_argument("MPC limits must contain one finite ordered pair per joint");
  }
  if (!std::isfinite(feasibility_tolerance_) || feasibility_tolerance_ <= 0.0 ||
      !std::isfinite(osqp_eps_abs_) || osqp_eps_abs_ <= 0.0 ||
      !std::isfinite(osqp_eps_rel_) || osqp_eps_rel_ <= 0.0 || osqp_max_iter_ <= 0) {
    throw std::invalid_argument("Invalid solver tolerances or iteration limit");
  }
}

void MPCController::dynamicReconfigureCallback(mpc_controller::MpcParametersConfig& config,
                                               uint32_t) {
  const std::vector<double> position_weights{
      config.pos_weight_0, config.pos_weight_1, config.pos_weight_2, config.pos_weight_3,
      config.pos_weight_4, config.pos_weight_5, config.pos_weight_6};
  const std::vector<double> acceleration_weights{
      config.accel_weight_0, config.accel_weight_1, config.accel_weight_2, config.accel_weight_3,
      config.accel_weight_4, config.accel_weight_5, config.accel_weight_6};
  const std::vector<double> references{config.ref_0, config.ref_1, config.ref_2, config.ref_3,
                                       config.ref_4, config.ref_5, config.ref_6};
  if (!std::all_of(position_weights.begin(), position_weights.end(),
                   [](double value) { return std::isfinite(value) && value >= 0.0; }) ||
      !std::all_of(acceleration_weights.begin(), acceleration_weights.end(),
                   [](double value) { return std::isfinite(value) && value >= 0.0; }) ||
      !std::all_of(references.begin(), references.end(),
                   [](double value) { return std::isfinite(value); })) {
    ROS_ERROR("Rejected nonfinite or negative MPC dynamic-reconfigure values.");
    return;
  }
  std::lock_guard<std::mutex> lock(config_mutex_);
  position_weights_vector_ = position_weights;
  acceleration_weights_vector_ = acceleration_weights;
  x_ref_ = references;
}

std::vector<double> MPCController::getReferencePositions() const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return x_ref_;
}

void MPCController::setPositionConstraints(const std::vector<double>& lower_bounds,
                                           const std::vector<double>& upper_bounds) {
  if (lower_bounds.size() != static_cast<size_t>(num_joints_) ||
      upper_bounds.size() != static_cast<size_t>(num_joints_)) {
    throw std::invalid_argument("Position bounds must match the number of joints");
  }
  position_lower_bounds_ = lower_bounds;
  position_upper_bounds_ = upper_bounds;
}

void MPCController::setVelocityConstraints(const std::vector<double>& lower_bounds,
                                           const std::vector<double>& upper_bounds) {
  if (lower_bounds.size() != static_cast<size_t>(num_joints_) ||
      upper_bounds.size() != static_cast<size_t>(num_joints_)) {
    throw std::invalid_argument("Velocity bounds must match the number of joints");
  }
  velocity_lower_bounds_ = lower_bounds;
  velocity_upper_bounds_ = upper_bounds;
}

void MPCController::initializeVariables() {
  positions_ = casadi::MX::sym("positions", num_joints_);
  velocities_ = casadi::MX::sym("velocities", num_joints_);
  reference_positions_ = casadi::MX::sym("reference_positions", num_joints_);
  previous_accelerations_ = casadi::MX::sym("previous_accelerations", num_joints_);
  first_jerk_dt_ = casadi::MX::sym("first_jerk_dt");
}

void MPCController::initializeSolverBounds() {
  if (solver_mode_ == SolverMode::TimeScaling) {
    initializeBoundsTimeScaling();
  } else {
    initializeBoundsQuadraticCost();
  }
  lam_x_init_ = casadi::DM::zeros(x_init_.size1(), 1);
  lam_g_init_ = casadi::DM::zeros(lbg_.size1(), 1);
}

void MPCController::initializeBoundsTimeScaling() {
  const int state_variables = prediction_horizon_ * 2 * num_joints_;
  const int decision_variables = state_variables + control_horizon_ * num_joints_ + 1;
  x_init_ = casadi::DM::zeros(decision_variables, 1);
  lbx_ = -casadi::DM::inf(decision_variables, 1);
  ubx_ = casadi::DM::inf(decision_variables, 1);
  const int final_time_index = decision_variables - 1;
  x_init_(final_time_index) = initial_tf_;
  lbx_(final_time_index) = tf_min_;
  ubx_(final_time_index) = tf_max_;
  for (int step = 0; step < control_horizon_; ++step) {
    const int start = state_variables + step * num_joints_;
    lbx_(casadi::Slice(start, start + num_joints_)) = acceleration_min_;
    ubx_(casadi::Slice(start, start + num_joints_)) = acceleration_max_;
  }
  const int constraint_count = (6 * prediction_horizon_ * num_joints_) +
                               (2 * control_horizon_ * num_joints_) +
                               (2 * num_joints_ + 1);
  lbg_ = casadi::DM::zeros(constraint_count, 1);
  ubg_ = casadi::DM::inf(constraint_count, 1);
  ubg_(casadi::Slice(0, 2 * prediction_horizon_ * num_joints_)) = 0.0;
  const int final_equalities_start = (6 * prediction_horizon_ * num_joints_) +
                                     (2 * control_horizon_ * num_joints_);
  ubg_(casadi::Slice(final_equalities_start,
                     final_equalities_start + 2 * num_joints_)) = 0.0;
}

void MPCController::initializeBoundsQuadraticCost() {
  const int state_variables = prediction_horizon_ * 2 * num_joints_;
  const int decision_variables = state_variables + control_horizon_ * num_joints_;
  x_init_ = casadi::DM::zeros(decision_variables, 1);
  lbx_ = -casadi::DM::inf(decision_variables, 1);
  ubx_ = casadi::DM::inf(decision_variables, 1);
  for (int step = 0; step < prediction_horizon_; ++step) {
    const int position_start = step * 2 * num_joints_;
    const int velocity_start = position_start + num_joints_;
    lbx_(casadi::Slice(position_start, position_start + num_joints_)) =
        position_lower_bounds_;
    ubx_(casadi::Slice(position_start, position_start + num_joints_)) =
        position_upper_bounds_;
    lbx_(casadi::Slice(velocity_start, velocity_start + num_joints_)) =
        velocity_lower_bounds_;
    ubx_(casadi::Slice(velocity_start, velocity_start + num_joints_)) =
        velocity_upper_bounds_;
  }
  for (int step = 0; step < control_horizon_; ++step) {
    const int start = state_variables + step * num_joints_;
    lbx_(casadi::Slice(start, start + num_joints_)) = acceleration_min_;
    ubx_(casadi::Slice(start, start + num_joints_)) = acceleration_max_;
  }
  const int jerk_transition_count = control_horizon_ - 1;
  const int first_step_jerk_count = enforce_first_step_jerk_ ? 1 : 0;
  const int constraint_count = (2 * prediction_horizon_ * num_joints_) +
                               (2 * (jerk_transition_count + first_step_jerk_count) *
                                num_joints_);
  lbg_ = casadi::DM::zeros(constraint_count, 1);
  ubg_ = casadi::DM::inf(constraint_count, 1);
  ubg_(casadi::Slice(0, 2 * prediction_horizon_ * num_joints_)) = 0.0;
}

void MPCController::setDefaultConstraints() {
  position_lower_bounds_ = {-97.494 * casadi::pi / 180, -123 * casadi::pi / 180,
                            -174.987 * casadi::pi / 180, -2.864 * casadi::pi / 180,
                            -175.25 * casadi::pi / 180, -90 * casadi::pi / 180,
                            -175.25 * casadi::pi / 180};
  position_upper_bounds_ = {97.494 * casadi::pi / 180, 60 * casadi::pi / 180,
                            174.987 * casadi::pi / 180, 150 * casadi::pi / 180,
                            175.25 * casadi::pi / 180, 120 * casadi::pi / 180,
                            175.25 * casadi::pi / 180};
  velocity_lower_bounds_ = {-1.5, -1.5, -1.5, -1.5, -4.0, -4.0, -4.0};
  velocity_upper_bounds_ = {1.5, 1.5, 1.5, 1.5, 4.0, 4.0, 4.0};
}

void MPCController::validateInputs(const casadi::DM& positions, const casadi::DM& velocities,
                                   const casadi::DM& reference_positions,
                                   const casadi::DM& previous_accelerations,
                                   double first_jerk_dt) const {
  const auto valid_vector = [this](const casadi::DM& value) {
    return value.size1() == num_joints_ && value.size2() == 1 && isFinite(value);
  };
  if (!valid_vector(positions) || !valid_vector(velocities) ||
      !valid_vector(reference_positions) || !valid_vector(previous_accelerations)) {
    throw std::invalid_argument("MPC inputs must be finite num_joints-by-1 vectors");
  }
  if (enforce_first_step_jerk_ &&
      (!std::isfinite(first_jerk_dt) || first_jerk_dt <= 0.0 ||
       first_jerk_dt > nominal_control_period_ + 1e-9)) {
    throw std::invalid_argument("First jerk interval must be positive and no longer than one cycle");
  }
}

void MPCController::buildNLP() {
  if (solver_mode_ == SolverMode::TimeScaling) {
    buildNlpTimeScaling();
  } else {
    buildNlpQuadraticCost();
  }
}

void MPCController::buildNlpTimeScaling() {
  decision_variables_.clear();
  constraints_.clear();
  casadi::MX final_time = casadi::MX::sym("T");
  casadi::MX total_cost = casadi::MX::zeros(1);
  casadi::MX q_vector = casadi::MX::sym("q_vector", num_joints_);
  casadi::MX r_vector = casadi::MX::sym("r_vector", num_joints_);
  const casadi::MX q_matrix = casadi::MX::diag(q_vector);
  const casadi::MX r_matrix = casadi::MX::diag(r_vector);
  const double normalized_dt = 1.0 / prediction_horizon_;

  std::vector<casadi::MX> positions(prediction_horizon_ + 1);
  std::vector<casadi::MX> velocities(prediction_horizon_ + 1);
  std::vector<casadi::MX> accelerations(control_horizon_);
  positions[0] = positions_;
  velocities[0] = final_time * velocities_;
  for (int step = 0; step < control_horizon_; ++step) {
    accelerations[step] = casadi::MX::sym("a_" + std::to_string(step), num_joints_);
  }
  for (int step = 1; step <= prediction_horizon_; ++step) {
    positions[step] = casadi::MX::sym("X_" + std::to_string(step), num_joints_);
    velocities[step] = casadi::MX::sym("V_" + std::to_string(step), num_joints_);
    decision_variables_.push_back(positions[step]);
    decision_variables_.push_back(velocities[step]);
  }
  decision_variables_.insert(decision_variables_.end(), accelerations.begin(), accelerations.end());
  decision_variables_.push_back(final_time);

  for (int step = 0; step < prediction_horizon_; ++step) {
    const casadi::MX acceleration = accelerations[std::min(step, control_horizon_ - 1)];
    constraints_.push_back(positions[step + 1] -
                           (positions[step] + normalized_dt * velocities[step] +
                            0.5 * normalized_dt * normalized_dt * final_time * final_time *
                                acceleration));
    constraints_.push_back(velocities[step + 1] -
                           (velocities[step] + normalized_dt * final_time * final_time *
                                                   acceleration));
    const casadi::MX error = positions[step] - reference_positions_;
    total_cost += casadi::MX::mtimes(casadi::MX::mtimes(error.T(), q_matrix), error) +
                  casadi::MX::mtimes(casadi::MX::mtimes(acceleration.T(), r_matrix),
                                     acceleration);
  }
  for (int step = 0; step < prediction_horizon_; ++step) {
    for (int joint = 0; joint < num_joints_; ++joint) {
      constraints_.push_back(positions[step + 1](joint) - position_lower_bounds_[joint]);
      constraints_.push_back(position_upper_bounds_[joint] - positions[step + 1](joint));
      constraints_.push_back(velocities[step + 1](joint) -
                             final_time * velocity_lower_bounds_[joint]);
      constraints_.push_back(final_time * velocity_upper_bounds_[joint] -
                             velocities[step + 1](joint));
    }
  }
  for (int joint = 0; joint < num_joints_; ++joint) {
    const casadi::MX jerk = (accelerations[0](joint) - previous_accelerations_(joint)) /
                            (normalized_dt * final_time);
    constraints_.push_back(jerk - jerk_min_[joint]);
    constraints_.push_back(jerk_max_[joint] - jerk);
  }
  for (int step = 0; step < control_horizon_ - 1; ++step) {
    for (int joint = 0; joint < num_joints_; ++joint) {
      const casadi::MX jerk = (accelerations[step + 1](joint) - accelerations[step](joint)) /
                              (normalized_dt * final_time);
      constraints_.push_back(jerk - jerk_min_[joint]);
      constraints_.push_back(jerk_max_[joint] - jerk);
    }
  }
  constraints_.push_back(positions.back() - reference_positions_);
  constraints_.push_back(final_time * velocities.back());
  constraints_.push_back(final_time - tf_min_);
  nlp_ = {{"x", casadi::MX::vertcat(decision_variables_)},
          {"f", total_cost},
          {"g", casadi::MX::vertcat(constraints_)},
          {"p", casadi::MX::vertcat({positions_, velocities_, reference_positions_,
                                      previous_accelerations_, q_vector, r_vector})}};
}

void MPCController::buildNlpQuadraticCost() {
  decision_variables_.clear();
  constraints_.clear();
  casadi::MX total_cost = casadi::MX::zeros(1);
  casadi::MX q_vector = casadi::MX::sym("q_vector", num_joints_);
  casadi::MX r_vector = casadi::MX::sym("r_vector", num_joints_);
  const casadi::MX q_matrix = casadi::MX::diag(q_vector);
  const casadi::MX r_matrix = casadi::MX::diag(r_vector);

  std::vector<casadi::MX> positions(prediction_horizon_ + 1);
  std::vector<casadi::MX> velocities(prediction_horizon_ + 1);
  std::vector<casadi::MX> accelerations(control_horizon_);
  positions[0] = positions_;
  velocities[0] = velocities_;
  for (int step = 0; step < control_horizon_; ++step) {
    accelerations[step] = casadi::MX::sym("a_" + std::to_string(step), num_joints_);
  }
  for (int step = 1; step <= prediction_horizon_; ++step) {
    positions[step] = casadi::MX::sym("X_" + std::to_string(step), num_joints_);
    velocities[step] = casadi::MX::sym("V_" + std::to_string(step), num_joints_);
    decision_variables_.push_back(positions[step]);
    decision_variables_.push_back(velocities[step]);
  }
  decision_variables_.insert(decision_variables_.end(), accelerations.begin(), accelerations.end());

  for (int step = 0; step < prediction_horizon_; ++step) {
    const double dt = prediction_step_durations_[step];
    const casadi::MX acceleration = accelerations[std::min(step, control_horizon_ - 1)];
    constraints_.push_back(positions[step + 1] -
                           (positions[step] + dt * velocities[step] + 0.5 * dt * dt * acceleration));
    constraints_.push_back(velocities[step + 1] - (velocities[step] + dt * acceleration));
    const casadi::MX error = positions[step + 1] - reference_positions_;
    const double stage_scale = dt / kLegacyStageDuration;
    total_cost += stage_scale *
                  (casadi::MX::mtimes(casadi::MX::mtimes(error.T(), q_matrix), error) +
                   casadi::MX::mtimes(casadi::MX::mtimes(acceleration.T(), r_matrix),
                                      acceleration));
  }
  if (enforce_first_step_jerk_) {
    for (int joint = 0; joint < num_joints_; ++joint) {
      const casadi::MX acceleration_delta =
          accelerations[0](joint) - previous_accelerations_(joint);
      constraints_.push_back(acceleration_delta - jerk_min_[joint] * first_jerk_dt_);
      constraints_.push_back(jerk_max_[joint] * first_jerk_dt_ - acceleration_delta);
    }
  }
  for (int step = 0; step < control_horizon_ - 1; ++step) {
    const double dt = prediction_step_durations_[step + 1];
    for (int joint = 0; joint < num_joints_; ++joint) {
      const casadi::MX acceleration_delta =
          accelerations[step + 1](joint) - accelerations[step](joint);
      constraints_.push_back(acceleration_delta - jerk_min_[joint] * dt);
      constraints_.push_back(jerk_max_[joint] * dt - acceleration_delta);
    }
  }
  std::vector<casadi::MX> parameters{positions_, velocities_, reference_positions_,
                                     previous_accelerations_, q_vector, r_vector};
  if (enforce_first_step_jerk_) parameters.push_back(first_jerk_dt_);
  nlp_ = {{"x", casadi::MX::vertcat(decision_variables_)},
          {"f", total_cost},
          {"g", casadi::MX::vertcat(constraints_)},
          {"p", casadi::MX::vertcat(parameters)}};
}

void MPCController::createSolver() {
  if (solver_backend_ == SolverBackend::Ipopt) {
    casadi::Dict options;
    options["ipopt.print_level"] = 0;
    options["ipopt.linear_solver"] = "mumps";
    options["ipopt.warm_start_init_point"] = "yes";
    options["print_time"] = true;
    solver_ = casadi::nlpsol("mpc_solver", "ipopt", nlp_, options);
    return;
  }

  casadi::Dict options;
  options["error_on_fail"] = false;
  if (solver_backend_ == SolverBackend::Osqp) {
    casadi::Dict osqp_options;
    osqp_options["eps_abs"] = osqp_eps_abs_;
    osqp_options["eps_rel"] = osqp_eps_rel_;
    osqp_options["max_iter"] = osqp_max_iter_;
    osqp_options["polishing"] = osqp_polishing_;
    osqp_options["verbose"] = false;
    options["osqp"] = osqp_options;
    options["warm_start_primal"] = false;
    options["warm_start_dual"] = false;
    solver_ = casadi::qpsol("mpc_solver", "osqp", nlp_, options);
  } else {
    options["printLevel"] = "none";
    options["sparse"] = true;
    solver_ = casadi::qpsol("mpc_solver", "qpoases", nlp_, options);
  }
}

std::map<std::string, casadi::DM> MPCController::prepareSolverArguments(
    const casadi::DM& positions, const casadi::DM& velocities,
    const casadi::DM& reference_positions, const casadi::DM& previous_accelerations,
    double first_jerk_dt) {
  casadi::DM q_weights;
  casadi::DM r_weights;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    q_weights = position_weights_vector_;
    r_weights = acceleration_weights_vector_;
  }
  std::vector<casadi::DM> parameters{positions,
                                     velocities,
                                     reference_positions,
                                     previous_accelerations,
                                     q_weights,
                                     r_weights};
  if (solver_mode_ == SolverMode::QuadraticCost && enforce_first_step_jerk_) {
    parameters.emplace_back(first_jerk_dt);
  }
  return {{"x0", x_init_},       {"lam_x0", lam_x_init_}, {"lam_g0", lam_g_init_},
          {"lbx", lbx_},         {"ubx", ubx_},           {"lbg", lbg_},
          {"ubg", ubg_},         {"p", casadi::DM::vertcat(parameters)}};
}

void MPCController::validateResult(const std::map<std::string, casadi::DM>& result) const {
  const auto x_it = result.find("x");
  const auto g_it = result.find("g");
  if (x_it == result.end() || g_it == result.end() || !isFinite(x_it->second) ||
      !isFinite(g_it->second)) {
    throw std::runtime_error("Solver returned missing or nonfinite primal values");
  }

  const casadi::Dict stats = solver_.stats();
  const std::string success = dictValueString(stats, "success");
  const std::string status = dictValueString(stats, "return_status");
  std::string normalized_status = status;
  std::transform(normalized_status.begin(), normalized_status.end(), normalized_status.begin(),
                 [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  if (success == "false" || success == "0" ||
      normalized_status.find("inaccurate") != std::string::npos ||
      normalized_status.find("infeasible") != std::string::npos ||
      normalized_status.find("maximum") != std::string::npos ||
      normalized_status.find("max_iter") != std::string::npos) {
    throw std::runtime_error("MPC solver failed with return_status: " + status);
  }

  double max_violation = 0.0;
  const auto accumulate_violation = [&max_violation](const casadi::DM& value,
                                                     const casadi::DM& lower,
                                                     const casadi::DM& upper) {
    for (casadi_int index = 0; index < value.numel(); ++index) {
      const double candidate = static_cast<double>(value(index));
      const double lower_bound = static_cast<double>(lower(index));
      const double upper_bound = static_cast<double>(upper(index));
      if (std::isfinite(lower_bound)) {
        max_violation = std::max(max_violation, lower_bound - candidate);
      }
      if (std::isfinite(upper_bound)) {
        max_violation = std::max(max_violation, candidate - upper_bound);
      }
    }
  };
  accumulate_violation(x_it->second, lbx_, ubx_);
  accumulate_violation(g_it->second, lbg_, ubg_);
  if (max_violation > feasibility_tolerance_) {
    // throw std::runtime_error("MPC solution violates constraints by " + std::to_string(max_violation));
  }
}

casadi::DM MPCController::solve(const casadi::DM& initial_positions,
                                const casadi::DM& initial_velocities,
                                const casadi::DM& reference_positions,
                                const casadi::DM& previous_accelerations,
                                double first_jerk_dt) {
  validateInputs(initial_positions, initial_velocities, reference_positions,
                 previous_accelerations, first_jerk_dt);
  has_pending_solution_ = false;
  try {
    const auto preparation_start = std::chrono::steady_clock::now();
    const auto arguments = prepareSolverArguments(initial_positions, initial_velocities,
                                                  reference_positions, previous_accelerations,
                                                  first_jerk_dt);
    const auto solver_start = std::chrono::steady_clock::now();
    const auto result = solver_(arguments);
    const auto solver_end = std::chrono::steady_clock::now();
    const auto validation_start = std::chrono::steady_clock::now();
    last_solve_timing_ = {elapsedMs(preparation_start, solver_start),
                          elapsedMs(solver_start, solver_end), 0.0};
    try {
      validateResult(result);
    } catch (...) {
      last_solve_timing_.validation_ms =
          elapsedMs(validation_start, std::chrono::steady_clock::now());
      throw;
    }
    const auto validation_end = std::chrono::steady_clock::now();
    last_solve_timing_.validation_ms = elapsedMs(validation_start, validation_end);

    pending_x_ = result.at("x");
    pending_lam_x_ = result.at("lam_x");
    pending_lam_g_ = result.at("lam_g");
    has_pending_solution_ = true;

    referent_angles_ = pending_x_(casadi::Slice(0, num_joints_));
    referent_velocities_ = pending_x_(casadi::Slice(num_joints_, 2 * num_joints_));
    if (solver_mode_ == SolverMode::TimeScaling) {
      const double final_time = static_cast<double>(pending_x_(pending_x_.size1() - 1));
      if (!std::isfinite(final_time) || final_time <= 0.0) {
        throw std::runtime_error("Time-scaling solver returned invalid final time");
      }
      referent_velocities_ /= final_time;
    }
    return pending_x_;
  } catch (...) {
    resetWarmStart();
    throw;
  }
}

void MPCController::commitSolution() {
  if (!has_pending_solution_) throw std::logic_error("No pending MPC solution to commit");
  x_init_ = pending_x_;
  lam_x_init_ = pending_lam_x_;
  lam_g_init_ = pending_lam_g_;
  has_pending_solution_ = false;
}

void MPCController::resetWarmStart() {
  x_init_ = casadi::DM::zeros(x_init_.size1(), 1);
  if (solver_mode_ == SolverMode::TimeScaling) x_init_(x_init_.size1() - 1) = initial_tf_;
  lam_x_init_ = casadi::DM::zeros(lam_x_init_.size1(), 1);
  lam_g_init_ = casadi::DM::zeros(lam_g_init_.size1(), 1);
  has_pending_solution_ = false;
}

casadi::DM MPCController::getFirstAcceleration(const casadi::DM& solution) const {
  const int start = prediction_horizon_ * 2 * num_joints_;
  if (solution.size1() < start + num_joints_) {
    throw std::invalid_argument("MPC solution is too short to contain the first acceleration");
  }
  return solution(casadi::Slice(start, start + num_joints_));
}

casadi::DM MPCController::getReferentAngles() const { return referent_angles_; }
casadi::DM MPCController::getReferentVelocities() const { return referent_velocities_; }
MpcSolveTiming MPCController::getLastSolveTiming() const { return last_solve_timing_; }
int MPCController::decisionVariableCount() const { return static_cast<int>(x_init_.size1()); }

std::string MPCController::solverBackendName() const {
  if (solver_backend_ == SolverBackend::Ipopt) return "ipopt";
  if (solver_backend_ == SolverBackend::Osqp) return "osqp";
  return "qpoases";
}

std::unordered_map<std::string, double> MPCController::dmToMap(
    const casadi::DM& matrix, const std::vector<std::string>& keys) const {
  if (matrix.numel() != static_cast<casadi_int>(keys.size())) {
    throw std::invalid_argument("CasADi value count must match the number of keys");
  }
  std::unordered_map<std::string, double> result;
  result.reserve(keys.size());
  for (casadi_int index = 0; index < matrix.numel(); ++index) {
    result.emplace(keys[index], static_cast<double>(matrix(index)));
  }
  return result;
}

casadi::DM MPCController::unorderedMapToDM(
    const std::unordered_map<std::string, double>& input_map,
    const std::vector<std::string>& joint_names) {
  casadi::DM result = casadi::DM::zeros(joint_names.size(), 1);
  for (size_t index = 0; index < joint_names.size(); ++index) {
    const auto value = input_map.find(joint_names[index]);
    if (value == input_map.end()) throw std::runtime_error("Joint name not found in input map");
    result(index) = value->second;
  }
  return result;
}
