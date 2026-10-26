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
  } else if (backend == "qrqp") {
    solver_backend_ = SolverBackend::Qrqp;  // CasADi built-in sparse active-set QP
  } else if (backend == "ipqp") {
    solver_backend_ = SolverBackend::Ipqp;  // CasADi built-in interior-point QP
  } else {
    throw std::invalid_argument(
        "robot/solver_backend must be 'ipopt', 'osqp', 'qpoases', 'qrqp', or 'ipqp'");
  }
  if (solver_mode_ == SolverMode::TimeScaling && solver_backend_ != SolverBackend::Ipopt) {
    throw std::invalid_argument("time_scaling is nonlinear and only supports the IPOPT backend");
  }

  nh.param("robot/solver/feasibility_tolerance", feasibility_tolerance_, feasibility_tolerance_);
  nh.param("robot/solver/osqp/eps_abs", osqp_eps_abs_, osqp_eps_abs_);
  nh.param("robot/solver/osqp/eps_rel", osqp_eps_rel_, osqp_eps_rel_);
  nh.param("robot/solver/osqp/max_iter", osqp_max_iter_, osqp_max_iter_);
  nh.param("robot/solver/osqp/polishing", osqp_polishing_, osqp_polishing_);
  nh.param("robot/solver/osqp/warm_start", osqp_warm_start_, osqp_warm_start_);
  nh.param("robot/solver/soft_state_bounds", soft_state_bounds_, soft_state_bounds_);
  nh.param("robot/solver/slack_per_step", slack_per_step_, slack_per_step_);
  nh.param("robot/solver/slack_linear_weight", slack_linear_weight_, slack_linear_weight_);
  nh.param("robot/solver/slack_quadratic_weight", slack_quadratic_weight_,
           slack_quadratic_weight_);
  nh.param("robot/solver/velocity_weight_ratio", velocity_weight_ratio_, velocity_weight_ratio_);
  if (!std::isfinite(velocity_weight_ratio_) || velocity_weight_ratio_ < 0.0) {
    throw std::invalid_argument("robot/solver/velocity_weight_ratio must be finite and nonnegative");
  }
  nh.param("robot/control/enforce_first_step_jerk", enforce_first_step_jerk_, false);
  if (solver_mode_ == SolverMode::TimeScaling && soft_state_bounds_) {
    ROS_WARN("Soft state bounds are only implemented for the qc formulation; ignoring.");
    soft_state_bounds_ = false;
  }
  setDefaultConstraints();
  validateConfiguration();
  computeQcLayout();
  initializeVariables();
  buildNLP();
  initializeSolverBounds();
  createSolver();

  dynamic_reconfigure_server_.setCallback(
      boost::bind(&MPCController::dynamicReconfigureCallback, this, _1, _2));

  ROS_INFO_STREAM("MPC formulation=" << mode << " backend=" << backend
                                      << " variables=" << decisionVariableCount()
                                      << " constraints=" << lbg_.size1()
                                      << " soft_state_bounds="
                                      << (soft_state_bounds_ ? (slack_per_step_ ? "per-step" : "shared") : "off"));
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
  if (!std::isfinite(slack_linear_weight_) || slack_linear_weight_ < 0.0 ||
      !std::isfinite(slack_quadratic_weight_) || slack_quadratic_weight_ < 0.0 ||
      (soft_state_bounds_ && slack_linear_weight_ + slack_quadratic_weight_ <= 0.0)) {
    throw std::invalid_argument("Slack weights must be finite, nonnegative and not both zero");
  }
}

void MPCController::computeQcLayout() {
  QcLayout layout;
  layout.num_joints = num_joints_;
  layout.state_vars = prediction_horizon_ * 2 * num_joints_;
  layout.accel_vars = control_horizon_ * num_joints_;
  layout.bound_slack_blocks = soft_state_bounds_ ? (slack_per_step_ ? prediction_horizon_ : 1) : 0;
  layout.bound_slack_vars = layout.bound_slack_blocks * 2 * num_joints_;
  layout.accel_offset = layout.state_vars;
  layout.bound_slack_offset = layout.accel_offset + layout.accel_vars;
  layout.decision_vars = layout.bound_slack_offset + layout.bound_slack_vars;

  layout.dyn_rows = 2 * prediction_horizon_ * num_joints_;
  layout.first_jerk_rows = enforce_first_step_jerk_ ? 2 * num_joints_ : 0;
  layout.jerk_rows = 2 * (control_horizon_ - 1) * num_joints_;
  layout.bound_rows = soft_state_bounds_ ? 4 * prediction_horizon_ * num_joints_ : 0;
  layout.first_jerk_row_offset = layout.dyn_rows;
  layout.jerk_row_offset = layout.first_jerk_row_offset + layout.first_jerk_rows;
  layout.bound_row_offset = layout.jerk_row_offset + layout.jerk_rows;
  layout.constraint_rows = layout.bound_row_offset + layout.bound_rows;
  layout_ = layout;
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
  const QcLayout& layout = layout_;
  x_init_ = casadi::DM::zeros(layout.decision_vars, 1);
  lbx_ = -casadi::DM::inf(layout.decision_vars, 1);
  ubx_ = casadi::DM::inf(layout.decision_vars, 1);
  if (!soft_state_bounds_) {
    for (int step = 1; step <= prediction_horizon_; ++step) {
      const int position_start = layout.xOffset(step);
      const int velocity_start = layout.vOffset(step);
      lbx_(casadi::Slice(position_start, position_start + num_joints_)) =
          position_lower_bounds_;
      ubx_(casadi::Slice(position_start, position_start + num_joints_)) =
          position_upper_bounds_;
      lbx_(casadi::Slice(velocity_start, velocity_start + num_joints_)) =
          velocity_lower_bounds_;
      ubx_(casadi::Slice(velocity_start, velocity_start + num_joints_)) =
          velocity_upper_bounds_;
    }
  }
  for (int step = 0; step < control_horizon_; ++step) {
    const int start = layout.aOffset(step);
    lbx_(casadi::Slice(start, start + num_joints_)) = acceleration_min_;
    ubx_(casadi::Slice(start, start + num_joints_)) = acceleration_max_;
  }
  if (soft_state_bounds_) {
    lbx_(casadi::Slice(layout.bound_slack_offset,
                       layout.bound_slack_offset + layout.bound_slack_vars)) = 0.0;
  }
  lbg_ = casadi::DM::zeros(layout.constraint_rows, 1);
  ubg_ = casadi::DM::inf(layout.constraint_rows, 1);
  // Only the dynamics rows are equalities; every other row is "g >= 0".
  ubg_(casadi::Slice(0, layout.dyn_rows)) = 0.0;
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
  // 80 % (70 % on the wrists) of the URDF velocity limits (1.5 / 4.0 rad/s): Gazebo and ros_control
  // enforce the URDF limits by clamping the effort to one sign, which breaks the
  // computed-torque model if the MPC rides the limit itself.
  velocity_lower_bounds_ = {-1.2, -1.2, -1.2, -1.2, -2.8, -2.8, -2.8};
  velocity_upper_bounds_ = {1.2, 1.2, 1.2, 1.2, 2.8, 2.8, 2.8};
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
  // Variable and row order below must follow layout_ (see QcLayout); the
  // checkpoints verify each block boundary while the problem is built.
  const auto symbolic_size = [](const std::vector<casadi::MX>& blocks) {
    casadi_int size = 0;
    for (const casadi::MX& block : blocks) size += block.size1();
    return size;
  };
  const auto expect_offset = [&](casadi_int actual, int expected, const char* what) {
    if (actual != expected) {
      throw std::logic_error(std::string("QP layout mismatch at ") + what + ": built " +
                             std::to_string(actual) + ", layout expects " +
                             std::to_string(expected));
    }
  };
  casadi::MX total_cost = casadi::MX::zeros(1);
  casadi::MX q_vector = casadi::MX::sym("q_vector", num_joints_);
  casadi::MX r_vector = casadi::MX::sym("r_vector", num_joints_);
  const casadi::MX q_matrix = casadi::MX::diag(q_vector);
  const casadi::MX r_matrix = casadi::MX::diag(r_vector);

  std::vector<casadi::MX> positions(prediction_horizon_ + 1);
  std::vector<casadi::MX> velocities(prediction_horizon_ + 1);
  std::vector<casadi::MX> accelerations(control_horizon_);
  std::vector<casadi::MX> slacks;
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
  expect_offset(symbolic_size(decision_variables_), layout_.accel_offset, "acceleration variables");
  decision_variables_.insert(decision_variables_.end(), accelerations.begin(), accelerations.end());
  expect_offset(symbolic_size(decision_variables_), layout_.bound_slack_offset, "bound slack variables");
  if (soft_state_bounds_) {
    slacks.reserve(layout_.bound_slack_blocks);
    for (int index = 0; index < layout_.bound_slack_blocks; ++index) {
      slacks.push_back(casadi::MX::sym("S_" + std::to_string(index), 2 * num_joints_));
      decision_variables_.push_back(slacks.back());
    }
  }

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
    if (velocity_weight_ratio_ > 0.0) {
      const casadi::MX& velocity = velocities[step + 1];
      total_cost += stage_scale * velocity_weight_ratio_ *
                    casadi::MX::mtimes(casadi::MX::mtimes(velocity.T(), q_matrix), velocity);
    }
  }
  if (soft_state_bounds_) {
    for (const casadi::MX& slack : slacks) {
      total_cost += slack_linear_weight_ * casadi::MX::sum1(slack) +
                    slack_quadratic_weight_ * casadi::MX::dot(slack, slack);
    }
  }
  expect_offset(symbolic_size(constraints_), layout_.first_jerk_row_offset, "first-step jerk rows");
  if (enforce_first_step_jerk_) {
    for (int joint = 0; joint < num_joints_; ++joint) {
      const casadi::MX acceleration_delta =
          accelerations[0](joint) - previous_accelerations_(joint);
      constraints_.push_back(acceleration_delta - jerk_min_[joint] * first_jerk_dt_);
      constraints_.push_back(jerk_max_[joint] * first_jerk_dt_ - acceleration_delta);
    }
  }
  expect_offset(symbolic_size(constraints_), layout_.jerk_row_offset, "jerk transition rows");
  for (int step = 0; step < control_horizon_ - 1; ++step) {
    const double dt = prediction_step_durations_[step + 1];
    for (int joint = 0; joint < num_joints_; ++joint) {
      const casadi::MX acceleration_delta =
          accelerations[step + 1](joint) - accelerations[step](joint);
      constraints_.push_back(acceleration_delta - jerk_min_[joint] * dt);
      constraints_.push_back(jerk_max_[joint] * dt - acceleration_delta);
    }
  }
  expect_offset(symbolic_size(constraints_), layout_.bound_row_offset, "soft bound rows");
  if (soft_state_bounds_) {
    // Soft bound rows start at layout_.bound_row_offset: all are "g >= 0".
    const casadi::DM position_lower(position_lower_bounds_);
    const casadi::DM position_upper(position_upper_bounds_);
    const casadi::DM velocity_lower(velocity_lower_bounds_);
    const casadi::DM velocity_upper(velocity_upper_bounds_);
    for (int step = 1; step <= prediction_horizon_; ++step) {
      const casadi::MX& slack = slacks[slack_per_step_ ? step - 1 : 0];
      const casadi::MX position_slack = slack(casadi::Slice(0, num_joints_));
      const casadi::MX velocity_slack = slack(casadi::Slice(num_joints_, 2 * num_joints_));
      constraints_.push_back(positions[step] - position_lower + position_slack);
      constraints_.push_back(position_upper - positions[step] + position_slack);
      constraints_.push_back(velocities[step] - velocity_lower + velocity_slack);
      constraints_.push_back(velocity_upper - velocities[step] + velocity_slack);
    }
  }
  std::vector<casadi::MX> parameters{positions_, velocities_, reference_positions_,
                                     previous_accelerations_, q_vector, r_vector};
  if (enforce_first_step_jerk_) parameters.push_back(first_jerk_dt_);
  const casadi::MX decision_vector = casadi::MX::vertcat(decision_variables_);
  const casadi::MX constraint_vector = casadi::MX::vertcat(constraints_);
  expect_offset(decision_vector.size1(), layout_.decision_vars, "decision vector length");
  expect_offset(constraint_vector.size1(), layout_.constraint_rows, "constraint vector length");
  nlp_ = {{"x", decision_vector},
          {"f", total_cost},
          {"g", constraint_vector},
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
    options["warm_start_primal"] = osqp_warm_start_;
    options["warm_start_dual"] = osqp_warm_start_;
    solver_ = casadi::qpsol("mpc_solver", "osqp", nlp_, options);
  } else if (solver_backend_ == SolverBackend::Qpoases) {
    options["printLevel"] = "none";
    options["sparse"] = true;
    solver_ = casadi::qpsol("mpc_solver", "qpoases", nlp_, options);
  } else if (solver_backend_ == SolverBackend::Qrqp) {
    options["print_iter"] = false;
    options["print_header"] = false;
    options["print_info"] = false;
    options["max_iter"] = osqp_max_iter_;
    options["constr_viol_tol"] = feasibility_tolerance_;
    options["dual_inf_tol"] = feasibility_tolerance_;
    solver_ = casadi::qpsol("mpc_solver", "qrqp", nlp_, options);
  } else {
    options["print_iter"] = false;
    options["print_header"] = false;
    options["print_info"] = false;
    options["max_iter"] = osqp_max_iter_;
    options["constr_viol_tol"] = feasibility_tolerance_;
    options["dual_inf_tol"] = feasibility_tolerance_;
    solver_ = casadi::qpsol("mpc_solver", "ipqp", nlp_, options);
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
  // "solved inaccurate" (OSQP) means the iteration cap was hit before the
  // requested tolerance; the returned point is still usable when it passes the
  // feasibility check below, and it is a far better warm start than zeros.
  const bool inaccurate = normalized_status.find("inaccurate") != std::string::npos;
  if (!inaccurate &&
      (success == "false" || success == "0" ||
       normalized_status.find("infeasible") != std::string::npos ||
       normalized_status.find("maximum") != std::string::npos ||
       normalized_status.find("max_iter") != std::string::npos)) {
    throw std::runtime_error("MPC solver failed with return_status: " + status);
  }
  if (inaccurate) {
    ROS_WARN_THROTTLE(1.0, "MPC solver hit its iteration limit (%s); using the result if feasible.",
                      status.c_str());
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
    throw std::runtime_error("MPC solution violates constraints by " +
                             std::to_string(max_violation));
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
      if (isFinite(result.at("x")) && isFinite(result.at("lam_x")) && isFinite(result.at("lam_g"))) {
        x_init_ = result.at("x");
        lam_x_init_ = result.at("lam_x");
        lam_g_init_ = result.at("lam_g");
      }
      throw;
    }
    const auto validation_end = std::chrono::steady_clock::now();
    last_solve_timing_.validation_ms = elapsedMs(validation_start, validation_end);

    pending_x_ = result.at("x");
    pending_lam_x_ = result.at("lam_x");
    pending_lam_g_ = result.at("lam_g");
    has_pending_solution_ = true;
    // Seed the next solve from this iterate even if the caller rejects it (deadline).
    x_init_ = pending_x_;
    lam_x_init_ = pending_lam_x_;
    lam_g_init_ = pending_lam_g_;

    referent_angles_ =
        pending_x_(casadi::Slice(layout_.xOffset(1), layout_.xOffset(1) + num_joints_));
    referent_velocities_ =
        pending_x_(casadi::Slice(layout_.vOffset(1), layout_.vOffset(1) + num_joints_));
    if (solver_mode_ == SolverMode::TimeScaling) {
      const double final_time = static_cast<double>(pending_x_(pending_x_.size1() - 1));
      if (!std::isfinite(final_time) || final_time <= 0.0) {
        throw std::runtime_error("Time-scaling solver returned invalid final time");
      }
      referent_velocities_ /= final_time;
    }
    return pending_x_;
  } catch (...) {
    // Keep the previous seed: a solver that stalled on one cycle converges far
    // faster from its last iterate than from zeros on the next one.
    has_pending_solution_ = false;
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
  if (solver_mode_ == SolverMode::QuadraticCost) {
    x_init_ = casadi::DM::zeros(layout_.decision_vars, 1);
    lam_x_init_ = casadi::DM::zeros(layout_.decision_vars, 1);
    lam_g_init_ = casadi::DM::zeros(layout_.constraint_rows, 1);
  } else {
    x_init_ = casadi::DM::zeros(x_init_.size1(), 1);
    x_init_(x_init_.size1() - 1) = initial_tf_;
    lam_x_init_ = casadi::DM::zeros(lam_x_init_.size1(), 1);
    lam_g_init_ = casadi::DM::zeros(lam_g_init_.size1(), 1);
  }
  has_pending_solution_ = false;
}

casadi::DM MPCController::getFirstAcceleration(const casadi::DM& solution) const {
  const int start = layout_.aOffset(0);
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
  if (solver_backend_ == SolverBackend::Qpoases) return "qpoases";
  if (solver_backend_ == SolverBackend::Qrqp) return "qrqp";
  return "ipqp";
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
