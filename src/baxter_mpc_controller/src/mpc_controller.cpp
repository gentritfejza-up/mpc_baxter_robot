#include "mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {

/// Stage duration the position/acceleration weights were tuned for [s]: every stage cost
/// is scaled by dt_k / kStageCostReferenceDuration so the tuning survives grid changes.
constexpr double kStageCostReferenceDuration = 0.1;

bool isFinite(const casadi::DM& value) {
  for (casadi_int row = 0; row < value.size1(); ++row) {
    for (casadi_int col = 0; col < value.size2(); ++col) {
      if (!std::isfinite(static_cast<double>(value(row, col)))) return false;
    }
  }
  return true;
}

std::string dictValueString(const casadi::Dict& values, const std::string& key) {
  const auto entry = values.find(key);
  if (entry == values.end()) return {};
  std::ostringstream stream;
  stream << entry->second;
  return stream.str();
}

double elapsedMs(const std::chrono::steady_clock::time_point& start,
                 const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string toLower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  return text;
}

bool allFiniteAndNonnegative(const std::vector<double>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value) && value >= 0.0; });
}

/// Largest violation of lower <= value <= upper over all entries (infinite bounds are skipped).
double maxBoundViolation(const casadi::DM& value, const casadi::DM& lower, const casadi::DM& upper) {
  // All three are dense column vectors: nonzeros() is the raw storage.
  const std::vector<double>& values = casadi::DM::densify(value).nonzeros();
  const std::vector<double>& lower_bounds = casadi::DM::densify(lower).nonzeros();
  const std::vector<double>& upper_bounds = casadi::DM::densify(upper).nonzeros();
  if (values.size() != lower_bounds.size() || values.size() != upper_bounds.size()) {
    throw std::runtime_error("Solver result and bounds have different sizes");
  }
  double max_violation = 0.0;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (std::isfinite(lower_bounds[index])) {
      max_violation = std::max(max_violation, lower_bounds[index] - values[index]);
    }
    if (std::isfinite(upper_bounds[index])) {
      max_violation = std::max(max_violation, values[index] - upper_bounds[index]);
    }
  }
  return max_violation;
}

}  // namespace

MPCController::MPCController(ros::NodeHandle& node_handle, int num_joints, int prediction_horizon,
                             int control_horizon,
                             const std::vector<double>& prediction_step_durations,
                             double nominal_control_period, int geometry_sphere_count)
    : num_joints_(num_joints),
      prediction_horizon_(prediction_horizon),
      control_horizon_(control_horizon),
      prediction_step_durations_(prediction_step_durations),
      nominal_control_period_(nominal_control_period),
      geometry_sphere_count_(geometry_sphere_count),
      position_weights_(num_joints, 0.0),
      acceleration_weights_(num_joints, 0.0),
      reference_positions_(num_joints, 0.0) {
  loadSolverParameters(node_handle);
  setDefaultConstraints();
  validateConfiguration();
  computeQcLayout();
  initializeVariables();
  buildNLP();
  initializeSolverBounds();
  createSolver();

  dynamic_reconfigure_server_.setCallback(
      boost::bind(&MPCController::dynamicReconfigureCallback, this, _1, _2));

  ROS_INFO_STREAM("MPC formulation=" << (solver_mode_ == SolverMode::QuadraticCost ? "qc" : "time_scaling")
                                     << " backend=" << solverBackendName()
                                     << " variables=" << decisionVariableCount()
                                     << " constraints=" << constraint_lower_bounds_.size1()
                                     << " soft_state_bounds="
                                     << (soft_state_bounds_ ? (slack_per_step_ ? "per-step" : "shared") : "off")
                                     << " geometry_rows=" << layout_.geometry_rows
                                     << " geometry_slacks=" << layout_.geometry_slack_variables);
}

void MPCController::loadSolverParameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("robot/jerk_max", jerk_max_) ||
      !node_handle.getParam("robot/jerk_min", jerk_min_) ||
      !node_handle.getParam("robot/acceleration_max", acceleration_max_) ||
      !node_handle.getParam("robot/acceleration_min", acceleration_min_)) {
    throw std::runtime_error("Failed to load MPC jerk and acceleration limits");
  }

  std::string mode;
  node_handle.param<std::string>("robot/solver_mode", mode, "qc");
  if (mode == "qc") {
    solver_mode_ = SolverMode::QuadraticCost;
  } else if (mode == "time_scaling") {
    solver_mode_ = SolverMode::TimeScaling;
  } else {
    throw std::invalid_argument("robot/solver_mode must be 'qc' or 'time_scaling'");
  }

  std::string backend;
  node_handle.param<std::string>("robot/solver_backend", backend,
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

  node_handle.param("robot/solver/feasibility_tolerance", feasibility_tolerance_, feasibility_tolerance_);
  node_handle.param("robot/solver/osqp/eps_abs", osqp_eps_abs_, osqp_eps_abs_);
  node_handle.param("robot/solver/osqp/eps_rel", osqp_eps_rel_, osqp_eps_rel_);
  node_handle.param("robot/solver/osqp/max_iter", osqp_max_iter_, osqp_max_iter_);
  node_handle.param("robot/solver/osqp/polishing", osqp_polishing_, osqp_polishing_);
  node_handle.param("robot/solver/osqp/warm_start", osqp_warm_start_, osqp_warm_start_);
  node_handle.param("robot/solver/osqp/rho", osqp_rho_, osqp_rho_);
  node_handle.param("robot/solver/osqp/adaptive_rho", osqp_adaptive_rho_, osqp_adaptive_rho_);
  node_handle.param("robot/solver/osqp/adaptive_rho_interval", osqp_adaptive_rho_interval_,
                    osqp_adaptive_rho_interval_);
  node_handle.param("robot/solver/osqp/time_limit_sec", osqp_time_limit_sec_, osqp_time_limit_sec_);
  node_handle.param("robot/solver/soft_state_bounds", soft_state_bounds_, soft_state_bounds_);
  node_handle.param("robot/solver/slack_per_step", slack_per_step_, slack_per_step_);
  node_handle.param("robot/solver/slack_linear_weight", slack_linear_weight_, slack_linear_weight_);
  node_handle.param("robot/solver/slack_quadratic_weight", slack_quadratic_weight_,
                    slack_quadratic_weight_);
  node_handle.param("robot/solver/velocity_weight_ratio", velocity_weight_ratio_,
                    velocity_weight_ratio_);
  if (!std::isfinite(velocity_weight_ratio_) || velocity_weight_ratio_ < 0.0) {
    throw std::invalid_argument("robot/solver/velocity_weight_ratio must be finite and nonnegative");
  }
  node_handle.param("robot/control/enforce_first_step_jerk", enforce_first_step_jerk_, false);
  node_handle.param("robot/geometry/rows_per_sphere/floor", geometry_rows_floor_, geometry_rows_floor_);
  node_handle.param("robot/geometry/rows_per_sphere/body", geometry_rows_body_, geometry_rows_body_);
  node_handle.param("robot/geometry/rows_per_sphere/other_arm", geometry_rows_other_arm_,
                    geometry_rows_other_arm_);
  node_handle.param("robot/geometry/slack_quadratic_weight", geometry_slack_weight_,
                    geometry_slack_weight_);
  node_handle.param("robot/geometry/velocity_lookahead_sec", geometry_velocity_lookahead_sec_,
                    geometry_velocity_lookahead_sec_);
  if (!std::isfinite(geometry_velocity_lookahead_sec_) || geometry_velocity_lookahead_sec_ < 0.0) {
    throw std::invalid_argument("robot/geometry/velocity_lookahead_sec must be finite and >= 0");
  }
  if (solver_mode_ == SolverMode::TimeScaling && soft_state_bounds_) {
    ROS_WARN("Soft state bounds are only implemented for the qc formulation; ignoring.");
    soft_state_bounds_ = false;
  }
  if (solver_mode_ == SolverMode::TimeScaling && geometry_sphere_count_ > 0) {
    ROS_WARN("The collision guard is only implemented for the qc formulation; ignoring.");
    geometry_sphere_count_ = 0;
  }
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
  // 80 % (70 % on the wrists) of the URDF velocity limits (1.5 / 4.0 rad/s): Gazebo and
  // ros_control enforce the URDF limits by clamping the effort to one sign, which breaks
  // the computed-torque model if the MPC rides the limit itself.
  velocity_lower_bounds_ = {-1.2, -1.2, -1.2, -1.2, -2.8, -2.8, -2.8};
  velocity_upper_bounds_ = {1.2, 1.2, 1.2, 1.2, 2.8, 2.8, 2.8};
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
  for (double step_duration : prediction_step_durations_) {
    if (!std::isfinite(step_duration) || step_duration <= 0.0) {
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
  if (geometry_sphere_count_ < 0 || geometry_rows_floor_ < 0 || geometry_rows_body_ < 0 ||
      geometry_rows_other_arm_ < 0 || !std::isfinite(geometry_slack_weight_) ||
      geometry_slack_weight_ <= 0.0 ||
      (geometry_sphere_count_ > 0 &&
       geometry_rows_floor_ + geometry_rows_body_ + geometry_rows_other_arm_ == 0)) {
    throw std::invalid_argument("Invalid collision-guard sphere count, row counts or slack weight");
  }
  if (geometry_sphere_count_ > 0 && num_joints_ != mpc_geometry::kArmJoints) {
    throw std::invalid_argument("The collision guard expects a 7-joint arm");
  }
}

void MPCController::computeQcLayout() {
  QcLayout layout;
  layout.num_joints = num_joints_;
  layout.state_variables = prediction_horizon_ * 2 * num_joints_;
  layout.acceleration_variables = control_horizon_ * num_joints_;
  layout.bound_slack_blocks = soft_state_bounds_ ? (slack_per_step_ ? prediction_horizon_ : 1) : 0;
  layout.bound_slack_variables = layout.bound_slack_blocks * 2 * num_joints_;
  layout.acceleration_offset = layout.state_variables;
  layout.bound_slack_offset = layout.acceleration_offset + layout.acceleration_variables;
  layout.geometry_slack_offset = layout.bound_slack_offset + layout.bound_slack_variables;
  const int slots_per_sphere = geometry_rows_floor_ + geometry_rows_body_ + geometry_rows_other_arm_;
  layout.geometry_slots_per_step = geometry_sphere_count_ * slots_per_sphere;
  layout.geometry_slack_variables = prediction_horizon_ * geometry_sphere_count_;
  layout.geometry_rows = prediction_horizon_ * layout.geometry_slots_per_step;
  layout.decision_variables = layout.geometry_slack_offset + layout.geometry_slack_variables;

  layout.dynamics_rows = 2 * prediction_horizon_ * num_joints_;
  layout.first_jerk_rows = enforce_first_step_jerk_ ? 2 * num_joints_ : 0;
  layout.jerk_rows = 2 * (control_horizon_ - 1) * num_joints_;
  layout.bound_rows = soft_state_bounds_ ? 4 * prediction_horizon_ * num_joints_ : 0;
  layout.first_jerk_row_offset = layout.dynamics_rows;
  layout.jerk_row_offset = layout.first_jerk_row_offset + layout.first_jerk_rows;
  layout.bound_row_offset = layout.jerk_row_offset + layout.jerk_rows;
  layout.geometry_row_offset = layout.bound_row_offset + layout.bound_rows;
  layout.constraint_rows = layout.geometry_row_offset + layout.geometry_rows;
  layout_ = layout;
}

mpc_geometry::GeometryLayout MPCController::geometryLayout() const {
  mpc_geometry::GeometryLayout layout;
  layout.prediction_horizon = prediction_horizon_;
  layout.sphere_count = geometry_sphere_count_;
  layout.rows_floor = geometry_rows_floor_;
  layout.rows_body = geometry_rows_body_;
  layout.rows_other_arm = geometry_rows_other_arm_;
  // Time of each prediction step from now (cumulative step durations).
  layout.step_times.reserve(prediction_step_durations_.size());
  double elapsed = 0.0;
  for (double step_duration : prediction_step_durations_) {
    elapsed += step_duration;
    layout.step_times.push_back(elapsed);
  }
  return layout;
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
  if (!allFiniteAndNonnegative(position_weights) || !allFiniteAndNonnegative(acceleration_weights) ||
      !std::all_of(references.begin(), references.end(),
                   [](double value) { return std::isfinite(value); })) {
    ROS_ERROR("Rejected nonfinite or negative MPC dynamic-reconfigure values.");
    return;
  }
  std::lock_guard<std::mutex> lock(config_mutex_);
  position_weights_ = position_weights;
  acceleration_weights_ = acceleration_weights;
  reference_positions_ = references;
  ++reference_updates_;
}

std::vector<double> MPCController::getReferencePositions() const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return reference_positions_;
}

bool MPCController::hasExternalReference() const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return reference_updates_ >= 2;
}

void MPCController::initializeVariables() {
  positions_symbol_ = casadi::MX::sym("positions", num_joints_);
  velocities_symbol_ = casadi::MX::sym("velocities", num_joints_);
  reference_positions_symbol_ = casadi::MX::sym("reference_positions", num_joints_);
  previous_accelerations_symbol_ = casadi::MX::sym("previous_accelerations", num_joints_);
  first_jerk_dt_symbol_ = casadi::MX::sym("first_jerk_dt");
  geometry_coefficients_symbol_ =
      casadi::MX::sym("geometry_coefficients", std::max(1, layout_.geometry_rows * num_joints_));
}

void MPCController::initializeSolverBounds() {
  if (solver_mode_ == SolverMode::TimeScaling) {
    initializeBoundsTimeScaling();
  } else {
    initializeBoundsQuadraticCost();
  }
  dual_bounds_warm_start_ = casadi::DM::zeros(primal_warm_start_.size1(), 1);
  dual_constraints_warm_start_ = casadi::DM::zeros(constraint_lower_bounds_.size1(), 1);
}

/**
 * Time-scaling bounds. Decision vector: states | accelerations | final time T.
 * Constraint rows: dynamics (2 N nj, == 0) | state bounds (4 N nj, >= 0) |
 * jerk (2 Nc nj, >= 0) | final position and velocity (2 nj, == 0) | T - T_min (>= 0).
 */
void MPCController::initializeBoundsTimeScaling() {
  const int state_variables = prediction_horizon_ * 2 * num_joints_;
  const int decision_variables = state_variables + control_horizon_ * num_joints_ + 1;
  primal_warm_start_ = casadi::DM::zeros(decision_variables, 1);
  decision_lower_bounds_ = -casadi::DM::inf(decision_variables, 1);
  decision_upper_bounds_ = casadi::DM::inf(decision_variables, 1);
  const int final_time_index = decision_variables - 1;
  primal_warm_start_(final_time_index) = final_time_initial_;
  decision_lower_bounds_(final_time_index) = final_time_min_;
  decision_upper_bounds_(final_time_index) = final_time_max_;
  for (int step = 0; step < control_horizon_; ++step) {
    const int start = state_variables + step * num_joints_;
    decision_lower_bounds_(casadi::Slice(start, start + num_joints_)) = acceleration_min_;
    decision_upper_bounds_(casadi::Slice(start, start + num_joints_)) = acceleration_max_;
  }
  const int dynamics_rows = 2 * prediction_horizon_ * num_joints_;
  const int final_equalities_start =
      (6 * prediction_horizon_ * num_joints_) + (2 * control_horizon_ * num_joints_);
  const int constraint_count = final_equalities_start + 2 * num_joints_ + 1;
  constraint_lower_bounds_ = casadi::DM::zeros(constraint_count, 1);
  constraint_upper_bounds_ = casadi::DM::inf(constraint_count, 1);
  constraint_upper_bounds_(casadi::Slice(0, dynamics_rows)) = 0.0;
  constraint_upper_bounds_(casadi::Slice(final_equalities_start,
                                         final_equalities_start + 2 * num_joints_)) = 0.0;
}

void MPCController::initializeBoundsQuadraticCost() {
  const QcLayout& layout = layout_;
  primal_warm_start_ = casadi::DM::zeros(layout.decision_variables, 1);
  decision_lower_bounds_ = -casadi::DM::inf(layout.decision_variables, 1);
  decision_upper_bounds_ = casadi::DM::inf(layout.decision_variables, 1);
  if (!soft_state_bounds_) {
    for (int step = 1; step <= prediction_horizon_; ++step) {
      const int position_start = layout.positionOffset(step);
      const int velocity_start = layout.velocityOffset(step);
      decision_lower_bounds_(casadi::Slice(position_start, position_start + num_joints_)) =
          position_lower_bounds_;
      decision_upper_bounds_(casadi::Slice(position_start, position_start + num_joints_)) =
          position_upper_bounds_;
      decision_lower_bounds_(casadi::Slice(velocity_start, velocity_start + num_joints_)) =
          velocity_lower_bounds_;
      decision_upper_bounds_(casadi::Slice(velocity_start, velocity_start + num_joints_)) =
          velocity_upper_bounds_;
    }
  }
  for (int step = 0; step < control_horizon_; ++step) {
    const int start = layout.accelerationOffset(step);
    decision_lower_bounds_(casadi::Slice(start, start + num_joints_)) = acceleration_min_;
    decision_upper_bounds_(casadi::Slice(start, start + num_joints_)) = acceleration_max_;
  }
  if (soft_state_bounds_) {
    decision_lower_bounds_(casadi::Slice(
        layout.bound_slack_offset, layout.bound_slack_offset + layout.bound_slack_variables)) = 0.0;
  }
  if (layout.geometry_slack_variables > 0) {
    decision_lower_bounds_(casadi::Slice(
        layout.geometry_slack_offset,
        layout.geometry_slack_offset + layout.geometry_slack_variables)) = 0.0;
  }
  constraint_lower_bounds_ = casadi::DM::zeros(layout.constraint_rows, 1);
  constraint_upper_bounds_ = casadi::DM::inf(layout.constraint_rows, 1);
  // Only the dynamics rows are equalities; every other row is "g >= 0", except the
  // geometry rows whose lower bound is set per cycle (-inf = inactive).
  constraint_upper_bounds_(casadi::Slice(0, layout.dynamics_rows)) = 0.0;
  if (layout.geometry_rows > 0) {
    constraint_lower_bounds_(casadi::Slice(layout.geometry_row_offset, layout.constraint_rows)) =
        -std::numeric_limits<double>::infinity();
  }
  constraint_lower_bounds_template_ = constraint_lower_bounds_.nonzeros();
  constraint_lower_bounds_cycle_ = constraint_lower_bounds_template_;
  inactive_geometry_coefficients_.assign(std::max(1, layout.geometry_rows * num_joints_), 0.0);
}

void MPCController::validateInputs(const MpcSolveInput& input) const {
  const auto valid_vector = [this](const casadi::DM& value) {
    return value.size1() == num_joints_ && value.size2() == 1 && isFinite(value);
  };
  if (!valid_vector(input.positions) || !valid_vector(input.velocities) ||
      !valid_vector(input.reference_positions) || !valid_vector(input.previous_accelerations)) {
    throw std::invalid_argument("MPC inputs must be finite num_joints-by-1 vectors");
  }
  if (enforce_first_step_jerk_ &&
      (!std::isfinite(input.first_jerk_dt) || input.first_jerk_dt <= 0.0 ||
       input.first_jerk_dt > nominal_control_period_ + 1e-9)) {
    throw std::invalid_argument("First jerk interval must be positive and no longer than one cycle");
  }
}

void MPCController::validateGeometryBatch(const mpc_geometry::GeometryRowBatch* batch) const {
  if (!batch || layout_.geometry_rows == 0) return;
  if (batch->coefficients.size() != static_cast<std::size_t>(layout_.geometry_rows * num_joints_) ||
      batch->lower_bounds.size() != static_cast<std::size_t>(layout_.geometry_rows)) {
    throw std::invalid_argument("Geometry row batch does not match the QP layout");
  }
  for (double coefficient : batch->coefficients) {
    if (!std::isfinite(coefficient)) {
      throw std::invalid_argument("Geometry row coefficients must be finite");
    }
  }
  for (double lower_bound : batch->lower_bounds) {
    if (std::isnan(lower_bound) || lower_bound == std::numeric_limits<double>::infinity()) {
      throw std::invalid_argument("Geometry row lower bounds must be finite or -inf");
    }
  }
}

void MPCController::buildNLP() {
  if (solver_mode_ == SolverMode::TimeScaling) {
    buildNlpTimeScaling();
  } else {
    buildNlpQuadraticCost();
  }
}

/**
 * Time-scaling formulation: the horizon is normalised to [0, 1] and the final time T is
 * a decision variable, so the velocities are scaled by T (V' = T V) and the dynamics by
 * T^2. The plan must reach the reference at rest at the end of the horizon.
 */
void MPCController::buildNlpTimeScaling() {
  decision_variables_.clear();
  constraints_.clear();
  casadi::MX final_time = casadi::MX::sym("T");
  casadi::MX total_cost = casadi::MX::zeros(1);
  casadi::MX position_weight_parameters = casadi::MX::sym("position_weights", num_joints_);
  casadi::MX acceleration_weight_parameters = casadi::MX::sym("acceleration_weights", num_joints_);
  const casadi::MX position_weight_matrix = casadi::MX::diag(position_weight_parameters);
  const casadi::MX acceleration_weight_matrix = casadi::MX::diag(acceleration_weight_parameters);
  const double normalized_dt = 1.0 / prediction_horizon_;

  std::vector<casadi::MX> positions(prediction_horizon_ + 1);
  std::vector<casadi::MX> velocities(prediction_horizon_ + 1);
  std::vector<casadi::MX> accelerations(control_horizon_);
  positions[0] = positions_symbol_;
  velocities[0] = final_time * velocities_symbol_;
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
    constraints_.push_back(velocities[step + 1] - (velocities[step] + normalized_dt * final_time *
                                                                          final_time * acceleration));
    const casadi::MX error = positions[step] - reference_positions_symbol_;
    total_cost += casadi::MX::mtimes(casadi::MX::mtimes(error.T(), position_weight_matrix), error) +
                  casadi::MX::mtimes(casadi::MX::mtimes(acceleration.T(), acceleration_weight_matrix),
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
    const casadi::MX jerk = (accelerations[0](joint) - previous_accelerations_symbol_(joint)) /
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
  constraints_.push_back(positions.back() - reference_positions_symbol_);
  constraints_.push_back(final_time * velocities.back());
  constraints_.push_back(final_time - final_time_min_);
  problem_ = {{"x", casadi::MX::vertcat(decision_variables_)},
              {"f", total_cost},
              {"g", casadi::MX::vertcat(constraints_)},
              {"p", casadi::MX::vertcat({positions_symbol_, velocities_symbol_,
                                          reference_positions_symbol_,
                                          previous_accelerations_symbol_,
                                          position_weight_parameters,
                                          acceleration_weight_parameters})}};
}

/**
 * Quadratic-cost QP (see the file header of mpc_controller.hpp for the formulation).
 * Variable and row order must follow layout_ (QcLayout); the checkpoints verify each
 * block boundary while the problem is built, so a layout/builder mismatch fails at
 * construction instead of producing a silently misaligned QP.
 */
void MPCController::buildNlpQuadraticCost() {
  decision_variables_.clear();
  constraints_.clear();
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
  casadi::MX position_weight_parameters = casadi::MX::sym("position_weights", num_joints_);
  casadi::MX acceleration_weight_parameters = casadi::MX::sym("acceleration_weights", num_joints_);
  const casadi::MX position_weight_matrix = casadi::MX::diag(position_weight_parameters);
  const casadi::MX acceleration_weight_matrix = casadi::MX::diag(acceleration_weight_parameters);

  // Decision variables: states, accelerations, bound slacks, geometry slacks.
  std::vector<casadi::MX> positions(prediction_horizon_ + 1);
  std::vector<casadi::MX> velocities(prediction_horizon_ + 1);
  std::vector<casadi::MX> accelerations(control_horizon_);
  std::vector<casadi::MX> bound_slacks;
  std::vector<casadi::MX> geometry_slacks;
  positions[0] = positions_symbol_;
  velocities[0] = velocities_symbol_;
  for (int step = 0; step < control_horizon_; ++step) {
    accelerations[step] = casadi::MX::sym("a_" + std::to_string(step), num_joints_);
  }
  for (int step = 1; step <= prediction_horizon_; ++step) {
    positions[step] = casadi::MX::sym("X_" + std::to_string(step), num_joints_);
    velocities[step] = casadi::MX::sym("V_" + std::to_string(step), num_joints_);
    decision_variables_.push_back(positions[step]);
    decision_variables_.push_back(velocities[step]);
  }
  expect_offset(symbolic_size(decision_variables_), layout_.acceleration_offset,
                "acceleration variables");
  decision_variables_.insert(decision_variables_.end(), accelerations.begin(), accelerations.end());
  expect_offset(symbolic_size(decision_variables_), layout_.bound_slack_offset,
                "bound slack variables");
  if (soft_state_bounds_) {
    bound_slacks.reserve(layout_.bound_slack_blocks);
    for (int block = 0; block < layout_.bound_slack_blocks; ++block) {
      bound_slacks.push_back(casadi::MX::sym("S_" + std::to_string(block), 2 * num_joints_));
      decision_variables_.push_back(bound_slacks.back());
    }
  }
  expect_offset(symbolic_size(decision_variables_), layout_.geometry_slack_offset,
                "geometry slack variables");
  if (layout_.geometry_rows > 0) {
    geometry_slacks.reserve(prediction_horizon_);
    for (int step = 1; step <= prediction_horizon_; ++step) {
      geometry_slacks.push_back(
          casadi::MX::sym("G_" + std::to_string(step), geometry_sphere_count_));
      decision_variables_.push_back(geometry_slacks.back());
    }
  }

  // Dynamics rows and the stage cost.
  for (int step = 0; step < prediction_horizon_; ++step) {
    const double step_duration = prediction_step_durations_[step];
    const casadi::MX acceleration = accelerations[std::min(step, control_horizon_ - 1)];
    constraints_.push_back(positions[step + 1] -
                           (positions[step] + step_duration * velocities[step] +
                            0.5 * step_duration * step_duration * acceleration));
    constraints_.push_back(velocities[step + 1] - (velocities[step] + step_duration * acceleration));
    const casadi::MX error = positions[step + 1] - reference_positions_symbol_;
    const double stage_scale = step_duration / kStageCostReferenceDuration;
    total_cost += stage_scale *
                  (casadi::MX::mtimes(casadi::MX::mtimes(error.T(), position_weight_matrix), error) +
                   casadi::MX::mtimes(casadi::MX::mtimes(acceleration.T(), acceleration_weight_matrix),
                                      acceleration));
    if (velocity_weight_ratio_ > 0.0) {
      const casadi::MX& velocity = velocities[step + 1];
      total_cost += stage_scale * velocity_weight_ratio_ *
                    casadi::MX::mtimes(casadi::MX::mtimes(velocity.T(), position_weight_matrix),
                                       velocity);
    }
  }
  for (const casadi::MX& slack : bound_slacks) {
    total_cost += slack_linear_weight_ * casadi::MX::sum1(slack) +
                  slack_quadratic_weight_ * casadi::MX::dot(slack, slack);
  }
  for (const casadi::MX& slack : geometry_slacks) {
    total_cost += geometry_slack_weight_ * casadi::MX::dot(slack, slack);
  }

  // Jerk rows: a_0 against the previously applied acceleration, then consecutive steps.
  expect_offset(symbolic_size(constraints_), layout_.first_jerk_row_offset, "first-step jerk rows");
  if (enforce_first_step_jerk_) {
    for (int joint = 0; joint < num_joints_; ++joint) {
      const casadi::MX acceleration_delta =
          accelerations[0](joint) - previous_accelerations_symbol_(joint);
      constraints_.push_back(acceleration_delta - jerk_min_[joint] * first_jerk_dt_symbol_);
      constraints_.push_back(jerk_max_[joint] * first_jerk_dt_symbol_ - acceleration_delta);
    }
  }
  expect_offset(symbolic_size(constraints_), layout_.jerk_row_offset, "jerk transition rows");
  for (int step = 0; step < control_horizon_ - 1; ++step) {
    const double step_duration = prediction_step_durations_[step + 1];
    for (int joint = 0; joint < num_joints_; ++joint) {
      const casadi::MX acceleration_delta =
          accelerations[step + 1](joint) - accelerations[step](joint);
      constraints_.push_back(acceleration_delta - jerk_min_[joint] * step_duration);
      constraints_.push_back(jerk_max_[joint] * step_duration - acceleration_delta);
    }
  }

  // Soft bound rows, all "g >= 0": X - lower + S_pos, upper - X + S_pos, and the same for V.
  expect_offset(symbolic_size(constraints_), layout_.bound_row_offset, "soft bound rows");
  if (soft_state_bounds_) {
    const casadi::DM position_lower(position_lower_bounds_);
    const casadi::DM position_upper(position_upper_bounds_);
    const casadi::DM velocity_lower(velocity_lower_bounds_);
    const casadi::DM velocity_upper(velocity_upper_bounds_);
    for (int step = 1; step <= prediction_horizon_; ++step) {
      const casadi::MX& slack = bound_slacks[slack_per_step_ ? step - 1 : 0];
      const casadi::MX position_slack = slack(casadi::Slice(0, num_joints_));
      const casadi::MX velocity_slack = slack(casadi::Slice(num_joints_, 2 * num_joints_));
      constraints_.push_back(positions[step] - position_lower + position_slack);
      constraints_.push_back(position_upper - positions[step] + position_slack);
      constraints_.push_back(velocities[step] - velocity_lower + velocity_slack);
      constraints_.push_back(velocity_upper - velocities[step] + velocity_slack);
    }
  }

  // Geometry rows of step k:  g = C_k (X_k + tau V_k) + E G_k, with C_k the slots x nj
  // coefficient block of the parameter vector (row-major) and E selecting the sphere's
  // slack for each of its slots.
  expect_offset(symbolic_size(constraints_), layout_.geometry_row_offset, "geometry rows");
  if (layout_.geometry_rows > 0) {
    const int slots = layout_.geometry_slots_per_step;
    const int slots_per_sphere = slots / geometry_sphere_count_;
    casadi::DM slack_selection = casadi::DM::zeros(slots, geometry_sphere_count_);
    for (int sphere = 0; sphere < geometry_sphere_count_; ++sphere) {
      for (int slot = 0; slot < slots_per_sphere; ++slot) {
        slack_selection(sphere * slots_per_sphere + slot, sphere) = 1.0;
      }
    }
    for (int step = 1; step <= prediction_horizon_; ++step) {
      const int start = (step - 1) * slots * num_joints_;
      const casadi::MX block =
          geometry_coefficients_symbol_(casadi::Slice(start, start + slots * num_joints_));
      // reshape is column-major: column r holds entries [nj r, nj r + nj) = row r's coefficients.
      const casadi::MX coefficients = casadi::MX::reshape(block, num_joints_, slots).T();
      // Approach-rate damper: the row holds for the point tau seconds ahead.
      const casadi::MX lookahead_position =
          positions[step] + geometry_velocity_lookahead_sec_ * velocities[step];
      constraints_.push_back(casadi::MX::mtimes(coefficients, lookahead_position) +
                             casadi::MX::mtimes(slack_selection, geometry_slacks[step - 1]));
    }
  }

  std::vector<casadi::MX> parameters{positions_symbol_, velocities_symbol_,
                                     reference_positions_symbol_, previous_accelerations_symbol_,
                                     position_weight_parameters, acceleration_weight_parameters};
  if (enforce_first_step_jerk_) parameters.push_back(first_jerk_dt_symbol_);
  if (layout_.geometry_rows > 0) parameters.push_back(geometry_coefficients_symbol_);
  const casadi::MX decision_vector = casadi::MX::vertcat(decision_variables_);
  const casadi::MX constraint_vector = casadi::MX::vertcat(constraints_);
  expect_offset(decision_vector.size1(), layout_.decision_variables, "decision vector length");
  expect_offset(constraint_vector.size1(), layout_.constraint_rows, "constraint vector length");
  problem_ = {{"x", decision_vector},
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
    solver_ = casadi::nlpsol("mpc_solver", "ipopt", problem_, options);
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
    osqp_options["rho"] = osqp_rho_;
    osqp_options["adaptive_rho"] = osqp_adaptive_rho_;
    if (osqp_adaptive_rho_interval_ > 0) {
      osqp_options["adaptive_rho_interval"] = osqp_adaptive_rho_interval_;
    }
    if (osqp_time_limit_sec_ > 0.0) osqp_options["time_limit"] = osqp_time_limit_sec_;
    osqp_options["verbose"] = false;
    options["osqp"] = osqp_options;
    options["warm_start_primal"] = osqp_warm_start_;
    options["warm_start_dual"] = osqp_warm_start_;
    solver_ = casadi::qpsol("mpc_solver", "osqp", problem_, options);
  } else if (solver_backend_ == SolverBackend::Qpoases) {
    options["printLevel"] = "none";
    options["sparse"] = true;
    solver_ = casadi::qpsol("mpc_solver", "qpoases", problem_, options);
  } else {
    // CasADi's built-in qrqp (active set) and ipqp (interior point) share their options.
    options["print_iter"] = false;
    options["print_header"] = false;
    options["print_info"] = false;
    options["max_iter"] = osqp_max_iter_;
    options["constr_viol_tol"] = feasibility_tolerance_;
    options["dual_inf_tol"] = feasibility_tolerance_;
    solver_ = casadi::qpsol("mpc_solver", solver_backend_ == SolverBackend::Qrqp ? "qrqp" : "ipqp",
                            problem_, options);
  }
}

void MPCController::seedWarmStartFromState(const casadi::DM& positions,
                                           const casadi::DM& velocities) {
  // An all-zero primal seed is ~1 rad from the trajectory and costs OSQP its whole
  // iteration budget. Seed every predicted state with the measured one (accelerations
  // and slacks zero).
  std::vector<double> seed(static_cast<std::size_t>(layout_.decision_variables), 0.0);
  const std::vector<double>& position_values = casadi::DM::densify(positions).nonzeros();
  const std::vector<double>& velocity_values = casadi::DM::densify(velocities).nonzeros();
  for (int step = 1; step <= prediction_horizon_; ++step) {
    for (int joint = 0; joint < num_joints_; ++joint) {
      seed[static_cast<std::size_t>(layout_.positionOffset(step) + joint)] = position_values[joint];
      seed[static_cast<std::size_t>(layout_.velocityOffset(step) + joint)] = velocity_values[joint];
    }
  }
  primal_warm_start_ = casadi::DM(seed);
  warm_start_seeded_ = true;
}

casadi::DMDict MPCController::prepareSolverArguments(const MpcSolveInput& input) {
  casadi::DM position_weights;
  casadi::DM acceleration_weights;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    position_weights = position_weights_;
    acceleration_weights = acceleration_weights_;
  }
  std::vector<casadi::DM> parameters{input.positions,      input.velocities,
                                     input.reference_positions, input.previous_accelerations,
                                     position_weights,     acceleration_weights};
  if (solver_mode_ == SolverMode::QuadraticCost && enforce_first_step_jerk_) {
    parameters.emplace_back(input.first_jerk_dt);
  }
  const bool has_geometry_rows = layout_.geometry_rows > 0;
  if (has_geometry_rows) {
    const mpc_geometry::GeometryRowBatch* batch = input.geometry;
    parameters.emplace_back(casadi::DM(batch ? batch->coefficients : inactive_geometry_coefficients_));
    constraint_lower_bounds_cycle_ = constraint_lower_bounds_template_;
    if (batch) {
      std::copy(batch->lower_bounds.begin(), batch->lower_bounds.end(),
                constraint_lower_bounds_cycle_.begin() + layout_.geometry_row_offset);
    }
  }
  const casadi::DM constraint_lower_bounds =
      has_geometry_rows ? casadi::DM(constraint_lower_bounds_cycle_) : constraint_lower_bounds_;
  if (!warm_start_seeded_ && solver_mode_ == SolverMode::QuadraticCost) {
    seedWarmStartFromState(input.positions, input.velocities);
  }
  return {{"x0", primal_warm_start_},
          {"lam_x0", dual_bounds_warm_start_},
          {"lam_g0", dual_constraints_warm_start_},
          {"lbx", decision_lower_bounds_},
          {"ubx", decision_upper_bounds_},
          {"lbg", constraint_lower_bounds},
          {"ubg", constraint_upper_bounds_},
          {"p", casadi::DM::vertcat(parameters)}};
}

void MPCController::adoptWarmStart(const casadi::DMDict& result) {
  const auto primal = result.find("x");
  const auto dual_bounds = result.find("lam_x");
  const auto dual_constraints = result.find("lam_g");
  if (primal == result.end() || dual_bounds == result.end() || dual_constraints == result.end()) {
    return;
  }
  if (isFinite(primal->second) && isFinite(dual_bounds->second) &&
      isFinite(dual_constraints->second)) {
    primal_warm_start_ = primal->second;
    dual_bounds_warm_start_ = dual_bounds->second;
    dual_constraints_warm_start_ = dual_constraints->second;
  }
}

/**
 * Accepts a solve when the solver reports success, or when it stopped early at its
 * iteration cap (OSQP "solved inaccurate" within 10x the tolerance, "maximum iterations
 * reached" beyond it) or time limit, as long as the iterate satisfies the dynamics, bounds
 * and rows within feasibility_tolerance_: a slightly suboptimal plan beats a braking
 * cycle, and it is the warm start anyway. Infeasible or failed solves throw.
 */
void MPCController::validateResult(const casadi::DMDict& result,
                                   const casadi::DM& constraint_lower_bounds) {
  const auto primal = result.find("x");
  const auto constraint_values = result.find("g");
  if (primal == result.end() || constraint_values == result.end() || !isFinite(primal->second) ||
      !isFinite(constraint_values->second)) {
    throw std::runtime_error("Solver returned missing or nonfinite primal values");
  }

  const casadi::Dict stats = solver_.stats();
  const std::string success = dictValueString(stats, "success");
  const std::string status = dictValueString(stats, "return_status");
  const std::string normalized_status = toLower(status);
  const bool inaccurate = normalized_status.find("inaccurate") != std::string::npos ||
                          normalized_status.find("maximum") != std::string::npos ||
                          normalized_status.find("max_iter") != std::string::npos ||
                          normalized_status.find("time limit") != std::string::npos ||
                          normalized_status.find("time_limit") != std::string::npos;
  if (!inaccurate &&
      (success == "false" || success == "0" ||
       normalized_status.find("infeasible") != std::string::npos)) {
    throw std::runtime_error("MPC solver failed with return_status: " + status);
  }
  last_solve_inaccurate_ = inaccurate;
  if (inaccurate) {
    ROS_WARN_THROTTLE(5.0, "MPC solver hit its iteration limit (%s); using the result if feasible.",
                      status.c_str());
  }

  const double max_violation =
      std::max(maxBoundViolation(primal->second, decision_lower_bounds_, decision_upper_bounds_),
               maxBoundViolation(constraint_values->second, constraint_lower_bounds,
                                 constraint_upper_bounds_));
  if (max_violation > feasibility_tolerance_) {
    throw std::runtime_error("MPC solution violates constraints by " +
                             std::to_string(max_violation));
  }
  if (solver_mode_ == SolverMode::TimeScaling) {
    const double final_time =
        static_cast<double>(primal->second(primal->second.size1() - 1));
    if (!std::isfinite(final_time) || final_time <= 0.0) {
      throw std::runtime_error("Time-scaling solver returned invalid final time");
    }
  }
}

casadi::DM MPCController::solve(const MpcSolveInput& input) {
  validateInputs(input);
  validateGeometryBatch(input.geometry);
  has_pending_solution_ = false;

  const auto preparation_start = std::chrono::steady_clock::now();
  const casadi::DMDict arguments = prepareSolverArguments(input);
  const auto solver_start = std::chrono::steady_clock::now();
  const casadi::DMDict result = solver_(arguments);
  const auto solver_end = std::chrono::steady_clock::now();
  last_solve_timing_ = {elapsedMs(preparation_start, solver_start),
                        elapsedMs(solver_start, solver_end), 0.0};

  // Whatever the validation says, the iterate seeds the next solve: a solver that
  // stalled on one cycle converges far faster from its last iterate than from zeros,
  // and a solution the caller rejects (deadline) is still the best warm start.
  adoptWarmStart(result);
  try {
    validateResult(result, arguments.at("lbg"));
  } catch (...) {
    last_solve_timing_.validation_ms = elapsedMs(solver_end, std::chrono::steady_clock::now());
    throw;
  }
  last_solve_timing_.validation_ms = elapsedMs(solver_end, std::chrono::steady_clock::now());

  pending_primal_ = result.at("x");
  pending_dual_bounds_ = result.at("lam_x");
  pending_dual_constraints_ = result.at("lam_g");
  has_pending_solution_ = true;
  return pending_primal_;
}

void MPCController::commitSolution() {
  if (!has_pending_solution_) throw std::logic_error("No pending MPC solution to commit");
  primal_warm_start_ = pending_primal_;
  dual_bounds_warm_start_ = pending_dual_bounds_;
  dual_constraints_warm_start_ = pending_dual_constraints_;
  has_pending_solution_ = false;
}

void MPCController::resetDualWarmStart() {
  dual_bounds_warm_start_ = casadi::DM::zeros(dual_bounds_warm_start_.size1(), 1);
  dual_constraints_warm_start_ = casadi::DM::zeros(dual_constraints_warm_start_.size1(), 1);
}

void MPCController::resetWarmStart() {
  primal_warm_start_ = casadi::DM::zeros(primal_warm_start_.size1(), 1);
  if (solver_mode_ == SolverMode::TimeScaling) {
    primal_warm_start_(primal_warm_start_.size1() - 1) = final_time_initial_;
  }
  dual_bounds_warm_start_ = casadi::DM::zeros(dual_bounds_warm_start_.size1(), 1);
  dual_constraints_warm_start_ = casadi::DM::zeros(dual_constraints_warm_start_.size1(), 1);
  warm_start_seeded_ = false;
  has_pending_solution_ = false;
}

std::vector<mpc_geometry::JointVector> MPCController::predictedPositions(
    const casadi::DM& solution) const {
  if (solution.size1() < layout_.acceleration_offset) {
    throw std::invalid_argument("MPC solution is too short to contain the predicted positions");
  }
  const std::vector<double> values = casadi::DM::densify(solution).nonzeros();
  std::vector<mpc_geometry::JointVector> predicted(prediction_horizon_);
  for (int step = 1; step <= prediction_horizon_; ++step) {
    const int offset = layout_.positionOffset(step);
    for (int joint = 0; joint < num_joints_ && joint < mpc_geometry::kArmJoints; ++joint) {
      predicted[step - 1][joint] = values[offset + joint];
    }
  }
  return predicted;
}

double MPCController::maxGeometrySlack(const casadi::DM& solution) const {
  if (layout_.geometry_slack_variables == 0) return 0.0;
  if (solution.size1() < layout_.geometry_slack_offset + layout_.geometry_slack_variables) {
    throw std::invalid_argument("MPC solution is too short to contain the geometry slacks");
  }
  const std::vector<double> values = casadi::DM::densify(solution).nonzeros();
  double worst = 0.0;
  for (int index = 0; index < layout_.geometry_slack_variables; ++index) {
    worst = std::max(worst, values[layout_.geometry_slack_offset + index]);
  }
  return worst;
}

casadi::DM MPCController::getFirstAcceleration(const casadi::DM& solution) const {
  const int start = layout_.accelerationOffset(0);
  if (solution.size1() < start + num_joints_) {
    throw std::invalid_argument("MPC solution is too short to contain the first acceleration");
  }
  return solution(casadi::Slice(start, start + num_joints_));
}

std::string MPCController::solverBackendName() const {
  switch (solver_backend_) {
    case SolverBackend::Ipopt: return "ipopt";
    case SolverBackend::Osqp: return "osqp";
    case SolverBackend::Qpoases: return "qpoases";
    case SolverBackend::Qrqp: return "qrqp";
    case SolverBackend::Ipqp: return "ipqp";
  }
  return "unknown";
}

std::unordered_map<std::string, double> MPCController::dmToMap(
    const casadi::DM& matrix, const std::vector<std::string>& keys) {
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
