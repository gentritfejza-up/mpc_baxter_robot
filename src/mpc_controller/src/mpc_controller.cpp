#include "mpc_controller/mpc_controller.hpp"
#include <cmath>
#include <sstream>
#include <stdexcept>

// Constructor to initialize the MPC controller
MPCController::MPCController(ros::NodeHandle& nh, int num_joints, int prediction_horizon,
                             int control_horizon, double sampling_time)
    : num_joints_(num_joints),
      prediction_horizon_(prediction_horizon),
      control_horizon_(control_horizon),
      sampling_time_(sampling_time),
      position_weights_vector_(num_joints, 0.0),
      acceleration_weights_vector_(num_joints, 0.0),
      x_ref_(num_joints, 0.0),
      ee_z_jacobian_row_(casadi::DM::zeros(num_joints, 1)) {
  std::vector<double> jerk_min, jerk_max, acceleration_min, acceleration_max;
  if (nh.getParam("robot/jerk_max", jerk_max) && nh.getParam("robot/jerk_min", jerk_min) &&
      nh.getParam("robot/acceleration_max", acceleration_max) &&
      nh.getParam("robot/acceleration_min", acceleration_min)) {
    jerk_max_ = jerk_max;
    jerk_min_ = jerk_min;
    acceleration_max_ = acceleration_max;
    acceleration_min_ = acceleration_min;
    ROS_INFO_STREAM("Jerk Parameters:");
    ROS_INFO_STREAM("Jerk acceleration: " << jerk_max_);
    ROS_INFO_STREAM("Jerk acceleration: " << jerk_min_);
    ROS_INFO_STREAM("Acceleration Parameters:");
    ROS_INFO_STREAM("Max acceleration: " << acceleration_max_);
    ROS_INFO_STREAM("Min acceleration: " << acceleration_min_);
  } else {
    ROS_ERROR("Failed to load Jerk and Acceleration parameters.");
    ros::shutdown();
  }

  std::string mode_str;
  nh.param<std::string>("robot/solver_mode", mode_str, "qc");
  solver_mode_ = (mode_str == "time_scaling") ? SolverMode::TimeScaling : SolverMode::QuadraticCost;
  ROS_INFO_STREAM("MPC solver mode: " << mode_str);

  nh.param("robot/safety/ee_z_constraint_steps", ee_z_constraint_steps_, prediction_horizon_);
  ee_z_constraint_steps_ = std::max(0, std::min(ee_z_constraint_steps_, prediction_horizon_));
  ROS_INFO_STREAM("MPC EE-z linear constraints per solve: " << ee_z_constraint_steps_);

  initializeVariables();
  setDefaultConstraints();
  buildNLP();
  createSolver();
  initializeSolverBounds();

  dynamic_reconfigure_server_.setCallback(
      boost::bind(&MPCController::dynamicReconfigureCallback, this, _1, _2));
}

// ----------------------------------------------------------------------------
// Dynamic reconfigure callback
// ----------------------------------------------------------------------------
void MPCController::dynamicReconfigureCallback(mpc_controller::MpcParametersConfig& config,
                                               uint32_t level) {
  std::lock_guard<std::mutex> lock(config_mutex_);
  position_weights_vector_[0] = config.pos_weight_0;
  position_weights_vector_[1] = config.pos_weight_1;
  position_weights_vector_[2] = config.pos_weight_2;
  position_weights_vector_[3] = config.pos_weight_3;
  position_weights_vector_[4] = config.pos_weight_4;
  position_weights_vector_[5] = config.pos_weight_5;
  position_weights_vector_[6] = config.pos_weight_6;

  acceleration_weights_vector_[0] = config.accel_weight_0;
  acceleration_weights_vector_[1] = config.accel_weight_1;
  acceleration_weights_vector_[2] = config.accel_weight_2;
  acceleration_weights_vector_[3] = config.accel_weight_3;
  acceleration_weights_vector_[4] = config.accel_weight_4;
  acceleration_weights_vector_[5] = config.accel_weight_5;
  acceleration_weights_vector_[6] = config.accel_weight_6;

  x_ref_[0] = config.ref_0;
  x_ref_[1] = config.ref_1;
  x_ref_[2] = config.ref_2;
  x_ref_[3] = config.ref_3;
  x_ref_[4] = config.ref_4;
  x_ref_[5] = config.ref_5;
  x_ref_[6] = config.ref_6;

  ROS_INFO("Dynamic Reconfigure: Updated weights and reference positions.");
}

std::vector<double> MPCController::getReferencePositions() const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return x_ref_;
}

// ----------------------------------------------------------------------------
// Set constraints / cost weights
// ----------------------------------------------------------------------------
void MPCController::setPositionConstraints(const std::vector<double>& lower_bounds,
                                           const std::vector<double>& upper_bounds) {
  if (lower_bounds.size() != num_joints_ || upper_bounds.size() != num_joints_) {
    ROS_ERROR_STREAM("Position bounds size " << lower_bounds.size()
                                             << " must match the number of joints " << num_joints_
                                             << ". Shutting down!");
    ros::shutdown();
    return;
  }
  position_lower_bounds_ = lower_bounds;
  position_upper_bounds_ = upper_bounds;
}

void MPCController::setVelocityConstraints(const std::vector<double>& lower_bounds,
                                           const std::vector<double>& upper_bounds) {
  if (lower_bounds.size() != num_joints_ || upper_bounds.size() != num_joints_) {
    ROS_ERROR_STREAM("Velocity bounds size " << lower_bounds.size()
                                             << " must match the number of joints " << num_joints_);
    ros::shutdown();
    return;
  }
  velocity_lower_bounds_ = lower_bounds;
  velocity_upper_bounds_ = upper_bounds;
}

void MPCController::setEndEffectorZLinearConstraint(const std::vector<double>& jz_row,
                                                    double z_offset, bool enabled) {
  if (jz_row.size() != static_cast<size_t>(num_joints_)) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "EE-z Jacobian row size " << jz_row.size() << " must match num_joints "
                                        << num_joints_ << ". Disabling EE-z constraint.");
    ee_z_gate_ = 0.0;
    return;
  }

  ee_z_jacobian_row_ = casadi::DM(jz_row);
  ee_z_offset_ = z_offset;
  ee_z_gate_ = enabled ? 1.0 : 0.0;
}



// ----------------------------------------------------------------------------
// Initialize symbolic parameters
// ----------------------------------------------------------------------------
void MPCController::initializeVariables() {
  positions_ = casadi::MX::sym("positions", num_joints_);
  velocities_ = casadi::MX::sym("velocities", num_joints_);
  reference_positions_ = casadi::MX::sym("reference_positions", num_joints_);
  previous_accelerations_ = casadi::MX::sym("previous_accelerations", num_joints_);
}

//-------------------------------------------------------------------
// Initialize Solver Bounds and Initial Guess
//-------------------------------------------------------------------
void MPCController::initializeSolverBounds() {
  if (solver_mode_ == SolverMode::TimeScaling) {
    initializeBoundsTimeScaling();
  } else {
    initializeBoundsQuadraticCost();
  }
}

void MPCController::initializeBoundsTimeScaling() {
  int numStateVars = prediction_horizon_ * 2 * num_joints_;
  int numControlVars = control_horizon_ * num_joints_;
  int decisionVars = numStateVars + numControlVars + 1;  // +1 for final time T

  x_init_ = casadi::DM::zeros(decisionVars);
  lbx_ = casadi::DM::zeros(decisionVars);
  ubx_ = casadi::DM::zeros(decisionVars);

  int T_index = decisionVars - 1;
  x_init_(T_index) = initial_tf_;
  lbx_(T_index) = tf_min_;
  ubx_(T_index) = tf_max_;

  for (int i = 0; i < numStateVars; ++i) {
    lbx_(i) = -casadi::inf;
    ubx_(i) = casadi::inf;
  }

  int controlStartIndex = numStateVars;
  for (int i = 0; i < control_horizon_; ++i) {
    int start = controlStartIndex + i * num_joints_;
    int end = start + num_joints_;
    lbx_(casadi::Slice(start, end)) = acceleration_min_;
    ubx_(casadi::Slice(start, end)) = acceleration_max_;
  }

  int numConstraints = (6 * prediction_horizon_ * num_joints_) +
                       (2 * control_horizon_ * num_joints_) + (2 * num_joints_ + 1) +
                       ee_z_constraint_steps_;
  lbg_ = casadi::DM::zeros(numConstraints);
  ubg_ = casadi::DM::inf(numConstraints);

  int num_eq_constraints = 2 * prediction_horizon_ * num_joints_;
  for (int i = 0; i < num_eq_constraints; ++i) {
    lbg_(i) = 0.0;
    ubg_(i) = 0.0;
  }

  int posEqStart =
      (6 * prediction_horizon_ * num_joints_) + (2 * control_horizon_ * num_joints_) +
      ee_z_constraint_steps_;
  int posEqEnd = posEqStart + (2 * num_joints_);
  for (int i = posEqStart; i < posEqEnd; ++i) { ubg_(i) = 0.0; }

  ROS_INFO("Time-scaling solver bounds initialized.");
}

void MPCController::initializeBoundsQuadraticCost() {
  int numStateVars = prediction_horizon_ * 2 * num_joints_;
  int numControlVars = control_horizon_ * num_joints_;
  int decisionVars = numStateVars + numControlVars;

  x_init_ = casadi::DM::zeros(decisionVars);
  lbx_ = casadi::DM::zeros(decisionVars);
  ubx_ = casadi::DM::zeros(decisionVars);

  for (int i = 0; i < numStateVars; ++i) {
    lbx_(i) = -casadi::inf;
    ubx_(i) = casadi::inf;
  }

  for (int i = 0; i < control_horizon_; ++i) {
    int start = numStateVars + i * num_joints_;
    int end = start + num_joints_;
    lbx_(casadi::Slice(start, end)) = acceleration_min_;
    ubx_(casadi::Slice(start, end)) = acceleration_max_;
  }

  int numConstraints = (6 * prediction_horizon_ * num_joints_) +
                       (2 * control_horizon_ * num_joints_) + ee_z_constraint_steps_;
  lbg_ = casadi::DM::zeros(numConstraints);
  ubg_ = casadi::DM::inf(numConstraints);

  int num_eq_constraints = 2 * prediction_horizon_ * num_joints_;
  for (int i = 0; i < num_eq_constraints; ++i) {
    lbg_(i) = 0.0;
    ubg_(i) = 0.0;
  }

  ROS_INFO("QC solver bounds initialized.");
}

// ----------------------------------------------------------------------------
// Default constraints
// ----------------------------------------------------------------------------
void MPCController::setDefaultConstraints() {
  position_lower_bounds_ = {-97.494 * casadi::pi / 180,
                            -123 * casadi::pi / 180,
                            -174.987 * casadi::pi / 180,
                            -2.864 * casadi::pi / 180,
                            -175.25 * casadi::pi / 180,
                            -90 * casadi::pi / 180,
                            -175.25 * casadi::pi / 180};
  position_upper_bounds_ = {97.494 * casadi::pi / 180,
                            60 * casadi::pi / 180,
                            174.987 * casadi::pi / 180,
                            150 * casadi::pi / 180,
                            175.25 * casadi::pi / 180,
                            120 * casadi::pi / 180,
                            175.25 * casadi::pi / 180};

  velocity_lower_bounds_ = {-1.5, -1.5, -1.5, -1.5, -4.0, -4.0, -4.0};
  velocity_upper_bounds_ = {1.5, 1.5, 1.5, 1.5, 4.0, 4.0, 4.0};
}

void MPCController::validateInputs(const casadi::DM& positions, const casadi::DM& velocities,
                                   const casadi::DM& reference_positions,
                                   const casadi::DM& previous_accelerations) const {
  if (positions.size1() != num_joints_ || velocities.size1() != num_joints_ ||
      reference_positions.size1() != num_joints_ ||
      previous_accelerations.size1() != num_joints_) {
    throw std::invalid_argument("MPC input dimensions must match the number of joints");
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

  casadi::MX T = casadi::MX::sym("T");
  casadi::MX total_cost = casadi::MX::zeros(1);

  casadi::MX q_vector = casadi::MX::sym("q_vector", num_joints_, 1);
  casadi::MX r_vector = casadi::MX::sym("r_vector", num_joints_, 1);
  casadi::MX ee_jz_vector = casadi::MX::sym("ee_jz_vector", num_joints_, 1);
  casadi::MX ee_z_offset = casadi::MX::sym("ee_z_offset", 1, 1);
  casadi::MX ee_z_gate = casadi::MX::sym("ee_z_gate", 1, 1);
  casadi::MX q_mat = casadi::MX::diag(q_vector);
  casadi::MX r_mat = casadi::MX::diag(r_vector);

  std::vector<casadi::MX> X_vec(prediction_horizon_ + 1);
  std::vector<casadi::MX> V_vec(prediction_horizon_ + 1);
  double d_tau = 1.0 / prediction_horizon_;

  std::vector<casadi::MX> A_vec(control_horizon_);
  for (int i = 0; i < control_horizon_; ++i) {
    A_vec[i] = casadi::MX::sym("a_" + std::to_string(i), num_joints_);
  }

  X_vec[0] = positions_;
  V_vec[0] = T * velocities_;

  for (int k = 1; k <= prediction_horizon_; ++k) {
    X_vec[k] = casadi::MX::sym("X_" + std::to_string(k), num_joints_);
    V_vec[k] = casadi::MX::sym("V_" + std::to_string(k), num_joints_);
    decision_variables_.push_back(X_vec[k]);
    decision_variables_.push_back(V_vec[k]);
  }

  for (int i = 0; i < control_horizon_; ++i) { decision_variables_.push_back(A_vec[i]); }
  decision_variables_.push_back(T);

  for (int k = 0; k < prediction_horizon_; ++k) {
    int ctrl_idx = std::min(k, control_horizon_ - 1);
    casadi::MX a_k = A_vec[ctrl_idx];
    casadi::MX dyn_pos =
        X_vec[k + 1] - (X_vec[k] + d_tau * V_vec[k] + 0.5 * std::pow(d_tau, 2) * T * T * a_k);
    casadi::MX dyn_vel = V_vec[k + 1] - (V_vec[k] + d_tau * T * T * a_k);

    constraints_.push_back(dyn_pos);
    constraints_.push_back(dyn_vel);
    total_cost +=
        casadi::MX::mtimes(casadi::MX::mtimes((X_vec[k] - reference_positions_).T(), q_mat),
                           (X_vec[k] - reference_positions_)) +
        casadi::MX::mtimes(casadi::MX::mtimes(A_vec[ctrl_idx].T(), r_mat), A_vec[ctrl_idx]);
  }

  for (int k = 0; k < prediction_horizon_; ++k) {
    for (int j = 0; j < num_joints_; ++j) {
      constraints_.push_back(X_vec[k + 1](j) - position_lower_bounds_[j]);
      constraints_.push_back(position_upper_bounds_[j] - X_vec[k + 1](j));
      constraints_.push_back(V_vec[k + 1](j) - T * velocity_lower_bounds_[j]);
      constraints_.push_back(T * velocity_upper_bounds_[j] - V_vec[k + 1](j));
    }
  }

  for (int j = 0; j < num_joints_; ++j) {
    const casadi::MX initial_jerk =
        (A_vec[0](j) - previous_accelerations_(j)) / (d_tau * T);
    constraints_.push_back(initial_jerk - jerk_min_[j]);
    constraints_.push_back(jerk_max_[j] - initial_jerk);
  }

  for (int k = 0; k < control_horizon_ - 1; ++k) {
    for (int j = 0; j < num_joints_; ++j) {
      constraints_.push_back(((A_vec[k + 1](j) - A_vec[k](j)) / (d_tau * T)) - jerk_min_[j]);
      constraints_.push_back(jerk_max_[j] - ((A_vec[k + 1](j) - A_vec[k](j)) / (d_tau * T)));
    }
  }

  for (int k = 0; k < ee_z_constraint_steps_; ++k) {
    casadi::MX z_lin_margin = casadi::MX::mtimes(ee_jz_vector.T(), X_vec[k + 1]) - ee_z_offset;
    constraints_.push_back(ee_z_gate * z_lin_margin);
  }

  casadi::MX final_pos_error = X_vec[prediction_horizon_] - reference_positions_;
  constraints_.push_back(final_pos_error);
  constraints_.push_back(T * V_vec[prediction_horizon_]);
  constraints_.push_back(T - tf_min_);

  nlp_ = {{"x", casadi::MX::vertcat(decision_variables_)},
          {"f", total_cost},
          {"g", casadi::MX::vertcat(constraints_)},
        {"p",
         casadi::MX::vertcat(
           {positions_, velocities_, reference_positions_, previous_accelerations_, q_vector,
            r_vector, ee_jz_vector, ee_z_offset, ee_z_gate})}};
}

void MPCController::buildNlpQuadraticCost() {
  decision_variables_.clear();
  constraints_.clear();

  casadi::MX total_cost = casadi::MX::zeros(1);

  casadi::MX q_vector = casadi::MX::sym("q_vector", num_joints_, 1);
  casadi::MX r_vector = casadi::MX::sym("r_vector", num_joints_, 1);
  casadi::MX ee_jz_vector = casadi::MX::sym("ee_jz_vector", num_joints_, 1);
  casadi::MX ee_z_offset = casadi::MX::sym("ee_z_offset", 1, 1);
  casadi::MX ee_z_gate = casadi::MX::sym("ee_z_gate", 1, 1);
  casadi::MX q_mat = casadi::MX::diag(q_vector);
  casadi::MX r_mat = casadi::MX::diag(r_vector);

  std::vector<casadi::MX> X_vec(prediction_horizon_ + 1);
  std::vector<casadi::MX> V_vec(prediction_horizon_ + 1);
  std::vector<casadi::MX> A_vec(control_horizon_);

  for (int i = 0; i < control_horizon_; ++i) {
    A_vec[i] = casadi::MX::sym("a_" + std::to_string(i), num_joints_);
  }

  X_vec[0] = positions_;
  V_vec[0] = velocities_;

  for (int k = 1; k <= prediction_horizon_; ++k) {
    X_vec[k] = casadi::MX::sym("X_" + std::to_string(k), num_joints_);
    V_vec[k] = casadi::MX::sym("V_" + std::to_string(k), num_joints_);
    decision_variables_.push_back(X_vec[k]);
    decision_variables_.push_back(V_vec[k]);
  }

  for (int i = 0; i < control_horizon_; ++i) { decision_variables_.push_back(A_vec[i]); }

  for (int k = 0; k < prediction_horizon_; ++k) {
    int ctrl_idx = std::min(k, control_horizon_ - 1);
    casadi::MX a_k = A_vec[ctrl_idx];
    casadi::MX dyn_pos = X_vec[k + 1] - (X_vec[k] + sampling_time_ * V_vec[k] +
                                          0.5 * std::pow(sampling_time_, 2) * a_k);
    casadi::MX dyn_vel = V_vec[k + 1] - (V_vec[k] + sampling_time_ * a_k);

    constraints_.push_back(dyn_pos);
    constraints_.push_back(dyn_vel);
    total_cost +=
        casadi::MX::mtimes(casadi::MX::mtimes((X_vec[k] - reference_positions_).T(), q_mat),
                           (X_vec[k] - reference_positions_)) +
        casadi::MX::mtimes(casadi::MX::mtimes(A_vec[ctrl_idx].T(), r_mat), A_vec[ctrl_idx]);
  }

  for (int k = 0; k < prediction_horizon_; ++k) {
    for (int j = 0; j < num_joints_; ++j) {
      constraints_.push_back(X_vec[k + 1](j) - position_lower_bounds_[j]);
      constraints_.push_back(position_upper_bounds_[j] - X_vec[k + 1](j));
      constraints_.push_back(V_vec[k + 1](j) - velocity_lower_bounds_[j]);
      constraints_.push_back(velocity_upper_bounds_[j] - V_vec[k + 1](j));
    }
  }

  for (int j = 0; j < num_joints_; ++j) {
    const casadi::MX initial_jerk =
        (A_vec[0](j) - previous_accelerations_(j)) / sampling_time_;
    constraints_.push_back(initial_jerk - jerk_min_[j]);
    constraints_.push_back(jerk_max_[j] - initial_jerk);
  }

  for (int k = 0; k < control_horizon_ - 1; ++k) {
    for (int j = 0; j < num_joints_; ++j) {
      constraints_.push_back(((A_vec[k + 1](j) - A_vec[k](j)) / sampling_time_) - jerk_min_[j]);
      constraints_.push_back(jerk_max_[j] - ((A_vec[k + 1](j) - A_vec[k](j)) / sampling_time_));
    }
  }

  for (int k = 0; k < ee_z_constraint_steps_; ++k) {
    casadi::MX z_lin_margin = casadi::MX::mtimes(ee_jz_vector.T(), X_vec[k + 1]) - ee_z_offset;
    constraints_.push_back(ee_z_gate * z_lin_margin);
  }

  nlp_ = {{"x", casadi::MX::vertcat(decision_variables_)},
          {"f", total_cost},
          {"g", casadi::MX::vertcat(constraints_)},
        {"p",
         casadi::MX::vertcat(
           {positions_, velocities_, reference_positions_, previous_accelerations_, q_vector,
            r_vector, ee_jz_vector, ee_z_offset, ee_z_gate})}};
}

// ----------------------------------------------------------------------------
// Create the solver
// ----------------------------------------------------------------------------
void MPCController::createSolver() {
  casadi::Dict solver_options;
  solver_options["ipopt.print_level"] = 1;
  solver_options["print_time"] = 0;
  solver_options["ipopt.linear_solver"] = "mumps";
  solver_ = casadi::nlpsol("solver_", "ipopt", nlp_, solver_options);

  ROS_INFO("MPCController solver created with IPOPT backend!");
}

std::map<std::string, casadi::DM> MPCController::prepareSolverArguments(
    const casadi::DM& positions, const casadi::DM& velocities,
    const casadi::DM& reference_positions, const casadi::DM& previous_accelerations) {
  std::map<std::string, casadi::DM> args;
  args["x0"] = x_init_;
  args["lbx"] = lbx_;
  args["ubx"] = ubx_;
  casadi::DM q_weights;
  casadi::DM r_weights;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    q_weights = position_weights_vector_;
    r_weights = acceleration_weights_vector_;
  }
  args["p"] = casadi::DM::vertcat(
      {positions, velocities, reference_positions, previous_accelerations, q_weights, r_weights,
       ee_z_jacobian_row_, casadi::DM(ee_z_offset_), casadi::DM(ee_z_gate_)});
  args["lbg"] = lbg_;
  args["ubg"] = ubg_;

  return args;
}

casadi::DM MPCController::solve(const casadi::DM& initial_positions,
                                const casadi::DM& initial_velocities,
                                const casadi::DM& reference_positions,
                                const casadi::DM& previous_accelerations) {
  validateInputs(initial_positions, initial_velocities, reference_positions,
                 previous_accelerations);

  std::map<std::string, casadi::DM> solver_args =
      prepareSolverArguments(initial_positions, initial_velocities, reference_positions,
                             previous_accelerations);

  auto result = solver_(solver_args);
  
  // Check solver status via stats (available after calling the solver)
  auto stats = solver_.stats();
  std::ostringstream oss;
  oss << stats.at("return_status");
  const std::string return_status = oss.str();
  if (return_status != "Solve_Succeeded" && return_status != "Solved_To_Acceptable_Level") {
    throw std::runtime_error("MPC solver failed with return_status: " + return_status);
  }
  
  casadi::DM optimal_solution = result["x"];

  int num_state_vars = prediction_horizon_ * 2 * num_joints_;

  std::vector<casadi::DM> accelerations;
  for (int k = 0; k < control_horizon_; ++k) {
    int start_index = num_state_vars + k * num_joints_;
    int end_index = start_index + num_joints_;
    casadi::DM acceleration_value = optimal_solution(casadi::Slice(start_index, end_index));
    accelerations.push_back(acceleration_value);
  }

  if (solver_mode_ == SolverMode::TimeScaling) {
    casadi::DM tf_value = optimal_solution(optimal_solution.size1() - 1);
    ROS_INFO_STREAM("----- MPC SOLUTION -----\n"
                    << " reference positions: " << reference_positions << "\n"
                    << " total_time: " << tf_value);

    casadi::DM X = initial_positions;
    casadi::DM V = tf_value * initial_velocities;
    double d_tau = 1.0 / prediction_horizon_;

    for (int k = 1; k <= prediction_horizon_; ++k) {
      casadi::DM applied_acceleration =
          (k - 1 < control_horizon_) ? accelerations[k - 1] : accelerations.back();
      X = X + d_tau * V + 0.5 * std::pow(d_tau, 2) * tf_value * tf_value * applied_acceleration;
      V = V + d_tau * tf_value * tf_value * applied_acceleration;
      if (k == 1) {
        referent_angles_ = X;
        referent_velocities_ = V / tf_value;
      }
    }
  } else {
    casadi::DM X = initial_positions;
    casadi::DM V = initial_velocities;

    for (int k = 1; k <= prediction_horizon_; ++k) {
      casadi::DM applied_acceleration =
          (k - 1 < control_horizon_) ? accelerations[k - 1] : accelerations.back();
      X = X + sampling_time_ * V + 0.5 * sampling_time_ * sampling_time_ * applied_acceleration;
      V = V + sampling_time_ * applied_acceleration;
      if (k == 1) {
        referent_angles_ = X;
        referent_velocities_ = V;
      }
    }
  }

  x_init_ = optimal_solution;
  return result["x"];
}

casadi::DM MPCController::getReferentAngles() const {
  return referent_angles_;
}

casadi::DM MPCController::getReferentVelocities() const {
  return referent_velocities_;
}

std::unordered_map<std::string, double> MPCController::dmToMap(
    const casadi::DM& matrix, const std::vector<std::string>& keys) const {
  std::unordered_map<std::string, double> result;
  int keyIndex = 0;

  for (casadi_int i = 0; i < matrix.size1(); ++i) {
    for (casadi_int j = 0; j < matrix.size2(); ++j) {
      result[keys[keyIndex]] = static_cast<double>(matrix(i, j));
      ++keyIndex;
    }
  }
  return result;
}

casadi::DM MPCController::unorderedMapToDM(const std::unordered_map<std::string, double>& input_map,
                                           const std::vector<std::string>& joint_names) {
  casadi::DM result = casadi::DM::zeros(joint_names.size(), 1);
  for (size_t i = 0; i < joint_names.size(); ++i) {
    if (input_map.find(joint_names[i]) != input_map.end()) {
      result(i, 0) = input_map.at(joint_names[i]);
    } else {
      throw std::runtime_error("Joint name not found in input map.");
    }
  }
  return result;
}
