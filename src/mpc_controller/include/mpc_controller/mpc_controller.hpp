#pragma once

#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <mpc_controller/MpcParametersConfig.h>
#include <baxter_interface_cpp/limb.hpp>

enum class SolverMode { TimeScaling, QuadraticCost };

/**
 * @class MPCController
 * @brief A Model Predictive Controller (MPC) for joint-space trajectory
 * planning.
 *
 * This class sets up an MPC optimization problem using CasADi's low-level
 * nlpsol interface (e.g., IPOPT). It can optimize over a specified prediction
 * horizon in discrete time.
 */
class MPCController {
 public:
  /**
   * @brief Constructs the MPCController.
   * @param num_joints         Number of robot joints.
   * @param prediction_horizon Number of discrete steps in the prediction
   * horizon.
   * @param sampling_time      Sampling time for each control step (seconds).
   *
   * Initializes symbolic variables, default constraints, builds the NLP, and
   * creates the solver. Also sets up dynamic reconfigure for run-time tuning of
   * some parameters.
   */
  MPCController(ros::NodeHandle& nh, int num_joints, int prediction_horizon, int control_horizon,
                double sampling_time);

  /**
   * @brief Dynamic reconfigure callback to update parameters.
   * @param config Configuration parameters from dynamic reconfigure.
   * @param level Level indicator for the callback.
   */
  void dynamicReconfigureCallback(mpc_controller::MpcParametersConfig& config, uint32_t level);

  /**
   * @brief Solve the MPC problem for a given set of initial and reference
   * states.
   * @param initial_positions   A DM of size (num_joints_, 1) with the current
   * joint positions.
   * @param initial_velocities  A DM of size (num_joints_, 1) with the current
   * joint velocities.
   * @param reference_positions A DM of size (num_joints_, 1) with the desired
   * final positions.
   * @param previous_accelerations A DM containing the accelerations applied in
   * the previous control cycle.
   * @return The raw CasADi DM solution vector containing [total_time,
   * a_0...a_{N-1}].
   *
   * The first entry is total_time, followed by the accelerations for each
   * discrete step. The function also prints debug info about the solution and
   * simulates the trajectory steps in console output.
   */
  casadi::DM solve(const casadi::DM& initial_positions, const casadi::DM& initial_velocities,
                   const casadi::DM& reference_positions,
                   const casadi::DM& previous_accelerations);

  /**
   * @brief Update linearized end-effector z-floor constraint used in MPC.
   *
   * The constraint enforced is: z_gate * (jz_row * q_k - z_offset) >= 0,
   * for the configured number of prediction steps.
   *
   * @param jz_row   Row vector d(z)/d(q) in MPC joint order (size num_joints).
   * @param z_offset Affine offset term for linearized bound.
   * @param enabled  Whether the z-floor constraint is active in this solve.
   */
  void setEndEffectorZLinearConstraint(const std::vector<double>& jz_row, double z_offset,
                                       bool enabled);

  /**
   * @brief Sets position constraints for the joints.
   * @param lower_bounds Lower bounds for joint positions.
   * @param upper_bounds Upper bounds for joint positions.
   */
  void setPositionConstraints(const std::vector<double>& lower_bounds,
                              const std::vector<double>& upper_bounds);

  /**
   * @brief Sets velocity constraints for the joints.
   * @param lower_bounds Lower bounds for joint velocities.
   * @param upper_bounds Upper bounds for joint velocities.
   */
  void setVelocityConstraints(const std::vector<double>& lower_bounds,
                              const std::vector<double>& upper_bounds);

  /**
   * @brief Converts a CasADi DM matrix to an unordered map.
   * @param matrix Input CasADi DM matrix.
   * @param keys Keys for the map corresponding to matrix elements.
   * @return Unordered map containing matrix elements (key -> value).
   */
  std::unordered_map<std::string, double> dmToMap(const casadi::DM& matrix,
                                                  const std::vector<std::string>& keys) const;

  /**
   * @brief Converts an unordered map to a CasADi DM matrix.
   * @param input_map Input map with joint names as keys and values.
   * @param joint_names Vector of joint names in desired order.
   * @return CasADi DM matrix representing the map values.
   * @throws std::runtime_error if a joint name is not found in input_map.
   */
  casadi::DM unorderedMapToDM(const std::unordered_map<std::string, double>& input_map,
                              const std::vector<std::string>& joint_names);
  
  /**
   * @brief Get the referent (target) joint angles from the last solve.
   * @return CasADi DM vector of joint angles.
   */
  casadi::DM getReferentAngles() const;
  
  /**
   * @brief Get the referent (target) joint velocities from the last solve.
   * @return CasADi DM vector of joint velocities.
   */
  casadi::DM getReferentVelocities() const;

  /**
   * @brief Get a thread-safe snapshot of the dynamic-reconfigure reference positions.
   */
  std::vector<double> getReferencePositions() const;

 private:
  /**
   * @brief Initializes symbolic variables for positions, velocities, and
   * references.
   */
  void initializeVariables();
  void initializeSolverBounds();
  void initializeBoundsTimeScaling();
  void initializeBoundsQuadraticCost();

  /**
   * @brief Sets default constraints for positions and velocities.
   */
  void setDefaultConstraints();

  /**
   * @brief Validate input dimensions for positions, velocities, etc.
   * @param positions            DM for initial positions.
   * @param velocities           DM for initial velocities.
   * @param reference_positions  DM for reference positions.
   * @param previous_accelerations DM for accelerations applied in the prior cycle.
   *
   * @throws std::invalid_argument if any dimension does not match num_joints_.
   */
  void validateInputs(const casadi::DM& positions, const casadi::DM& velocities,
                      const casadi::DM& reference_positions,
                      const casadi::DM& previous_accelerations) const;

  /**
   * @brief Build the symbolic NLP (decision variables, constraints, cost).
   *
   * Creates total_time as a decision variable, plus an acceleration decision
   * variable for each step. Defines the cost as total_time, plus constraints
   * for positions, velocities, final position, etc. Finally, populates nlp_
   * with (x, f, g, p).
   */
  void buildNLP();
  void buildNlpTimeScaling();
  void buildNlpQuadraticCost();

  /**
   * @brief Helper to build common NLP structure with parameterized dynamics.
   * @param use_time_scaling If true, uses time-scaled dynamics with T variable.
   */
  void buildNlpCommon(bool use_time_scaling);

  /**
   * @brief Create the solver (e.g. IPOPT) from the nlp_ dictionary.
   */
  void createSolver();

  /**
   * @brief Prepare the solver arguments (initial guess, bounds, parameters,
   * etc.) for solve().
   * @param positions            The current joint positions.
   * @param velocities           The current joint velocities.
   * @param reference_positions  The desired final positions.
   * @param previous_accelerations Accelerations applied in the prior cycle.
   * @return A std::map<std::string, casadi::DM> containing x0, lbx, ubx, p,
   * lbg, ubg.
   */
  std::map<std::string, casadi::DM> prepareSolverArguments(const casadi::DM& positions,
                                                           const casadi::DM& velocities,
                                                           const casadi::DM& reference_positions,
                                                           const casadi::DM& previous_accelerations);

  int num_joints_;
  int prediction_horizon_;
  int control_horizon_;
  double sampling_time_;
  double tf_min_{0.1};
  double initial_tf_{1.0};
  double tf_max_{3};

  casadi::DM x_init_;  ///< Initial guess for decision variables
  casadi::DM lbx_;     ///< Lower bound on decision variables
  casadi::DM ubx_;     ///< Upper bound on decision variables
  casadi::DM lbg_;     ///< Lower bound on constraints
  casadi::DM ubg_;     ///< Upper bound on constraints
  casadi::MX positions_;
  casadi::MX velocities_;
  casadi::MX reference_positions_;
  casadi::MX previous_accelerations_;
  std::vector<casadi::MX> decision_variables_;
  std::vector<casadi::MX> constraints_;

  std::vector<double> position_lower_bounds_;
  std::vector<double> position_upper_bounds_;
  std::vector<double> velocity_lower_bounds_;
  std::vector<double> velocity_upper_bounds_;
  std::vector<double> position_weights_vector_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> acceleration_weights_vector_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> x_ref_;
  mutable std::mutex config_mutex_;
  std::vector<double> acceleration_max_;
  std::vector<double> acceleration_min_;
  std::vector<double> jerk_min_;
  std::vector<double> jerk_max_;

  int ee_z_constraint_steps_{0};
  casadi::DM ee_z_jacobian_row_;
  double ee_z_offset_{0.0};
  double ee_z_gate_{0.0};

  casadi::DM referent_velocities_;
  casadi::DM referent_angles_;

  SolverMode solver_mode_;

  casadi::MXDict nlp_;
  casadi::Function solver_;

  dynamic_reconfigure::Server<mpc_controller::MpcParametersConfig> dynamic_reconfigure_server_;
};
