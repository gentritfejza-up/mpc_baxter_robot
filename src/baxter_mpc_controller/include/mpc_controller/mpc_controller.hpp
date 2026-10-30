#pragma once

/**
 * @file mpc_controller.hpp
 * @brief Joint-space model-predictive controller for one Baxter arm: a convex QP
 *        (CasADi + OSQP by default) over decoupled double integrators per joint with
 *        soft state bounds, jerk limits and an optional linearised collision-guard block.
 *
 * Quadratic-cost formulation (the one used at run time):
 * - states X_k, V_k (positions [rad], velocities [rad/s]) for k = 1..N, inputs a_k
 *   (accelerations [rad/s^2]) for k = 0..Nc-1, held constant after the control horizon;
 * - dynamics X_{k+1} = X_k + dt_k V_k + dt_k^2/2 a_k, V_{k+1} = V_k + dt_k a_k;
 * - cost sum_k (dt_k / 0.1) [ (X_k - ref)^T Q (X_k - ref) + a^T R a + ratio V_k^T Q V_k ]
 *   plus quadratic (optionally linear) penalties on the bound slacks and the geometry slacks;
 * - soft position/velocity bounds X_k +/- S >= / <= limits, hard acceleration bounds,
 *   jerk limits on consecutive accelerations, and parametric geometry rows
 *   c^T (X_k + tau V_k) + G_k[sphere] >= lb (see geometry_rows.hpp).
 *
 * The legacy time-scaling formulation (final time as a decision variable, IPOPT) is kept
 * for comparison and has neither soft bounds nor the collision guard.
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include <casadi/casadi.hpp>
#include <ros/ros.h>

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <baxter_interface_cpp/limb.hpp>
#include <dynamic_reconfigure/server.h>
#include <baxter_mpc_controller/MpcParametersConfig.h>

#include "mpc_controller/geometry_rows.hpp"

/** @brief Optimal-control formulation (robot/solver_mode: "time_scaling" or "qc"). */
enum class SolverMode { TimeScaling, QuadraticCost };

/** @brief Solver plugin (robot/solver_backend). */
enum class SolverBackend { Ipopt, Osqp, Qpoases, Qrqp, Ipqp };

/** @brief Wall-clock breakdown of the last solve() call. */
struct MpcSolveTiming {
  double argument_preparation_ms{0.0};  ///< Building the parameter vector and warm start [ms].
  double solver_ms{0.0};                ///< Time inside the CasADi solver call [ms].
  double validation_ms{0.0};            ///< Feasibility and status check of the result [ms].
};

/** @brief Inputs of one MPC solve. */
struct MpcSolveInput {
  casadi::DM positions;                ///< Measured joint positions, num_joints x 1 [rad].
  casadi::DM velocities;               ///< Measured joint velocities, num_joints x 1 [rad/s].
  casadi::DM reference_positions;      ///< Reference joint positions, num_joints x 1 [rad].
  casadi::DM previous_accelerations;   ///< Accelerations applied last cycle, num_joints x 1 [rad/s^2].
  double first_jerk_dt{0.0};           ///< Time since the previous command, for the first jerk row [s].
  /// Linearised collision-guard rows for this cycle; nullptr leaves every geometry row
  /// inactive. Ignored when the controller has no geometry block.
  const mpc_geometry::GeometryRowBatch* geometry{nullptr};
};

/**
 * @brief Builds the MPC problem once and solves it every control cycle.
 *
 * Thread safety: solve() and the solution accessors belong to the control thread; the
 * dynamic-reconfigure callback (weights and reference) may run on a ROS spinner thread
 * and is guarded by a mutex.
 */
class MPCController {
 public:
  /**
   * @param node_handle Node handle for the robot/... parameters and the reconfigure server.
   * @param num_joints Joints of the arm (7 for Baxter).
   * @param prediction_horizon N, prediction steps.
   * @param control_horizon Nc <= N, steps with a free acceleration.
   * @param prediction_step_durations One duration per prediction step [s].
   * @param nominal_control_period Control period [s]; upper limit of the first jerk interval.
   * @param geometry_sphere_count > 0 adds the collision-guard block (rows and slacks sized
   *        from robot/geometry/rows_per_sphere) to the quadratic-cost QP; 0 disables it.
   */
  MPCController(ros::NodeHandle& node_handle, int num_joints, int prediction_horizon,
                int control_horizon, const std::vector<double>& prediction_step_durations,
                double nominal_control_period, int geometry_sphere_count = 0);

  /**
   * @brief Dynamic-reconfigure callback: position/acceleration weights and the reference.
   * @param config New values (pos_weight_i, accel_weight_i, ref_i).
   * @param level Unused.
   */
  void dynamicReconfigureCallback(mpc_controller::MpcParametersConfig& config, uint32_t level);

  /**
   * @brief Solves the QP for the given state.
   *
   * On success the solution is kept pending: it already seeds the next warm start, and
   * commitSolution() marks it as applied. Throws std::invalid_argument on bad inputs and
   * std::runtime_error when the solver fails or the result violates the constraints by
   * more than robot/solver/feasibility_tolerance.
   * @param input Measured state, reference and geometry rows.
   * @return The full decision vector (see QcLayout for the layout).
   */
  casadi::DM solve(const MpcSolveInput& input);

  /** @brief Marks the pending solution as applied. Throws std::logic_error if none is pending. */
  void commitSolution();

  /** @brief Clears the warm start; the next solve is seeded from the measured state. */
  void resetWarmStart();

  /**
   * @brief Zeroes the dual warm start (keeps the primal). An ADMM solve that stalls at the
   *        cap right after a converged one usually carries a bad dual estimate.
   */
  void resetDualWarmStart();

  /**
   * @brief Converts a CasADi vector into a joint-name map.
   * @param matrix Vector with one entry per key.
   * @param keys Joint names in the vector's order.
   * @return Map joint name -> value.
   */
  static std::unordered_map<std::string, double> dmToMap(const casadi::DM& matrix,
                                                         const std::vector<std::string>& keys);

  /**
   * @brief First acceleration a_0 of a solution.
   * @param solution Decision vector returned by solve().
   * @return num_joints x 1 [rad/s^2].
   */
  casadi::DM getFirstAcceleration(const casadi::DM& solution) const;

  /** @brief Latest reference positions set through dynamic reconfigure [rad]. */
  std::vector<double> getReferencePositions() const;

  /**
   * @brief True once a client has set the reference (the dynamic-reconfigure server itself
   *        calls back once at start-up with the defaults, which do not count).
   */
  bool hasExternalReference() const;

  /** @brief Joint position lower bounds used by the QP [rad]. */
  const std::vector<double>& positionLowerBounds() const { return position_lower_bounds_; }
  /** @brief Joint position upper bounds used by the QP [rad]. */
  const std::vector<double>& positionUpperBounds() const { return position_upper_bounds_; }
  /** @brief Joint velocity lower bounds used by the QP [rad/s]. */
  const std::vector<double>& velocityLowerBounds() const { return velocity_lower_bounds_; }
  /** @brief Joint velocity upper bounds used by the QP [rad/s]. */
  const std::vector<double>& velocityUpperBounds() const { return velocity_upper_bounds_; }

  /** @brief True when the last solve ended at the iteration cap (its feasible iterate was used). */
  bool lastSolveInaccurate() const { return last_solve_inaccurate_; }
  /** @brief Timing of the last solve() call. */
  MpcSolveTiming getLastSolveTiming() const { return last_solve_timing_; }
  /** @brief Name of the solver plugin ("osqp", "ipopt", ...). */
  std::string solverBackendName() const;
  /** @brief Length of the decision vector. */
  int decisionVariableCount() const { return static_cast<int>(primal_warm_start_.size1()); }
  /** @brief True when the position/velocity bounds are soft (slack variables). */
  bool softStateBounds() const { return soft_state_bounds_; }
  /** @brief True when the QP has a collision-guard block. */
  bool geometryEnabled() const { return layout_.geometry_rows > 0; }
  /** @brief Layout of the collision-guard block (sphere count, rows per sphere, step times). */
  mpc_geometry::GeometryLayout geometryLayout() const;

  /**
   * @brief Predicted joint positions X_1..X_N of a solution.
   * @param solution Decision vector returned by solve().
   * @return One joint vector per prediction step [rad].
   */
  std::vector<mpc_geometry::JointVector> predictedPositions(const casadi::DM& solution) const;

  /**
   * @brief Largest collision-guard slack of a solution.
   * @param solution Decision vector returned by solve().
   * @return Slack [m]; 0 when there is no geometry block.
   */
  double maxGeometrySlack(const casadi::DM& solution) const;

 private:
  /**
   * @brief Variable and constraint-row layout of the quadratic-cost QP.
   *
   * Filled once by computeQcLayout() right after validateConfiguration(). Every offset
   * used by buildNlpQuadraticCost(), initializeBoundsQuadraticCost() and the solution
   * accessors comes from here, so a block appended to the decision vector or to the
   * constraint vector only has to be added to this struct.
   *
   * Decision vector: X_1,V_1,...,X_N,V_N | a_0..a_{Nc-1} | bound slacks S | geometry slacks G
   * Constraint rows: dynamics (== 0) | first-step jerk | jerk transitions |
   *                  soft bounds (all >= 0) | geometry rows (>= per-cycle lower bound)
   *
   * The state/acceleration prefix is shared with the time-scaling formulation (which
   * appends only the final time), so positionOffset/velocityOffset/accelerationOffset
   * are valid for both modes.
   */
  struct QcLayout {
    int num_joints{0};
    int state_variables{0};          ///< 2 * N * num_joints.
    int acceleration_variables{0};   ///< Nc * num_joints.
    int bound_slack_blocks{0};       ///< Number of S blocks: N (per step), 1 (shared) or 0.
    int bound_slack_variables{0};    ///< bound_slack_blocks * 2 * num_joints.
    int geometry_slack_variables{0}; ///< N * spheres.
    int decision_variables{0};       ///< Total length of the decision vector.
    int dynamics_rows{0};            ///< 2 * N * num_joints equality rows.
    int first_jerk_rows{0};          ///< 2 * num_joints, or 0 when the first-step jerk is not enforced.
    int jerk_rows{0};                ///< 2 * (Nc - 1) * num_joints.
    int bound_rows{0};               ///< 4 * N * num_joints, or 0 with hard bounds.
    int geometry_slots_per_step{0};  ///< spheres * slots per sphere.
    int geometry_rows{0};            ///< N * geometry_slots_per_step.
    int constraint_rows{0};          ///< Total number of constraint rows.
    int acceleration_offset{0};      ///< Variable offset of a_0.
    int bound_slack_offset{0};       ///< Variable offset of the first S block.
    int geometry_slack_offset{0};    ///< Variable offset of G_1.
    int first_jerk_row_offset{0};    ///< Row offset of the first-step jerk rows.
    int jerk_row_offset{0};          ///< Row offset of the jerk transition rows.
    int bound_row_offset{0};         ///< Row offset of the soft bound rows.
    int geometry_row_offset{0};      ///< Row offset of the geometry rows.
    /// Variable offset of X_step, step in 1..N.
    int positionOffset(int step) const { return (step - 1) * 2 * num_joints; }
    /// Variable offset of V_step, step in 1..N.
    int velocityOffset(int step) const { return positionOffset(step) + num_joints; }
    /// Variable offset of a_step, step in 0..Nc-1.
    int accelerationOffset(int step) const { return acceleration_offset + step * num_joints; }
  };

  void loadSolverParameters(ros::NodeHandle& node_handle);
  void setDefaultConstraints();
  void validateConfiguration() const;
  void computeQcLayout();
  void initializeVariables();
  void buildNLP();
  void buildNlpTimeScaling();
  void buildNlpQuadraticCost();
  void initializeSolverBounds();
  void initializeBoundsTimeScaling();
  void initializeBoundsQuadraticCost();
  void createSolver();

  void validateInputs(const MpcSolveInput& input) const;
  void validateGeometryBatch(const mpc_geometry::GeometryRowBatch* batch) const;
  casadi::DMDict prepareSolverArguments(const MpcSolveInput& input);
  /// Seeds the primal warm start with the measured state repeated over the horizon.
  void seedWarmStartFromState(const casadi::DM& positions, const casadi::DM& velocities);
  /// Adopts a solver result as the warm start of the next solve (when all parts are finite).
  void adoptWarmStart(const casadi::DMDict& result);
  /// Throws unless the result is finite, the solver succeeded (or stopped at a cap) and
  /// the constraints hold within feasibility_tolerance_.
  void validateResult(const casadi::DMDict& result, const casadi::DM& constraint_lower_bounds);

  // Problem dimensions and timing.
  int num_joints_;
  int prediction_horizon_;
  int control_horizon_;
  std::vector<double> prediction_step_durations_;  ///< [s]
  double nominal_control_period_;                  ///< [s]

  // Time-scaling formulation: bounds and seed of the final time [s].
  double final_time_min_{0.1};
  double final_time_initial_{1.0};
  double final_time_max_{3.0};

  // Solver options (robot/solver/*).
  double feasibility_tolerance_{5e-3};
  double osqp_eps_abs_{1e-4};
  double osqp_eps_rel_{1e-4};
  int osqp_max_iter_{3000};
  bool osqp_polishing_{true};
  double osqp_rho_{0.1};
  bool osqp_adaptive_rho_{true};
  int osqp_adaptive_rho_interval_{0};  ///< 0: OSQP decides from the setup/solve time ratio.
  double osqp_time_limit_sec_{0.0};    ///< 0: no limit; else the iterate at the limit is used if feasible [s].
  bool osqp_warm_start_{true};
  bool enforce_first_step_jerk_{false};
  /// Soft joint position/velocity bounds: non-negative slack variables keep the QP
  /// feasible when the measured state is already outside the hard limits (contact
  /// impacts, velocity overshoot). Acceleration bounds stay hard. Benchmarked on a
  /// 2 rad reference jump (OSQP, 1.5 s horizon): an L1 (linear) slack penalty stalls
  /// OSQP at its iteration cap, a purely quadratic penalty with one slack pair per
  /// joint per step converges in ~2 ms warm.
  bool soft_state_bounds_{true};
  bool slack_per_step_{true};  ///< false: one slack pair per joint for the whole horizon.
  double slack_linear_weight_{0.0};
  double slack_quadratic_weight_{1.0e4};
  /// Velocity penalty as a fraction of the position weight (towards zero velocity):
  /// damps the otherwise bang-bang approach and reduces overshoot.
  double velocity_weight_ratio_{0.05};

  /// Collision-guard block (quadratic-cost formulation only). Per prediction step and
  /// sphere there are slots-per-sphere linear rows  c^T X_k + G_k[sphere] >= lb  whose
  /// coefficients c are parameters and whose lower bound is set per cycle (-inf when the
  /// slot is inactive); G_k >= 0 is one quadratic slack per sphere and step shared by
  /// that sphere's slots.
  int geometry_sphere_count_{0};
  int geometry_rows_floor_{1};
  int geometry_rows_body_{1};
  int geometry_rows_other_arm_{1};
  double geometry_slack_weight_{1.0e4};
  /// Geometry rows act on X_k + tau V_k, the position tau seconds ahead along the
  /// predicted velocity: an approach-rate damper, so a plan cannot ride a keep-out
  /// margin with a nonzero approach speed (re-linearisation chatter). 0 disables [s].
  double geometry_velocity_lookahead_sec_{0.0};

  QcLayout layout_;

  // Symbolic problem.
  casadi::MX positions_symbol_;               ///< Parameter: measured positions.
  casadi::MX velocities_symbol_;              ///< Parameter: measured velocities.
  casadi::MX reference_positions_symbol_;     ///< Parameter: reference positions.
  casadi::MX previous_accelerations_symbol_;  ///< Parameter: last applied accelerations.
  casadi::MX first_jerk_dt_symbol_;           ///< Parameter: first jerk interval.
  casadi::MX geometry_coefficients_symbol_;   ///< Parameter: geometry_rows * num_joints row gradients.
  std::vector<casadi::MX> decision_variables_;
  std::vector<casadi::MX> constraints_;
  casadi::MXDict problem_;
  casadi::Function solver_;

  // Numeric bounds, warm start and the pending solution.
  casadi::DM decision_lower_bounds_;
  casadi::DM decision_upper_bounds_;
  casadi::DM constraint_lower_bounds_;  ///< Geometry rows at -inf (inactive).
  casadi::DM constraint_upper_bounds_;
  std::vector<double> constraint_lower_bounds_template_;  ///< constraint_lower_bounds_ as a plain vector.
  std::vector<double> constraint_lower_bounds_cycle_;     ///< Scratch for the per-cycle lower bounds.
  std::vector<double> inactive_geometry_coefficients_;    ///< All zeros, used when no batch is given.
  casadi::DM primal_warm_start_;
  casadi::DM dual_bounds_warm_start_;       ///< lam_x
  casadi::DM dual_constraints_warm_start_;  ///< lam_g
  bool warm_start_seeded_{false};  ///< false: seed the next solve from the measured state.
  casadi::DM pending_primal_;
  casadi::DM pending_dual_bounds_;
  casadi::DM pending_dual_constraints_;
  bool has_pending_solution_{false};
  bool last_solve_inaccurate_{false};
  MpcSolveTiming last_solve_timing_;

  // Joint limits [rad], [rad/s], [rad/s^2], [rad/s^3].
  std::vector<double> position_lower_bounds_;
  std::vector<double> position_upper_bounds_;
  std::vector<double> velocity_lower_bounds_;
  std::vector<double> velocity_upper_bounds_;
  std::vector<double> acceleration_min_;
  std::vector<double> acceleration_max_;
  std::vector<double> jerk_min_;
  std::vector<double> jerk_max_;

  // Reconfigurable values (guarded by config_mutex_).
  mutable std::mutex config_mutex_;
  std::vector<double> position_weights_;      ///< Q diagonal.
  std::vector<double> acceleration_weights_;  ///< R diagonal.
  std::vector<double> reference_positions_;   ///< Reference set through reconfigure [rad].
  int reference_updates_{0};

  SolverMode solver_mode_{SolverMode::QuadraticCost};
  SolverBackend solver_backend_{SolverBackend::Osqp};

  dynamic_reconfigure::Server<mpc_controller::MpcParametersConfig> dynamic_reconfigure_server_;
};
