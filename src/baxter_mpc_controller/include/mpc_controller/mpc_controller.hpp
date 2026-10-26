#pragma once

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

enum class SolverMode { TimeScaling, QuadraticCost };
enum class SolverBackend { Ipopt, Osqp, Qpoases, Qrqp, Ipqp };

struct MpcSolveTiming {
  double argument_preparation_ms{0.0};
  double solver_ms{0.0};
  double validation_ms{0.0};
};

class MPCController {
 public:
  MPCController(ros::NodeHandle& nh, int num_joints, int prediction_horizon,
                int control_horizon, const std::vector<double>& prediction_step_durations,
                double nominal_control_period);

  void dynamicReconfigureCallback(mpc_controller::MpcParametersConfig& config, uint32_t level);

  casadi::DM solve(const casadi::DM& initial_positions, const casadi::DM& initial_velocities,
                   const casadi::DM& reference_positions,
                   const casadi::DM& previous_accelerations, double first_jerk_dt);

  void commitSolution();
  void resetWarmStart();

  void setPositionConstraints(const std::vector<double>& lower_bounds,
                              const std::vector<double>& upper_bounds);
  void setVelocityConstraints(const std::vector<double>& lower_bounds,
                              const std::vector<double>& upper_bounds);

  std::unordered_map<std::string, double> dmToMap(
      const casadi::DM& matrix, const std::vector<std::string>& keys) const;
  casadi::DM unorderedMapToDM(const std::unordered_map<std::string, double>& input_map,
                              const std::vector<std::string>& joint_names);

  casadi::DM getReferentAngles() const;
  casadi::DM getReferentVelocities() const;
  casadi::DM getFirstAcceleration(const casadi::DM& solution) const;
  std::vector<double> getReferencePositions() const;
  MpcSolveTiming getLastSolveTiming() const;
  std::string solverBackendName() const;
  int decisionVariableCount() const;
  bool softStateBounds() const { return soft_state_bounds_; }

 private:
  void initializeVariables();
  void initializeSolverBounds();
  void initializeBoundsTimeScaling();
  void initializeBoundsQuadraticCost();
  void setDefaultConstraints();
  void validateConfiguration() const;
  void computeQcLayout();
  void validateInputs(const casadi::DM& positions, const casadi::DM& velocities,
                      const casadi::DM& reference_positions,
                      const casadi::DM& previous_accelerations, double first_jerk_dt) const;

  void buildNLP();
  void buildNlpTimeScaling();
  void buildNlpQuadraticCost();
  void createSolver();

  std::map<std::string, casadi::DM> prepareSolverArguments(
      const casadi::DM& positions, const casadi::DM& velocities,
      const casadi::DM& reference_positions, const casadi::DM& previous_accelerations,
      double first_jerk_dt);
  void validateResult(const std::map<std::string, casadi::DM>& result) const;

  int num_joints_;
  int prediction_horizon_;
  int control_horizon_;
  std::vector<double> prediction_step_durations_;
  double nominal_control_period_;
  double tf_min_{0.1};
  double initial_tf_{1.0};
  double tf_max_{3.0};
  double feasibility_tolerance_{5e-3};
  double osqp_eps_abs_{1e-4};
  double osqp_eps_rel_{1e-4};
  int osqp_max_iter_{3000};
  bool osqp_polishing_{true};
  bool osqp_warm_start_{true};
  bool enforce_first_step_jerk_{false};
  // Soft joint position/velocity bounds: non-negative slack variables keep the
  // QP feasible when the measured state is already outside the hard limits
  // (contact impacts, velocity overshoot). Acceleration bounds stay hard.
  // Benchmarked on a 2 rad reference jump (OSQP, 1.5 s horizon): an L1 (linear)
  // slack penalty stalls OSQP at its iteration cap, a purely quadratic penalty
  // with one slack pair per joint per step converges in ~2 ms warm.
  bool soft_state_bounds_{true};
  bool slack_per_step_{true};   // false: one slack pair per joint for the whole horizon
  double slack_linear_weight_{0.0};
  double slack_quadratic_weight_{1.0e4};
  // Velocity penalty as a fraction of the position weight (towards zero velocity):
  // damps the otherwise bang-bang approach and reduces overshoot.
  double velocity_weight_ratio_{0.05};

  // Variable and constraint-row layout of the quadratic-cost QP, filled once by
  // computeQcLayout() right after validateConfiguration(). Every offset used by
  // buildNlpQuadraticCost(), initializeBoundsQuadraticCost() and the solution
  // accessors comes from here, so a block appended to the decision vector or to
  // the constraint vector only has to be added to this struct.
  //
  // Decision vector: X_1,V_1,...,X_N,V_N | a_0..a_{Nc-1} | bound slacks S
  // Constraint rows: dynamics (== 0) | first-step jerk | jerk transitions |
  //                  soft bounds (all >= 0)
  // The state/acceleration prefix is shared with the time-scaling formulation
  // (which appends only the final time), so xOffset/vOffset/accel_offset are
  // valid for both modes.
  struct QcLayout {
    int num_joints{0};
    int state_vars{0};
    int accel_vars{0};
    int bound_slack_blocks{0};  // number of S blocks: N (per step), 1 (shared) or 0
    int bound_slack_vars{0};
    int decision_vars{0};
    int dyn_rows{0};
    int first_jerk_rows{0};
    int jerk_rows{0};
    int bound_rows{0};
    int constraint_rows{0};
    int accel_offset{0};        // variable offsets
    int bound_slack_offset{0};
    int first_jerk_row_offset{0};  // constraint-row offsets
    int jerk_row_offset{0};
    int bound_row_offset{0};
    int xOffset(int step) const { return (step - 1) * 2 * num_joints; }  // X_step, step in 1..N
    int vOffset(int step) const { return xOffset(step) + num_joints; }   // V_step, step in 1..N
    int aOffset(int step) const { return accel_offset + step * num_joints; }  // a_step, step in 0..Nc-1
  } layout_;

  casadi::DM x_init_;
  casadi::DM lam_x_init_;
  casadi::DM lam_g_init_;
  casadi::DM lbx_;
  casadi::DM ubx_;
  casadi::DM lbg_;
  casadi::DM ubg_;
  casadi::DM pending_x_;
  casadi::DM pending_lam_x_;
  casadi::DM pending_lam_g_;
  bool has_pending_solution_{false};

  casadi::MX positions_;
  casadi::MX velocities_;
  casadi::MX reference_positions_;
  casadi::MX previous_accelerations_;
  casadi::MX first_jerk_dt_;
  std::vector<casadi::MX> decision_variables_;
  std::vector<casadi::MX> constraints_;

  std::vector<double> position_lower_bounds_;
  std::vector<double> position_upper_bounds_;
  std::vector<double> velocity_lower_bounds_;
  std::vector<double> velocity_upper_bounds_;
  std::vector<double> position_weights_vector_;
  std::vector<double> acceleration_weights_vector_;
  std::vector<double> x_ref_;
  mutable std::mutex config_mutex_;
  std::vector<double> acceleration_max_;
  std::vector<double> acceleration_min_;
  std::vector<double> jerk_min_;
  std::vector<double> jerk_max_;

  casadi::DM referent_velocities_;
  casadi::DM referent_angles_;
  MpcSolveTiming last_solve_timing_;

  SolverMode solver_mode_{SolverMode::QuadraticCost};
  SolverBackend solver_backend_{SolverBackend::Osqp};

  casadi::MXDict nlp_;
  casadi::Function solver_;

  dynamic_reconfigure::Server<mpc_controller::MpcParametersConfig> dynamic_reconfigure_server_;
};
