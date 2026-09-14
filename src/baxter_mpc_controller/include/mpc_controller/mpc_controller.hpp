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
#include <mpc_controller/MpcParametersConfig.h>

enum class SolverMode { TimeScaling, QuadraticCost };
enum class SolverBackend { Ipopt, Osqp, Qpoases };

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

 private:
  void initializeVariables();
  void initializeSolverBounds();
  void initializeBoundsTimeScaling();
  void initializeBoundsQuadraticCost();
  void setDefaultConstraints();
  void validateConfiguration() const;
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
  double feasibility_tolerance_{1e-5};
  double osqp_eps_abs_{1e-6};
  double osqp_eps_rel_{1e-6};
  int osqp_max_iter_{4000};
  bool osqp_polishing_{true};
  bool enforce_first_step_jerk_{false};

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
