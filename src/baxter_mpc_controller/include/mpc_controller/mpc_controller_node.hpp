#pragma once

/**
 * @file mpc_controller_node.hpp
 * @brief Control loop of one arm: reads the joint state, gates on the robot enable and
 *        settle conditions, linearises the collision guard, solves the MPC, checks the
 *        plan with the guard and publishes the acceleration command (or a braking
 *        fallback) to the torque loop.
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <baxter_core_msgs/AssemblyState.h>
#include <std_msgs/Float64MultiArray.h>

#include "mpc_controller/arm_collision_model.hpp"
#include "mpc_controller/geometry_markers.hpp"
#include "mpc_controller/geometry_rows.hpp"
#include "mpc_controller/joint_state_listener.hpp"
#include "mpc_controller/mpc_controller.hpp"

/**
 * @brief Runs the MPC of one arm at the control rate.
 *
 * Threading: run() executes the control loop on the calling thread; ROS callbacks
 * (joint states inside Limb, /robot/state, the other arm, dynamic reconfigure) run on
 * the AsyncSpinner threads and only touch atomics or mutex-guarded state.
 */
class MPCControlManager {
 public:
  /**
   * @param node_handle Node handle for parameters, publishers and subscribers.
   * @param limb_side "left" or "right".
   */
  MPCControlManager(ros::NodeHandle& node_handle, const std::string& limb_side);

  /** @brief Control loop; returns when ROS shuts down. */
  void run();

 private:
  /** @brief Everything the cycle knows about the measured state. */
  struct MeasuredState {
    std::vector<double> positions;   ///< [rad], in Limb::jointNames() order.
    std::vector<double> velocities;  ///< [rad/s]
    ros::Time stamp;                 ///< Measurement time (ROS clock).
  };

  void loadAndValidateConfiguration();
  void initializeGeometry();
  void initializeGeometryPublishers();
  /// Prediction grid: one control period, then equal shares of the remaining duration [s].
  std::vector<double> makePredictionGrid() const;

  /**
   * @brief One control cycle.
   * @param reference_positions Reference from dynamic reconfigure [rad].
   * @param cycle_start Start of the cycle (for the deadline).
   * @return True when an MPC command was published.
   */
  bool processControl(const std::vector<double>& reference_positions,
                      const std::chrono::steady_clock::time_point& cycle_start);

  /// Reads a fresh, complete, finite joint state; false (with a throttled warning) otherwise.
  bool readMeasuredState(MeasuredState& state) const;
  /// Enable and settle gates; false while the MPC must idle (plan discarded meanwhile).
  bool passEnableAndSettleGates(const std::vector<double>& velocities);
  /// Overspeed recovery; true while braking (no MPC command this cycle).
  bool handleOverspeedRecovery(const MeasuredState& state);
  /// Start-up hold or external reference; false when the reference is unusable.
  bool selectReference(const std::vector<double>& reference_in,
                       const std::vector<double>& positions, std::vector<double>& reference_out);
  /// Other-arm trajectory, floor arming, reference projection and row linearisation.
  /// False when the other arm is required but stale (braking fallback published).
  bool prepareGeometry(const MeasuredState& state, std::vector<double>& reference_positions);
  /// Builds the other-arm trajectory over the horizon (constant velocity, capped in time).
  void buildOtherArmTrajectory(const mpc_geometry::JointVector& other_positions,
                               const mpc_geometry::JointVector& other_velocities);
  /// Arms the floor rows once the arm has reached its home position.
  void updateFloorArming(const std::vector<double>& positions);
  /// Moves the reference out of the keep-outs and joint limits.
  void projectReference(std::vector<double>& reference_positions);
  /// Linearisation points for this cycle: the previous accepted prediction shifted by one
  /// step, or the measured state everywhere when there is none.
  void buildLinearisationPoints(const std::vector<double>& positions);
  /// Solves, with the dual reset on a first stall and the first-solve convergence retries.
  casadi::DM solveWithRetries(const MpcSolveInput& input);
  /// Guard evaluation, markers and diagnostics; false when the plan is rejected.
  bool runCollisionGuard(const casadi::DM& solution, const MeasuredState& state,
                         const std::vector<double>& reference_positions);
  /// Forgets the warm start and the linearisation trajectory.
  void discardPlan();
  void publishGeometryDiagnostics();

  /**
   * @brief Commands a braking acceleration (decelerate every joint to rest within the
   *        acceleration limits) when no valid MPC solution is available, so the torque
   *        loop keeps a fresh reference instead of timing out into position mode.
   */
  void publishBrakingFallback(const MeasuredState& state,
                              const std::vector<double>& reference_positions);

  /**
   * @brief Publishes an acceleration command with its base state.
   *
   * The base state carried with the command is propagated from `state.stamp` to publish
   * time with the acceleration that was active meanwhile, so the torque loop's reference
   * does not lag the plant by the solve latency (which otherwise makes its PD fight the
   * feedforward).
   * @param accelerations Commanded accelerations [rad/s^2].
   * @param state Measured state the accelerations were planned from.
   * @param reference_positions Reference carried for diagnostics [rad].
   */
  void publishCommand(const std::vector<double>& accelerations, const MeasuredState& state,
                      const std::vector<double>& reference_positions);

  void onAssemblyState(const baxter_core_msgs::AssemblyState::ConstPtr& message);

  ros::NodeHandle node_handle_;
  std::string limb_side_;
  std::unique_ptr<Limb> limb_;
  std::vector<std::string> joint_names_;  ///< Limb joint names, cached once.
  std::unique_ptr<MPCController> mpc_controller_;

  // Timing and horizon (robot/control/*).
  int num_joints_{7};
  int prediction_horizon_{20};
  int control_horizon_{15};
  int spinner_threads_{10};
  double control_rate_hz_{50.0};
  double prediction_duration_sec_{2.0};  ///< Length of the prediction horizon [s].
  double control_period_sec_{0.02};      ///< 1 / control_rate_hz_ [s].
  double state_timeout_sec_{0.04};       ///< Oldest joint state still commanded on [s].
  double cycle_deadline_sec_{0.02};      ///< Cycle time counted as a deadline miss [s].
  bool enforce_cycle_deadline_{true};
  /// true: a solution that misses the cycle deadline is still commanded (late is safer
  /// than letting the torque loop time out into position mode mid-motion); false: the
  /// late solution is only kept as the next warm start.
  bool publish_on_deadline_miss_{true};
  double max_latency_compensation_sec_{0.15};  ///< Cap on the base-state propagation [s].

  // Command state.
  std::vector<double> previous_applied_accelerations_;  ///< [rad/s^2]
  std::vector<double> acceleration_min_;                ///< [rad/s^2]
  std::vector<double> acceleration_max_;                ///< [rad/s^2]
  bool braking_fallback_enabled_{true};
  bool has_last_command_time_{false};
  ros::Time last_command_time_;

  // Cycle statistics.
  double last_state_preparation_ms_{0.0};
  double last_postprocessing_ms_{0.0};
  uint64_t successful_cycles_{0};
  uint64_t failed_cycles_{0};
  uint64_t deadline_misses_{0};
  uint64_t fallback_cycles_{0};
  uint64_t dual_resets_{0};

  // Enable gate: the MPC idles (no command, internal state reset) until /robot/state
  // reports the robot enabled: before that the simulator holds the joints in position
  // mode and commands wind up the plan without moving the arm.
  bool require_enabled_{true};
  std::atomic<bool> robot_enabled_{false};
  bool was_enabled_{false};
  double settle_velocity_rad_s_{0.1};  ///< Joints count as quiet below this speed [rad/s].
  double settle_hold_sec_{0.5};        ///< Joints must stay quiet this long [s].
  double enable_delay_sec_{0.0};       ///< Extra idle time after the enable (simulator set-up) [s].
  ros::WallTime enabled_at_;
  bool settled_after_enable_{false};
  ros::WallTime quiet_since_;
  bool quiet_timer_running_{false};
  ros::Subscriber assembly_state_subscriber_;

  // Reference handling.
  std::vector<double> hold_reference_;  ///< Pose held until the first external reference [rad].
  bool had_external_reference_{false};
  bool project_reference_{true};
  std::vector<double> projection_lower_bounds_;  ///< Joint limits minus a margin for the projection [rad].
  std::vector<double> projection_upper_bounds_;  ///< [rad]
  double last_reference_shift_{0.0};             ///< Largest joint shift of the last projection [rad].

  // Solver convergence handling. After enabling, the first solve is repeated
  // (warm-started on its own iterate) until it converges, so the first command never
  // comes from a capped iterate.
  int first_solve_max_retries_{6};
  bool first_solve_converged_{false};
  bool last_solve_was_inaccurate_{false};
  // Recovery: a joint faster than overspeed_ratio x its velocity bound (a kick, a
  // contact) is braked with the fallback instead of trusting the capped iterates such a
  // state produces, until every joint is back within bounds.
  double overspeed_ratio_{1.25};
  bool recovering_{false};

  // Collision guard (robot/geometry parameters).
  bool geometry_enabled_{false};
  std::string guard_mode_{"reject"};  ///< reject | warn | off
  bool other_arm_required_{true};
  double other_arm_timeout_sec_{0.1};
  /// The other arm is assumed to keep its measured joint velocity for this long into
  /// the horizon, then to stand still (0: static at the measured state) [s].
  double other_arm_extrapolation_sec_{0.3};
  /// Floor latch: with activate_at_home the floor rows/guard stay off until the
  /// measured joints are within home_tolerance_rad of the home position, then on.
  bool floor_activate_at_home_{false};
  bool floor_armed_{true};
  double home_tolerance_rad_{0.05};
  mpc_geometry::JointVector home_position_{};
  mpc_geometry::GeometryParams geometry_params_;
  std::unique_ptr<mpc_geometry::ArmCollisionModel> collision_model_;
  std::unique_ptr<mpc_geometry::GeometryRowLinearizer> linearizer_;
  std::unique_ptr<mpc_geometry::OtherArmStateListener> other_arm_listener_;
  std::unique_ptr<mpc_geometry::GeometryMarkerPublisher> marker_publisher_;
  ros::Publisher geometry_diagnostics_publisher_;
  mpc_geometry::OtherArmTrajectory other_arm_trajectory_;  ///< Empty when the other arm is unknown.
  mpc_geometry::GeometryRowBatch geometry_batch_;
  std::vector<mpc_geometry::JointVector> linearisation_points_;
  bool has_linearisation_trajectory_{false};
  mpc_geometry::GuardReport last_guard_report_;
  uint64_t guard_rejections_{0};
  uint64_t other_arm_stale_cycles_{0};
  uint64_t geometry_sequence_{0};
  double last_linearize_ms_{0.0};
  double last_guard_ms_{0.0};
  double last_max_geometry_slack_{0.0};
};
