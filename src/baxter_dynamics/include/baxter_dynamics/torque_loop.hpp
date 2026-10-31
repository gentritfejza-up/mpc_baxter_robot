/**
 * @file torque_loop.hpp
 * @brief Fixed-rate computed-torque loop for one Baxter arm, independent of
 *        the dynamics model: tracks the MPC acceleration reference, holds the
 *        pose while idle, detects simulator teleports and publishes
 *        diagnostics.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * The same loop runs behind `kdl_inverse_dynamics_node` (KDL model) and
 * `lagrange_euler_node` (Lagrange-Euler model); only the model object and the
 * parameter namespace differ.
 */
#pragma once

#include "computed_torque_law.hpp"
#include "inverse_dynamics_model.hpp"

#include <baxter_interface_cpp/limb.hpp>
#include <ros/ros.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace baxter {

/// Where the loop reads its parameters; differs per backend so both yaml files stay separate.
struct TorqueLoopParameterKeys {
  /// Namespace of the law and hold settings, e.g. `robot/kdl_dynamics`.
  std::string settings_namespace;
  /// Key of the loop rate [Hz], e.g. `robot/control/kdl_dynamics_rate_hz`.
  std::string loop_rate_key;
  /// Key of the diagnostics rate [Hz], e.g. `robot/control/kdl_diagnostics_rate_hz`.
  std::string diagnostics_rate_key;
};

/// Parameters of the loop (the model and the torque law load their own).
struct TorqueLoopSettings {
  int spinner_threads{2};
  double loop_rate_hz{800.0};
  double diagnostics_rate_hz{50.0};
  /// An MPC reference older than this is stale; also the joint-command timeout [s].
  double acceleration_timeout_sec{0.2};
  /// A joint state older than this (wall clock) is stale: no torque is commanded and
  /// the arm is handed back to position mode so the robot's own watchdog can act [s].
  double joint_state_timeout_sec{0.2};
  bool hold_when_idle{true};
  std::string hold_mode{"torque"};  ///< "torque" or "position".
  std::vector<double> hold_kp;      ///< Torque-space hold gain per joint [Nm/rad].
  std::vector<double> hold_kd;      ///< Torque-space hold gain per joint [Nm s/rad].
  /// A joint step between two loop iterations above this is a teleport [rad].
  double teleport_jump_rad{0.2};

  /**
   * @brief Read the settings (`robot/control/spinner_threads`,
   *        `robot/control/acceleration_ref_timeout_sec`,
   *        `robot/control/joint_state_timeout_sec`, the two rate keys and
   *        `<settings_namespace>/{hold_when_idle,hold_mode,hold_kp,hold_kd,teleport_jump_rad}`).
   * @throw std::runtime_error on an invalid value.
   */
  static TorqueLoopSettings load(ros::NodeHandle& nh, const TorqueLoopParameterKeys& keys,
                                 std::size_t joint_count);
};

/**
 * @brief The control loop and its state; runs in the calling thread while an
 *        AsyncSpinner serves the Limb subscribers.
 *
 * With a fresh MPC reference the loop commands the tracking torque. Without one
 * it holds the pose: in torque mode with a torque-space PD anchored where the
 * hold engaged (re-anchored when a joint jumps, i.e. the simulator teleported
 * the arm), in position mode, or it leaves torque control, depending on the
 * settings. Holding in torque mode moves the simulator's position-to-torque
 * hand-over (which can kick the arm) to the enable instant, before the MPC
 * commands anything.
 */
class TorqueLoop {
 public:
  /**
   * @param nh Node handle for the diagnostics publisher.
   * @param limb_name "left" or "right".
   * @param model Dynamics model; the loop takes ownership.
   * @param law_settings Torque-law settings (gravity, damping, inner PD).
   * @param settings Loop settings (rates, hold).
   * @throw std::invalid_argument when a gain vector does not match the model.
   */
  TorqueLoop(ros::NodeHandle& nh, const std::string& limb_name,
             std::unique_ptr<InverseDynamicsModel> model, ComputedTorqueSettings law_settings,
             TorqueLoopSettings settings);

  /// Run until ROS shuts down, then return the arm to position mode if needed.
  void run();

  /// Diagnostics topic suffix under `/robot/limb/<side>/` (layout "v1").
  static constexpr const char* kDiagnosticsTopic = "torque_loop_diagnostics";

 private:
  using SteadyClock = std::chrono::steady_clock;

  void step(SteadyClock::time_point loop_start, double loop_dt_ms);
  /// @return True when a joint moved more than teleport_jump_rad since the last step.
  bool updateTeleportDetector(const JointMap& positions);
  void holdPose(const JointStateSnapshot& state, bool teleported);
  void logHoldError(const JointMap& positions, const ComputedTorqueResult& hold) const;
  void trackReference(const JointStateSnapshot& state,
                      const AccelerationReferenceSnapshot& reference,
                      SteadyClock::time_point loop_start, double loop_dt_ms);
  void publishDiagnostics(const ComputedTorqueResult& result, double loop_dt_ms,
                          double compute_ms, double reference_age_sec) const;
  /// Hand the joints back to position mode at the measured pose.
  void leaveTorqueMode();

  const std::string limb_name_;
  const TorqueLoopSettings settings_;
  std::unique_ptr<InverseDynamicsModel> model_;
  ComputedTorqueLaw law_;
  Limb limb_;
  ros::Publisher diagnostics_publisher_;
  const double diagnostics_period_sec_;

  bool tracking_reference_{false};   ///< Last iteration commanded MPC-tracking torque.
  bool torque_mode_engaged_{false};  ///< A torque command has been sent since the last exit.
  bool holding_{false};              ///< The idle hold is engaged and anchored.
  JointMap hold_positions_;          ///< Hold anchor [rad].
  JointMap previous_positions_;      ///< Pose of the previous iteration [rad].
  uint64_t sequence_{0};             ///< Number of MPC-tracking torque commands sent.
  SteadyClock::time_point previous_diagnostics_time_;
};

/**
 * @brief Entry point shared by both backend executables: parse the limb
 *        argument, build the model with @p make_model, load the settings from
 *        @p keys and run the loop.
 * @param argc, argv Program arguments after `ros::init`; argv[1] is the limb.
 * @param node_name Name used in log messages.
 * @param keys Parameter keys of this backend.
 * @param make_model Builds the model for the limb (may throw).
 * @return Process exit code.
 */
int runTorqueLoopNode(int argc, char** argv, const std::string& node_name,
                      const TorqueLoopParameterKeys& keys,
                      const std::function<std::unique_ptr<InverseDynamicsModel>(
                          ros::NodeHandle&, const std::string&)>& make_model);

}  // namespace baxter
