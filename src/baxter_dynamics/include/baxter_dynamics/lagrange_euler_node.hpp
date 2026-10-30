/**
 * @file lagrange_euler_node.hpp
 * @brief Computed-torque control loop for one Baxter arm using the
 *        Lagrange-Euler inverse dynamics as model backend.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * The node reads the measured joint state and the MPC acceleration reference
 * from the Limb interface, optionally adds a PD correction around the MPC
 * trajectory, evaluates tau = M(q) qdd + C(q, qd) + g(q) and publishes tau as
 * torque command at a fixed rate. It is the drop-in alternative to the KDL
 * backend (selected with `dynamics_backend:=lagrange_euler`).
 */
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include "baxter_dynamics/lagrange_euler.hpp"
#include "baxter_interface_cpp/limb.hpp"

namespace baxter {

/**
 * @brief Fixed-rate computed-torque loop around LagrangeEuler.
 *
 * Parameters (all optional, read once in the constructor):
 *  - `robot/control/spinner_threads` (int, default 5)
 *  - `robot/control/lagrange_euler_rate_hz` (int, default 800) [Hz]
 *  - `robot/control/acceleration_ref_timeout_sec` (double, default 0.2) [s]
 *  - `robot/lagrange_euler/kp`, `robot/lagrange_euler/kd` (one finite
 *    non-negative gain per joint, default 0 = pure feed-forward) [1/s^2], [1/s]
 *  - `robot/lagrange_euler/extrapolate_reference` (bool, default true)
 *  - `robot/lagrange_euler/max_reference_extrapolation_sec` (double,
 *    default 0.1) [s]
 */
class LagrangeEulerNode {
public:
  /**
   * @brief Builds the dynamics model and the limb interface for one arm.
   * @param nh Node handle used to read parameters.
   * @param limb_side `"left"` or `"right"`.
   * @throws std::invalid_argument if the PD gains or the extrapolation limit
   *         are malformed; std::runtime_error from LagrangeEuler on missing
   *         model parameters.
   */
  LagrangeEulerNode(ros::NodeHandle &nh, const std::string &limb_side);

  /**
   * @brief Runs the control loop until ROS shuts down, then leaves torque
   *        control mode.
   */
  void runNode();

private:
  /// One control step: read state, compute and publish torques (or bail out).
  void runControlStep();

  /**
   * @brief Acceleration command fed to the inverse dynamics,
   *        qdd = a_mpc + kp (q_ref - q) + kd (qd_ref - qd).
   *
   * The reference (q_ref, qd_ref) is the MPC sample (q0, qd0) propagated with
   * a_mpc over the (clamped) age of the command when extrapolation is enabled.
   * @return Commanded joint accelerations keyed by full joint name [rad/s^2].
   * @throws std::out_of_range if the reference maps lack a joint.
   */
  std::unordered_map<std::string, double> commandedAccelerations() const;

  /// Leaves torque control mode once, if it is active.
  void exitTorqueControl();

  std::unique_ptr<LagrangeEuler> lagrange_euler_{nullptr};
  std::unique_ptr<Limb> limb_{nullptr};

  std::unordered_map<std::string, double> joint_angles_;        ///< q [rad]
  std::unordered_map<std::string, double> joint_velocities_;    ///< qd [rad/s]
  std::unordered_map<std::string, double> joint_accelerations_; ///< MPC qdd [rad/s^2]

  int spinner_threads_{5};
  int loop_rate_hz_{800};
  double acceleration_ref_timeout_sec_{0.2};
  bool torque_control_active_{false};

  std::vector<double> kp_; ///< Position gain per joint [1/s^2].
  std::vector<double> kd_; ///< Velocity gain per joint [1/s].
  bool extrapolate_reference_{true};
  double max_reference_extrapolation_sec_{0.1};
};

} // namespace baxter
