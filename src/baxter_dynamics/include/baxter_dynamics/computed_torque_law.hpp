/**
 * @file computed_torque_law.hpp
 * @brief Computed-torque control law on top of any InverseDynamicsModel:
 *        tracking of the MPC acceleration reference with an optional inner PD,
 *        and a torque-space hold of an anchor pose.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include "inverse_dynamics_model.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros {
class NodeHandle;
}

namespace baxter {

/// Joint values keyed by joint name.
using JointMap = std::unordered_map<std::string, double>;

/**
 * @brief Settings of the torque law, read from one parameter namespace
 *        (`robot/kdl_dynamics` or `robot/lagrange_euler`).
 */
struct ComputedTorqueSettings {
  /// Add g(q). Gazebo: true. The real Baxter compensates gravity itself: false.
  bool add_gravity_compensation{true};
  /// Add b qd with the model's viscous damping (the URDF damping in Gazebo).
  bool compensate_viscous_damping{true};
  std::vector<double> inner_pd_kp;  ///< Acceleration-space PD gain per joint [1/s^2].
  std::vector<double> inner_pd_kd;  ///< Acceleration-space PD gain per joint [1/s].
  /// Track q0 + v0 t + a t^2 / 2 from the MPC sample time instead of a fixed point.
  bool extrapolate_reference{true};
  double max_reference_extrapolation_sec{0.1};  ///< Clamp of t above [s].

  /**
   * @brief Read the settings from `<parameter_namespace>/...`; absent keys
   *        keep the defaults above (gains default to zero = pure feedforward).
   * @throw std::runtime_error on a malformed value.
   */
  static ComputedTorqueSettings load(ros::NodeHandle& nh, const std::string& parameter_namespace,
                                     std::size_t joint_count);
};

/**
 * @brief Read a per-joint gain vector parameter.
 * @return The gains, or `size` zeros when the parameter is absent.
 * @throw std::runtime_error when the parameter has the wrong size or holds a
 *        negative or non-finite value.
 */
std::vector<double> loadGainVector(ros::NodeHandle& nh, const std::string& name, std::size_t size);

/**
 * @brief One evaluation of the torque law with every intermediate term kept
 *        for diagnostics. All vectors are in model joint order.
 */
struct ComputedTorqueResult {
  JointMap torques;                            ///< Saturated torque to command [Nm].
  std::vector<double> position;                ///< Measured joint position [rad].
  std::vector<double> velocity;                ///< Measured joint velocity [rad/s].
  std::vector<double> reference_position;      ///< Tracked position reference [rad].
  std::vector<double> reference_velocity;      ///< Tracked velocity reference [rad/s].
  std::vector<double> mpc_acceleration;        ///< Feedforward acceleration from the MPC [rad/s^2].
  std::vector<double> pd_acceleration;         ///< Inner PD correction [rad/s^2].
  std::vector<double> commanded_acceleration;  ///< mpc_acceleration + pd_acceleration [rad/s^2].
  std::vector<double> inertia_torque;          ///< M(q) commanded_acceleration [Nm].
  std::vector<double> coriolis_torque;         ///< C(q, qd) qd [Nm].
  std::vector<double> gravity_torque;          ///< g(q), zero when gravity compensation is off [Nm].
  std::vector<double> damping_torque;          ///< b qd, zero when damping compensation is off [Nm].
  std::vector<double> raw_torque;              ///< Sum of the terms before saturation [Nm].
  std::vector<double> commanded_torque;        ///< raw_torque clamped to +-torque_limit [Nm].
  std::vector<double> torque_limit;            ///< Per-joint saturation limit [Nm].
};

/**
 * @brief tau = M(q) qdd + C(q, qd) qd [+ g(q)] [+ b qd], saturated to the
 *        model's effort limits, for any InverseDynamicsModel.
 *
 * `track()` follows the MPC reference with qdd = a_mpc + kp (q_ref - q) +
 * kd (qd_ref - qd). `hold()` is the idle fallback: model torque at zero
 * acceleration plus a torque-space PD toward an anchor pose. Not thread-safe
 * (the model's scratch storage is shared).
 */
class ComputedTorqueLaw {
 public:
  /**
   * @param model Dynamics model; must outlive the law.
   * @param settings Law settings; the gain vectors must match the model's joint count.
   * @throw std::invalid_argument when a gain vector has the wrong size.
   */
  ComputedTorqueLaw(const InverseDynamicsModel& model, ComputedTorqueSettings settings);

  /**
   * @brief Torque to track the MPC acceleration reference.
   * @param positions Measured joint positions keyed by joint name [rad].
   * @param velocities Measured joint velocities keyed by joint name [rad/s].
   * @param accelerations MPC feedforward accelerations keyed by joint name [rad/s^2].
   * @param reference_positions MPC base positions q0; only required for joints
   *        whose inner PD gains are nonzero.
   * @param reference_velocities MPC base velocities v0; same condition.
   * @param reference_age_sec Age of the MPC command [s], used for the extrapolation.
   * @throw std::runtime_error when a value is missing or non-finite, or the model fails.
   */
  ComputedTorqueResult track(const JointMap& positions, const JointMap& velocities,
                             const JointMap& accelerations, const JointMap& reference_positions,
                             const JointMap& reference_velocities,
                             double reference_age_sec = 0.0) const;

  /**
   * @brief Torque that holds `hold_positions` while no MPC reference exists.
   *
   * Model torque at zero acceleration (gravity, Coriolis and damping
   * compensation, no inner PD) plus tau_pd = kp (q_hold - q) - kd qd. The PD
   * acts in torque space because an acceleration-space PD scales with the
   * joint inertia, and the simulated wrists are too light for it to pull a
   * joint back from its limit stop.
   * @param kp Proportional gain per joint in model order [Nm/rad].
   * @param kd Derivative gain per joint in model order [Nm s/rad].
   * @throw std::invalid_argument when a gain vector has the wrong size.
   * @throw std::runtime_error when a value is missing or non-finite, or the model fails.
   */
  ComputedTorqueResult hold(const JointMap& positions, const JointMap& velocities,
                            const JointMap& hold_positions, const std::vector<double>& kp,
                            const std::vector<double>& kd) const;

  const std::vector<std::string>& jointNames() const { return model_.jointNames(); }
  const ComputedTorqueSettings& settings() const { return settings_; }

 private:
  /// @throw std::runtime_error when the joint is missing or its value is not finite.
  static double requireFiniteValue(const JointMap& values, const std::string& joint,
                                   const char* source);
  /// A result whose vectors are sized for the model, optional terms zeroed.
  ComputedTorqueResult makeEmptyResult() const;
  /// Fill the measured state into `q`, `qd` and the result.
  void readMeasuredState(const JointMap& positions, const JointMap& velocities,
                         Eigen::VectorXd& q, Eigen::VectorXd& qd,
                         ComputedTorqueResult& result) const;
  /// Evaluate the model and write every torque term and `raw_torque`.
  void evaluateModelTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
                           const Eigen::VectorXd& qdd, ComputedTorqueResult& result) const;
  /// Clamp `raw_torque` into `commanded_torque` and `torques`, warning on saturation.
  void saturateTorque(ComputedTorqueResult& result) const;

  const InverseDynamicsModel& model_;
  const ComputedTorqueSettings settings_;
  mutable JointSpaceDynamics dynamics_;  ///< Scratch for the model terms.
};

}  // namespace baxter
