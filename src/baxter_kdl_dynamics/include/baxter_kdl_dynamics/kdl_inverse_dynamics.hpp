/**
 * @file kdl_inverse_dynamics.hpp
 * @brief URDF-derived KDL inverse dynamics for one Baxter arm: computed-torque
 *        tracking of the MPC acceleration reference and a torque-space hold.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>
#include <urdf/model.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace baxter_kdl_dynamics {

/**
 * @brief One evaluation of the torque law, with every intermediate term kept for
 *        diagnostics. All vectors are indexed in chain order
 *        (see KdlInverseDynamics::jointNames()).
 */
struct DynamicsResult {
  /// Saturated torque to command, keyed by joint name [Nm].
  std::unordered_map<std::string, double> torques;
  std::vector<double> position;                ///< Measured joint position [rad].
  std::vector<double> velocity;                ///< Measured joint velocity [rad/s].
  std::vector<double> reference_position;      ///< Tracked position reference [rad].
  std::vector<double> reference_velocity;      ///< Tracked velocity reference [rad/s].
  std::vector<double> mpc_acceleration;        ///< Feedforward acceleration from the MPC [rad/s^2].
  std::vector<double> pd_acceleration;         ///< Inner PD correction [rad/s^2].
  std::vector<double> commanded_acceleration;  ///< mpc_acceleration + pd_acceleration [rad/s^2].
  std::vector<double> inertia_torque;          ///< M(q) * commanded_acceleration [Nm].
  std::vector<double> coriolis_torque;         ///< C(q, qd) * qd [Nm].
  std::vector<double> gravity_torque;          ///< g(q), zero when gravity compensation is off [Nm].
  std::vector<double> damping_torque;          ///< URDF viscous damping * qd, zero when off [Nm].
  std::vector<double> raw_torque;              ///< Sum of the terms before saturation [Nm].
  std::vector<double> commanded_torque;        ///< raw_torque clamped to +-torque_limit [Nm].
  std::vector<double> torque_limit;            ///< Per-joint saturation limit [Nm].
};

/**
 * @brief Computed-torque law tau = M(q) qdd + C(q, qd) qd + g(q) + b qd for one arm.
 *
 * The model is a KDL chain from the configured root link to the arm tip, built
 * from `/robot_description`. The inertia of every link below the tip (hand,
 * gripper, fingers) is lumped into the tip segment at zero descendant-joint
 * position, so the chain stays seven-DOF but carries the payload mass.
 *
 * `compute()` tracks the MPC reference: qdd = a_mpc + kp (q_ref - q) + kd (qd_ref - qd),
 * where the reference is optionally extrapolated from the MPC sample time.
 * `computeHold()` is the idle fallback: model torque at zero acceleration plus a
 * torque-space PD toward an anchor pose.
 *
 * The object is not thread-safe: both compute methods share KDL scratch storage.
 */
class KdlInverseDynamics {
 public:
  /**
   * @brief Build the chain and load the `robot/kdl_dynamics` parameters.
   * @param nh Node handle used for parameter lookup (relative to its namespace).
   * @param limb "left" or "right".
   * @throw std::invalid_argument for an unknown limb.
   * @throw std::runtime_error when the URDF, the chain, the effort limits or a
   *        parameter are invalid.
   */
  KdlInverseDynamics(ros::NodeHandle& nh, const std::string& limb);

  /**
   * @brief Torque to track the MPC acceleration reference.
   * @param positions Measured joint positions keyed by joint name [rad].
   * @param velocities Measured joint velocities keyed by joint name [rad/s].
   * @param accelerations MPC feedforward accelerations keyed by joint name [rad/s^2].
   * @param reference_positions MPC base positions q0 keyed by joint name [rad]; only
   *        required for joints whose inner PD gains are nonzero.
   * @param reference_velocities MPC base velocities v0 keyed by joint name [rad/s];
   *        only required for joints whose inner PD gains are nonzero.
   * @param reference_age_sec Age of the MPC command [s]. When extrapolation is enabled
   *        the tracked reference is q0 + v0 t + a t^2 / 2 with t clamped to
   *        [0, max_reference_extrapolation_sec].
   * @return Every torque term and the saturated command.
   * @throw std::runtime_error when a value is missing or non-finite, or KDL fails.
   */
  DynamicsResult compute(const std::unordered_map<std::string, double>& positions,
                         const std::unordered_map<std::string, double>& velocities,
                         const std::unordered_map<std::string, double>& accelerations,
                         const std::unordered_map<std::string, double>& reference_positions,
                         const std::unordered_map<std::string, double>& reference_velocities,
                         double reference_age_sec = 0.0) const;

  /**
   * @brief Torque that holds `hold_positions` while no MPC reference exists.
   *
   * Model torque at zero acceleration (gravity, Coriolis and damping compensation,
   * no inner PD) plus a torque-space PD, tau_pd = kp (q_hold - q) - kd qd. The PD
   * acts in torque space because the computed-torque PD scales with the joint
   * inertia, and the simulated wrists are too light for it to pull a joint back
   * from its limit stop.
   * @param positions Measured joint positions keyed by joint name [rad].
   * @param velocities Measured joint velocities keyed by joint name [rad/s].
   * @param hold_positions Anchor pose keyed by joint name [rad].
   * @param kp Proportional gain per joint in chain order [Nm/rad].
   * @param kd Derivative gain per joint in chain order [Nm s/rad].
   * @return Every torque term and the saturated command; the acceleration fields
   *         are zero and `reference_position` is the anchor.
   * @throw std::invalid_argument when a gain vector has the wrong size.
   * @throw std::runtime_error when a value is missing or non-finite, or KDL fails.
   */
  DynamicsResult computeHold(const std::unordered_map<std::string, double>& positions,
                             const std::unordered_map<std::string, double>& velocities,
                             const std::unordered_map<std::string, double>& hold_positions,
                             const std::vector<double>& kp, const std::vector<double>& kd) const;

  /**
   * @brief Read a per-joint gain vector parameter.
   * @param nh Node handle used for the lookup.
   * @param name Parameter key.
   * @param size Required number of entries (one per arm joint).
   * @return The gains, or `size` zeros when the parameter is absent.
   * @throw std::runtime_error when the parameter has the wrong size or holds a
   *        negative or non-finite value.
   */
  static std::vector<double> loadGainVector(ros::NodeHandle& nh, const std::string& name,
                                            std::size_t size);

  /// @return Movable joint names in chain (root to tip) order.
  const std::vector<std::string>& jointNames() const { return joint_names_; }

 private:
  /**
   * @brief Look up a joint value and reject missing or non-finite entries.
   * @param values Map keyed by joint name.
   * @param joint Joint to look up.
   * @param source Human-readable name of the map, used in the error message.
   * @return The finite value.
   * @throw std::runtime_error when the joint is missing or the value is not finite.
   */
  static double requireFiniteValue(const std::unordered_map<std::string, double>& values,
                                   const std::string& joint, const std::string& source);

  /// @return A result whose vectors are sized for the chain and whose optional
  ///         terms (gravity, damping) are zero.
  DynamicsResult makeEmptyResult() const;

  /**
   * @brief Fill the measured state into `q`, `qd` and `result`.
   * @param positions Measured joint positions keyed by joint name [rad].
   * @param velocities Measured joint velocities keyed by joint name [rad/s].
   * @param q Output joint positions in chain order [rad].
   * @param qd Output joint velocities in chain order [rad/s].
   * @param result `position` and `velocity` are filled.
   */
  void readMeasuredState(const std::unordered_map<std::string, double>& positions,
                         const std::unordered_map<std::string, double>& velocities,
                         KDL::JntArray& q, KDL::JntArray& qd, DynamicsResult& result) const;

  /**
   * @brief Evaluate M(q) qdd + C(q, qd) qd + g(q) + b qd into `result.raw_torque`
   *        and the per-term vectors.
   * @param q Joint positions in chain order [rad].
   * @param qd Joint velocities in chain order [rad/s].
   * @param qdd Commanded joint accelerations in chain order [rad/s^2].
   * @param result Receives inertia, Coriolis, gravity, damping and raw torques [Nm].
   * @throw std::runtime_error when a KDL solver reports an error.
   */
  void evaluateModelTorque(const KDL::JntArray& q, const KDL::JntArray& qd,
                           const KDL::JntArray& qdd, DynamicsResult& result) const;

  /**
   * @brief Clamp `result.raw_torque` to the effort limits into `commanded_torque`
   *        and `torques`, warning (throttled) on saturation.
   */
  void saturateTorque(DynamicsResult& result) const;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dynamics_solver_;
  std::vector<std::string> joint_names_;
  std::vector<double> effort_limits_;   ///< Torque saturation per joint [Nm].
  std::vector<double> urdf_damping_;    ///< Viscous damping per joint from the URDF [Nm s/rad].
  std::vector<double> inner_pd_kp_;     ///< Acceleration-space PD gain [1/s^2].
  std::vector<double> inner_pd_kd_;     ///< Acceleration-space PD gain [1/s].
  bool add_gravity_compensation_{true};
  bool compensate_urdf_damping_{true};
  /// The MPC sends (q0, v0, a) sampled at its solve time; the tracked reference is
  /// q0 + v0 t + a t^2 / 2 with t the command age, clamped.
  bool extrapolate_reference_{true};
  double max_reference_extrapolation_sec_{0.1};
};

}  // namespace baxter_kdl_dynamics
