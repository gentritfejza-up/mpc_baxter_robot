/**
 * @file kdl_inverse_dynamics.hpp
 * @brief URDF-derived KDL joint-space dynamics model of one Baxter arm behind
 *        the InverseDynamicsModel interface of `baxter_dynamics`.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include <baxter_dynamics/inverse_dynamics_model.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

namespace baxter_kdl_dynamics {

/**
 * @brief M(q), C(q, qd) qd and g(q) of a KDL chain built from `/robot_description`.
 *
 * The chain runs from the configured root link to the arm tip. The inertia of
 * every link below the tip (hand, gripper, fingers) is lumped into the tip
 * segment at zero descendant-joint position, so the chain stays seven-DOF but
 * carries the payload mass. Effort limits and viscous damping come from the
 * URDF joints.
 *
 * Parameters (`robot/kdl_dynamics/...`): `root_link` (default `torso`),
 * `tip_link` (default `<limb>_gripper_base`, overridable by the private
 * `~tip_link`), `aggregate_tip_subtree_inertia`, `s1_torque_limit_override`
 * [Nm] (positive replaces the URDF limit), `gravity` (3-vector in the root
 * frame, default (0, 0, -9.81)), `robot_description_timeout_sec`.
 *
 * Not thread-safe: `evaluate()` shares KDL scratch storage.
 */
class KdlInverseDynamics : public baxter::InverseDynamicsModel {
 public:
  /**
   * @param nh Node handle used for parameter lookup.
   * @param limb "left" or "right".
   * @throw std::invalid_argument for an unknown limb.
   * @throw std::runtime_error when the URDF, the chain or the effort limits are invalid.
   */
  KdlInverseDynamics(ros::NodeHandle& nh, const std::string& limb);

  const std::vector<std::string>& jointNames() const override { return joint_names_; }
  const std::vector<double>& effortLimits() const override { return effort_limits_; }
  const std::vector<double>& viscousDamping() const override { return urdf_damping_; }
  std::string description() const override { return description_; }

  /// @copydoc baxter::InverseDynamicsModel::evaluate
  void evaluate(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                baxter::JointSpaceDynamics& dynamics) const override;

 private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dynamics_solver_;
  std::vector<std::string> joint_names_;
  std::vector<double> effort_limits_;  ///< Torque saturation per joint [Nm].
  std::vector<double> urdf_damping_;   ///< Viscous damping per joint from the URDF [Nm s/rad].
  std::string description_;

  /// KDL scratch, reused between evaluations.
  mutable KDL::JntArray q_, qd_, coriolis_, gravity_;
  mutable KDL::JntSpaceInertiaMatrix mass_matrix_;
};

}  // namespace baxter_kdl_dynamics
