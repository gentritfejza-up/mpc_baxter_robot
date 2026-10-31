/**
 * @file inverse_dynamics_model.hpp
 * @brief Backend-independent interface of a joint-space rigid-body model:
 *        M(q), C(q, qd) qd and g(q) of a serial chain, plus the per-joint
 *        effort limits and viscous damping the torque law needs.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * The computed-torque law and the torque loop only see this interface, so any
 * model that can deliver the three joint-space terms can drive the robot: the
 * Lagrange-Euler model of this package (closed form from a DH table), the KDL
 * model of `baxter_kdl_dynamics` (recursive Newton-Euler from the URDF), or a
 * future one (identified parameters, Pinocchio, ...).
 */
#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <vector>

namespace baxter {

/**
 * @brief The joint-space terms of tau = M(q) qdd + C(q, qd) qd + g(q),
 *        evaluated at one state. All vectors are in model joint order.
 */
struct JointSpaceDynamics {
  Eigen::MatrixXd mass_matrix;      ///< M(q), symmetric positive definite [kg m^2].
  Eigen::VectorXd coriolis_torque;  ///< C(q, qd) qd [Nm].
  Eigen::VectorXd gravity_torque;   ///< g(q) [Nm].

  /// Size every term for `joint_count` joints and zero it.
  void resize(std::size_t joint_count) {
    const auto n = static_cast<Eigen::Index>(joint_count);
    mass_matrix = Eigen::MatrixXd::Zero(n, n);
    coriolis_torque = Eigen::VectorXd::Zero(n);
    gravity_torque = Eigen::VectorXd::Zero(n);
  }
};

/**
 * @brief Abstract joint-space dynamics model of one serial chain.
 *
 * Implementations are evaluated from a single thread by the torque loop; they
 * may keep mutable scratch storage and need not be thread-safe.
 */
class InverseDynamicsModel {
 public:
  virtual ~InverseDynamicsModel() = default;

  /// @return Joint names in model order (base to tip).
  virtual const std::vector<std::string>& jointNames() const = 0;

  /// @return Number of joints.
  std::size_t jointCount() const { return jointNames().size(); }

  /**
   * @brief Evaluate M(q), C(q, qd) qd and g(q).
   * @param positions q in model order [rad] (or [m] for prismatic joints).
   * @param velocities qd in model order [rad/s].
   * @param dynamics Receives the terms, resized by the implementation.
   * @throw std::runtime_error when the input size is wrong or the backend fails.
   */
  virtual void evaluate(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                        JointSpaceDynamics& dynamics) const = 0;

  /// @return Symmetric torque saturation limit per joint [Nm].
  virtual const std::vector<double>& effortLimits() const = 0;

  /// @return Viscous damping coefficient per joint, b in tau_damping = b qd [Nm s/rad].
  virtual const std::vector<double>& viscousDamping() const = 0;

  /// @return One line describing the model for the start-up log.
  virtual std::string description() const = 0;
};

}  // namespace baxter
