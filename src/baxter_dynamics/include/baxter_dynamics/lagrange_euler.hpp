/**
 * @file lagrange_euler.hpp
 * @brief Closed-form Lagrange-Euler inverse dynamics of a serial manipulator
 *        built from a classic DH model: joint-space mass matrix M(q),
 *        Coriolis/centrifugal vector C(q, qd) qd and gravity vector g(q).
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * The model is generic in the number of joints and supports revolute and
 * prismatic joints; it is the thesis' own derivation and the alternative to the
 * URDF-based KDL model behind the same InverseDynamicsModel interface.
 *
 * @section le_equations Equations
 *
 * The implementation follows the matrix form of the Lagrange-Euler equations
 * (Fu, Gonzalez and Lee, "Robotics: Control, Sensing, Vision and
 * Intelligence", ch. 3). With n links, link i's frame attached by the classic
 * DH convention and ^0T_i the homogeneous transform from the base to link
 * frame i, the joint torques are
 *
 *     tau_i = sum_k M_ik(q) qdd_k + sum_k sum_m h_ikm(q) qd_k qd_m + G_i(q)
 *
 * with
 *
 *     M_ik  = sum_{j = max(i,k)}^{n}   Tr( U_jk  J_j U_ji^T )          [kg m^2]
 *     h_ikm = sum_{j = max(i,k,m)}^{n} Tr( U_jkm J_j U_ji^T )          [kg m^2]
 *     C_i   = sum_k sum_m h_ikm qd_k qd_m                               [N m]
 *     G_i   = - sum_{j = i}^{n} m_j g^T U_ji r_j                        [N m]
 *
 * where g = (g_x, g_y, g_z, 0)^T is the gravity vector in the base frame and
 * r_j = (x, y, z, 1)^T is the centre of mass of link j in link frame j.
 *
 * The U matrices are the partial derivatives of the link transforms with
 * respect to the joint variables:
 *
 *     U_ij  = d(^0T_i)/dq_j = ^0T_{j-1} Q_j ^{j-1}T_i                    for j <= i, else 0
 *     U_ijk = dU_ij/dq_k    = ^0T_{j-1} Q_j ^{j-1}T_{k-1} Q_k ^{k-1}T_i  for i >= k >= j
 *                           = ^0T_{k-1} Q_k ^{k-1}T_{j-1} Q_j ^{j-1}T_i  for i >= j >= k
 *                           = 0                                         otherwise
 *
 * Q_j is the constant derivative operator of joint j with respect to its
 * variable: for a revolute joint (rotation about z_{j-1})
 * Q = [[0,-1,0,0],[1,0,0,0],[0,0,0,0],[0,0,0,0]], for a prismatic joint
 * (translation along z_{j-1}) Q = [[0,0,0,0],[0,0,0,0],[0,0,0,1],[0,0,0,0]].
 * Because ^{j-1}T_i = (^0T_{j-1})^{-1} ^0T_i, every U_ij is the product of the
 * joint operator in the base frame, D_j = ^0T_{j-1} Q_j (^0T_{j-1})^{-1}, and
 * ^0T_i; the code precomputes the n operators D_j once per evaluation.
 *
 * J_i is the pseudo-inertia matrix of link i, the second moment of the mass
 * distribution about the origin of link frame i:
 *
 *     J_i = int r r^T dm = [ S_i        m_i c_i ]
 *                          [ m_i c_i^T  m_i     ]
 *
 * with c_i the centre of mass in frame i and S_i = int (x,y,z)(x,y,z)^T dm.
 * The description stores the inertia tensor I_C about the centre of mass
 * (URDF sign convention I_xy = -int xy dm) expressed in the DH frame, so S_i is
 * obtained by converting to second moments and shifting to the frame origin
 * with the parallel-axis theorem:
 *
 *     S_i = ( 1/2 Tr(I_C) 1 - I_C ) + m_i c_i c_i^T
 *
 * i.e. the diagonal is (-Ixx+Iyy+Izz)/2, (Ixx-Iyy+Izz)/2, (Ixx+Iyy-Izz)/2, the
 * off-diagonals are -Ixy, -Iyz, -Izx, and m c c^T is added to every entry.
 *
 * h_ikm is symmetric in (k, m) because U_jkm = U_jmk, so the Coriolis sum is
 * evaluated over k <= m only.
 *
 * Indices in the code are zero based: link l in {0..n-1} is link l+1 of the
 * formulas, `transforms[l+1]` is ^0T_{l+1} and `transforms[l]` is ^0T_l, the
 * frame in which the axis of joint l+1 is defined.
 */
#pragma once

#include "arm_description.hpp"
#include "forward_kinematics.hpp"
#include "inverse_dynamics_model.hpp"

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace baxter {

/**
 * @brief Lagrange-Euler model tau = M(q) qdd + C(q, qd) qd + g(q) of one arm.
 *
 * The constructor validates the description and precomputes the constant
 * pseudo-inertia matrices and joint operators. `evaluate()` is allocation-free
 * after the first call (scratch tables are kept as mutable members), which
 * makes it suitable for the 800 Hz torque loop; it is not thread-safe.
 */
class LagrangeEuler : public InverseDynamicsModel {
 public:
  using LinkMatrices = std::vector<Eigen::Matrix4d>;

  /**
   * @param arm Arm description; copied, so the caller need not keep it.
   * @throw std::invalid_argument when the description is inconsistent.
   */
  explicit LagrangeEuler(ArmDescription arm);

  const std::vector<std::string>& jointNames() const override { return joint_names_; }
  const std::vector<double>& effortLimits() const override { return effort_limits_; }
  const std::vector<double>& viscousDamping() const override { return viscous_damping_; }
  std::string description() const override;

  /**
   * @brief Evaluate M(q), C(q, qd) qd and g(q).
   *
   * Positions outside the configured limits only produce a throttled warning
   * (the model is still evaluated): the torque loop must keep commanding a
   * joint that sits on its stop.
   * @copydoc InverseDynamicsModel::evaluate
   */
  void evaluate(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                JointSpaceDynamics& dynamics) const override;

  /// @return The arm description the model was built from.
  const ArmDescription& arm() const { return arm_; }

  /// Derivative operator Q of a joint with respect to its variable (see file documentation).
  static Eigen::Matrix4d jointOperator(JointType type);

 private:
  /// Flat index of U_ij in a LinkMatrices table of n^2 entries.
  std::size_t indexUij(std::size_t i, std::size_t j) const { return i * joint_count_ + j; }
  /// Flat index of U_ijk in a LinkMatrices table of n^3 entries.
  std::size_t indexUijk(std::size_t i, std::size_t j, std::size_t k) const {
    return (i * joint_count_ + j) * joint_count_ + k;
  }

  /// Pseudo-inertia J_i of every link in its own frame [kg m^2, kg m, kg].
  LinkMatrices buildPseudoInertiaMatrices() const;

  /// D_j = ^0T_{j-1} Q_j (^0T_{j-1})^{-1} for every joint.
  void computeJointDerivativeOperators(const LinkMatrices& transforms,
                                       LinkMatrices& joint_operators) const;
  /// U_ij = D_j ^0T_i for j <= i (zero otherwise).
  void computeUij(const LinkMatrices& transforms, const LinkMatrices& joint_operators,
                  LinkMatrices& U_ij) const;
  /// U_ijk = D_j U_ik for k >= j, U_ikj by symmetry; zero unless i >= max(j, k).
  void computeUijk(const LinkMatrices& U_ij, const LinkMatrices& joint_operators,
                   LinkMatrices& U_ijk) const;
  /// W_ji = J_j U_ji^T, the factor shared by every term of M and h.
  void computeInertiaWeightedUij(const LinkMatrices& U_ij, LinkMatrices& weighted_U_ij) const;
  /// M_ik = sum_j Tr(U_jk W_ji).
  void computeMassMatrix(const LinkMatrices& U_ij, const LinkMatrices& weighted_U_ij,
                         Eigen::MatrixXd& mass_matrix) const;
  /// C_i = sum_k sum_m h_ikm qd_k qd_m with h_ikm = sum_j Tr(U_jkm W_ji).
  void computeCoriolisVector(const LinkMatrices& U_ijk, const LinkMatrices& weighted_U_ij,
                             const Eigen::VectorXd& velocities, Eigen::VectorXd& coriolis) const;
  /// G_i = -sum_j m_j g^T U_ji r_j.
  void computeGravityVector(const LinkMatrices& U_ij, Eigen::VectorXd& gravity) const;

  ArmDescription arm_;
  std::size_t joint_count_{0};
  std::vector<std::string> joint_names_;
  std::vector<double> effort_limits_;
  std::vector<double> viscous_damping_;
  ForwardKinematics forward_kinematics_;
  Eigen::Vector4d gravity_;              ///< (g_x, g_y, g_z, 0) in the base frame.
  std::vector<Eigen::Matrix4d> joint_Q_;  ///< Q_j per joint (revolute or prismatic).
  std::vector<Eigen::Vector4d> centre_of_mass_;  ///< r_j, homogeneous.
  LinkMatrices pseudo_inertia_;           ///< J_i, constant for a rigid arm.

  /// Scratch tables reused between evaluations (no allocation in the loop).
  struct Workspace {
    LinkMatrices transforms;
    LinkMatrices joint_operators;
    LinkMatrices U_ij;
    LinkMatrices U_ijk;
    LinkMatrices weighted_U_ij;
  };
  mutable Workspace workspace_;
};

}  // namespace baxter
