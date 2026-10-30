/**
 * @file lagrange_euler.hpp
 * @brief Closed-form Lagrange-Euler inverse dynamics of the 7-DOF Baxter arm
 *        built from a classic DH model: joint-space mass matrix M(q),
 *        Coriolis/centrifugal vector C(q, qd) and gravity vector g(q).
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * @section le_equations Equations
 *
 * The implementation follows the matrix form of the Lagrange-Euler equations
 * (Fu, Gonzalez and Lee, "Robotics: Control, Sensing, Vision and
 * Intelligence", ch. 3). With n = 7 links, link i's frame attached by the
 * classic DH convention and ^0T_i the homogeneous transform from the arm base
 * to link frame i, the joint torques are
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
 * where g = (0, 0, -9.81, 0)^T is the gravity vector in the base frame and
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
 * Q_j is the constant derivative operator of a revolute joint about its
 * z axis, Q_j = [[0,-1,0,0],[1,0,0,0],[0,0,0,0],[0,0,0,0]] (identical for all
 * seven joints, read from `robot/limb/Q_j`). Because ^{j-1}T_i =
 * (^0T_{j-1})^{-1} ^0T_i, every U_ij is the product of the joint operator in
 * the base frame, D_j = ^0T_{j-1} Q_j (^0T_{j-1})^{-1}, and ^0T_i; the code
 * precomputes the seven D_j once per call.
 *
 * J_i is the pseudo-inertia matrix of link i, the second moment of the mass
 * distribution about the origin of link frame i:
 *
 *     J_i = int r r^T dm = [ S_i        m_i c_i ]
 *                          [ m_i c_i^T  m_i     ]
 *
 * with c_i the centre of mass in frame i and S_i = int (x,y,z)(x,y,z)^T dm.
 * The YAML stores the URDF inertia tensor I_C about the centre of mass
 * (URDF sign convention I_xy = -int xy dm) rotated into the DH frame, so S_i is
 * obtained by converting to second moments and shifting to the frame origin
 * with the parallel-axis theorem:
 *
 *     S_i = ( 1/2 Tr(I_C) 1 - I_C ) + m_i c_i c_i^T
 *
 * i.e. the diagonal is (-Ixx+Iyy+Izz)/2, (Ixx-Iyy+Izz)/2, (Ixx+Iyy-Izz)/2, the
 * off-diagonals are -Ixy, -Iyz, -Izx, and m c c^T is added to every entry.
 *
 * Indices in the code are zero based: link l in {0..6} is link l+1 of the
 * formulas, `link_transforms[l+1]` is ^0T_{l+1} and `link_transforms[l]` is
 * ^0T_l, the frame in which the joint axis of link l+1 is defined.
 */
#pragma once

#include "forward_kinematics.hpp"

#include <Eigen/Dense>
#include <ros/ros.h>

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace baxter {

/**
 * @brief Inertial parameters of one link as stored in `dynamics_params.yaml`.
 *
 * The inertia tensor is the URDF tensor about the link centre of mass, already
 * rotated into the DH link frame (URDF sign convention, I_xy = -int xy dm).
 */
struct LinkInertialParameters {
  double mass{0.0}; ///< Link mass [kg].
  /// Centre of mass in the DH link frame as a homogeneous point (x, y, z, 1) [m].
  Eigen::Vector4d centre_of_mass{Eigen::Vector4d::Zero()};
  double Ixx{0.0}; ///< Moment of inertia about x, at the COM [kg m^2].
  double Iyy{0.0}; ///< Moment of inertia about y, at the COM [kg m^2].
  double Izz{0.0}; ///< Moment of inertia about z, at the COM [kg m^2].
  double Ixy{0.0}; ///< Product of inertia xy, URDF sign, at the COM [kg m^2].
  double Iyz{0.0}; ///< Product of inertia yz, URDF sign, at the COM [kg m^2].
  double Izx{0.0}; ///< Product of inertia zx, URDF sign, at the COM [kg m^2].
};

/**
 * @brief Lagrange-Euler inverse dynamics tau = M(q) qdd + C(q, qd) + g(q) of
 *        one Baxter arm.
 *
 * All model parameters are read once from the parameter server in the
 * constructor (`robot/limb/...` and `robot/torque_limits/...`). The computed
 * torques are clamped to the configured per-joint-group limits before they are
 * returned. The constructor throws `std::runtime_error` when a parameter is
 * missing or has the wrong size.
 */
class LagrangeEuler {
public:
  /// Number of moving links (= number of joints) of one Baxter arm.
  static constexpr int kNumLinks = 7;

  /// One 4x4 matrix per link, or a flat row-major table of them (see indexUij).
  using LinkMatrices = std::vector<Eigen::Matrix4d>;
  using JointVector = Eigen::Matrix<double, kNumLinks, 1>;
  using JointMatrix = Eigen::Matrix<double, kNumLinks, kNumLinks>;

  /**
   * @brief Loads the dynamic model and uses an existing kinematics solver.
   * @param limb Limb side, `"left"` or `"right"`; also the joint-name prefix.
   * @param nh Node handle used to read the model parameters.
   * @param forward_kinematics Solver providing ^0T_i for this limb.
   * @throws std::runtime_error on missing or malformed parameters.
   */
  LagrangeEuler(const std::string &limb, ros::NodeHandle &nh,
                std::shared_ptr<ForwardKinematics> forward_kinematics);

  /**
   * @brief Loads the dynamic model and builds its own kinematics solver.
   * @param limb Limb side, `"left"` or `"right"`.
   * @param nh Node handle used to read the model parameters.
   * @throws std::runtime_error on missing or malformed parameters.
   */
  LagrangeEuler(const std::string &limb, ros::NodeHandle &nh);

  /**
   * @brief Inverse dynamics tau = M(q) qdd + C(q, qd) + g(q), clamped to the
   *        configured torque limits.
   * @param joint_angles q keyed by full joint name [rad].
   * @param joint_velocities qd keyed by full joint name [rad/s].
   * @param joint_accelerations Desired qdd keyed by full joint name [rad/s^2].
   * @return Joint torques tau keyed by full joint name [N m].
   * @throws std::runtime_error if any joint of the arm is missing from one of
   *         the input maps.
   */
  std::unordered_map<std::string, double> computeTorques(
      const std::unordered_map<std::string, double> &joint_angles,
      const std::unordered_map<std::string, double> &joint_velocities,
      const std::unordered_map<std::string, double> &joint_accelerations) const;

private:
  /// Gravitational acceleration along -z of the arm base frame [m/s^2].
  static constexpr double kGravity = 9.81;

  /// Flat index of U_ij in a LinkMatrices table of kNumLinks^2 entries.
  static constexpr int indexUij(int i, int j) { return i * kNumLinks + j; }
  /// Flat index of U_ijk in a LinkMatrices table of kNumLinks^3 entries.
  static constexpr int indexUijk(int i, int j, int k) {
    return (i * kNumLinks + j) * kNumLinks + k;
  }

  /// Reads `robot/limb/<limb>` and checks it lists exactly kNumLinks joints.
  void loadJointNames(ros::NodeHandle &nh);
  /// Reads masses, centres of mass, inertia tensors and Q_j from `robot/limb/`.
  void loadLinkInertialParameters(ros::NodeHandle &nh);
  /// Reads the per-joint-group torque limits from `robot/torque_limits/`.
  void loadTorqueLimits(ros::NodeHandle &nh);

  /**
   * @brief Builds the pseudo-inertia matrices J_i (see file documentation).
   * @return J_i for every link, in link frame i [kg m^2, kg m, kg].
   */
  LinkMatrices buildPseudoInertiaMatrices() const;

  /**
   * @brief Joint derivative operators in the base frame,
   *        D_j = ^0T_{j-1} Q_j (^0T_{j-1})^{-1}.
   * @param link_transforms ^0T_0 ... ^0T_7 from the kinematics solver.
   * @return One D_j per joint.
   */
  LinkMatrices computeJointDerivativeOperators(
      const LinkMatrices &link_transforms) const;

  /**
   * @brief First derivatives U_ij = d(^0T_i)/dq_j.
   * @param link_transforms ^0T_0 ... ^0T_7.
   * @param joint_operators D_j from computeJointDerivativeOperators().
   * @return Flat table indexed by indexUij(i, j); zero for j > i.
   */
  LinkMatrices computeUij(const LinkMatrices &link_transforms,
                          const LinkMatrices &joint_operators) const;

  /**
   * @brief Second derivatives U_ijk = dU_ij/dq_k.
   * @param U_ij Table from computeUij().
   * @param joint_operators D_j from computeJointDerivativeOperators().
   * @return Flat table indexed by indexUijk(i, j, k); zero unless
   *         i >= max(j, k).
   */
  LinkMatrices computeUijk(const LinkMatrices &U_ij,
                           const LinkMatrices &joint_operators) const;

  /**
   * @brief Inertia-weighted transposed derivatives W_ji = J_j U_ji^T, the
   *        factor shared by every term of M and h.
   * @param U_ij Table from computeUij().
   * @return Flat table indexed by indexUij(j, i).
   */
  LinkMatrices computeInertiaWeightedUij(const LinkMatrices &U_ij) const;

  /**
   * @brief Mass matrix M_ik = sum_j Tr(U_jk W_ji).
   * @param U_ij Table from computeUij().
   * @param weighted_U_ij Table from computeInertiaWeightedUij().
   * @return Symmetric positive definite M(q) [kg m^2].
   */
  JointMatrix computeMassMatrix(const LinkMatrices &U_ij,
                                const LinkMatrices &weighted_U_ij) const;

  /**
   * @brief Coriolis/centrifugal torques C_i = sum_k sum_m h_ikm qd_k qd_m with
   *        h_ikm = sum_j Tr(U_jkm W_ji).
   * @param U_ijk Table from computeUijk().
   * @param weighted_U_ij Table from computeInertiaWeightedUij().
   * @param joint_velocities qd in joint order [rad/s].
   * @return C(q, qd) [N m].
   */
  JointVector computeCoriolisVector(const LinkMatrices &U_ijk,
                                    const LinkMatrices &weighted_U_ij,
                                    const JointVector &joint_velocities) const;

  /**
   * @brief Gravity torques G_i = -sum_j m_j g^T U_ji r_j.
   * @param U_ij Table from computeUij().
   * @return g(q) [N m].
   */
  JointVector computeGravityVector(const LinkMatrices &U_ij) const;

  /**
   * @brief Reads the arm's joints from a name->value map in joint order.
   * @param values Map keyed by full joint name.
   * @param quantity_name Used in the error message, e.g. "joint_velocities".
   * @return Values ordered like joint_names_.
   * @throws std::runtime_error if a joint is missing.
   */
  JointVector mapToJointVector(
      const std::unordered_map<std::string, double> &values,
      const char *quantity_name) const;

  /**
   * @brief Clamps every torque to the limit of its joint group (s1, other
   *        shoulder/elbow joints, wrist) and keys it by joint name.
   * @param torques Raw torques in joint order [N m].
   * @return Clamped torques keyed by full joint name [N m].
   */
  std::unordered_map<std::string, double>
  clampToTorqueLimits(const JointVector &torques) const;

  std::string limb_;
  std::shared_ptr<ForwardKinematics> forward_kinematics_{nullptr};
  std::vector<std::string> joint_names_;
  std::array<LinkInertialParameters, kNumLinks> links_;
  /// Revolute joint derivative operator Q_j (same for every joint).
  Eigen::Matrix4d Q_j_{Eigen::Matrix4d::Zero()};
  /// Pseudo-inertia matrices J_i, constant for a rigid arm, built once.
  LinkMatrices pseudo_inertia_;

  double torque_limit_shoulder_elbow_max_{50.0}; ///< s0, e0, e1 upper limit [N m].
  double torque_limit_shoulder_elbow_min_{-50.0}; ///< s0, e0, e1 lower limit [N m].
  double torque_limit_s1_max_{50.0};  ///< s1 upper limit [N m].
  double torque_limit_s1_min_{-50.0}; ///< s1 lower limit [N m].
  double torque_limit_wrist_max_{20.0};  ///< w0, w1, w2 upper limit [N m].
  double torque_limit_wrist_min_{-20.0}; ///< w0, w1, w2 lower limit [N m].
};

} // namespace baxter
