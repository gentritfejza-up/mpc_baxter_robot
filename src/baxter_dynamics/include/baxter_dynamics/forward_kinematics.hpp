/**
 * @file forward_kinematics.hpp
 * @brief Forward kinematics of a serial chain from its classic DH table,
 *        returning the homogeneous transform of every link frame with respect
 *        to the base frame.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include "arm_description.hpp"

#include <Eigen/Dense>

#include <vector>

namespace baxter {

/**
 * @brief Chain forward kinematics ^0T_i for every link of an arm.
 *
 * Link frame i is attached to the distal end of link i following the classic
 * DH convention. Frame 0 is the base frame of the DH table (the frame the
 * Lagrange-Euler gravity vector is expressed in). The class only references
 * the description it was built with; the caller keeps it alive.
 */
class ForwardKinematics {
 public:
  /// One 4x4 homogeneous transform per frame, frame 0 (identity) included.
  using Transforms = std::vector<Eigen::Matrix4d>;

  /**
   * @param arm Validated arm description; must outlive the solver.
   */
  explicit ForwardKinematics(const ArmDescription& arm) : arm_(arm) {}

  /**
   * @brief Compute ^0T_0 ... ^0T_n for the joint values @p positions.
   * @param positions Joint positions in chain order [rad] or [m].
   * @param transforms Receives n + 1 transforms; element 0 is the identity.
   * @throw std::runtime_error when @p positions has the wrong size.
   */
  void compute(const Eigen::VectorXd& positions, Transforms& transforms) const;

  /**
   * @brief Count the joints whose position lies outside their limits.
   * @param positions Joint positions in chain order.
   * @return Number of joints out of range (0 when all are within limits).
   */
  std::size_t countLimitViolations(const Eigen::VectorXd& positions) const;

  /**
   * @brief Homogeneous transform of one classic DH row,
   *        `Rz(theta) Tz(d) Tx(a) Rx(alpha)`.
   * @param d Offset along z [m].
   * @param theta Rotation about z [rad].
   * @param a Length along x [m].
   * @param alpha Twist about x [rad].
   * @return ^{i-1}T_i.
   */
  static Eigen::Matrix4d dhTransform(double d, double theta, double a, double alpha);

 private:
  const ArmDescription& arm_;
};

}  // namespace baxter
