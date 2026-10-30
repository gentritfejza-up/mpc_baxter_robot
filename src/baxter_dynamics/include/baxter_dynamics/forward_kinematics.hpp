/**
 * @file forward_kinematics.hpp
 * @brief Forward kinematics of one Baxter arm from its classic DH table,
 *        returning the homogeneous transform of every link frame with respect
 *        to the arm base frame.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include "robot_model.hpp"

#include <Eigen/Dense>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace baxter {

/**
 * @brief Chain forward kinematics ^0T_i for all links of a 7-DOF Baxter arm.
 *
 * Link frame i is attached to the distal end of link i following the classic
 * DH convention. Frame 0 is the arm base frame of the DH table (the frame the
 * Lagrange-Euler gravity vector is expressed in).
 */
class ForwardKinematics {
public:
  /**
   * @brief Builds the solver and loads the arm description for @p limb.
   * @param limb Limb side, `"left"` or `"right"`.
   * @param nh Node handle used to read the `robot/limb/...` parameters.
   */
  ForwardKinematics(const std::string &limb, ros::NodeHandle &nh);

  /**
   * @brief Builds the solver on an already loaded arm description.
   * @param nh Unused; kept so both constructors share one signature style.
   * @param robot Arm description shared with the caller.
   */
  ForwardKinematics(ros::NodeHandle &nh, std::shared_ptr<Robot> robot);

  /**
   * @brief Computes the base-to-link transforms ^0T_i for i = 0..7.
   *
   * Joint angles outside the configured limits only produce a throttled
   * warning; the transforms are still computed.
   *
   * @param joint_angles Joint angles q keyed by full joint name [rad]. Must
   *        contain every joint of the arm.
   * @return Vector of 8 homogeneous transforms: element 0 is the identity
   *         (base frame), element i is ^0T_i for link i [m for translations].
   * @throws std::out_of_range if a joint of the arm is missing from
   *         @p joint_angles.
   */
  std::vector<Eigen::Matrix4d> solveIntermediateFK(
      const std::unordered_map<std::string, double> &joint_angles) const;

private:
  /**
   * @brief Homogeneous transform of one classic DH row,
   *        `Rz(theta) Tz(d) Tx(a) Rx(alpha)`.
   * @param d Offset along z [m].
   * @param theta Rotation about z (offset plus joint angle) [rad].
   * @param a Length along x [m].
   * @param alpha Twist about x [rad].
   * @return 4x4 homogeneous transform ^{i-1}T_i.
   */
  static Eigen::Matrix4d dhTransform(double d, double theta, double a,
                                     double alpha);

  /**
   * @brief Logs a throttled warning for every joint outside its angle limits.
   * @param joint_angles Joint angles keyed by full joint name [rad].
   * @return true if every joint is within its limits.
   */
  bool checkJointLimits(
      const std::unordered_map<std::string, double> &joint_angles) const;

  std::shared_ptr<Robot> robot_{nullptr};
};

} // namespace baxter
