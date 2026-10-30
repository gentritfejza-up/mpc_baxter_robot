/**
 * @file robot_model.hpp
 * @brief Kinematic description of one Baxter arm loaded from the parameter
 *        server: ordered joint names, joint angle limits and the classic
 *        Denavit-Hartenberg table of the 7-DOF chain.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * All values are read from the `robot/limb/...` namespace populated by
 * `config/dynamics_params.yaml`. The DH table uses the classic (distal)
 * convention: the transform from link frame i-1 to link frame i is
 * `Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)`.
 */
#pragma once

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace baxter {

/**
 * @brief Classic Denavit-Hartenberg parameters of a single revolute joint.
 *
 * The joint variable q_i is added to @ref theta_offset, i.e. the rotation
 * about the joint axis is `theta_i = theta_offset + q_i`.
 */
struct DhParameters {
  double d{0.0};            ///< Offset along the previous z axis [m].
  double theta_offset{0.0}; ///< Constant rotation about the previous z axis [rad].
  double a{0.0};            ///< Length along the rotated x axis [m].
  double alpha{0.0};        ///< Twist about the rotated x axis [rad].
};

/**
 * @brief Joint names, angle limits and DH table of one Baxter arm.
 *
 * Every accessor returns data keyed by the full joint name as published by
 * Baxter (e.g. `left_s0`). The constructor throws `std::runtime_error` if a
 * required parameter is missing or malformed, so a fully constructed object is
 * always complete.
 */
class Robot {
public:
  /**
   * @brief Loads the arm description for one limb from the parameter server.
   * @param limb Limb side, `"left"` or `"right"`.
   * @param nh Node handle used to read `robot/limb/...` parameters.
   * @throws std::runtime_error if joint names, angle limits or DH parameters
   *         are missing or malformed.
   */
  Robot(const std::string &limb, ros::NodeHandle &nh);

  /**
   * @brief DH parameters of every joint, keyed by full joint name.
   * @return Map joint name -> DH row (d [m], theta_offset [rad], a [m],
   *         alpha [rad]).
   */
  const std::unordered_map<std::string, DhParameters> &getDhParams() const;

  /**
   * @brief Joint angle limits, keyed by full joint name.
   * @return Map joint name -> (lower, upper) limit [rad].
   */
  const std::unordered_map<std::string, std::pair<double, double>> &
  getAngleLimits() const;

  /**
   * @brief Joint names in kinematic order from the shoulder to the wrist.
   * @return Ordered list of the seven joint names (`<limb>_s0` ... `<limb>_w2`).
   */
  const std::vector<std::string> &getJointNames() const;

private:
  /// Reads `robot/limb/<limb>` (ordered joint names).
  void loadJointNames(ros::NodeHandle &nh);
  /// Reads `robot/limb/angle_limits/<joint suffix>` for every joint.
  void loadAngleLimits(ros::NodeHandle &nh);
  /// Reads `robot/limb/dh_parameters/<joint suffix>` for every joint.
  void loadDhParameters(ros::NodeHandle &nh);

  /// Two-letter joint suffix (`s0`, `s1`, ...) used as key in the YAML tables.
  static std::string jointSuffix(const std::string &joint_name);

  std::string limb_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::pair<double, double>> angle_limits_;
  std::unordered_map<std::string, DhParameters> dh_parameters_;
};

} // namespace baxter
