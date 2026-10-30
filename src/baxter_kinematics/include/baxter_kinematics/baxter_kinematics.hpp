/**
 * @file baxter_kinematics.hpp
 * @brief KDL forward kinematics, joint-limited inverse kinematics and Jacobian
 *        for a Baxter arm chain built from `robot_description`.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <ros/ros.h>
#include <urdf/model.h>

/**
 * @brief Kinematics of one serial chain of the Baxter URDF.
 *
 * The chain runs from `base_link` to `end_link`; joint limits come from the URDF
 * and bound the Newton-Raphson IK solver (`ChainIkSolverPos_NR_JL`). Joint
 * vectors are in chain order, see getJointNames().
 *
 * Typical usage:
 * @code
 *   BaxterKinematicsInterface kinematics("base", "right_hand");
 *   KDL::Frame tip_pose = kinematics.forward(q);
 *   std::vector<double> solution = kinematics.inverse(tip_pose, q_seed);
 *   KDL::Jacobian jacobian = kinematics.jacobian(q);
 * @endcode
 */
class BaxterKinematicsInterface {
 public:
  /**
   * @brief Load the URDF from `robot_description`, extract the chain and its joint
   *        limits, and create the solvers.
   * @param base_link Root link of the chain.
   * @param end_link Tip link of the chain.
   * @throw std::runtime_error when the URDF cannot be loaded or the chain does not exist.
   */
  BaxterKinematicsInterface(const std::string& base_link, const std::string& end_link);

  /// @return Number of movable joints in the chain.
  size_t numJoints() const { return joint_count_; }

  /// @return Movable joint names in chain order.
  const std::vector<std::string>& getJointNames() const { return joint_names_; }

  /// @return Lower joint limits in chain order [rad]; -inf when the URDF has none.
  const std::vector<double>& getLowerLimits() const { return joint_limits_lower_; }

  /// @return Upper joint limits in chain order [rad]; +inf when the URDF has none.
  const std::vector<double>& getUpperLimits() const { return joint_limits_upper_; }

  /**
   * @brief Forward kinematics of the tip link.
   * @param q Joint angles in chain order, size numJoints() [rad].
   * @return Tip pose in the `base_link` frame; identity when the size is wrong or
   *         the solver fails (logged as an error).
   */
  KDL::Frame forward(const std::vector<double>& q) const;

  /**
   * @brief Joint-limited inverse kinematics.
   * @param desired_pose Target tip pose in the `base_link` frame.
   * @param seed Initial guess in chain order, size numJoints() [rad].
   * @return Joint angles in chain order [rad], or an empty vector when the size is
   *         wrong or the solver fails (logged as an error).
   */
  std::vector<double> inverse(const KDL::Frame& desired_pose,
                              const std::vector<double>& seed) const;

  /**
   * @brief Geometric Jacobian of the tip link, 6 x numJoints().
   * @param q Joint angles in chain order, size numJoints() [rad].
   * @return The Jacobian in the `base_link` frame; zero columns when the size is
   *         wrong or the solver fails (logged as an error).
   */
  KDL::Jacobian jacobian(const std::vector<double>& q) const;

 private:
  /// @return False (after logging) when the URDF, the tree or the chain cannot be built.
  bool buildChain(const std::string& base_link, const std::string& end_link);
  /// Fill the limit vectors from the URDF; joints without limits keep +-inf.
  void parseJointLimits(const urdf::Model& urdf_model);

  KDL::JntArray toJntArray(const std::vector<double>& values) const;
  std::vector<double> toStdVector(const KDL::JntArray& joint_array) const;

  KDL::Chain chain_;
  size_t joint_count_{0};
  std::vector<std::string> joint_names_;
  std::vector<double> joint_limits_lower_;  ///< [rad]
  std::vector<double> joint_limits_upper_;  ///< [rad]

  // The IK position solver references the FK and IK velocity solvers, so these
  // are declared (and therefore destroyed) in dependency order.
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_velocity_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_position_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
};
