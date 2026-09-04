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
 * @class BaxterKinematicsInterface
 * @brief A KDL-based kinematics class that:
 *   - Always loads the URDF from the "robot_description" param
 *   - Always uses ChainIkSolverPos_NR_JL to enforce joint limits
 *   - Provides forward kinematics, inverse kinematics, and Jacobian
 *   - The chain is built from a specified base_link to an end_link
 *
 * Typical usage:
 *   BaxterKinematicsInterface kin("base", "right_hand");
 *   auto fk = kin.forward(q);  // KDL::Frame
 *   auto q_sol = kin.inverse(fk, q_seed); // respects joint limits
 *   auto jac = kin.jacobian(q);
 */
class BaxterKinematicsInterface
{
public:
	/**
   * @param base_link The starting link of the chain
   * @param end_link  The tip link of the chain
   * @note This constructor loads the URDF from "robot_description",
   *       parses the chain, parses joint limits, and creates the KDL solvers.
   * @throw std::runtime_error if URDF or chain creation fails.
   */
	BaxterKinematicsInterface(const std::string& base_link, const std::string& end_link);

	/**
   * @brief Number of joints in the chain
   */
	size_t numJoints() const
	{
		return num_joints_;
	}

	/**
   * @brief Joint names in the chain's order
   */
	const std::vector<std::string>& getJointNames() const
	{
		return joint_names_;
	}

	/**
   * @brief Lower joint limits
   */
	const std::vector<double>& getLowerLimits() const
	{
		return joint_limits_lower_;
	}

	/**
   * @brief Upper joint limits
   */
	const std::vector<double>& getUpperLimits() const
	{
		return joint_limits_upper_;
	}

	/**
   * @brief Forward kinematics: compute end-link pose from joint angles (in chain order).
   * @param q A vector of joint angles of size numJoints().
   * @return A KDL::Frame in the base_link frame. If solver fails, returns identity.
   */
	KDL::Frame forward(const std::vector<double>& q) const;

	/**
   * @brief Inverse Kinematics: solves for joint angles that achieve desired_pose,
   *        respecting URDF joint limits with NR_JL solver.
   * @param desired_pose The target pose in base_link frame.
   * @param seed         Initial guess (size must be numJoints()).
   * @return A vector of angles if success, or empty if solver fails.
   */
	std::vector<double> inverse(const KDL::Frame& desired_pose,
								const std::vector<double>& seed) const;

	/**
   * @brief Compute the 6xN Jacobian at the given joint configuration q.
   * @param q A vector of joint angles (size numJoints()).
   * @return A KDL::Jacobian. If solver fails, returned Jacobian has 0 columns.
   */
	KDL::Jacobian jacobian(const std::vector<double>& q) const;

private:
	// Build the chain from the URDF in robot_description
	bool buildChain(const std::string& base_link, const std::string& end_link);
	// Parse URDF joint limits
	void parseJointLimits(const urdf::Model& urdf);

	// Helper to convert vector -> KDL::JntArray
	KDL::JntArray toKDL(const std::vector<double>& v) const;
	// Helper to convert KDL::JntArray -> vector
	std::vector<double> toStd(const KDL::JntArray& arr) const;

private:
	KDL::Chain chain_;
	size_t num_joints_{0};

	// Joint info
	std::vector<std::string> joint_names_;
	std::vector<double> joint_limits_lower_;
	std::vector<double> joint_limits_upper_;

	// Kinematics solvers
	std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
	std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
	// We always use the joint-limited IK solver
	std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
	std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
};
