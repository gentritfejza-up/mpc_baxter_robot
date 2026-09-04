#pragma once

#include <ros/ros.h>

#include <casadi/casadi.hpp>
#include <casadi/core/dm_fwd.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <baxter_kinematics/baxter_kinematics.hpp>

#include "mpc_controller.hpp"

namespace {
constexpr auto kSamplingTime{0.1};
constexpr auto kRosLoopRate{10};
constexpr auto kNumJoints{7};
constexpr auto kPredictionHorizon{20};
constexpr auto kControlHorizon{15};
constexpr auto kNumThreads{10};
}  // namespace

/**
 * @class MPCControlManager
 * @brief Manages ROS-based MPC control operations for Baxter robot limbs.
 *
 * This class encapsulates the ROS node for managing an MPC controller.
 * It handles joint angles and velocities using ROS topics, and controls
 * a specific limb of the robot by setting joint accelerations.
 * Supports end-effector z-axis safety constraints via forward kinematics.
 */
class MPCControlManager {
 public:
  /**
   * @brief Constructor to initialize the MPCControlManager.
   *
   * @param node_handle The ROS NodeHandle used for communication with ROS
   * topics.
   * @param limb_side The side of the robot limb ("left" or "right") to control.
   */
  MPCControlManager(ros::NodeHandle& nh, const std::string& limb_side);

  /**
   * @brief Start the ROS event loop and handle MPC control.
   * @param x_ref The reference configuration (e.g. target joint angles).
   */
  void run();

 private:
  ros::NodeHandle nh_;
  std::string limb_side_;
  std::unique_ptr<Limb> limb_;
  std::unique_ptr<BaxterKinematicsInterface> kinematics_;
  MPCController mpc_;

  bool ee_z_constraint_enabled_{false};
  double ee_z_min_m_{-0.20};
  std::string ee_base_link_{"base"};
  std::string ee_end_link_;
  std::vector<int> mpc_to_kdl_index_;
  std::vector<double> previous_applied_accelerations_ =
      std::vector<double>(kNumJoints, 0.0);

  /**
   * @brief Get joint angles from the robot limb.
   * @return A map containing joint names and their current angles.
   */
  std::unordered_map<std::string, double> getJointAngles() const;

  /**
   * @brief Get joint velocities from the robot limb.
   * @return A map containing joint names and their current velocities.
   */
  std::unordered_map<std::string, double> getJointVelocities() const;

  /**
   * @brief Process MPC control at each iteration.
   *
   * Retrieves joint states, computes optimal accelerations,
   * and sends the control command to the robot limb.
   * @param x_ref The reference configuration (e.g. target joint angles).
   */
  void processControl(const std::vector<double>& x_ref);

  bool initializeZConstraintKinematics();
  void updateLinearizedZConstraint(const std::vector<double>& x0_vals);
};
