#pragma once

#include <baxter_core_msgs/SolvePositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

/**
 * @brief Thin ROS service wrapper for Baxter’s IK solver.
 */
class BaxterIKClient {
 public:
  /// Construct for either the "left" or "right" arm.
  explicit BaxterIKClient(const std::string& arm_side);

  /**
   * @brief Solve IK with an arbitrary seed mode.
   * @param desired_pose   Target tool‑frame pose (header.frame_id = "base").
   * @param joint_solution Returns the first valid joint solution.
   * @param seed_mode      One of SolvePositionIKRequest::SEED_*.
   * @param user_seed      Used only when seed_mode == SEED_USER.
   * @return true if a valid solution was found.
   */
  bool computeIK(const geometry_msgs::PoseStamped& desired_pose,
                 sensor_msgs::JointState& joint_solution,
                 uint8_t seed_mode = baxter_core_msgs::SolvePositionIKRequest::SEED_CURRENT,
                 const sensor_msgs::JointState& user_seed = sensor_msgs::JointState());

  /**
   * @brief Convenience overload for user‑provided seeds.
   *        Internally sets seed_mode = SEED_USER.
   */
  inline bool computeIK(const geometry_msgs::PoseStamped& desired_pose,
                        sensor_msgs::JointState& joint_solution,
                        const sensor_msgs::JointState& user_seed) {
    return computeIK(desired_pose,
                     joint_solution,
                     baxter_core_msgs::SolvePositionIKRequest::SEED_USER,
                     user_seed);
  }

 private:
  ros::NodeHandle nh_;
  ros::ServiceClient ik_client_;
  std::string service_name_;
};
