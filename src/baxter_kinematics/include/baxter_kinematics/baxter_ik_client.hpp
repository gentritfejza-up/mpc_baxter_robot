/**
 * @file baxter_ik_client.hpp
 * @brief Thin client for Baxter's `SolvePositionIK` service
 *        (`/ExternalTools/<arm>/PositionKinematicsNode/IKService`).
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include <baxter_core_msgs/SolvePositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>

/// @brief Wrapper around the SDK's position IK service for one arm.
class BaxterIKClient {
 public:
  /**
   * @brief Connect to the IK service of one arm; waits up to 5 s for it and warns
   *        when it is absent (calls then fail).
   * @param arm_side "left" or "right".
   */
  explicit BaxterIKClient(const std::string& arm_side);

  /**
   * @brief Solve IK for one pose.
   * @param desired_pose Target tool-frame pose (header.frame_id = "base") [m].
   * @param joint_solution Receives the first valid joint solution [rad].
   * @param seed_mode One of `SolvePositionIKRequest::SEED_*`.
   * @param user_seed Seed used only when `seed_mode == SEED_USER` [rad].
   * @return True when the service returned a valid solution.
   */
  bool computeIK(const geometry_msgs::PoseStamped& desired_pose,
                 sensor_msgs::JointState& joint_solution,
                 uint8_t seed_mode = baxter_core_msgs::SolvePositionIKRequest::SEED_CURRENT,
                 const sensor_msgs::JointState& user_seed = sensor_msgs::JointState());

  /**
   * @brief Solve IK seeded from `user_seed` (`SEED_USER`).
   * @param desired_pose Target tool-frame pose (header.frame_id = "base") [m].
   * @param joint_solution Receives the first valid joint solution [rad].
   * @param user_seed Seed joint angles [rad].
   * @return True when the service returned a valid solution.
   */
  bool computeIK(const geometry_msgs::PoseStamped& desired_pose,
                 sensor_msgs::JointState& joint_solution,
                 const sensor_msgs::JointState& user_seed) {
    return computeIK(desired_pose, joint_solution,
                     baxter_core_msgs::SolvePositionIKRequest::SEED_USER, user_seed);
  }

 private:
  ros::NodeHandle nh_;
  ros::ServiceClient ik_client_;
  std::string service_name_;
};
