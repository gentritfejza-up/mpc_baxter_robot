#pragma once

/**
 * @file joint_state_listener.hpp
 * @brief Keeps the latest joint positions and velocities of one arm from
 *        /robot/joint_states.
 *
 * Used by the MPC of the opposite arm for the arm-vs-arm collision rows. `Limb` is
 * not reused here because it advertises command topics for the limb it is created
 * for.
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include "mpc_controller/arm_collision_model.hpp"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <array>
#include <mutex>
#include <string>

namespace mpc_geometry {

/**
 * @brief Thread-safe cache of the latest complete joint state of one arm.
 *
 * The subscriber callback runs on a ROS spinner thread while the control loop
 * reads through latest(); a mutex guards the stored state.
 */
class OtherArmStateListener {
 public:
  /**
   * @param node_handle Node handle used for the subscription.
   * @param side Arm to listen for, "left" or "right".
   */
  explicit OtherArmStateListener(ros::NodeHandle& node_handle, const std::string& side);

  /**
   * @brief Copies the latest complete joint state (s0 s1 e0 e1 w0 w1 w2).
   * @param[out] positions Joint positions [rad].
   * @param[out] velocities Joint velocities [rad/s]; zeros when the message carried none.
   * @param timeout_sec Maximum age of the data (wall clock) [s].
   * @param[out] age_sec Age of the data when available [s]; may be nullptr.
   * @return False when nothing complete was received yet or the data is older than `timeout_sec`.
   */
  bool latest(JointVector& positions, JointVector& velocities, double timeout_sec,
              double* age_sec = nullptr) const;

  /** @brief Arm this listener follows. */
  const std::string& side() const { return side_; }

 private:
  void onJointStates(const sensor_msgs::JointState::ConstPtr& message);

  std::string side_;
  std::array<std::string, kArmJoints> joint_names_;
  mutable std::mutex mutex_;
  JointVector positions_{};   ///< Latest complete positions [rad] (guarded by mutex_).
  JointVector velocities_{};  ///< Latest velocities [rad/s] (guarded by mutex_).
  bool has_complete_state_{false};
  ros::WallTime received_at_;
  ros::Subscriber subscriber_;
};

}  // namespace mpc_geometry
