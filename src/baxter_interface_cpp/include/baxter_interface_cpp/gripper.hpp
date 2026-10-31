/**
 * @file gripper.hpp
 * @brief C++ wrapper around the Baxter SDK end-effector topics of one electric
 *        gripper: calibrate, open, close and configure, mirroring the Python
 *        `baxter_interface.Gripper`.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <ros/ros.h>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

/**
 * @brief One Baxter electric gripper ("left" or "right").
 *
 * Commands are JSON-encoded `EndEffectorCommand` messages; the gripper echoes
 * the sender and sequence of the last accepted command in its state, which is
 * how blocking calls detect acknowledgement. Blocking calls spin the global
 * callback queue themselves (`ros::spinOnce`), so they work without a spinner.
 */
class Gripper {
 public:
  /**
   * @brief Subscribe to the gripper state, wait for a valid state and send the
   *        default configuration (velocity 50, moving force 40, holding force 30,
   *        dead zone 5, all in SDK percent units).
   * @param gripper "left" or "right".
   * @param nh Node handle used for the publisher and subscriber.
   * @param versioned Accepted for parity with the Python SDK constructor; unused.
   */
  Gripper(const std::string& gripper, ros::NodeHandle& nh, bool versioned = false);

  /**
   * @brief Command the fingers fully open (position 100 %).
   * @param block When true, wait for the acknowledgement and then for the
   *        fingers to reach the target (within the dead zone) or to grip.
   * @param timeout Maximum wait per phase [s].
   * @return True when the command was sent (and, if blocking, completed in time).
   */
  bool open(bool block = false, float timeout = 5.0);

  /**
   * @brief Command the fingers fully closed (position 0 %).
   * @param block When true, wait for the acknowledgement and then for the
   *        fingers to reach the target (within the dead zone) or to grip.
   * @param timeout Maximum wait per phase [s].
   * @return True when the command was sent (and, if blocking, completed in time).
   */
  bool close(bool block = false, float timeout = 5.0);

  /**
   * @brief Run the gripper's calibration routine.
   * @param block When true, wait for the acknowledgement and then for the state
   *        to report calibrated and ready.
   * @param timeout Maximum wait per phase [s].
   * @return True when the command was sent (and, if blocking, completed in time).
   */
  bool calibrate(bool block = true, float timeout = 5.0);

  /// @return True when the latest state reports the gripper calibrated.
  bool calibrated() const;

  /**
   * @brief Update gripper parameters and send the full configuration.
   * @param params Subset of {velocity, moving_force, holding_force, dead_zone} in
   *        SDK percent units; values are clamped to [0, 100], unknown keys are
   *        logged and ignored.
   */
  void set_parameters(const std::unordered_map<std::string, double>& params);

 private:
  void onEndEffectorState(const baxter_core_msgs::EndEffectorState::ConstPtr& message);

  /**
   * @brief Publish one end-effector command.
   * @param command_name One of `EndEffectorCommand::CMD_*`.
   * @param arguments JSON arguments of the command.
   * @param block When true, republish until the gripper echoes the command's
   *        sender and sequence or `timeout` elapses.
   * @param timeout Maximum wait for the acknowledgement [s].
   * @return True when not blocking, or when the command was acknowledged in time.
   */
  bool command(const std::string& command_name, const nlohmann::json& arguments = {},
               bool block = false, float timeout = 5.0);

  /// @return The next command sequence number, wrapping within the positive int32 range.
  uint32_t nextCommandSequence();

  /// @return Copy of the latest end-effector state.
  baxter_core_msgs::EndEffectorState currentState() const;

  std::string name_;  ///< "<side>_gripper", as used in the topic names.
  ros::Publisher command_publisher_;
  ros::Subscriber state_subscriber_;
  mutable std::mutex state_mutex_;
  baxter_core_msgs::EndEffectorState state_;  ///< Guarded by state_mutex_.
  std::unordered_map<std::string, double> parameters_;  ///< SDK percent units.
  uint32_t command_sequence_;
  std::string command_sender_;  ///< Sender prefix echoed back in the state.
};
