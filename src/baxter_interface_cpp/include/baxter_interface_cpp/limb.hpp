/**
 * @file limb.hpp
 * @brief C++ wrapper around the Baxter SDK joint topics of one arm: thread-safe
 *        access to the measured state and the MPC reference, and joint command
 *        publishers for position, velocity and torque mode.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */
#pragma once

#include <baxter_core_msgs/AccelerationCommand.h>
#include <baxter_core_msgs/AccelerationMax.h>
#include <baxter_core_msgs/AccelerationMin.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/ReferentJointAngles.h>
#include <baxter_core_msgs/SEAJointState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

/// @brief A 3D vector (position [m], linear velocity [m/s] or force [N]).
struct Point {
  double x, y, z;
};

/// @brief An orientation quaternion (x, y, z, w).
struct Quaternion {
  double x, y, z, w;
};

/// @brief Measured joint state captured under one lock, so that positions and
///        velocities come from the same `/robot/joint_states` message.
struct JointStateSnapshot {
  std::unordered_map<std::string, double> positions;   ///< Joint positions by name [rad].
  std::unordered_map<std::string, double> velocities;  ///< Joint velocities by name [rad/s].
  ros::WallTime received_at;  ///< Wall-clock receipt time; zero until the first message.
  ros::Time stamp;  ///< Message header stamp (ROS/sim clock); receipt time when the header is unstamped.
};

/// @brief MPC acceleration reference captured under one lock, so that the
///        accelerations, base velocities and base angles come from the same
///        `acceleration_command` message.
struct AccelerationReferenceSnapshot {
  std::unordered_map<std::string, double> accelerations;  ///< Feedforward accelerations by name [rad/s^2].
  std::unordered_map<std::string, double> velocities;     ///< Base velocities v0 by name [rad/s].
  std::unordered_map<std::string, double> angles;         ///< Base angles q0 by name [rad].
  /// Seconds since the last reference was received on the ROS clock [s];
  /// `std::numeric_limits<double>::max()` when none has been received.
  double age_sec;
};

/**
 * @brief One Baxter arm ("left" or "right") seen through the SDK topics.
 *
 * Subscribers run on the node's spinner threads and write into the state maps
 * under `state_mutex_`; every getter copies the requested map under the same
 * lock, so callers in a control loop always see a consistent message. Setters
 * publish immediately and do not touch the shared state.
 */
class Limb {
 public:
  /**
   * @brief Advertise and subscribe to every `/robot/limb/<limb_name>/...` topic.
   * @param limb_name "left" or "right".
   */
  explicit Limb(const std::string& limb_name);

  /// @return The seven arm joint names in SDK order (s0, s1, e0, e1, w0, w1, w2).
  std::vector<std::string> jointNames() const;

  /**
   * @brief Measured angle of one joint.
   * @param joint Joint name.
   * @return Angle [rad].
   * @throw std::out_of_range when no joint state has been received for `joint`.
   */
  double jointAngle(const std::string& joint) const;

  /// @return Measured joint angles by name [rad]; empty until the first joint state.
  std::unordered_map<std::string, double> jointAngles() const;

  /// @return Measured joint velocities by name [rad/s]; empty until the first joint state.
  std::unordered_map<std::string, double> jointVelocities() const;

  /// @return Angles, velocities and timing of the latest joint state under one lock.
  JointStateSnapshot jointStateSnapshot() const;

  /// @return Measured joint efforts by name [Nm].
  std::unordered_map<std::string, double> jointEfforts() const;

  /// @return Latest MPC feedforward accelerations by name [rad/s^2].
  std::unordered_map<std::string, double> jointAccelerationsRef() const;

  /// @return Latest MPC base velocities v0 by name [rad/s].
  std::unordered_map<std::string, double> jointVelocitiesRef() const;

  /// @return Latest MPC base angles q0 by name [rad].
  std::unordered_map<std::string, double> jointAnglesRef() const;

  /**
   * @brief Age of the latest MPC acceleration reference.
   * @return Seconds since receipt on the ROS clock [s], or
   *         `std::numeric_limits<double>::max()` when none has been received.
   */
  double accelerationRefAgeSec() const;

  /// @return Accelerations, base velocities, base angles and age of the latest MPC
  ///         reference under one lock.
  AccelerationReferenceSnapshot accelerationReferenceSnapshot() const;

  /// @return Latest acceleration upper bounds by name [rad/s^2].
  std::unordered_map<std::string, double> jointAccelerationsMax() const;

  /// @return Latest acceleration lower bounds by name [rad/s^2].
  std::unordered_map<std::string, double> jointAccelerationsMin() const;

  /// @return Latest referent joint angles by name [rad].
  std::unordered_map<std::string, double> referentJointAngles() const;

  /// @return Commanded effort reported by the SEA joint state by name [Nm].
  std::unordered_map<std::string, double> commandedEffort() const;

  /// @return Actual effort reported by the SEA joint state by name [Nm].
  std::unordered_map<std::string, double> actualEffort() const;

  /// @return Gravity-model effort reported by the SEA joint state by name [Nm].
  std::unordered_map<std::string, double> gravityModelEffort() const;

  /**
   * @brief Publish one position-mode joint command.
   * @param positions Target angles by joint name [rad].
   */
  void setJointPositions(const std::unordered_map<std::string, double>& positions);

  /**
   * @brief Publish one velocity-mode joint command.
   * @param velocities Target velocities by joint name [rad/s].
   */
  void setJointVelocities(const std::unordered_map<std::string, double>& velocities);

  /**
   * @brief Publish an MPC acceleration reference for all seven joints.
   * @param accelerations Feedforward accelerations by joint name [rad/s^2].
   * @param velocities Base velocities v0 by joint name [rad/s].
   * @param angles Base angles q0 by joint name [rad].
   * @throw std::out_of_range when a map lacks one of the arm joints.
   */
  void setJointAccelerations(const std::unordered_map<std::string, double>& accelerations,
                             const std::unordered_map<std::string, double>& velocities,
                             const std::unordered_map<std::string, double>& angles);

  /**
   * @brief Publish referent joint angles for all seven joints.
   * @param referent_angles Angles by joint name [rad].
   * @throw std::out_of_range when the map lacks one of the arm joints.
   */
  void setReferentAngles(const std::unordered_map<std::string, double>& referent_angles);

  /**
   * @brief Publish one torque-mode joint command.
   * @param torques Target torques by joint name [Nm].
   */
  void setJointTorques(const std::unordered_map<std::string, double>& torques);

  /**
   * @brief Leave velocity or torque mode by commanding the measured angles in
   *        position mode.
   * @param timeout Joint-command timeout to set before the mode change [s].
   */
  void exitControlMode(double timeout = 0.2);

  /**
   * @brief Drive the arm to the SDK neutral pose in position mode.
   *
   * The position command is republished at 100 Hz (Baxter drops commands older
   * than the joint-command timeout) until every joint is within the SDK's
   * 0.0087 rad tolerance of the target or `timeout` elapses.
   * @param timeout Maximum time to spend [s].
   */
  void moveToNeutral(double timeout = 15.0);

  /// @return Endpoint position under the key "position" [m].
  std::unordered_map<std::string, Point> endpointPose() const;

  /// @return Endpoint orientation under the key "orientation".
  std::unordered_map<std::string, Quaternion> endpointOrientation() const;

  /// @return Endpoint twist under the keys "linear" [m/s] and "angular" [rad/s].
  std::unordered_map<std::string, Point> endpointVelocity() const;

  /// @return Endpoint wrench under the keys "force" [N] and "torque" [Nm].
  std::unordered_map<std::string, Point> endpointEffort() const;

  /**
   * @brief Set the time after which the robot discards the last joint command.
   * @param timeout Timeout [s].
   */
  void setCommandTimeout(double timeout);

  /**
   * @brief Set the position-mode speed ratio.
   * @param speed Fraction of the maximum joint speed, 0.0 to 1.0.
   */
  void setJointPositionSpeed(double speed);

  /// @param accelerations Acceleration upper bounds by joint name [rad/s^2].
  void setJointAccelerationsMax(const std::unordered_map<std::string, double>& accelerations);

  /// @param accelerations Acceleration lower bounds by joint name [rad/s^2].
  void setJointAccelerationsMin(const std::unordered_map<std::string, double>& accelerations);

 private:
  /// @return True when `joint` belongs to this arm.
  bool isLimbJoint(const std::string& joint) const;

  void onJointStates(const sensor_msgs::JointState::ConstPtr& message);
  void onEndpointState(const baxter_core_msgs::EndpointState::ConstPtr& message);
  void onAccelerationCommand(const baxter_core_msgs::AccelerationCommand::ConstPtr& message);
  void onAccelerationMax(const baxter_core_msgs::AccelerationMax::ConstPtr& message);
  void onAccelerationMin(const baxter_core_msgs::AccelerationMin::ConstPtr& message);
  void onReferentJointAngles(const baxter_core_msgs::ReferentJointAngles::ConstPtr& message);
  void onSeaJointState(const baxter_core_msgs::SEAJointState::ConstPtr& message);

  const std::string limb_name_;
  const std::vector<std::string> joint_names_;

  ros::NodeHandle nh_;
  ros::Publisher joint_command_publisher_;
  ros::Publisher acceleration_command_publisher_;
  ros::Publisher acceleration_max_publisher_;
  ros::Publisher acceleration_min_publisher_;
  ros::Publisher speed_ratio_publisher_;
  ros::Publisher command_timeout_publisher_;
  ros::Publisher referent_angles_publisher_;
  ros::Subscriber joint_state_subscriber_;
  ros::Subscriber endpoint_state_subscriber_;
  ros::Subscriber acceleration_command_subscriber_;
  ros::Subscriber acceleration_max_subscriber_;
  ros::Subscriber acceleration_min_subscriber_;
  ros::Subscriber sea_joint_state_subscriber_;
  ros::Subscriber referent_angles_subscriber_;

  // Shared state, guarded by state_mutex_.
  mutable std::mutex state_mutex_;
  std::unordered_map<std::string, double> joint_angles_;             ///< [rad]
  std::unordered_map<std::string, double> joint_velocities_;         ///< [rad/s]
  std::unordered_map<std::string, double> joint_efforts_;            ///< [Nm]
  std::unordered_map<std::string, double> reference_accelerations_;  ///< [rad/s^2]
  std::unordered_map<std::string, double> reference_velocities_;     ///< [rad/s]
  std::unordered_map<std::string, double> reference_angles_;         ///< [rad]
  std::unordered_map<std::string, double> max_accelerations_;        ///< [rad/s^2]
  std::unordered_map<std::string, double> min_accelerations_;        ///< [rad/s^2]
  std::unordered_map<std::string, double> commanded_efforts_;        ///< [Nm]
  std::unordered_map<std::string, double> actual_efforts_;           ///< [Nm]
  std::unordered_map<std::string, double> gravity_model_efforts_;    ///< [Nm]
  std::unordered_map<std::string, double> referent_angles_;          ///< [rad]
  std::unordered_map<std::string, Point> endpoint_position_;
  std::unordered_map<std::string, Quaternion> endpoint_orientation_;
  std::unordered_map<std::string, Point> endpoint_velocity_;
  std::unordered_map<std::string, Point> endpoint_effort_;
  ros::Time last_acceleration_reference_stamp_;  ///< Zero until the first reference.
  ros::WallTime last_joint_state_received_at_;
  ros::Time last_joint_state_stamp_;
};
