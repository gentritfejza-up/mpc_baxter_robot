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
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Struct representing a 3D point (x, y, z).
 */
struct Point {
  double x, y, z;
};

/**
 * @brief Struct representing a quaternion for orientation.
 */
struct Quaternion {
  double x, y, z, w;
};

/**
 * @brief Class to represent and control a limb (left or right) of the Baxter
 * robot.
 *
 * This class allows access to the limb's joint positions, velocities, efforts,
 * and accelerations. It also allows setting commands for joint control,
 * including position, velocity, acceleration, and torque.
 */
class Limb {
 public:
  /**
   * @brief Constructor for the Limb class.
   * @param limb_name The name of the limb ("left" or "right").
   */
  Limb(const std::string& limb_name);

  /**
   * @brief Get the list of joint names for the limb.
   * @return A vector of joint names.
   */
  std::vector<std::string> jointNames() const;

  /**
   * @brief Get the current angle of a specific joint.
   * @param joint The name of the joint.
   * @return The current angle (in radians) of the specified joint.
   */
  double jointAngle(const std::string& joint) const;

  /**
   * @brief Get the current angles for all joints in the limb.
   * @return A map from joint names to their current angles (in radians).
   */
  std::unordered_map<std::string, double> jointAngles() const;

  /**
   * @brief Get the current velocities for all joints in the limb.
   * @return A map from joint names to their current velocities (in radians per
   * second).
   */
  std::unordered_map<std::string, double> jointVelocities() const;

  /**
   * @brief Get the current efforts (torques) for all joints in the limb.
   * @return A map from joint names to their current efforts (in Newton-meters).
   */
  std::unordered_map<std::string, double> jointEfforts() const;

  /**
   * @brief Get the current accelerations for all joints in the limb.
   * @return A map from joint names to their current accelerations (in meters
   * per second squared).
   */
  std::unordered_map<std::string, double> jointAccelerationsRef() const;

  /**
   * @brief Seconds elapsed since the last acceleration reference was received.
   * @return Age in seconds, or a large value if never received.
   */
  double accelerationRefAgeSec() const;

  std::unordered_map<std::string, double> jointAccelerationsMax() const;

  std::unordered_map<std::string, double> jointAccelerationsMin() const;

  std::unordered_map<std::string, double> commandedEffort() const;
  std::unordered_map<std::string, double> actualEffort() const;
  std::unordered_map<std::string, double> gravityModelEffort() const;

  /**
   * @brief Set desired joint positions for the limb.
   * @param positions A map from joint names to desired joint positions (in
   * radians).
   */
  void setJointPositions(const std::unordered_map<std::string, double>& positions);

  /**
   * @brief Set desired joint velocities for the limb.
   * @param velocities A map from joint names to desired joint velocities (in
   * radians per second).
   */
  void setJointVelocities(const std::unordered_map<std::string, double>& velocities);

  /**
   * @brief Set desired joint accelerations for the limb.
   * @param accelerations A map from joint names to desired joint accelerations
   * (in meters per second squared).
   */
  void setJointAccelerations(const std::unordered_map<std::string, double>& accelerations,
                             const std::unordered_map<std::string, double>& velocities,
                             const std::unordered_map<std::string, double>& angles);

  void setReferentAngles(const std::unordered_map<std::string, double>& referent_angles);

  /**
   * @brief Set desired joint torques (efforts) for the limb.
   * @param torques A map from joint names to desired joint torques (in
   * Newton-meters).
   */
  void setJointTorques(const std::unordered_map<std::string, double>& torques);

  /**
   * @brief Move the limb to its neutral position.
   * @param timeout Time (in seconds) to allow the limb to reach the neutral
   * position.
   */
  void moveToNeutral(double timeout = 15.0);

  /**
   * @brief Get the current Cartesian pose (position) of the limb's endpoint.
   * @return A map from endpoint names to their current 3D positions.
   */
  std::unordered_map<std::string, Point> endpointPose() const;

  /**
   * @brief Get the current Cartesian velocity of the limb's endpoint.
   * @return A map from endpoint names to their current 3D velocities.
   */
  std::unordered_map<std::string, Point> endpointVelocity() const;

  /**
   * @brief Get the current Cartesian effort (force) of the limb's endpoint.
   * @return A map from endpoint names to their current 3D efforts (forces).
   */
  std::unordered_map<std::string, Point> endpointEffort() const;

  /**
   * @brief Set the timeout for joint commands.
   * @param timeout Time (in seconds) after which the joint command times out.
   */
  void setCommandTimeout(double timeout);

  /**
   * @brief Set the speed ratio for joint position movement.
   * @param speed A ratio of the maximum speed (0.0 to 1.0).
   */
  void setJointPositionSpeed(double speed);

  void setJointAccelerationsMax(const std::unordered_map<std::string, double>& accelerations);

  void setJointAccelerationsMin(const std::unordered_map<std::string, double>& accelerations);
  void onGravityCompenstation(const baxter_core_msgs::SEAJointState::ConstPtr& msg);
  std::unordered_map<std::string, double> jointVelocitiesRef() const;
  std::unordered_map<std::string, double> jointAnglesRef() const;
  std::unordered_map<std::string, double> referentJointAngles() const;

 private:
  // ROS communication variables
  ros::NodeHandle nh_;
  ros::Publisher pub_joint_cmd_;
  ros::Publisher pub_acceleration_cmd_;
  ros::Publisher pub_acceleration_max_cmd_;
  ros::Publisher pub_acceleration_min_cmd_;
  ros::Publisher pub_speed_ratio_;
  ros::Publisher pub_joint_cmd_timeout_;
  ros::Publisher pub_ref_angles_;
  ros::Subscriber sub_joint_state_;
  ros::Subscriber sub_endpoint_state_;
  ros::Subscriber sub_acceleration_cmd_;
  ros::Subscriber sub_acceleration_max_cmd_;
  ros::Subscriber sub_acceleration_min_cmd_;
  ros::Subscriber gravity_compensation_;
  ros::Subscriber sub_ref_angles_;

  // State storage
  std::unordered_map<std::string, double> joint_angle_;
  std::unordered_map<std::string, double> joint_velocity_;
  std::unordered_map<std::string, double> joint_acceleration_;
  std::unordered_map<std::string, double> joint_velocity_ref_;
  std::unordered_map<std::string, double> joint_angle_ref_;
  std::unordered_map<std::string, double> joint_acceleration_max_;
  std::unordered_map<std::string, double> joint_acceleration_min_;
  std::unordered_map<std::string, double> joint_effort_;
  std::unordered_map<std::string, double> commanded_effort_;
  std::unordered_map<std::string, double> actual_effort_;
  std::unordered_map<std::string, double> gravity_model_effort_;
  std::unordered_map<std::string, double> referent_angles_;

  std::unordered_map<std::string, Point> cartesian_pose_;
  std::unordered_map<std::string, Quaternion> cartesian_orientation_;
  std::unordered_map<std::string, Point> cartesian_velocity_;
  std::unordered_map<std::string, Point> cartesian_effort_;
  mutable std::mutex mutex_;
  ros::Time last_acceleration_ref_stamp_;

  std::unordered_map<std::string, std::vector<std::string>> joint_names_;
  std::string name;

  /**
   * @brief Callback for processing joint states.
   * @param msg The joint state message.
   */
  void onJointStates(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * @brief Callback for processing endpoint states.
   * @param msg The endpoint state message.
   */
  void onEndpointStates(const baxter_core_msgs::EndpointState::ConstPtr& msg);

  /**
   * @brief Callback for processing joint acceleration commands.
   * @param msg The acceleration command message.
   */
  void onJointAccelerations(const baxter_core_msgs::AccelerationCommand::ConstPtr& msg);

  void onJointAccelerationsMax(const baxter_core_msgs::AccelerationMax::ConstPtr& msg);

  void onJointAccelerationsMin(const baxter_core_msgs::AccelerationMin::ConstPtr& msg);
  void onJointReferentAngles(const baxter_core_msgs::ReferentJointAngles::ConstPtr& msg);
};
