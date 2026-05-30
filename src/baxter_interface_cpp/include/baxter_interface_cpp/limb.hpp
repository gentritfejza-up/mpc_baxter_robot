#pragma once

#include <baxter_core_msgs/AccelerationCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class Limb {
public:
  explicit Limb(const std::string &limb_name);

  std::vector<std::string> jointNames() const;
  std::unordered_map<std::string, double> jointAngles() const;
  std::unordered_map<std::string, double> jointVelocities() const;
  std::unordered_map<std::string, double> jointAccelerationsRef() const;
  void setJointTorques(const std::unordered_map<std::string, double> &torques);

private:
  void onJointStates(const sensor_msgs::JointState::ConstPtr &msg);
  void onJointAccelerations(
      const baxter_core_msgs::AccelerationCommand::ConstPtr &msg);
  bool ownsJoint(const std::string &joint_name) const;

  ros::NodeHandle nh_;
  ros::Publisher pub_joint_cmd_;
  ros::Subscriber sub_joint_state_;
  ros::Subscriber sub_acceleration_cmd_;

  std::unordered_map<std::string, double> joint_angle_;
  std::unordered_map<std::string, double> joint_velocity_;
  std::unordered_map<std::string, double> joint_acceleration_;
  std::unordered_map<std::string, std::vector<std::string>> joint_names_;
  std::string name_;
  mutable std::mutex mutex_;
};