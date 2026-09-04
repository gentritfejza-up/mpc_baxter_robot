#pragma once

#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <ros/ros.h>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

class Gripper {
 public:
  Gripper(const std::string& gripper, ros::NodeHandle& nh, bool versioned = false);
  bool open(bool block = false, float timeout = 5.0);
  bool close(bool block = false, float timeout = 5.0);
  bool calibrate(bool block = true, float timeout = 5.0);
  bool calibrated() const;
  void set_parameters(const std::unordered_map<std::string, double>& params);

 private:
  void stateCallback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg);
  bool command(const std::string& cmd, const nlohmann::json& args = {}, bool block = false,
               float timeout = 5.0);
  uint32_t incrementSequence();

  std::string name_;
  ros::Publisher cmd_pub_;
  ros::Subscriber state_sub_;
  baxter_core_msgs::EndEffectorState state_;
  std::unordered_map<std::string, double> parameters_;
  uint32_t cmd_sequence_;
  std::string cmd_sender_;
  mutable std::mutex state_mutex_;
  bool versioned_;
};