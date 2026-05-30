#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <ros/ros.h>

#include "baxter_dynamics/lagrange_euler.hpp"
#include "baxter_interface_cpp/limb.hpp"

class LagrangeEulerNode {
public:
  LagrangeEulerNode(ros::NodeHandle &nh, const std::string &limb_side);
  void runNode();

private:
  void processComputedTorque();

  std::unique_ptr<baxter::LagrangeEuler> lagrange_euler_{nullptr};
  std::unique_ptr<Limb> limb_{nullptr};
  std::unordered_map<std::string, double> joint_angles_;
  std::unordered_map<std::string, double> joint_velocities_;
  std::unordered_map<std::string, double> joint_accelerations_;
  int spinner_threads_{5};
  int loop_rate_hz_{800};
};