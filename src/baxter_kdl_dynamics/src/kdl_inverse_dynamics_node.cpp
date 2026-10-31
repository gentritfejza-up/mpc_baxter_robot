/**
 * @file kdl_inverse_dynamics_node.cpp
 * @brief `kdl_inverse_dynamics_node`: the shared computed-torque loop of
 *        `baxter_dynamics` with the KDL model built from `/robot_description`.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * Usage: run_kdl_inverse_dynamics <left|right>. Settings namespace
 * `robot/kdl_dynamics`, rates `robot/control/kdl_dynamics_rate_hz` and
 * `robot/control/kdl_diagnostics_rate_hz` (config/kdl_dynamics.yaml).
 */
#include "baxter_kdl_dynamics/kdl_inverse_dynamics.hpp"

#include <baxter_dynamics/torque_loop.hpp>
#include <ros/ros.h>

#include <memory>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kdl_inverse_dynamics_node");
  const baxter::TorqueLoopParameterKeys keys{"robot/kdl_dynamics",
                                             "robot/control/kdl_dynamics_rate_hz",
                                             "robot/control/kdl_diagnostics_rate_hz"};
  return baxter::runTorqueLoopNode(
      argc, argv, "kdl_inverse_dynamics_node", keys,
      [](ros::NodeHandle& nh, const std::string& limb) -> std::unique_ptr<baxter::InverseDynamicsModel> {
        return std::make_unique<baxter_kdl_dynamics::KdlInverseDynamics>(nh, limb);
      });
}
