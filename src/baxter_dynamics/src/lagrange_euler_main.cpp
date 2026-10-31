/**
 * @file lagrange_euler_main.cpp
 * @brief `lagrange_euler_node`: the computed-torque loop with the Lagrange-Euler
 *        model loaded from `robot/arm_model` (config/dynamics_params.yaml).
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * Usage: run_lagrange_euler <left|right>. Settings namespace `robot/lagrange_euler`
 * (same keys as `robot/kdl_dynamics`), rates `robot/control/lagrange_euler_rate_hz`
 * and `robot/control/lagrange_euler_diagnostics_rate_hz`. Per-joint
 * `robot/lagrange_euler/torque_limit_overrides/<key>` [Nm] replaces the effort
 * limit of the model (the simulator raises s1 to 100 Nm).
 */
#include "baxter_dynamics/arm_description.hpp"
#include "baxter_dynamics/lagrange_euler.hpp"
#include "baxter_dynamics/torque_loop.hpp"

#include <Eigen/Core>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace {

std::unique_ptr<baxter::InverseDynamicsModel> makeLagrangeEulerModel(ros::NodeHandle& nh,
                                                                     const std::string& limb) {
  baxter::ArmDescription arm = baxter::loadArmDescription(nh, "robot/arm_model", limb);
  for (baxter::JointDescription& joint : arm.joints) {
    const std::string key = joint.name.compare(0, limb.size() + 1, limb + "_") == 0
                                ? joint.name.substr(limb.size() + 1)
                                : joint.name;
    double override_nm = -1.0;
    nh.param("robot/lagrange_euler/torque_limit_overrides/" + key, override_nm, override_nm);
    if (override_nm > 0.0) {
      ROS_INFO_STREAM(joint.name << " effort limit " << joint.effort_limit << " -> "
                                 << override_nm << " Nm (override)");
      joint.effort_limit = override_nm;
    }
  }
  return std::make_unique<baxter::LagrangeEuler>(std::move(arm));
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "lagrange_euler_node");
  Eigen::initParallel();
  const baxter::TorqueLoopParameterKeys keys{"robot/lagrange_euler",
                                             "robot/control/lagrange_euler_rate_hz",
                                             "robot/control/lagrange_euler_diagnostics_rate_hz"};
  return baxter::runTorqueLoopNode(argc, argv, "lagrange_euler_node", keys,
                                   makeLagrangeEulerModel);
}
