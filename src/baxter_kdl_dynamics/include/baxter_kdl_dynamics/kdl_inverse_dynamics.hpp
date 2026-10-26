#pragma once

#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <ros/ros.h>
#include <urdf/model.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace baxter_kdl_dynamics {

struct DynamicsResult {
  std::unordered_map<std::string, double> torques;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> reference_position;
  std::vector<double> reference_velocity;
  std::vector<double> mpc_acceleration;
  std::vector<double> pd_acceleration;
  std::vector<double> commanded_acceleration;
  std::vector<double> inertia_torque;
  std::vector<double> coriolis_torque;
  std::vector<double> gravity_torque;
  std::vector<double> damping_torque;
  std::vector<double> raw_torque;
  std::vector<double> commanded_torque;
  std::vector<double> torque_limit;
};

class KdlInverseDynamics {
 public:
  KdlInverseDynamics(ros::NodeHandle& nh, const std::string& limb);

  DynamicsResult compute(
      const std::unordered_map<std::string, double>& positions,
      const std::unordered_map<std::string, double>& velocities,
      const std::unordered_map<std::string, double>& accelerations,
      const std::unordered_map<std::string, double>& reference_positions,
      const std::unordered_map<std::string, double>& reference_velocities,
      double reference_age_sec = 0.0);

  const std::vector<std::string>& jointNames() const { return joint_names_; }

 private:
  static std::vector<double> loadGainVector(ros::NodeHandle& nh, const std::string& name,
                                            std::size_t size);
  static double requiredValue(const std::unordered_map<std::string, double>& values,
                              const std::string& joint, const std::string& source);

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dynamics_;
  std::vector<std::string> joint_names_;
  std::vector<double> effort_limits_;
  std::vector<double> damping_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  bool add_gravity_compensation_{true};
  bool compensate_urdf_damping_{true};
  // The MPC sends (q0, v0, a) sampled at its solve time; the reference the PD
  // loop tracks is q0 + v0*t + a*t^2/2 with t = age of the command, clamped.
  bool extrapolate_reference_{true};
  double max_reference_extrapolation_sec_{0.1};
};

}  // namespace baxter_kdl_dynamics
