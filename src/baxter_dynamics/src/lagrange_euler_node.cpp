#include "baxter_dynamics/lagrange_euler_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>

namespace baxter {

LagrangeEulerNode::LagrangeEulerNode(ros::NodeHandle &nh,
                                     const std::string &limb_side)
    : lagrange_euler_(std::make_unique<LagrangeEuler>(limb_side, nh)),
      limb_(std::make_unique<Limb>(limb_side)) {
  nh.param("robot/control/spinner_threads", spinner_threads_, 5);
  nh.param("robot/control/lagrange_euler_rate_hz", loop_rate_hz_, 800);
  nh.param("robot/control/acceleration_ref_timeout_sec",
           acceleration_ref_timeout_sec_, 0.2);
  limb_->setCommandTimeout(acceleration_ref_timeout_sec_);

  const std::size_t joint_count = limb_->jointNames().size();
  kp_.assign(joint_count, 0.0);
  kd_.assign(joint_count, 0.0);
  nh.param("robot/lagrange_euler/kp", kp_, kp_);
  nh.param("robot/lagrange_euler/kd", kd_, kd_);
  nh.param("robot/lagrange_euler/extrapolate_reference",
           extrapolate_reference_, extrapolate_reference_);
  nh.param("robot/lagrange_euler/max_reference_extrapolation_sec",
           max_reference_extrapolation_sec_, max_reference_extrapolation_sec_);

  const auto gains_are_valid = [joint_count](const std::vector<double> &gains) {
    return gains.size() == joint_count &&
           std::all_of(gains.begin(), gains.end(), [](double gain) {
             return std::isfinite(gain) && gain >= 0.0;
           });
  };
  if (!gains_are_valid(kp_) || !gains_are_valid(kd_)) {
    throw std::invalid_argument(
        "robot/lagrange_euler/{kp,kd} must hold one finite nonnegative value "
        "per joint");
  }
  if (!std::isfinite(max_reference_extrapolation_sec_) ||
      max_reference_extrapolation_sec_ < 0.0) {
    throw std::invalid_argument(
        "robot/lagrange_euler/max_reference_extrapolation_sec must be finite "
        "and nonnegative");
  }
}

std::unordered_map<std::string, double>
LagrangeEulerNode::commandedAccelerations() const {
  const std::vector<std::string> joint_names = limb_->jointNames();
  const auto reference_angles = limb_->jointAnglesRef();
  const auto reference_velocities = limb_->jointVelocitiesRef();

  const double reference_age_sec = limb_->accelerationRefAgeSec();
  const double extrapolation_sec =
      extrapolate_reference_ && std::isfinite(reference_age_sec)
          ? std::clamp(reference_age_sec, 0.0, max_reference_extrapolation_sec_)
          : 0.0;

  std::unordered_map<std::string, double> commanded;
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const std::string &joint = joint_names[i];
    const double mpc_acceleration = joint_accelerations_.at(joint);
    double qdd = mpc_acceleration;
    if (kp_[i] > 0.0 || kd_[i] > 0.0) {
      const double q_ref = reference_angles.at(joint) +
                           reference_velocities.at(joint) * extrapolation_sec +
                           0.5 * mpc_acceleration * extrapolation_sec *
                               extrapolation_sec;
      const double qd_ref = reference_velocities.at(joint) +
                            mpc_acceleration * extrapolation_sec;
      qdd += kp_[i] * (q_ref - joint_angles_.at(joint)) +
             kd_[i] * (qd_ref - joint_velocities_.at(joint));
    }
    commanded.emplace(joint, qdd);
  }
  return commanded;
}

void LagrangeEulerNode::exitTorqueControl() {
  if (torque_control_active_) {
    limb_->exitControlMode(acceleration_ref_timeout_sec_);
    torque_control_active_ = false;
  }
}

void LagrangeEulerNode::runControlStep() {
  joint_angles_ = limb_->jointAngles();
  joint_velocities_ = limb_->jointVelocities();
  joint_accelerations_ = limb_->jointAccelerationsRef();

  if (joint_angles_.empty() || joint_velocities_.empty() ||
      joint_accelerations_.empty()) {
    ROS_WARN_THROTTLE(
        1, "Joint states or MPC acceleration references are not yet available.");
    exitTorqueControl();
    return;
  }

  if (limb_->accelerationRefAgeSec() > acceleration_ref_timeout_sec_) {
    ROS_WARN_THROTTLE(
        1, "Acceleration references are stale; exiting torque control.");
    exitTorqueControl();
    return;
  }

  try {
    const auto torques = lagrange_euler_->computeTorques(
        joint_angles_, joint_velocities_, commandedAccelerations());
    limb_->setJointTorques(torques);
    torque_control_active_ = true;
  } catch (const std::exception &exception) {
    ROS_ERROR_THROTTLE(1, "Torque calculation failed: %s", exception.what());
    exitTorqueControl();
  }
}

void LagrangeEulerNode::runNode() {
  ros::AsyncSpinner spinner(spinner_threads_);
  spinner.start();

  ros::Rate rate(loop_rate_hz_);
  while (ros::ok()) {
    runControlStep();
    rate.sleep();
  }
  exitTorqueControl();
}

} // namespace baxter
