#include "baxter_dynamics/lagrange_euler_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

LagrangeEulerNode::LagrangeEulerNode(ros::NodeHandle &nh,
                                     const std::string &limb_side)
    : lagrange_euler_(std::make_unique<baxter::LagrangeEuler>(limb_side, nh)),
      limb_(std::make_unique<Limb>(limb_side)) {
  nh.param("robot/control/spinner_threads", spinner_threads_, 5);
  nh.param("robot/control/lagrange_euler_rate_hz", loop_rate_hz_, 800);
  nh.param("robot/control/acceleration_ref_timeout_sec",
           acceleration_ref_timeout_sec_, 0.2);
  limb_->setCommandTimeout(acceleration_ref_timeout_sec_);

  const auto joint_count = limb_->jointNames().size();
  kp_.assign(joint_count, 0.0);
  kd_.assign(joint_count, 0.0);
  nh.param("robot/lagrange_euler/kp", kp_, kp_);
  nh.param("robot/lagrange_euler/kd", kd_, kd_);
  nh.param("robot/lagrange_euler/extrapolate_reference", extrapolate_reference_,
           extrapolate_reference_);
  nh.param("robot/lagrange_euler/max_reference_extrapolation_sec",
           max_reference_extrapolation_sec_, max_reference_extrapolation_sec_);
  const auto valid_gains = [joint_count](const std::vector<double>& gains) {
    return gains.size() == joint_count &&
           std::all_of(gains.begin(), gains.end(),
                       [](double value) { return std::isfinite(value) && value >= 0.0; });
  };
  if (!valid_gains(kp_) || !valid_gains(kd_) ||
      !std::isfinite(max_reference_extrapolation_sec_) ||
      max_reference_extrapolation_sec_ < 0.0) {
    throw std::invalid_argument("robot/lagrange_euler/{kp,kd} must hold one finite nonnegative value per joint");
  }
}

std::unordered_map<std::string, double> LagrangeEulerNode::commandedAccelerations() const {
  // qdd = a_mpc + kp*(q_ref - q) + kd*(v_ref - v), where the reference is the MPC
  // sample (q0, v0) propagated with a_mpc over the age of the command.
  const auto names = limb_->jointNames();
  const auto q0 = limb_->jointAnglesRef();
  const auto v0 = limb_->jointVelocitiesRef();
  const double age = limb_->accelerationRefAgeSec();
  const double t = extrapolate_reference_ && std::isfinite(age)
                       ? std::clamp(age, 0.0, max_reference_extrapolation_sec_)
                       : 0.0;
  std::unordered_map<std::string, double> result;
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string& joint = names[i];
    const double a = joint_accelerations_.at(joint);
    double qdd = a;
    if (kp_[i] > 0.0 || kd_[i] > 0.0) {
      const double q_ref = q0.at(joint) + v0.at(joint) * t + 0.5 * a * t * t;
      const double v_ref = v0.at(joint) + a * t;
      qdd += kp_[i] * (q_ref - joint_angles_.at(joint)) +
             kd_[i] * (v_ref - joint_velocities_.at(joint));
    }
    result.emplace(joint, qdd);
  }
  return result;
}

void LagrangeEulerNode::exitTorqueControl() {
  if (torque_control_active_) {
    limb_->exitControlMode(acceleration_ref_timeout_sec_);
    torque_control_active_ = false;
  }
}

void LagrangeEulerNode::processComputedTorque() {
  joint_angles_ = limb_->jointAngles();
  joint_velocities_ = limb_->jointVelocities();
  joint_accelerations_ = limb_->jointAccelerationsRef();

  if (joint_angles_.empty() || joint_velocities_.empty() ||
      joint_accelerations_.empty()) {
    ROS_WARN_THROTTLE(
        1,
        "Joint states or MPC acceleration references are not yet available.");
    exitTorqueControl();
    return;
  }

  if (limb_->accelerationRefAgeSec() > acceleration_ref_timeout_sec_) {
    ROS_WARN_THROTTLE(1, "Acceleration references are stale; exiting torque control.");
    exitTorqueControl();
    return;
  }

  try {
    auto tau = lagrange_euler_->Torque_calc(joint_angles_, joint_velocities_,
                                            commandedAccelerations());
    limb_->setJointTorques(tau);
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
    processComputedTorque();
    rate.sleep();
  }
  exitTorqueControl();
}
