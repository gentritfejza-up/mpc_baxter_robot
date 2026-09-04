#include "baxter_dynamics/lagrange_euler_node.hpp"

#include <memory>
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
                                            joint_accelerations_);
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
