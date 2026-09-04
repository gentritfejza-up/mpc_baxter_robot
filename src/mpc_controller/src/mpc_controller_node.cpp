#include "mpc_controller/mpc_controller_node.hpp"

#include <algorithm>
#include <casadi/core/dm_fwd.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>

MPCControlManager::MPCControlManager(ros::NodeHandle& nh, const std::string& limb_side)
    : nh_(nh),
      limb_side_(limb_side),
      limb_(std::make_unique<Limb>(limb_side)),
      mpc_(nh_, kNumJoints, kPredictionHorizon, kControlHorizon, kSamplingTime) {
  nh_.param("robot/safety/ee_z_constraint_enabled", ee_z_constraint_enabled_, false);
  nh_.param("robot/safety/ee_z_min_m", ee_z_min_m_, -0.20);
  nh_.param("robot/safety/ee_base_link", ee_base_link_, std::string("base"));

  const std::string default_end_link = (limb_side_ == "left") ? "left_gripper" : "right_gripper";
  nh_.param("robot/safety/ee_end_link", ee_end_link_, default_end_link);

  if (ee_z_constraint_enabled_ && !initializeZConstraintKinematics()) {
    ee_z_constraint_enabled_ = false;
    ROS_WARN("Disabling MPC EE-z constraint due to kinematics initialization failure.");
  }
}

void MPCControlManager::processControl(const std::vector<double>& x_ref) {
  auto joint_angles = getJointAngles();
  auto joint_velocities = getJointVelocities();

  if (joint_angles.empty() || joint_velocities.empty()) {
    ROS_WARN_THROTTLE(1, "Joint angles and velocities are not yet available!");
    return;
  }

  // Prepare state vectors
  std::vector<double> x0_vals;
  std::vector<double> v0_vals;
  x0_vals.reserve(kNumJoints);
  v0_vals.reserve(kNumJoints);

  // Fill current joint positions and velocities
  for (const auto& joint_name : limb_->jointNames()) {
    const auto angle_it = joint_angles.find(joint_name);
    const auto velocity_it = joint_velocities.find(joint_name);
    if (angle_it == joint_angles.end() || velocity_it == joint_velocities.end()) {
      ROS_WARN_THROTTLE(1.0, "Missing joint state for %s", joint_name.c_str());
      return;
    }
    x0_vals.push_back(angle_it->second);
    v0_vals.push_back(velocity_it->second);
  }

  // Convert to CasADi DM
  casadi::DM x0 = casadi::DM(x0_vals);
  casadi::DM v0 = casadi::DM(v0_vals);

  casadi::DM x_ref_val = casadi::DM(x_ref);

  updateLinearizedZConstraint(x0_vals);

  casadi::DM optimal_a;
  try {
    optimal_a = mpc_.solve(x0, v0, x_ref_val, casadi::DM(previous_applied_accelerations_));
  } catch (const std::exception& ex) {
    ROS_ERROR_THROTTLE(1.0, "MPC solve failed; suppressing command: %s", ex.what());
    return;
  }

  // Extract the first step's accelerations
  int num_state_vars = kPredictionHorizon * 2 * kNumJoints;
  casadi::DM first_step_a = optimal_a(casadi::Slice(num_state_vars, num_state_vars + kNumJoints));
  auto angles = mpc_.getReferentAngles();
  auto velocities = mpc_.getReferentVelocities();

  auto referent_accelerations = mpc_.dmToMap(first_step_a, limb_->jointNames());
  auto referent_velocities = mpc_.dmToMap(velocities, limb_->jointNames());
  auto referent_angles = mpc_.dmToMap(angles, limb_->jointNames());
  auto ik_referent_angles = mpc_.dmToMap(x_ref_val, limb_->jointNames());
  limb_->setReferentAngles(ik_referent_angles);

  // // Command the limb
  limb_->setJointAccelerations(referent_accelerations, referent_velocities, referent_angles);
  previous_applied_accelerations_.clear();
  for (const auto& joint_name : limb_->jointNames()) {
    previous_applied_accelerations_.push_back(referent_accelerations.at(joint_name));
  }
}

void MPCControlManager::run() {
  ros::AsyncSpinner spinner(kNumThreads);
  spinner.start();
  ros::Rate rate(kRosLoopRate);

  while (ros::ok()) {
    auto t_start = std::chrono::steady_clock::now();
    processControl(mpc_.getReferencePositions());
    auto t_end = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    ROS_INFO_THROTTLE(2, "MPC cycle: %.2f ms", ms);
    rate.sleep();
  }
}
std::unordered_map<std::string, double> MPCControlManager::getJointAngles() const {
  return limb_->jointAngles();
}

std::unordered_map<std::string, double> MPCControlManager::getJointVelocities() const {
  return limb_->jointVelocities();
}

bool MPCControlManager::initializeZConstraintKinematics() {
  try {
    kinematics_ = std::make_unique<BaxterKinematicsInterface>(ee_base_link_, ee_end_link_);
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("Failed to construct kinematics for EE-z constraint: " << ex.what());
    return false;
  }

  const auto& mpc_joint_names = limb_->jointNames();
  const auto& kdl_joint_names = kinematics_->getJointNames();

  if (mpc_joint_names.size() != kdl_joint_names.size()) {
    ROS_ERROR_STREAM("Joint count mismatch between MPC limb (" << mpc_joint_names.size()
                                                                 << ") and KDL chain ("
                                                                 << kdl_joint_names.size() << ").");
    return false;
  }

  mpc_to_kdl_index_.assign(mpc_joint_names.size(), -1);
  for (size_t i = 0; i < mpc_joint_names.size(); ++i) {
    const auto it = std::find(kdl_joint_names.begin(), kdl_joint_names.end(), mpc_joint_names[i]);
    if (it == kdl_joint_names.end()) {
      ROS_ERROR_STREAM("Joint " << mpc_joint_names[i]
                                 << " not found in KDL chain for EE-z constraint.");
      return false;
    }
    mpc_to_kdl_index_[i] = static_cast<int>(std::distance(kdl_joint_names.begin(), it));
  }

  ROS_INFO_STREAM("MPC EE-z constraint enabled: z >= " << ee_z_min_m_ << " m using links "
                                                        << ee_base_link_ << " -> " << ee_end_link_);
  return true;
}

void MPCControlManager::updateLinearizedZConstraint(const std::vector<double>& x0_vals) {
  if (!ee_z_constraint_enabled_ || !kinematics_) {
    mpc_.setEndEffectorZLinearConstraint(std::vector<double>(kNumJoints, 0.0), 0.0, false);
    return;
  }

  if (x0_vals.size() != static_cast<size_t>(kNumJoints)) {
    ROS_WARN_THROTTLE(1.0, "State vector size mismatch for EE-z constraint computation.");
    mpc_.setEndEffectorZLinearConstraint(std::vector<double>(kNumJoints, 0.0), 0.0, false);
    return;
  }

  std::vector<double> q_kdl(kNumJoints, 0.0);
  for (size_t i = 0; i < x0_vals.size(); ++i) {
    int kdl_idx = mpc_to_kdl_index_[i];
    if (kdl_idx < 0 || kdl_idx >= kNumJoints) {
      mpc_.setEndEffectorZLinearConstraint(std::vector<double>(kNumJoints, 0.0), 0.0, false);
      ROS_WARN_THROTTLE(1.0, "Invalid MPC->KDL index mapping. Disabling EE-z constraint this cycle.");
      return;
    }
    q_kdl[kdl_idx] = x0_vals[i];
  }

  try {
    KDL::Frame ee_frame = kinematics_->forward(q_kdl);
    KDL::Jacobian jac = kinematics_->jacobian(q_kdl);
    if (jac.columns() != static_cast<unsigned int>(kNumJoints)) {
      mpc_.setEndEffectorZLinearConstraint(std::vector<double>(kNumJoints, 0.0), 0.0, false);
      ROS_WARN_THROTTLE(1.0, "Jacobian dimension mismatch for EE-z constraint.");
      return;
    }

    const double z0 = ee_frame.p.z();
    std::vector<double> jz_mpc(kNumJoints, 0.0);
    for (size_t i = 0; i < jz_mpc.size(); ++i) {
      int kdl_idx = mpc_to_kdl_index_[i];
      jz_mpc[i] = jac(2, kdl_idx);
    }

    double z_offset = ee_z_min_m_ - z0;
    for (size_t i = 0; i < jz_mpc.size(); ++i) {
      z_offset += jz_mpc[i] * x0_vals[i];
    }

    mpc_.setEndEffectorZLinearConstraint(jz_mpc, z_offset, true);
  } catch (const std::exception& ex) {
    ROS_WARN_THROTTLE(1.0, "Exception in EE-z constraint update: %s", ex.what());
    mpc_.setEndEffectorZLinearConstraint(std::vector<double>(kNumJoints, 0.0), 0.0, false);
  }
}
