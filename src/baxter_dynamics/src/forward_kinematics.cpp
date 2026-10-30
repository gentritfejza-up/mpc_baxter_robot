#include "baxter_dynamics/forward_kinematics.hpp"

#include <cmath>
#include <memory>
#include <utility>

namespace baxter {

ForwardKinematics::ForwardKinematics(ros::NodeHandle & /*nh*/,
                                     std::shared_ptr<Robot> robot)
    : robot_(std::move(robot)) {}

ForwardKinematics::ForwardKinematics(const std::string &limb,
                                     ros::NodeHandle &nh)
    : ForwardKinematics(nh, std::make_shared<Robot>(limb, nh)) {}

bool ForwardKinematics::checkJointLimits(
    const std::unordered_map<std::string, double> &joint_angles) const {
  bool all_within_limits = true;
  const auto &angle_limits = robot_->getAngleLimits();
  for (const auto &joint : robot_->getJointNames()) {
    const auto angle_it = joint_angles.find(joint);
    if (angle_it == joint_angles.end()) {
      continue; // Reported by solveIntermediateFK when the angle is needed.
    }
    const double angle = angle_it->second;
    const auto &[lower, upper] = angle_limits.at(joint);
    if (angle < lower || angle > upper) {
      ROS_WARN_STREAM_THROTTLE(1, joint << " joint limits exceeded! (" << angle
                                        << " not in [" << lower << ", "
                                        << upper << "])");
      all_within_limits = false;
    }
  }
  return all_within_limits;
}

std::vector<Eigen::Matrix4d> ForwardKinematics::solveIntermediateFK(
    const std::unordered_map<std::string, double> &joint_angles) const {
  checkJointLimits(joint_angles);

  const auto &joint_names = robot_->getJointNames();
  const auto &dh_table = robot_->getDhParams();

  std::vector<Eigen::Matrix4d> link_transforms;
  link_transforms.reserve(joint_names.size() + 1);

  Eigen::Matrix4d base_to_link = Eigen::Matrix4d::Identity();
  link_transforms.push_back(base_to_link); // ^0T_0: base frame.

  for (const auto &joint : joint_names) {
    const DhParameters &dh = dh_table.at(joint);
    const double theta = dh.theta_offset + joint_angles.at(joint);
    base_to_link = base_to_link * dhTransform(dh.d, theta, dh.a, dh.alpha);
    link_transforms.push_back(base_to_link); // ^0T_i
  }

  return link_transforms;
}

Eigen::Matrix4d ForwardKinematics::dhTransform(double d, double theta,
                                               double a, double alpha) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  const double cos_alpha = std::cos(alpha);
  const double sin_alpha = std::sin(alpha);

  Eigen::Matrix4d transform;
  transform << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha,
      a * cos_theta,
      sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta,
      0.0, sin_alpha, cos_alpha, d,
      0.0, 0.0, 0.0, 1.0;
  return transform;
}

} // namespace baxter
