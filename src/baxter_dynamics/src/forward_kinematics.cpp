#include "baxter_dynamics/forward_kinematics.hpp"

#include <cmath>
#include <memory>

namespace baxter {

ForwardKinematics::ForwardKinematics(const std::string &limb,
                                     ros::NodeHandle &nh,
                                     std::shared_ptr<Robot> baxter)
    : robot_(baxter) {}

ForwardKinematics::ForwardKinematics(const std::string &limb,
                                     ros::NodeHandle &nh)
    : ForwardKinematics(limb, nh, std::make_shared<Robot>(limb, nh)) {}

bool ForwardKinematics::checkJointLimits(
    const std::unordered_map<std::string, double> &joint_angles) const {
  bool all_ok = true;
  for (const auto &[joint, angle] : joint_angles) {
    const auto &limits =
        robot_->getAngleLimits().at(joint.substr(joint.size() - 2));
    if (angle < limits.first || angle > limits.second) {
      ROS_WARN_STREAM_THROTTLE(1, joint << " joint limits exceeded! ("
                                        << angle << " not in ["
                                        << limits.first << ", "
                                        << limits.second << "])");
      all_ok = false;
    }
  }
  return all_ok;
}

std::vector<Eigen::Matrix4d> ForwardKinematics::solveIntermediateFK(
    const std::unordered_map<std::string, double> &joint_angles) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Matrix4d> intermediateT;
  intermediateT.reserve(robot_->getJointNames().size() + 1);
  intermediateT.push_back(T);

  checkJointLimits(joint_angles);

  const auto &joint_names = robot_->getJointNames();
  const auto &dh_params = robot_->getDhParams();

  for (const auto &joint : joint_names) {
    const auto &params = dh_params.at(joint);
    double a = params[2];
    double theta = params[1] + joint_angles.at(joint);
    double d = params[0];
    double alpha = params[3];
    T = T * DH2T(d, theta, a, alpha);
    intermediateT.push_back(T);
  }

  return intermediateT;
}

Eigen::Matrix4d ForwardKinematics::DH2T(const double &d, const double &theta,
                                        const double &a, const double &alpha) {
  Eigen::Matrix4d T;
  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
      a * cos(theta), sin(theta), cos(theta) * cos(alpha),
      -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d,
      0, 0, 0, 1;
  return T;
}

} // namespace baxter