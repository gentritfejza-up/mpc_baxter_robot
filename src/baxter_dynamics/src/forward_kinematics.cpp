#include "baxter_dynamics/forward_kinematics.hpp"

#include <cmath>
#include <stdexcept>

namespace baxter {

void ForwardKinematics::compute(const Eigen::VectorXd& positions, Transforms& transforms) const {
  const std::size_t joint_count = arm_.jointCount();
  if (static_cast<std::size_t>(positions.size()) != joint_count) {
    throw std::runtime_error("Forward kinematics: expected " + std::to_string(joint_count) +
                             " joint positions, got " + std::to_string(positions.size()));
  }
  transforms.resize(joint_count + 1);
  transforms[0] = Eigen::Matrix4d::Identity();  // ^0T_0: the base frame.

  for (std::size_t index = 0; index < joint_count; ++index) {
    const JointDescription& joint = arm_.joints[index];
    const double q = positions(static_cast<Eigen::Index>(index));
    const double d = joint.dh.d_offset + (joint.type == JointType::kPrismatic ? q : 0.0);
    const double theta = joint.dh.theta_offset + (joint.type == JointType::kRevolute ? q : 0.0);
    transforms[index + 1] =
        transforms[index] * dhTransform(d, theta, joint.dh.a, joint.dh.alpha);  // ^0T_{i+1}
  }
}

std::size_t ForwardKinematics::countLimitViolations(const Eigen::VectorXd& positions) const {
  std::size_t violations = 0;
  const auto count = std::min<std::size_t>(arm_.jointCount(), positions.size());
  for (std::size_t index = 0; index < count; ++index) {
    const double q = positions(static_cast<Eigen::Index>(index));
    const JointDescription& joint = arm_.joints[index];
    if (q < joint.lower_limit || q > joint.upper_limit) ++violations;
  }
  return violations;
}

Eigen::Matrix4d ForwardKinematics::dhTransform(double d, double theta, double a, double alpha) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  const double cos_alpha = std::cos(alpha);
  const double sin_alpha = std::sin(alpha);

  Eigen::Matrix4d transform;
  transform << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta,
      sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta,
      0.0, sin_alpha, cos_alpha, d,
      0.0, 0.0, 0.0, 1.0;
  return transform;
}

}  // namespace baxter
