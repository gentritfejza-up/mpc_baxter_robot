#include "mpc_controller/joint_state_listener.hpp"

#include <cmath>
#include <stdexcept>

namespace mpc_geometry {

OtherArmStateListener::OtherArmStateListener(ros::NodeHandle& node_handle, const std::string& side)
    : side_(side) {
  if (side != "left" && side != "right") throw std::invalid_argument("side must be left or right");
  for (int joint = 0; joint < kArmJoints; ++joint) {
    joint_names_[joint] = side + "_" + kArmJointSuffixes[joint];
  }
  subscriber_ = node_handle.subscribe("/robot/joint_states", 1,
                                      &OtherArmStateListener::onJointStates, this);
}

void OtherArmStateListener::onJointStates(const sensor_msgs::JointState::ConstPtr& message) {
  JointVector positions{};
  JointVector velocities{};
  std::array<bool, kArmJoints> seen{};
  for (std::size_t entry = 0; entry < message->name.size() && entry < message->position.size();
       ++entry) {
    for (int joint = 0; joint < kArmJoints; ++joint) {
      if (message->name[entry] != joint_names_[joint]) continue;
      if (!std::isfinite(message->position[entry])) return;
      positions[joint] = message->position[entry];
      if (entry < message->velocity.size()) {
        // A supplied but non-finite velocity is a corrupt message, not a resting joint.
        if (!std::isfinite(message->velocity[entry])) return;
        velocities[joint] = message->velocity[entry];
      }
      seen[joint] = true;
    }
  }
  for (bool joint_seen : seen) {
    if (!joint_seen) return;  // a partial message never replaces a complete one
  }
  std::lock_guard<std::mutex> lock(mutex_);
  positions_ = positions;
  velocities_ = velocities;
  has_complete_state_ = true;
  received_at_ = ros::WallTime::now();
}

bool OtherArmStateListener::latest(JointVector& positions, JointVector& velocities,
                                   double timeout_sec, double* age_sec) const {
  // One critical section for positions and velocities, so both come from the same message.
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_complete_state_) return false;
  const double age = (ros::WallTime::now() - received_at_).toSec();
  if (age_sec) *age_sec = age;
  if (!std::isfinite(age) || age < 0.0 || age > timeout_sec) return false;
  positions = positions_;
  velocities = velocities_;
  return true;
}

}  // namespace mpc_geometry
