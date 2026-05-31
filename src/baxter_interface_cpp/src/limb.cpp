#include "baxter_interface_cpp/limb.hpp"

#include <algorithm>
#include <limits>

Limb::Limb(const std::string &limb_name) : name_(limb_name) {
  const std::string ns = "/robot/limb/" + limb_name + "/";

  pub_joint_cmd_ =
      nh_.advertise<baxter_core_msgs::JointCommand>(ns + "joint_command", 1);
  sub_joint_state_ =
      nh_.subscribe("/robot/joint_states", 1, &Limb::onJointStates, this);
  sub_acceleration_cmd_ = nh_.subscribe(
      ns + "acceleration_command", 1, &Limb::onJointAccelerations, this);

  joint_names_["left"] = {"left_s0", "left_s1", "left_e0", "left_e1",
                           "left_w0", "left_w1", "left_w2"};
  joint_names_["right"] = {"right_s0", "right_s1", "right_e0",
                            "right_e1", "right_w0", "right_w1",
                            "right_w2"};
}

std::vector<std::string> Limb::jointNames() const { return joint_names_.at(name_); }

std::unordered_map<std::string, double> Limb::jointAngles() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_angle_;
}

std::unordered_map<std::string, double> Limb::jointVelocities() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_velocity_;
}

std::unordered_map<std::string, double> Limb::jointAccelerationsRef() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_acceleration_;
}

double Limb::accelerationRefAgeSec() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_acceleration_ref_stamp_.isZero()) {
    return std::numeric_limits<double>::infinity();
  }

  return (ros::Time::now() - last_acceleration_ref_stamp_).toSec();
}

void Limb::setJointTorques(
    const std::unordered_map<std::string, double> &torques) {
  baxter_core_msgs::JointCommand cmd_msg;
  cmd_msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;

  for (const auto &joint : jointNames()) {
    auto it = torques.find(joint);
    if (it == torques.end()) {
      continue;
    }

    cmd_msg.names.push_back(it->first);
    cmd_msg.command.push_back(it->second);
  }

  pub_joint_cmd_.publish(cmd_msg);
}

void Limb::onJointStates(const sensor_msgs::JointState::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t index = 0; index < msg->name.size(); ++index) {
    if (!ownsJoint(msg->name[index])) {
      continue;
    }

    if (index < msg->position.size()) {
      joint_angle_[msg->name[index]] = msg->position[index];
    }

    if (index < msg->velocity.size()) {
      joint_velocity_[msg->name[index]] = msg->velocity[index];
    }
  }
}

void Limb::onJointAccelerations(
    const baxter_core_msgs::AccelerationCommand::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  bool updated = false;
  for (size_t index = 0; index < msg->joint_names.size(); ++index) {
    if (!ownsJoint(msg->joint_names[index])) {
      continue;
    }

    if (index >= msg->accelerations.size()) {
      break;
    }

    joint_acceleration_[msg->joint_names[index]] = msg->accelerations[index];
    updated = true;
  }

  if (updated) {
    last_acceleration_ref_stamp_ = ros::Time::now();
  }
}

bool Limb::ownsJoint(const std::string &joint_name) const {
  const auto &joints = joint_names_.at(name_);
  return std::find(joints.begin(), joints.end(), joint_name) != joints.end();
}