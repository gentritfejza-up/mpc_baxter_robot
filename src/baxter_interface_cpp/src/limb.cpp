#include "baxter_interface_cpp/limb.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>

Limb::Limb(const std::string& limb_name) : name(limb_name) {
  std::string ns = "/robot/limb/" + limb_name + "/";

  joint_names_["left"] = {
      "left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
  joint_names_["right"] = {
      "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};

  pub_joint_cmd_ = nh_.advertise<baxter_core_msgs::JointCommand>(ns + "joint_command", 1);
  pub_speed_ratio_ = nh_.advertise<std_msgs::Float64>(ns + "set_speed_ratio", 10, true);
  pub_joint_cmd_timeout_ = nh_.advertise<std_msgs::Float64>(ns + "joint_command_timeout", 10, true);
  pub_acceleration_cmd_ =
      nh_.advertise<baxter_core_msgs::AccelerationCommand>(ns + "acceleration_command", 1);
  pub_acceleration_max_cmd_ =
      nh_.advertise<baxter_core_msgs::AccelerationMax>(ns + "acceleration_max_command", 1, true);
  pub_acceleration_min_cmd_ =
      nh_.advertise<baxter_core_msgs::AccelerationMin>(ns + "acceleration_min_command", 1, true);
  pub_ref_angles_ =
      nh_.advertise<baxter_core_msgs::ReferentJointAngles>(ns + "referent_joint_angles", 1, true);

  sub_joint_state_ = nh_.subscribe("/robot/joint_states", 1, &Limb::onJointStates, this);
  sub_endpoint_state_ = nh_.subscribe(ns + "endpoint_state", 1, &Limb::onEndpointStates, this);
  sub_acceleration_cmd_ =
      nh_.subscribe(ns + "acceleration_command", 1, &Limb::onJointAccelerations, this);
  sub_acceleration_max_cmd_ =
      nh_.subscribe(ns + "acceleration_max_command", 1, &Limb::onJointAccelerationsMax, this);
  sub_acceleration_min_cmd_ =
      nh_.subscribe(ns + "acceleration_min_command", 1, &Limb::onJointAccelerationsMin, this);
  sub_ref_angles_ =
      nh_.subscribe(ns + "referent_joint_angles", 1, &Limb::onJointReferentAngles, this);
  gravity_compensation_ =
      nh_.subscribe(ns + "gravity_compensation_torques", 1, &Limb::onGravityCompensation, this);

}

void Limb::onJointStates(const sensor_msgs::JointState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (std::find(joint_names_[name].begin(), joint_names_[name].end(), msg->name[i]) !=
        joint_names_[name].end()) {
      if (i < msg->position.size()) {
        joint_angle_[msg->name[i]] = msg->position[i];
      }
      if (i < msg->velocity.size()) {
        joint_velocity_[msg->name[i]] = msg->velocity[i];
      }
      if (i < msg->effort.size()) {
        joint_effort_[msg->name[i]] = msg->effort[i];
      }
    }
  }
}

void Limb::onJointReferentAngles(const baxter_core_msgs::ReferentJointAngles::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    if (std::find(joint_names_[name].begin(), joint_names_[name].end(), msg->joint_names[i]) !=
        joint_names_[name].end()) {
      if (i < msg->angles.size()) {
        referent_angles_[msg->joint_names[i]] = msg->angles[i];
      }
    }
  }
}

void Limb::onEndpointStates(const baxter_core_msgs::EndpointState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  cartesian_pose_["position"] =
      Point{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
  cartesian_orientation_["orientation"] = Quaternion{msg->pose.orientation.x,
                                                     msg->pose.orientation.y,
                                                     msg->pose.orientation.z,
                                                     msg->pose.orientation.w};

  cartesian_velocity_["linear"] =
      Point{msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
  cartesian_velocity_["angular"] =
      Point{msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};

  cartesian_effort_["force"] = Point{msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z};
  cartesian_effort_["torque"] =
      Point{msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z};
}

void Limb::onJointAccelerations(const baxter_core_msgs::AccelerationCommand::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  bool received_acceleration = false;
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    if (std::find(joint_names_[name].begin(), joint_names_[name].end(), msg->joint_names[i]) !=
        joint_names_[name].end()) {
      if (i < msg->accelerations.size()) {
        joint_acceleration_[msg->joint_names[i]] = msg->accelerations[i];
        received_acceleration = true;
      }
      if (i < msg->velocities.size()) {
        joint_velocity_ref_[msg->joint_names[i]] = msg->velocities[i];
      }
      if (i < msg->angles.size()) {
        joint_angle_ref_[msg->joint_names[i]] = msg->angles[i];
      }
    }
  }
  if (received_acceleration) {
    last_acceleration_ref_stamp_ = ros::Time::now();
  }
}

void Limb::onGravityCompensation(const baxter_core_msgs::SEAJointState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (std::find(joint_names_[name].begin(), joint_names_[name].end(), msg->name[i]) ==
        joint_names_[name].end()) {
      continue;
    }
    if (i < msg->commanded_effort.size()) {
      commanded_effort_[msg->name[i]] = msg->commanded_effort[i];
    }
    if (i < msg->actual_effort.size()) {
      actual_effort_[msg->name[i]] = msg->actual_effort[i];
    }
    if (i < msg->gravity_model_effort.size()) {
      gravity_model_effort_[msg->name[i]] = msg->gravity_model_effort[i];
    }
  }
}

double Limb::accelerationRefAgeSec() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_acceleration_ref_stamp_.isZero()) {
    return std::numeric_limits<double>::max();
  }
  return (ros::Time::now() - last_acceleration_ref_stamp_).toSec();
}

void Limb::onJointAccelerationsMax(const baxter_core_msgs::AccelerationMax::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    if (std::find(joint_names_[name].begin(), joint_names_[name].end(), msg->joint_names[i]) !=
        joint_names_[name].end()) {
      if (i < msg->accelerations.size()) {
        joint_acceleration_max_[msg->joint_names[i]] = msg->accelerations[i];
      }
    }
  }
}

void Limb::onJointAccelerationsMin(const baxter_core_msgs::AccelerationMin::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    if (std::find(joint_names_[name].begin(), joint_names_[name].end(), msg->joint_names[i]) !=
        joint_names_[name].end()) {
      if (i < msg->accelerations.size()) {
        joint_acceleration_min_[msg->joint_names[i]] = msg->accelerations[i];
      }
    }
  }
}

std::vector<std::string> Limb::jointNames() const {
  return joint_names_.at(name);
}

double Limb::jointAngle(const std::string& joint) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_angle_.at(joint);
}

std::unordered_map<std::string, double> Limb::jointAngles() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_angle_;
}

std::unordered_map<std::string, double> Limb::jointVelocities() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_velocity_;
}

std::unordered_map<std::string, double> Limb::jointEfforts() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_effort_;
}

std::unordered_map<std::string, double> Limb::jointAccelerationsRef() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_acceleration_;
}

std::unordered_map<std::string, double> Limb::jointVelocitiesRef() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_velocity_ref_;
}

std::unordered_map<std::string, double> Limb::jointAnglesRef() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_angle_ref_;
}

std::unordered_map<std::string, double> Limb::jointAccelerationsMax() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_acceleration_max_;
}

std::unordered_map<std::string, double> Limb::jointAccelerationsMin() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_acceleration_min_;
}

std::unordered_map<std::string, double> Limb::referentJointAngles() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return referent_angles_;
}

std::unordered_map<std::string, double> Limb::commandedEffort() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return commanded_effort_;
}

std::unordered_map<std::string, double> Limb::actualEffort() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_effort_;
}

std::unordered_map<std::string, double> Limb::gravityModelEffort() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return gravity_model_effort_;
}

void Limb::setJointPositions(const std::unordered_map<std::string, double>& positions) {
  baxter_core_msgs::JointCommand cmd_msg;
  cmd_msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
  for (const auto& joint : positions) {
    cmd_msg.names.push_back(joint.first);
    cmd_msg.command.push_back(joint.second);
  }
  pub_joint_cmd_.publish(cmd_msg);
}

void Limb::setJointAccelerations(const std::unordered_map<std::string, double>& accelerations,
                                 const std::unordered_map<std::string, double>& velocities,
                                 const std::unordered_map<std::string, double>& angles) {
  baxter_core_msgs::AccelerationCommand cmd_msg;
  for (const auto& joint : joint_names_[name]) {
    cmd_msg.joint_names.push_back(joint);
    cmd_msg.accelerations.push_back(accelerations.at(joint));
    cmd_msg.velocities.push_back(velocities.at(joint));
    cmd_msg.angles.push_back(angles.at(joint));
  }
  pub_acceleration_cmd_.publish(cmd_msg);
}

void Limb::setReferentAngles(const std::unordered_map<std::string, double>& referent_angles) {
  baxter_core_msgs::ReferentJointAngles cmd_msg;
  for (const auto& joint : joint_names_[name]) {
    cmd_msg.joint_names.push_back(joint);
    cmd_msg.angles.push_back(referent_angles.at(joint));
  }
  pub_ref_angles_.publish(cmd_msg);
}

void Limb::setJointAccelerationsMax(const std::unordered_map<std::string, double>& accelerations) {
  baxter_core_msgs::AccelerationMax cmd_msg;
  for (const auto& joint : accelerations) {
    cmd_msg.joint_names.push_back(joint.first);
    cmd_msg.accelerations.push_back(joint.second);
  }
  pub_acceleration_max_cmd_.publish(cmd_msg);
}

void Limb::setJointAccelerationsMin(const std::unordered_map<std::string, double>& accelerations) {
  baxter_core_msgs::AccelerationMin cmd_msg;
  for (const auto& joint : accelerations) {
    cmd_msg.joint_names.push_back(joint.first);
    cmd_msg.accelerations.push_back(joint.second);
  }
  pub_acceleration_min_cmd_.publish(cmd_msg);
}

void Limb::setJointVelocities(const std::unordered_map<std::string, double>& velocities) {
  baxter_core_msgs::JointCommand cmd_msg;
  cmd_msg.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
  for (const auto& joint : velocities) {
    cmd_msg.names.push_back(joint.first);
    cmd_msg.command.push_back(joint.second);
  }
  pub_joint_cmd_.publish(cmd_msg);
}

void Limb::setJointTorques(const std::unordered_map<std::string, double>& torques) {
  baxter_core_msgs::JointCommand cmd_msg;
  cmd_msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
  for (const auto& joint : torques) {
    cmd_msg.names.push_back(joint.first);
    cmd_msg.command.push_back(joint.second);
  }
  pub_joint_cmd_.publish(cmd_msg);
}

void Limb::exitControlMode(double timeout) {
  setCommandTimeout(timeout);
  setJointPositions(jointAngles());
}

void Limb::moveToNeutral(double timeout) {
  const std::unordered_map<std::string, double> neutral_positions = {
      {name + "_s0", 0.0},  {name + "_s1", -0.55}, {name + "_e0", 0.0},
      {name + "_e1", 0.75}, {name + "_w0", 0.0},   {name + "_w1", 1.26},
      {name + "_w2", 0.0}};

  setJointPositions(neutral_positions);
  ros::Duration(timeout).sleep();
}

std::unordered_map<std::string, Point> Limb::endpointPose() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cartesian_pose_;
}

std::unordered_map<std::string, Point> Limb::endpointVelocity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cartesian_velocity_;
}

std::unordered_map<std::string, Point> Limb::endpointEffort() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cartesian_effort_;
}

void Limb::setCommandTimeout(double timeout) {
  std_msgs::Float64 timeout_msg;
  timeout_msg.data = timeout;
  pub_joint_cmd_timeout_.publish(timeout_msg);
}

void Limb::setJointPositionSpeed(double speed) {
  std_msgs::Float64 speed_msg;
  speed_msg.data = speed;
  pub_speed_ratio_.publish(speed_msg);
}
