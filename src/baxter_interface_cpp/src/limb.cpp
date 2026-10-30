#include "baxter_interface_cpp/limb.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>

namespace {

/// Joint angle tolerance of the SDK's move_to_joint_positions [rad] (0.5 deg).
constexpr double kJointAngleToleranceRad = 0.008726646;
/// Republish rate of blocking position moves [Hz], matching the Python SDK.
constexpr double kPositionCommandRateHz = 100.0;

std::vector<std::string> armJointNames(const std::string& limb_name) {
  return {limb_name + "_s0", limb_name + "_s1", limb_name + "_e0", limb_name + "_e1",
          limb_name + "_w0", limb_name + "_w1", limb_name + "_w2"};
}

}  // namespace

Limb::Limb(const std::string& limb_name)
    : limb_name_(limb_name), joint_names_(armJointNames(limb_name)) {
  const std::string limb_namespace = "/robot/limb/" + limb_name + "/";

  joint_command_publisher_ =
      nh_.advertise<baxter_core_msgs::JointCommand>(limb_namespace + "joint_command", 1);
  speed_ratio_publisher_ =
      nh_.advertise<std_msgs::Float64>(limb_namespace + "set_speed_ratio", 10, true);
  command_timeout_publisher_ =
      nh_.advertise<std_msgs::Float64>(limb_namespace + "joint_command_timeout", 10, true);
  acceleration_command_publisher_ = nh_.advertise<baxter_core_msgs::AccelerationCommand>(
      limb_namespace + "acceleration_command", 1);
  acceleration_max_publisher_ = nh_.advertise<baxter_core_msgs::AccelerationMax>(
      limb_namespace + "acceleration_max_command", 1, true);
  acceleration_min_publisher_ = nh_.advertise<baxter_core_msgs::AccelerationMin>(
      limb_namespace + "acceleration_min_command", 1, true);
  referent_angles_publisher_ = nh_.advertise<baxter_core_msgs::ReferentJointAngles>(
      limb_namespace + "referent_joint_angles", 1, true);

  joint_state_subscriber_ = nh_.subscribe("/robot/joint_states", 1, &Limb::onJointStates, this);
  endpoint_state_subscriber_ =
      nh_.subscribe(limb_namespace + "endpoint_state", 1, &Limb::onEndpointState, this);
  acceleration_command_subscriber_ = nh_.subscribe(limb_namespace + "acceleration_command", 1,
                                                   &Limb::onAccelerationCommand, this);
  acceleration_max_subscriber_ = nh_.subscribe(limb_namespace + "acceleration_max_command", 1,
                                               &Limb::onAccelerationMax, this);
  acceleration_min_subscriber_ = nh_.subscribe(limb_namespace + "acceleration_min_command", 1,
                                               &Limb::onAccelerationMin, this);
  referent_angles_subscriber_ = nh_.subscribe(limb_namespace + "referent_joint_angles", 1,
                                              &Limb::onReferentJointAngles, this);
  sea_joint_state_subscriber_ = nh_.subscribe(limb_namespace + "gravity_compensation_torques", 1,
                                              &Limb::onSeaJointState, this);
}

bool Limb::isLimbJoint(const std::string& joint) const {
  return std::find(joint_names_.begin(), joint_names_.end(), joint) != joint_names_.end();
}

void Limb::onJointStates(const sensor_msgs::JointState::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  // /robot/joint_states carries every Baxter joint; keep this arm's seven.
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    const std::string& joint = message->name[index];
    if (!isLimbJoint(joint)) continue;
    if (index < message->position.size()) joint_angles_[joint] = message->position[index];
    if (index < message->velocity.size()) joint_velocities_[joint] = message->velocity[index];
    if (index < message->effort.size()) joint_efforts_[joint] = message->effort[index];
  }
  last_joint_state_received_at_ = ros::WallTime::now();
  last_joint_state_stamp_ =
      message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
}

void Limb::onReferentJointAngles(const baxter_core_msgs::ReferentJointAngles::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
    const std::string& joint = message->joint_names[index];
    if (isLimbJoint(joint) && index < message->angles.size()) {
      referent_angles_[joint] = message->angles[index];
    }
  }
}

void Limb::onEndpointState(const baxter_core_msgs::EndpointState::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  endpoint_position_["position"] =
      Point{message->pose.position.x, message->pose.position.y, message->pose.position.z};
  endpoint_orientation_["orientation"] =
      Quaternion{message->pose.orientation.x, message->pose.orientation.y,
                 message->pose.orientation.z, message->pose.orientation.w};
  endpoint_velocity_["linear"] =
      Point{message->twist.linear.x, message->twist.linear.y, message->twist.linear.z};
  endpoint_velocity_["angular"] =
      Point{message->twist.angular.x, message->twist.angular.y, message->twist.angular.z};
  endpoint_effort_["force"] =
      Point{message->wrench.force.x, message->wrench.force.y, message->wrench.force.z};
  endpoint_effort_["torque"] =
      Point{message->wrench.torque.x, message->wrench.torque.y, message->wrench.torque.z};
}

void Limb::onAccelerationCommand(
    const baxter_core_msgs::AccelerationCommand::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  bool received_acceleration = false;
  for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
    const std::string& joint = message->joint_names[index];
    if (!isLimbJoint(joint)) continue;
    if (index < message->accelerations.size()) {
      reference_accelerations_[joint] = message->accelerations[index];
      received_acceleration = true;
    }
    if (index < message->velocities.size()) {
      reference_velocities_[joint] = message->velocities[index];
    }
    if (index < message->angles.size()) {
      reference_angles_[joint] = message->angles[index];
    }
  }
  if (received_acceleration) {
    last_acceleration_reference_stamp_ = ros::Time::now();
  }
}

void Limb::onSeaJointState(const baxter_core_msgs::SEAJointState::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    const std::string& joint = message->name[index];
    if (!isLimbJoint(joint)) continue;
    if (index < message->commanded_effort.size()) {
      commanded_efforts_[joint] = message->commanded_effort[index];
    }
    if (index < message->actual_effort.size()) {
      actual_efforts_[joint] = message->actual_effort[index];
    }
    if (index < message->gravity_model_effort.size()) {
      gravity_model_efforts_[joint] = message->gravity_model_effort[index];
    }
  }
}

void Limb::onAccelerationMax(const baxter_core_msgs::AccelerationMax::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
    const std::string& joint = message->joint_names[index];
    if (isLimbJoint(joint) && index < message->accelerations.size()) {
      max_accelerations_[joint] = message->accelerations[index];
    }
  }
}

void Limb::onAccelerationMin(const baxter_core_msgs::AccelerationMin::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
    const std::string& joint = message->joint_names[index];
    if (isLimbJoint(joint) && index < message->accelerations.size()) {
      min_accelerations_[joint] = message->accelerations[index];
    }
  }
}

std::vector<std::string> Limb::jointNames() const { return joint_names_; }

double Limb::jointAngle(const std::string& joint) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return joint_angles_.at(joint);
}

std::unordered_map<std::string, double> Limb::jointAngles() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return joint_angles_;
}

std::unordered_map<std::string, double> Limb::jointVelocities() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return joint_velocities_;
}

JointStateSnapshot Limb::jointStateSnapshot() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return JointStateSnapshot{joint_angles_, joint_velocities_, last_joint_state_received_at_,
                            last_joint_state_stamp_};
}

std::unordered_map<std::string, double> Limb::jointEfforts() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return joint_efforts_;
}

std::unordered_map<std::string, double> Limb::jointAccelerationsRef() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return reference_accelerations_;
}

std::unordered_map<std::string, double> Limb::jointVelocitiesRef() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return reference_velocities_;
}

std::unordered_map<std::string, double> Limb::jointAnglesRef() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return reference_angles_;
}

double Limb::accelerationRefAgeSec() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (last_acceleration_reference_stamp_.isZero()) {
    return std::numeric_limits<double>::max();
  }
  return (ros::Time::now() - last_acceleration_reference_stamp_).toSec();
}

AccelerationReferenceSnapshot Limb::accelerationReferenceSnapshot() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const double age_sec = last_acceleration_reference_stamp_.isZero()
                             ? std::numeric_limits<double>::max()
                             : (ros::Time::now() - last_acceleration_reference_stamp_).toSec();
  return AccelerationReferenceSnapshot{reference_accelerations_, reference_velocities_,
                                       reference_angles_, age_sec};
}

std::unordered_map<std::string, double> Limb::jointAccelerationsMax() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return max_accelerations_;
}

std::unordered_map<std::string, double> Limb::jointAccelerationsMin() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return min_accelerations_;
}

std::unordered_map<std::string, double> Limb::referentJointAngles() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return referent_angles_;
}

std::unordered_map<std::string, double> Limb::commandedEffort() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return commanded_efforts_;
}

std::unordered_map<std::string, double> Limb::actualEffort() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return actual_efforts_;
}

std::unordered_map<std::string, double> Limb::gravityModelEffort() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return gravity_model_efforts_;
}

void Limb::setJointPositions(const std::unordered_map<std::string, double>& positions) {
  baxter_core_msgs::JointCommand command;
  command.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
  for (const auto& [joint, position] : positions) {
    command.names.push_back(joint);
    command.command.push_back(position);
  }
  joint_command_publisher_.publish(command);
}

void Limb::setJointAccelerations(const std::unordered_map<std::string, double>& accelerations,
                                 const std::unordered_map<std::string, double>& velocities,
                                 const std::unordered_map<std::string, double>& angles) {
  baxter_core_msgs::AccelerationCommand command;
  for (const std::string& joint : joint_names_) {
    command.joint_names.push_back(joint);
    command.accelerations.push_back(accelerations.at(joint));
    command.velocities.push_back(velocities.at(joint));
    command.angles.push_back(angles.at(joint));
  }
  acceleration_command_publisher_.publish(command);
}

void Limb::setReferentAngles(const std::unordered_map<std::string, double>& referent_angles) {
  baxter_core_msgs::ReferentJointAngles command;
  for (const std::string& joint : joint_names_) {
    command.joint_names.push_back(joint);
    command.angles.push_back(referent_angles.at(joint));
  }
  referent_angles_publisher_.publish(command);
}

void Limb::setJointAccelerationsMax(const std::unordered_map<std::string, double>& accelerations) {
  baxter_core_msgs::AccelerationMax command;
  for (const auto& [joint, acceleration] : accelerations) {
    command.joint_names.push_back(joint);
    command.accelerations.push_back(acceleration);
  }
  acceleration_max_publisher_.publish(command);
}

void Limb::setJointAccelerationsMin(const std::unordered_map<std::string, double>& accelerations) {
  baxter_core_msgs::AccelerationMin command;
  for (const auto& [joint, acceleration] : accelerations) {
    command.joint_names.push_back(joint);
    command.accelerations.push_back(acceleration);
  }
  acceleration_min_publisher_.publish(command);
}

void Limb::setJointVelocities(const std::unordered_map<std::string, double>& velocities) {
  baxter_core_msgs::JointCommand command;
  command.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
  for (const auto& [joint, velocity] : velocities) {
    command.names.push_back(joint);
    command.command.push_back(velocity);
  }
  joint_command_publisher_.publish(command);
}

void Limb::setJointTorques(const std::unordered_map<std::string, double>& torques) {
  baxter_core_msgs::JointCommand command;
  command.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
  for (const auto& [joint, torque] : torques) {
    command.names.push_back(joint);
    command.command.push_back(torque);
  }
  joint_command_publisher_.publish(command);
}

void Limb::exitControlMode(double timeout) {
  setCommandTimeout(timeout);
  setJointPositions(jointAngles());
}

void Limb::moveToNeutral(double timeout) {
  const std::unordered_map<std::string, double> neutral_positions = {
      {limb_name_ + "_s0", 0.0},  {limb_name_ + "_s1", -0.55}, {limb_name_ + "_e0", 0.0},
      {limb_name_ + "_e1", 0.75}, {limb_name_ + "_w0", 0.0},   {limb_name_ + "_w1", 1.26},
      {limb_name_ + "_w2", 0.0}};

  // A single joint command expires after the joint-command timeout, so the
  // target is republished until the arm arrives (as the Python SDK does).
  const ros::Time deadline = ros::Time::now() + ros::Duration(timeout);
  ros::Rate command_rate(kPositionCommandRateHz);
  while (ros::ok() && ros::Time::now() < deadline) {
    setJointPositions(neutral_positions);
    const std::unordered_map<std::string, double> measured_angles = jointAngles();
    const bool arrived =
        measured_angles.size() == neutral_positions.size() &&
        std::all_of(neutral_positions.begin(), neutral_positions.end(),
                    [&measured_angles](const std::pair<const std::string, double>& target) {
                      const auto measured = measured_angles.find(target.first);
                      return measured != measured_angles.end() &&
                             std::abs(measured->second - target.second) < kJointAngleToleranceRad;
                    });
    if (arrived) return;
    command_rate.sleep();
  }
}

std::unordered_map<std::string, Point> Limb::endpointPose() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return endpoint_position_;
}

std::unordered_map<std::string, Quaternion> Limb::endpointOrientation() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return endpoint_orientation_;
}

std::unordered_map<std::string, Point> Limb::endpointVelocity() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return endpoint_velocity_;
}

std::unordered_map<std::string, Point> Limb::endpointEffort() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return endpoint_effort_;
}

void Limb::setCommandTimeout(double timeout) {
  std_msgs::Float64 timeout_message;
  timeout_message.data = timeout;
  command_timeout_publisher_.publish(timeout_message);
}

void Limb::setJointPositionSpeed(double speed) {
  std_msgs::Float64 speed_message;
  speed_message.data = speed;
  speed_ratio_publisher_.publish(speed_message);
}
