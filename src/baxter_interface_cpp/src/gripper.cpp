#include "baxter_interface_cpp/gripper.hpp"
#include <cmath>

namespace {
// Custom wait_for to replace baxter_dataflow::wait_for
template <typename Func, typename Body>
bool wait_for(
    Func test, double timeout, bool raise_on_error, Body body = []() {}) {
  ros::Time start = ros::Time::now();
  ros::Rate rate(100.0);  // 100 Hz
  while (ros::ok() && (ros::Time::now() - start).toSec() < timeout) {
    ros::spinOnce();  // Process ROS callbacks
    if (test()) return true;
    body();
    rate.sleep();
  }
  if (raise_on_error) { ROS_ERROR("Wait timed out after %f seconds", timeout); }
  return false;
}
}  // namespace

Gripper::Gripper(const std::string& gripper, ros::NodeHandle& nh, bool versioned)
    : name_(gripper + "_gripper"),
      cmd_sequence_(0),
      cmd_sender_(ros::this_node::getName() + "_" + gripper),
      versioned_(versioned) {
  // Initialize parameters (defaults from Python Gripper)
  parameters_ = {
      {"velocity", 50.0}, {"moving_force", 40.0}, {"holding_force", 30.0}, {"dead_zone", 5.0}};

  // Setup ROS topics
  std::string ns = "/robot/end_effector/" + name_ + "/";
  std::string state_topic = ns + "state";
  std::string cmd_topic = ns + "command";
  ROS_INFO_STREAM("Subscribing to gripper state topic: " << state_topic);
  ROS_INFO_STREAM("Publishing to gripper command topic: " << cmd_topic);
  cmd_pub_ = nh.advertise<baxter_core_msgs::EndEffectorCommand>(cmd_topic, 10);
  state_sub_ = nh.subscribe(state_topic, 10, &Gripper::stateCallback, this);

  // Wait for initial state with retries
  bool state_received = false;
  int max_retries = 5;
  for (int attempt = 1; attempt <= max_retries && !state_received; ++attempt) {
    ROS_INFO_STREAM("Attempt " << attempt << "/" << max_retries << " to get gripper state for "
                               << name_);
    ROS_INFO_STREAM("State topic publishers: " << state_sub_.getNumPublishers());
    state_received = wait_for(
        [this]() {
          std::lock_guard<std::mutex> lock(state_mutex_);
          bool id_valid = state_.id != 0 && state_.timestamp != ros::Time(0);
          ROS_DEBUG_STREAM("Gripper state: id=" << state_.id << ", timestamp=" << state_.timestamp
                                                << (id_valid ? " (valid)" : ""));
          return id_valid;
        },
        15.0,
        true,
        []() {});
    if (!state_received && attempt < max_retries) {
      ROS_WARN_STREAM("Retrying gripper state subscription for " << name_);
      state_sub_ = nh.subscribe(state_topic, 10, &Gripper::stateCallback, this);
      ros::Duration(1.0).sleep();
    }
  }

  if (!state_received) {
    ROS_WARN_STREAM("Failed to get valid gripper state for " << name_
                                                             << ". Gripper may not function.");
  }

  // Set default parameters
  set_parameters(parameters_);
}

void Gripper::stateCallback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = *msg;
  ROS_DEBUG_STREAM("Received gripper state: id=" << state_.id << ", timestamp=" << state_.timestamp
                                                 << ", position=" << state_.position
                                                 << ", calibrated=" << state_.calibrated
                                                 << ", gripping=" << state_.gripping);
}

uint32_t Gripper::incrementSequence() {
  cmd_sequence_ = (cmd_sequence_ % 0x7FFFFFFF) + 1;
  return cmd_sequence_;
}

bool Gripper::command(const std::string& cmd, const nlohmann::json& args, bool block,
                      float timeout) {
  baxter_core_msgs::EndEffectorCommand ee_cmd;
  ee_cmd.id = state_.id;
  ee_cmd.command = cmd;
  ee_cmd.sender = cmd_sender_ + "_" + cmd;
  ee_cmd.sequence = incrementSequence();
  ee_cmd.args = args.dump();

  ROS_DEBUG_STREAM("Sending gripper command: " << cmd << ", args=" << ee_cmd.args);
  cmd_pub_.publish(ee_cmd);

  if (!block) return true;

  // Wait for command acknowledgement
  bool cmd_acked = wait_for(
      [this, &ee_cmd]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_.command_sender == ee_cmd.sender &&
               (state_.command_sequence == ee_cmd.sequence || state_.command_sequence == 0);
      },
      timeout,
      false,
      [&]() { cmd_pub_.publish(ee_cmd); });
  if (!cmd_acked) {
    ROS_DEBUG_STREAM("Timed out on gripper command acknowledgement for " << name_ << ":" << cmd);
  }

  return cmd_acked;
}

bool Gripper::open(bool block, float timeout) {
  if (!calibrated()) {
    ROS_WARN_STREAM(name_ << " not calibrated. Cannot open.");
    return false;
  }

  nlohmann::json args = {{"position", 100.0}};
  auto test = [this]() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return std::abs(state_.position - 100.0) < parameters_["dead_zone"] || state_.gripping;
  };

  return command(baxter_core_msgs::EndEffectorCommand::CMD_GO, args, block, timeout);
}

bool Gripper::close(bool block, float timeout) {
  if (!calibrated()) {
    ROS_WARN_STREAM(name_ << " not calibrated. Cannot close.");
    return false;
  }

  nlohmann::json args = {{"position", 0.0}};
  auto test = [this]() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return std::abs(state_.position - 0.0) < parameters_["dead_zone"] || state_.gripping;
  };

  return command(baxter_core_msgs::EndEffectorCommand::CMD_GO, args, block, timeout);
}

bool Gripper::calibrate(bool block, float timeout) {
  auto test = [this]() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_.calibrated && state_.ready;
  };

  bool success = command(baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE, {}, block, timeout);
  if (success && block) {
    success = wait_for(test, timeout, false, []() {});
  }
  return success;
}

bool Gripper::calibrated() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_.calibrated;
}

void Gripper::set_parameters(const std::unordered_map<std::string, double>& params) {
  for (const auto& param : params) {
    const std::string& key = param.first;
    double value = param.second;
    if (parameters_.count(key)) {
      parameters_[key] = std::max(0.0, std::min(100.0, value));
    } else {
      ROS_WARN_STREAM("Invalid parameter: " << key << " for " << name_);
    }
  }
  nlohmann::json args;
  for (const auto& param : parameters_) {
    const std::string& key = param.first;
    double value = param.second;
    args[key] = value;
  }
  command(baxter_core_msgs::EndEffectorCommand::CMD_CONFIGURE, args, false);
}