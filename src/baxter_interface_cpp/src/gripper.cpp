#include "baxter_interface_cpp/gripper.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPollRateHz = 100.0;
constexpr double kInitialStateTimeoutSec = 15.0;
constexpr int kInitialStateAttempts = 5;

/**
 * @brief Poll `condition` at 100 Hz, spinning the global callback queue, until it
 *        holds or `timeout_sec` elapses; `on_each_poll` runs after every failed check.
 * @return True when the condition held before the timeout.
 */
template <typename Condition, typename OnEachPoll>
bool waitFor(Condition condition, double timeout_sec, bool log_timeout_as_error,
             OnEachPoll on_each_poll) {
  const ros::Time start = ros::Time::now();
  ros::Rate poll_rate(kPollRateHz);
  while (ros::ok() && (ros::Time::now() - start).toSec() < timeout_sec) {
    ros::spinOnce();
    if (condition()) return true;
    on_each_poll();
    poll_rate.sleep();
  }
  if (log_timeout_as_error) ROS_ERROR("Wait timed out after %f seconds", timeout_sec);
  return false;
}

template <typename Condition>
bool waitFor(Condition condition, double timeout_sec, bool log_timeout_as_error) {
  return waitFor(condition, timeout_sec, log_timeout_as_error, []() {});
}

}  // namespace

Gripper::Gripper(const std::string& gripper, ros::NodeHandle& nh, bool /*versioned*/)
    : name_(gripper + "_gripper"),
      parameters_{{"velocity", 50.0},
                  {"moving_force", 40.0},
                  {"holding_force", 30.0},
                  {"dead_zone", 5.0}},
      command_sequence_(0),
      command_sender_(ros::this_node::getName() + "_" + gripper) {
  const std::string gripper_namespace = "/robot/end_effector/" + name_ + "/";
  const std::string state_topic = gripper_namespace + "state";
  const std::string command_topic = gripper_namespace + "command";
  ROS_INFO_STREAM("Subscribing to gripper state topic: " << state_topic);
  ROS_INFO_STREAM("Publishing to gripper command topic: " << command_topic);
  command_publisher_ = nh.advertise<baxter_core_msgs::EndEffectorCommand>(command_topic, 10);
  state_subscriber_ = nh.subscribe(state_topic, 10, &Gripper::onEndEffectorState, this);

  // The gripper id is needed for every command; wait for the first valid state.
  bool state_received = false;
  for (int attempt = 1; attempt <= kInitialStateAttempts && !state_received; ++attempt) {
    ROS_INFO_STREAM("Attempt " << attempt << "/" << kInitialStateAttempts
                               << " to get gripper state for " << name_);
    ROS_INFO_STREAM("State topic publishers: " << state_subscriber_.getNumPublishers());
    state_received = waitFor(
        [this]() {
          const baxter_core_msgs::EndEffectorState state = currentState();
          const bool id_valid = state.id != 0 && state.timestamp != ros::Time(0);
          ROS_DEBUG_STREAM("Gripper state: id=" << state.id << ", timestamp=" << state.timestamp
                                                << (id_valid ? " (valid)" : ""));
          return id_valid;
        },
        kInitialStateTimeoutSec, true);
    if (!state_received && attempt < kInitialStateAttempts) {
      ROS_WARN_STREAM("Retrying gripper state subscription for " << name_);
      state_subscriber_ = nh.subscribe(state_topic, 10, &Gripper::onEndEffectorState, this);
      ros::Duration(1.0).sleep();
    }
  }
  if (!state_received) {
    ROS_WARN_STREAM("Failed to get valid gripper state for " << name_
                                                             << ". Gripper may not function.");
  }

  set_parameters(parameters_);
}

void Gripper::onEndEffectorState(const baxter_core_msgs::EndEffectorState::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = *message;
  ROS_DEBUG_STREAM("Received gripper state: id=" << state_.id << ", timestamp=" << state_.timestamp
                                                 << ", position=" << state_.position
                                                 << ", calibrated=" << state_.calibrated
                                                 << ", gripping=" << state_.gripping);
}

baxter_core_msgs::EndEffectorState Gripper::currentState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

uint32_t Gripper::nextCommandSequence() {
  command_sequence_ = (command_sequence_ % 0x7FFFFFFF) + 1;
  return command_sequence_;
}

bool Gripper::command(const std::string& command_name, const nlohmann::json& arguments,
                      bool block, float timeout) {
  baxter_core_msgs::EndEffectorCommand command_message;
  command_message.id = currentState().id;
  command_message.command = command_name;
  command_message.sender = command_sender_ + "_" + command_name;
  command_message.sequence = nextCommandSequence();
  command_message.args = arguments.dump();

  ROS_DEBUG_STREAM("Sending gripper command: " << command_name
                                               << ", args=" << command_message.args);
  command_publisher_.publish(command_message);

  if (!block) return true;

  const bool acknowledged = waitFor(
      [this, &command_message]() {
        const baxter_core_msgs::EndEffectorState state = currentState();
        return state.command_sender == command_message.sender &&
               (state.command_sequence == command_message.sequence ||
                state.command_sequence == 0);
      },
      timeout, false, [&]() { command_publisher_.publish(command_message); });
  if (!acknowledged) {
    ROS_DEBUG_STREAM("Timed out on gripper command acknowledgement for " << name_ << ":"
                                                                         << command_name);
  }
  return acknowledged;
}

bool Gripper::open(bool block, float timeout) {
  if (!calibrated()) {
    ROS_WARN_STREAM(name_ << " not calibrated. Cannot open.");
    return false;
  }
  constexpr double kOpenPosition = 100.0;
  bool success = command(baxter_core_msgs::EndEffectorCommand::CMD_GO,
                         {{"position", kOpenPosition}}, block, timeout);
  if (success && block) {
    success = waitFor(
        [this]() {
          const baxter_core_msgs::EndEffectorState state = currentState();
          return std::abs(state.position - kOpenPosition) < parameters_.at("dead_zone") ||
                 state.gripping;
        },
        timeout, false);
  }
  return success;
}

bool Gripper::close(bool block, float timeout) {
  if (!calibrated()) {
    ROS_WARN_STREAM(name_ << " not calibrated. Cannot close.");
    return false;
  }
  constexpr double kClosedPosition = 0.0;
  bool success = command(baxter_core_msgs::EndEffectorCommand::CMD_GO,
                         {{"position", kClosedPosition}}, block, timeout);
  if (success && block) {
    success = waitFor(
        [this]() {
          const baxter_core_msgs::EndEffectorState state = currentState();
          return std::abs(state.position - kClosedPosition) < parameters_.at("dead_zone") ||
                 state.gripping;
        },
        timeout, false);
  }
  return success;
}

bool Gripper::calibrate(bool block, float timeout) {
  bool success =
      command(baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE, {}, block, timeout);
  if (success && block) {
    success = waitFor(
        [this]() {
          const baxter_core_msgs::EndEffectorState state = currentState();
          return state.calibrated && state.ready;
        },
        timeout, false);
  }
  return success;
}

bool Gripper::calibrated() const { return currentState().calibrated; }

void Gripper::set_parameters(const std::unordered_map<std::string, double>& params) {
  for (const auto& [key, value] : params) {
    if (parameters_.count(key)) {
      parameters_[key] = std::clamp(value, 0.0, 100.0);
    } else {
      ROS_WARN_STREAM("Invalid parameter: " << key << " for " << name_);
    }
  }
  nlohmann::json arguments;
  for (const auto& [key, value] : parameters_) arguments[key] = value;
  command(baxter_core_msgs::EndEffectorCommand::CMD_CONFIGURE, arguments, false);
}
