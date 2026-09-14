#include "mpc_controller/mpc_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point& start,
                 const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

MPCControlManager::MPCControlManager(ros::NodeHandle& nh, const std::string& limb_side)
    : nh_(nh), limb_side_(limb_side), limb_(std::make_unique<Limb>(limb_side)) {
  loadAndValidateConfiguration();
  previous_applied_accelerations_.assign(num_joints_, 0.0);
  mpc_ = std::make_unique<MPCController>(nh_, num_joints_, prediction_horizon_, control_horizon_,
                                         makePredictionGrid(), control_period_sec_);

  ROS_INFO_STREAM("MPC runtime rate=" << control_rate_hz_ << " Hz period="
                                      << control_period_sec_ * 1000.0 << " ms prediction="
                                      << prediction_duration_sec_ << " s");
}

void MPCControlManager::loadAndValidateConfiguration() {
  nh_.param("robot/control/mpc_rate_hz", control_rate_hz_, 50.0);
  nh_.param("robot/control/prediction_duration_sec", prediction_duration_sec_, 2.0);
  nh_.param("robot/control/prediction_horizon", prediction_horizon_, 20);
  nh_.param("robot/control/control_horizon", control_horizon_, 15);
  nh_.param("robot/control/mpc_spinner_threads", spinner_threads_, 10);
  if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
    throw std::invalid_argument("robot/control/mpc_rate_hz must be finite and positive");
  }
  control_period_sec_ = 1.0 / control_rate_hz_;
  nh_.param("robot/control/state_timeout_sec", state_timeout_sec_, 2.0 * control_period_sec_);
  nh_.param("robot/control/cycle_deadline_ms", cycle_deadline_sec_,
            control_period_sec_ * 1000.0);
  cycle_deadline_sec_ /= 1000.0;
  nh_.param("robot/control/enforce_cycle_deadline", enforce_cycle_deadline_, true);

  if (prediction_horizon_ < 2 || control_horizon_ <= 0 ||
      control_horizon_ > prediction_horizon_ || spinner_threads_ <= 0 ||
      !std::isfinite(prediction_duration_sec_) ||
      prediction_duration_sec_ <= control_period_sec_ || !std::isfinite(state_timeout_sec_) ||
      state_timeout_sec_ <= 0.0 || !std::isfinite(cycle_deadline_sec_) ||
      cycle_deadline_sec_ <= 0.0) {
    throw std::invalid_argument("Invalid MPC horizon, timing, or thread configuration");
  }
}

std::vector<double> MPCControlManager::makePredictionGrid() const {
  std::vector<double> durations(prediction_horizon_, control_period_sec_);
  const double remaining_duration = prediction_duration_sec_ - control_period_sec_;
  const double later_step = remaining_duration / static_cast<double>(prediction_horizon_ - 1);
  std::fill(durations.begin() + 1, durations.end(), later_step);
  return durations;
}

bool MPCControlManager::processControl(
    const std::vector<double>& x_ref,
    const std::chrono::steady_clock::time_point& cycle_start) {
  const JointStateSnapshot state = limb_->jointStateSnapshot();
  if (state.received_at.isZero()) {
    ROS_WARN_THROTTLE(1.0, "Joint state has not been received; suppressing MPC command.");
    return false;
  }
  const double state_age = (ros::WallTime::now() - state.received_at).toSec();
  if (!std::isfinite(state_age) || state_age < 0.0 || state_age > state_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "Joint state is stale (%.3f s); suppressing MPC command.", state_age);
    return false;
  }

  std::vector<double> positions;
  std::vector<double> velocities;
  positions.reserve(num_joints_);
  velocities.reserve(num_joints_);
  for (const auto& joint_name : limb_->jointNames()) {
    const auto position = state.positions.find(joint_name);
    const auto velocity = state.velocities.find(joint_name);
    if (position == state.positions.end() || velocity == state.velocities.end() ||
        !std::isfinite(position->second) || !std::isfinite(velocity->second)) {
      ROS_WARN_THROTTLE(1.0, "Joint state is incomplete or nonfinite; suppressing MPC command.");
      return false;
    }
    positions.push_back(position->second);
    velocities.push_back(velocity->second);
  }
  if (x_ref.size() != static_cast<size_t>(num_joints_) ||
      !std::all_of(x_ref.begin(), x_ref.end(), [](double value) { return std::isfinite(value); })) {
    ROS_ERROR_THROTTLE(1.0, "MPC reference is incomplete or nonfinite; suppressing command.");
    return false;
  }

  const auto solve_start = std::chrono::steady_clock::now();
  last_state_preparation_ms_ = elapsedMs(cycle_start, solve_start);

  double first_jerk_dt = control_period_sec_;
  if (has_last_command_time_) {
    first_jerk_dt = std::chrono::duration<double>(solve_start - last_command_time_).count();
    first_jerk_dt = std::max(1e-4, std::min(first_jerk_dt, control_period_sec_));
  }

  try {
    const casadi::DM solution =
        mpc_->solve(casadi::DM(positions), casadi::DM(velocities), casadi::DM(x_ref),
                    casadi::DM(previous_applied_accelerations_), first_jerk_dt);
    const casadi::DM first_acceleration = mpc_->getFirstAcceleration(solution);
    const auto joint_names = limb_->jointNames();
    const auto acceleration_command = mpc_->dmToMap(first_acceleration, joint_names);
    const auto velocity_command = mpc_->dmToMap(mpc_->getReferentVelocities(), joint_names);
    const auto angle_command = mpc_->dmToMap(mpc_->getReferentAngles(), joint_names);
    const auto target_angles = mpc_->dmToMap(casadi::DM(x_ref), joint_names);
    const auto publish_time = std::chrono::steady_clock::now();
    const MpcSolveTiming timing = mpc_->getLastSolveTiming();
    last_postprocessing_ms_ =
        std::max(0.0, elapsedMs(solve_start, publish_time) - timing.argument_preparation_ms -
                          timing.solver_ms - timing.validation_ms);
    const double cycle_elapsed_sec =
        std::chrono::duration<double>(publish_time - cycle_start).count();
    if (enforce_cycle_deadline_ && cycle_elapsed_sec > cycle_deadline_sec_) {
      ++deadline_misses_;
      // Keep the validated solution only as a numerical seed. No command or
      // applied-acceleration state is updated on a missed deadline.
      mpc_->commitSolution();
      ROS_ERROR_THROTTLE(1.0,
                         "MPC missed %.2f ms deadline (%.2f ms); suppressing command.",
                         cycle_deadline_sec_ * 1000.0, cycle_elapsed_sec * 1000.0);
      return false;
    }

    limb_->setReferentAngles(target_angles);
    limb_->setJointAccelerations(acceleration_command, velocity_command, angle_command);
    previous_applied_accelerations_.clear();
    for (const auto& joint_name : joint_names) {
      previous_applied_accelerations_.push_back(acceleration_command.at(joint_name));
    }
    mpc_->commitSolution();
    last_command_time_ = std::chrono::steady_clock::now();
    has_last_command_time_ = true;
    ++successful_cycles_;
    return true;
  } catch (const std::exception& exception) {
    ++failed_cycles_;
    mpc_->resetWarmStart();
    ROS_ERROR_THROTTLE(1.0, "MPC solve failed; suppressing command: %s", exception.what());
    return false;
  }
}

void MPCControlManager::run() {
  ros::AsyncSpinner spinner(spinner_threads_);
  spinner.start();
  ros::WallRate rate(control_rate_hz_);
  while (ros::ok()) {
    const auto cycle_start = std::chrono::steady_clock::now();
    processControl(mpc_->getReferencePositions(), cycle_start);
    const auto cycle_end = std::chrono::steady_clock::now();
    const MpcSolveTiming timing = mpc_->getLastSolveTiming();
    ROS_INFO_THROTTLE(2.0,
                      "MPC backend=%s cycle=%.2f ms state=%.2f ms args=%.2f ms solve=%.2f ms "
                      "validate=%.2f ms post=%.2f ms ok=%llu fail=%llu deadline=%llu",
                      mpc_->solverBackendName().c_str(), elapsedMs(cycle_start, cycle_end),
                      last_state_preparation_ms_, timing.argument_preparation_ms, timing.solver_ms,
                      timing.validation_ms, last_postprocessing_ms_,
                      static_cast<unsigned long long>(successful_cycles_),
                      static_cast<unsigned long long>(failed_cycles_),
                      static_cast<unsigned long long>(deadline_misses_));
    rate.sleep();
  }
}
