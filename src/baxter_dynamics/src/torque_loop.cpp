#include "baxter_dynamics/torque_loop.hpp"

#include <std_msgs/Float64MultiArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace baxter {
namespace {

/// Diagnostics layout "v1": sequence, loop dt [ms], compute time [ms],
/// reference age [s], torque-control flag, then kDiagnosticsFieldCount blocks of
/// one value per joint (the order of diagnosticsFields()).
constexpr std::size_t kDiagnosticsMetadataCount = 5;
constexpr std::size_t kDiagnosticsFieldCount = 14;

std::array<const std::vector<double>*, kDiagnosticsFieldCount> diagnosticsFields(
    const ComputedTorqueResult& result) {
  return {&result.position,           &result.velocity,
          &result.reference_position, &result.reference_velocity,
          &result.mpc_acceleration,   &result.pd_acceleration,
          &result.commanded_acceleration, &result.inertia_torque,
          &result.coriolis_torque,    &result.gravity_torque,
          &result.damping_torque,     &result.raw_torque,
          &result.commanded_torque,   &result.torque_limit};
}

std::string formatGainPairs(const std::vector<double>& kp, const std::vector<double>& kd) {
  std::ostringstream gains;
  for (std::size_t index = 0; index < kp.size(); ++index) {
    gains << (index ? ", " : "") << kp[index] << "/" << kd[index];
  }
  return gains.str();
}

}  // namespace

TorqueLoopSettings TorqueLoopSettings::load(ros::NodeHandle& nh,
                                            const TorqueLoopParameterKeys& keys,
                                            std::size_t joint_count) {
  const std::string ns = keys.settings_namespace + "/";
  TorqueLoopSettings settings;
  nh.param("robot/control/spinner_threads", settings.spinner_threads, settings.spinner_threads);
  nh.param(keys.loop_rate_key, settings.loop_rate_hz, settings.loop_rate_hz);
  nh.param(keys.diagnostics_rate_key, settings.diagnostics_rate_hz, settings.diagnostics_rate_hz);
  nh.param("robot/control/acceleration_ref_timeout_sec", settings.acceleration_timeout_sec,
           settings.acceleration_timeout_sec);
  nh.param("robot/control/joint_state_timeout_sec", settings.joint_state_timeout_sec,
           settings.joint_state_timeout_sec);
  nh.param(ns + "hold_when_idle", settings.hold_when_idle, settings.hold_when_idle);
  nh.param<std::string>(ns + "hold_mode", settings.hold_mode, settings.hold_mode);
  if (settings.hold_mode != "position" && settings.hold_mode != "torque") {
    throw std::runtime_error(ns + "hold_mode must be position or torque");
  }
  settings.hold_kp = loadGainVector(nh, ns + "hold_kp", joint_count);
  settings.hold_kd = loadGainVector(nh, ns + "hold_kd", joint_count);
  nh.param(ns + "teleport_jump_rad", settings.teleport_jump_rad, settings.teleport_jump_rad);
  if (settings.spinner_threads <= 0 || !std::isfinite(settings.loop_rate_hz) ||
      settings.loop_rate_hz <= 0.0 || !std::isfinite(settings.diagnostics_rate_hz) ||
      settings.diagnostics_rate_hz <= 0.0 ||
      settings.diagnostics_rate_hz > settings.loop_rate_hz ||
      !std::isfinite(settings.acceleration_timeout_sec) ||
      settings.acceleration_timeout_sec <= 0.0 || !std::isfinite(settings.joint_state_timeout_sec) ||
      settings.joint_state_timeout_sec <= 0.0) {
    throw std::runtime_error("Invalid torque loop timing parameters (" + keys.loop_rate_key +
                             ", " + keys.diagnostics_rate_key + ", robot/control/*)");
  }
  if (!std::isfinite(settings.teleport_jump_rad) || settings.teleport_jump_rad <= 0.0) {
    throw std::runtime_error(ns + "teleport_jump_rad must be finite and positive");
  }
  return settings;
}

TorqueLoop::TorqueLoop(ros::NodeHandle& nh, const std::string& limb_name,
                       std::unique_ptr<InverseDynamicsModel> model,
                       ComputedTorqueSettings law_settings, TorqueLoopSettings settings)
    : limb_name_(limb_name),
      settings_(std::move(settings)),
      model_(std::move(model)),
      law_(*model_, std::move(law_settings)),
      limb_(limb_name),
      diagnostics_publisher_(nh.advertise<std_msgs::Float64MultiArray>(
          "/robot/limb/" + limb_name + "/" + kDiagnosticsTopic, 2)),
      diagnostics_period_sec_(1.0 / settings_.diagnostics_rate_hz) {
  if (settings_.hold_kp.size() != model_->jointCount() ||
      settings_.hold_kd.size() != model_->jointCount()) {
    throw std::invalid_argument("hold gains must contain one value per model joint");
  }
  const ComputedTorqueSettings& law = law_.settings();
  ROS_INFO_STREAM("Torque loop for the " << limb_name << " arm: " << model_->description()
                                          << "; gravity compensation "
                                          << (law.add_gravity_compensation ? "on" : "off")
                                          << ", viscous damping compensation "
                                          << (law.compensate_viscous_damping ? "on" : "off")
                                          << ", inner PD kp/kd "
                                          << formatGainPairs(law.inner_pd_kp, law.inner_pd_kd)
                                          << ", hold kp/kd "
                                          << formatGainPairs(settings_.hold_kp, settings_.hold_kd));
  limb_.setCommandTimeout(settings_.acceleration_timeout_sec);
}

void TorqueLoop::run() {
  ros::AsyncSpinner spinner(settings_.spinner_threads);
  spinner.start();
  ros::Rate loop_rate(settings_.loop_rate_hz);
  SteadyClock::time_point previous_loop_start = SteadyClock::now();
  previous_diagnostics_time_ = previous_loop_start;
  while (ros::ok()) {
    const SteadyClock::time_point loop_start = SteadyClock::now();
    const double loop_dt_ms =
        std::chrono::duration<double, std::milli>(loop_start - previous_loop_start).count();
    previous_loop_start = loop_start;
    step(loop_start, loop_dt_ms);
    loop_rate.sleep();
  }
  if (torque_mode_engaged_) leaveTorqueMode();
}

void TorqueLoop::step(SteadyClock::time_point loop_start, double loop_dt_ms) {
  const std::size_t joint_count = model_->jointCount();
  const JointStateSnapshot state = limb_.jointStateSnapshot();
  const AccelerationReferenceSnapshot reference = limb_.accelerationReferenceSnapshot();
  const bool teleported = updateTeleportDetector(state.positions);

  if (state.positions.size() != joint_count || state.velocities.size() != joint_count) {
    ROS_WARN_THROTTLE(1.0, "Waiting for a complete joint state");
    return;
  }
  // A frozen joint state must not keep feeding torques to the robot: stop
  // commanding so the robot's command watchdog expires and hand back control.
  const double joint_state_age_sec =
      state.received_at.isZero() ? std::numeric_limits<double>::infinity()
                                 : (ros::WallTime::now() - state.received_at).toSec();
  if (!(joint_state_age_sec <= settings_.joint_state_timeout_sec)) {
    ROS_WARN_THROTTLE(1.0, "Joint state is stale (%.3f s); not commanding torque",
                      joint_state_age_sec);
    if (torque_mode_engaged_) leaveTorqueMode();
    holding_ = false;
    tracking_reference_ = false;
    return;
  }
  const bool reference_usable = reference.accelerations.size() == joint_count &&
                                reference.age_sec <= settings_.acceleration_timeout_sec;
  if (!reference_usable) {
    if (settings_.hold_when_idle) {
      holdPose(state, teleported);
    } else {
      ROS_WARN_THROTTLE(1.0, "Acceleration references are stale; exiting torque control");
      if (torque_mode_engaged_) leaveTorqueMode();
    }
    tracking_reference_ = false;
    return;
  }
  holding_ = false;
  trackReference(state, reference, loop_start, loop_dt_ms);
}

bool TorqueLoop::updateTeleportDetector(const JointMap& positions) {
  const std::size_t joint_count = model_->jointCount();
  if (positions.size() != joint_count) return false;
  bool teleported = false;
  if (previous_positions_.size() == joint_count) {
    for (const auto& [joint, position] : positions) {
      const auto previous = previous_positions_.find(joint);
      if (previous != previous_positions_.end() &&
          std::abs(position - previous->second) > settings_.teleport_jump_rad) {
        teleported = true;  // Physically impossible within one period: a simulator reset.
        break;
      }
    }
  }
  // Assign into the existing keys: no allocation once the map is populated.
  for (const auto& [joint, position] : positions) previous_positions_[joint] = position;
  return teleported;
}

void TorqueLoop::holdPose(const JointStateSnapshot& state, bool teleported) {
  // The anchor is captured when the hold engages or the arm teleports, and the
  // PD always pulls back to it (a target that followed the pose let the arm
  // coast away after the simulator's kicks).
  if (!holding_ || teleported) {
    hold_positions_ = state.positions;
    if (!holding_) {
      ROS_INFO_THROTTLE(5.0, "No acceleration reference; holding the current pose in torque mode.");
    }
    if (teleported) ROS_INFO("Joint positions jumped (teleport); holding the new pose.");
    holding_ = true;
  }
  if (settings_.hold_mode == "position") {
    // The simulator's own position controllers hold the matched target.
    limb_.setJointPositions(hold_positions_);
    torque_mode_engaged_ = false;
    return;
  }
  try {
    const ComputedTorqueResult hold = law_.hold(state.positions, state.velocities,
                                                hold_positions_, settings_.hold_kp,
                                                settings_.hold_kd);
    limb_.setJointTorques(hold.torques);
    torque_mode_engaged_ = true;
    logHoldError(state.positions, hold);
  } catch (const std::exception& exception) {
    ROS_ERROR_THROTTLE(1.0, "Hold torque calculation failed: %s", exception.what());
  }
}

void TorqueLoop::logHoldError(const JointMap& positions, const ComputedTorqueResult& hold) const {
  double max_anchor_error = 0.0;
  std::string worst_joint;
  for (const auto& [joint, position] : positions) {
    const double error = std::abs(hold_positions_.at(joint) - position);
    if (error >= max_anchor_error) {
      max_anchor_error = error;
      worst_joint = joint;
    }
  }
  ROS_INFO_THROTTLE(10.0, "Hold: max |anchor - q| %.3f rad on %s (torque %.1f Nm)",
                    max_anchor_error, worst_joint.c_str(), hold.torques.at(worst_joint));
}

void TorqueLoop::trackReference(const JointStateSnapshot& state,
                                const AccelerationReferenceSnapshot& reference,
                                SteadyClock::time_point loop_start, double loop_dt_ms) {
  try {
    const ComputedTorqueResult result =
        law_.track(state.positions, state.velocities, reference.accelerations,
                   reference.angles, reference.velocities, reference.age_sec);
    limb_.setJointTorques(result.torques);
    torque_mode_engaged_ = true;
    tracking_reference_ = true;
    ++sequence_;
    const SteadyClock::time_point compute_end = SteadyClock::now();
    if (std::chrono::duration<double>(compute_end - previous_diagnostics_time_).count() >=
        diagnostics_period_sec_) {
      previous_diagnostics_time_ = compute_end;
      publishDiagnostics(result, loop_dt_ms,
                         std::chrono::duration<double, std::milli>(compute_end - loop_start).count(),
                         reference.age_sec);
    }
  } catch (const std::exception& exception) {
    ROS_ERROR_THROTTLE(1.0, "Torque calculation failed: %s", exception.what());
    if (torque_mode_engaged_) leaveTorqueMode();
    tracking_reference_ = false;
  }
}

void TorqueLoop::publishDiagnostics(const ComputedTorqueResult& result, double loop_dt_ms,
                                    double compute_ms, double reference_age_sec) const {
  std_msgs::Float64MultiArray message;
  std_msgs::MultiArrayDimension dimension;
  dimension.label = "v1:";
  for (const std::string& joint : model_->jointNames()) dimension.label += joint + ",";
  dimension.size = kDiagnosticsMetadataCount + kDiagnosticsFieldCount * model_->jointCount();
  dimension.stride = dimension.size;
  message.layout.dim.push_back(dimension);
  message.data.reserve(dimension.size);
  message.data.push_back(static_cast<double>(sequence_));
  message.data.push_back(loop_dt_ms);
  message.data.push_back(compute_ms);
  message.data.push_back(reference_age_sec);
  message.data.push_back(tracking_reference_ ? 1.0 : 0.0);
  for (const std::vector<double>* field : diagnosticsFields(result)) {
    message.data.insert(message.data.end(), field->begin(), field->end());
  }
  diagnostics_publisher_.publish(message);
}

void TorqueLoop::leaveTorqueMode() {
  limb_.exitControlMode(settings_.acceleration_timeout_sec);
  torque_mode_engaged_ = false;
}

int runTorqueLoopNode(int argc, char** argv, const std::string& node_name,
                      const TorqueLoopParameterKeys& keys,
                      const std::function<std::unique_ptr<InverseDynamicsModel>(
                          ros::NodeHandle&, const std::string&)>& make_model) {
  // ros::init has stripped the remapping arguments: argv[1] is the limb side.
  if (argc < 2 || (std::string(argv[1]) != "left" && std::string(argv[1]) != "right")) {
    ROS_ERROR("Usage: %s <left|right>", node_name.c_str());
    return EXIT_FAILURE;
  }
  try {
    ros::NodeHandle nh;
    const std::string limb_name(argv[1]);
    std::unique_ptr<InverseDynamicsModel> model = make_model(nh, limb_name);
    const std::size_t joint_count = model->jointCount();
    ComputedTorqueSettings law_settings =
        ComputedTorqueSettings::load(nh, keys.settings_namespace, joint_count);
    TorqueLoopSettings loop_settings = TorqueLoopSettings::load(nh, keys, joint_count);
    TorqueLoop loop(nh, limb_name, std::move(model), std::move(law_settings),
                    std::move(loop_settings));
    loop.run();
  } catch (const std::exception& exception) {
    ROS_FATAL("%s startup failed: %s", node_name.c_str(), exception.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

}  // namespace baxter
