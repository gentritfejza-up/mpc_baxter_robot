// 800 Hz computed-torque loop for one Baxter arm.
//
// With a fresh MPC acceleration reference the loop commands
// tau = M(q) qdd + C + g + damping (KdlInverseDynamics::compute). Without one it
// holds the pose: in torque mode with a torque-space PD anchored where the hold
// engaged (re-anchored when a joint jumps, i.e. the simulator teleported the arm),
// or in position mode, or it leaves torque control, depending on the parameters.
#include "baxter_kdl_dynamics/kdl_inverse_dynamics.hpp"

#include <baxter_interface_cpp/limb.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

using baxter_kdl_dynamics::DynamicsResult;
using baxter_kdl_dynamics::KdlInverseDynamics;
using SteadyClock = std::chrono::steady_clock;
using JointMap = std::unordered_map<std::string, double>;

constexpr std::size_t kArmJointCount = 7;
/// Diagnostics layout "v1": sequence, loop dt [ms], compute time [ms],
/// reference age [s], torque-control flag, then kDiagnosticsFieldCount blocks of
/// one value per joint (the order of diagnosticsFields()).
constexpr std::size_t kDiagnosticsMetadataCount = 5;
constexpr std::size_t kDiagnosticsFieldCount = 14;

std::array<const std::vector<double>*, kDiagnosticsFieldCount> diagnosticsFields(
    const DynamicsResult& result) {
  return {&result.position,             &result.velocity,
          &result.reference_position,   &result.reference_velocity,
          &result.mpc_acceleration,     &result.pd_acceleration,
          &result.commanded_acceleration, &result.inertia_torque,
          &result.coriolis_torque,      &result.gravity_torque,
          &result.damping_torque,       &result.raw_torque,
          &result.commanded_torque,     &result.torque_limit};
}

/// Parameters of the loop (the dynamics model loads its own).
struct TorqueLoopSettings {
  int spinner_threads{2};
  double loop_rate_hz{800.0};
  double diagnostics_rate_hz{50.0};
  /// An MPC reference older than this is stale; also the joint-command timeout [s].
  double acceleration_timeout_sec{0.2};
  bool hold_when_idle{true};
  std::string hold_mode{"torque"};  ///< "torque" or "position".
  std::vector<double> hold_kp;      ///< Torque-space hold gain per joint [Nm/rad].
  std::vector<double> hold_kd;      ///< Torque-space hold gain per joint [Nm s/rad].
  /// A joint step between two loop iterations above this is a teleport [rad].
  double teleport_jump_rad{0.2};
};

TorqueLoopSettings loadSettings(ros::NodeHandle& nh) {
  TorqueLoopSettings settings;
  nh.param("robot/control/spinner_threads", settings.spinner_threads, settings.spinner_threads);
  nh.param("robot/control/kdl_dynamics_rate_hz", settings.loop_rate_hz, settings.loop_rate_hz);
  nh.param("robot/control/kdl_diagnostics_rate_hz", settings.diagnostics_rate_hz,
           settings.diagnostics_rate_hz);
  nh.param("robot/control/acceleration_ref_timeout_sec", settings.acceleration_timeout_sec,
           settings.acceleration_timeout_sec);
  nh.param("robot/kdl_dynamics/hold_when_idle", settings.hold_when_idle, settings.hold_when_idle);
  nh.param<std::string>("robot/kdl_dynamics/hold_mode", settings.hold_mode, settings.hold_mode);
  if (settings.hold_mode != "position" && settings.hold_mode != "torque") {
    throw std::runtime_error("robot/kdl_dynamics/hold_mode must be position or torque");
  }
  settings.hold_kp =
      KdlInverseDynamics::loadGainVector(nh, "robot/kdl_dynamics/hold_kp", kArmJointCount);
  settings.hold_kd =
      KdlInverseDynamics::loadGainVector(nh, "robot/kdl_dynamics/hold_kd", kArmJointCount);
  nh.param("robot/kdl_dynamics/teleport_jump_rad", settings.teleport_jump_rad,
           settings.teleport_jump_rad);
  if (settings.spinner_threads <= 0 || !std::isfinite(settings.loop_rate_hz) ||
      settings.loop_rate_hz <= 0.0 || !std::isfinite(settings.diagnostics_rate_hz) ||
      settings.diagnostics_rate_hz <= 0.0 ||
      settings.diagnostics_rate_hz > settings.loop_rate_hz ||
      !std::isfinite(settings.acceleration_timeout_sec) ||
      settings.acceleration_timeout_sec <= 0.0) {
    throw std::runtime_error("Invalid KDL dynamics control timing parameters");
  }
  if (!std::isfinite(settings.teleport_jump_rad) || settings.teleport_jump_rad <= 0.0) {
    throw std::runtime_error("robot/kdl_dynamics/teleport_jump_rad must be finite and positive");
  }
  return settings;
}

std::string formatHoldGains(const TorqueLoopSettings& settings) {
  std::ostringstream gains;
  for (std::size_t joint_index = 0; joint_index < settings.hold_kp.size(); ++joint_index) {
    gains << (joint_index ? ", " : "") << settings.hold_kp[joint_index] << "/"
          << settings.hold_kd[joint_index];
  }
  return gains.str();
}

/// The control loop and its state; runs in the main thread while an AsyncSpinner
/// serves the Limb subscribers.
class TorqueLoop {
 public:
  TorqueLoop(ros::NodeHandle& nh, const std::string& limb_name, TorqueLoopSettings settings)
      : limb_name_(limb_name),
        settings_(std::move(settings)),
        limb_(limb_name),
        dynamics_(nh, limb_name),
        diagnostics_publisher_(nh.advertise<std_msgs::Float64MultiArray>(
            "/robot/limb/" + limb_name + "/kdl_dynamics_diagnostics", 2)),
        diagnostics_period_sec_(1.0 / settings_.diagnostics_rate_hz) {
    ROS_INFO_STREAM("Idle hold gains kp/kd per joint (ns '" << nh.getNamespace()
                                                           << "'): " << formatHoldGains(settings_));
    limb_.setCommandTimeout(settings_.acceleration_timeout_sec);
  }

  /// Run until ROS shuts down, then return the arm to position mode if needed.
  void run() {
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

 private:
  void step(SteadyClock::time_point loop_start, double loop_dt_ms) {
    const JointStateSnapshot state = limb_.jointStateSnapshot();
    const AccelerationReferenceSnapshot reference = limb_.accelerationReferenceSnapshot();
    const bool teleported = updateTeleportDetector(state.positions);

    if (state.positions.size() != kArmJointCount || state.velocities.size() != kArmJointCount) {
      ROS_WARN_THROTTLE(1.0, "Waiting for a complete joint state");
      return;
    }
    const bool reference_usable = reference.accelerations.size() == kArmJointCount &&
                                  reference.age_sec <= settings_.acceleration_timeout_sec;
    if (!reference_usable) {
      // No (or a stale) MPC reference. Holding in torque mode moves the
      // simulator's position-to-torque hand-over (which can kick the arm) to the
      // enable instant, before the MPC commands anything.
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

  /**
   * @brief Compare the measured pose with the previous iteration's and remember it.
   * @return True when any joint moved more than teleport_jump_rad since the last
   *         complete joint state: physically impossible within one loop period, so
   *         the simulator reset or teleported the arm.
   */
  bool updateTeleportDetector(const JointMap& positions) {
    if (positions.size() != kArmJointCount) return false;
    bool teleported = false;
    if (previous_positions_.size() == kArmJointCount) {
      for (const auto& [joint, position] : positions) {
        const auto previous = previous_positions_.find(joint);
        if (previous != previous_positions_.end() &&
            std::abs(position - previous->second) > settings_.teleport_jump_rad) {
          teleported = true;
          break;
        }
      }
    }
    // Assign into the existing keys: no allocation once the map is populated.
    for (const auto& [joint, position] : positions) previous_positions_[joint] = position;
    return teleported;
  }

  /// Hold the anchor pose; the anchor is captured when the hold engages or the arm
  /// teleports, and the PD always pulls back to it (a target that followed the
  /// pose let the arm coast away after the simulator's kicks).
  void holdPose(const JointStateSnapshot& state, bool teleported) {
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
      const DynamicsResult hold =
          dynamics_.computeHold(state.positions, state.velocities, hold_positions_,
                                settings_.hold_kp, settings_.hold_kd);
      limb_.setJointTorques(hold.torques);
      torque_mode_engaged_ = true;
      logHoldError(state.positions, hold);
    } catch (const std::exception& exception) {
      ROS_ERROR_THROTTLE(1.0, "KDL hold torque calculation failed: %s", exception.what());
    }
  }

  void logHoldError(const JointMap& positions, const DynamicsResult& hold) const {
    double max_anchor_error = 0.0;
    for (const auto& [joint, position] : positions) {
      max_anchor_error = std::max(max_anchor_error, std::abs(hold_positions_.at(joint) - position));
    }
    const std::string s1 = limb_name_ + "_s1";
    ROS_INFO_THROTTLE(10.0, "Hold: max |anchor - q| %.3f rad, s1 anchor %.3f q %.3f torque %.1f Nm",
                      max_anchor_error, hold_positions_.at(s1), positions.at(s1),
                      hold.torques.at(s1));
  }

  void trackReference(const JointStateSnapshot& state,
                      const AccelerationReferenceSnapshot& reference,
                      SteadyClock::time_point loop_start, double loop_dt_ms) {
    try {
      const DynamicsResult result =
          dynamics_.compute(state.positions, state.velocities, reference.accelerations,
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
      ROS_ERROR_THROTTLE(1.0, "KDL torque calculation failed: %s", exception.what());
      if (torque_mode_engaged_) leaveTorqueMode();
      tracking_reference_ = false;
    }
  }

  void publishDiagnostics(const DynamicsResult& result, double loop_dt_ms, double compute_ms,
                          double reference_age_sec) const {
    std_msgs::Float64MultiArray message;
    std_msgs::MultiArrayDimension dimension;
    dimension.label = "v1:";
    for (const std::string& joint : dynamics_.jointNames()) dimension.label += joint + ",";
    dimension.size = kDiagnosticsMetadataCount + kDiagnosticsFieldCount * dynamics_.jointNames().size();
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

  /// Hand the joints back to position mode at the measured pose.
  void leaveTorqueMode() {
    limb_.exitControlMode(settings_.acceleration_timeout_sec);
    torque_mode_engaged_ = false;
  }

  const std::string limb_name_;
  const TorqueLoopSettings settings_;
  Limb limb_;
  KdlInverseDynamics dynamics_;
  ros::Publisher diagnostics_publisher_;
  const double diagnostics_period_sec_;

  bool tracking_reference_{false};   ///< Last iteration commanded MPC-tracking torque.
  bool torque_mode_engaged_{false};  ///< A torque command has been sent since the last exit.
  bool holding_{false};              ///< The idle hold is engaged and anchored.
  JointMap hold_positions_;          ///< Hold anchor [rad].
  JointMap previous_positions_;      ///< Pose of the previous iteration [rad].
  uint64_t sequence_{0};             ///< Number of MPC-tracking torque commands sent.
  SteadyClock::time_point previous_diagnostics_time_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "kdl_inverse_dynamics_node");
  if (argc < 2 || (std::string(argv[1]) != "left" && std::string(argv[1]) != "right")) {
    ROS_ERROR("Usage: run_kdl_inverse_dynamics <left|right>");
    return EXIT_FAILURE;
  }

  try {
    ros::NodeHandle nh;
    const std::string limb_name(argv[1]);
    TorqueLoop loop(nh, limb_name, loadSettings(nh));
    loop.run();
  } catch (const std::exception& exception) {
    ROS_FATAL("KDL inverse dynamics startup failed: %s", exception.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
