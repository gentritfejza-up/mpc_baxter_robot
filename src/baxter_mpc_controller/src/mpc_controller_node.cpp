#include "mpc_controller/mpc_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/// Margin kept from the joint limits when projecting a reference [rad]: a set point on a
/// soft bound pulls against it forever and chatters there.
constexpr double kReferenceJointLimitMargin = 0.02;
/// Gauss-Newton iterations and extra clearance [m] of the reference projection.
constexpr int kReferenceProjectionIterations = 8;
constexpr double kReferenceProjectionClearance = 0.005;
/// Reference shifts above this are reported [rad].
constexpr double kReportedReferenceShift = 0.01;

double elapsedMs(const std::chrono::steady_clock::time_point& start,
                 const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

mpc_geometry::JointVector toJointVector(const std::vector<double>& values) {
  mpc_geometry::JointVector joint_vector{};
  for (int joint = 0; joint < mpc_geometry::kArmJoints && joint < static_cast<int>(values.size());
       ++joint) {
    joint_vector[joint] = values[joint];
  }
  return joint_vector;
}

}  // namespace

MPCControlManager::MPCControlManager(ros::NodeHandle& node_handle, const std::string& limb_side)
    : node_handle_(node_handle),
      limb_side_(limb_side),
      limb_(std::make_unique<Limb>(limb_side)),
      joint_names_(limb_->jointNames()) {
  loadAndValidateConfiguration();
  previous_applied_accelerations_.assign(num_joints_, 0.0);
  node_handle_.param("robot/control/require_enabled", require_enabled_, require_enabled_);
  node_handle_.param("robot/control/first_solve_max_retries", first_solve_max_retries_,
                     first_solve_max_retries_);
  node_handle_.param("robot/control/settle_velocity_rad_s", settle_velocity_rad_s_,
                     settle_velocity_rad_s_);
  node_handle_.param("robot/control/settle_hold_sec", settle_hold_sec_, settle_hold_sec_);
  node_handle_.param("robot/control/enable_delay_sec", enable_delay_sec_, enable_delay_sec_);
  node_handle_.param("robot/control/overspeed_ratio", overspeed_ratio_, overspeed_ratio_);
  node_handle_.param("robot/geometry/project_reference", project_reference_, project_reference_);
  assembly_state_subscriber_ =
      node_handle_.subscribe("/robot/state", 1, &MPCControlManager::onAssemblyState, this);
  initializeGeometry();
  mpc_controller_ = std::make_unique<MPCController>(
      node_handle_, num_joints_, prediction_horizon_, control_horizon_, makePredictionGrid(),
      control_period_sec_, geometry_enabled_ ? collision_model_->sphereCount() : 0);
  if (geometry_enabled_) {
    linearizer_ = std::make_unique<mpc_geometry::GeometryRowLinearizer>(
        *collision_model_, limb_side_, geometry_params_, mpc_controller_->geometryLayout());
    linearizer_->setFloorEnabled(floor_armed_);
    geometry_batch_.reset(mpc_controller_->geometryLayout());
    projection_lower_bounds_ = mpc_controller_->positionLowerBounds();
    projection_upper_bounds_ = mpc_controller_->positionUpperBounds();
    for (double& bound : projection_lower_bounds_) bound += kReferenceJointLimitMargin;
    for (double& bound : projection_upper_bounds_) bound -= kReferenceJointLimitMargin;
    initializeGeometryPublishers();
    ROS_INFO_STREAM("MPC collision guard on: " << collision_model_->sphereCount()
                    << " spheres/arm, " << collision_model_->bodyObstacles().size()
                    << " body obstacles, z_min=" << geometry_params_.z_min << " m, guard="
                    << guard_mode_ << ", rows=" << mpc_controller_->geometryLayout().rowCount());
  }

  ROS_INFO_STREAM("MPC runtime rate=" << control_rate_hz_ << " Hz period="
                                      << control_period_sec_ * 1000.0 << " ms prediction="
                                      << prediction_duration_sec_ << " s");
}

void MPCControlManager::loadAndValidateConfiguration() {
  node_handle_.param("robot/control/mpc_rate_hz", control_rate_hz_, 50.0);
  node_handle_.param("robot/control/prediction_duration_sec", prediction_duration_sec_, 2.0);
  node_handle_.param("robot/control/prediction_horizon", prediction_horizon_, 20);
  node_handle_.param("robot/control/control_horizon", control_horizon_, 15);
  node_handle_.param("robot/control/mpc_spinner_threads", spinner_threads_, 10);
  if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
    throw std::invalid_argument("robot/control/mpc_rate_hz must be finite and positive");
  }
  control_period_sec_ = 1.0 / control_rate_hz_;
  node_handle_.param("robot/control/state_timeout_sec", state_timeout_sec_,
                     2.0 * control_period_sec_);
  double cycle_deadline_ms = control_period_sec_ * 1000.0;
  node_handle_.param("robot/control/cycle_deadline_ms", cycle_deadline_ms, cycle_deadline_ms);
  cycle_deadline_sec_ = cycle_deadline_ms / 1000.0;
  node_handle_.param("robot/control/enforce_cycle_deadline", enforce_cycle_deadline_, true);
  node_handle_.param("robot/control/braking_fallback", braking_fallback_enabled_, true);
  node_handle_.param("robot/control/publish_on_deadline_miss", publish_on_deadline_miss_,
                     publish_on_deadline_miss_);
  node_handle_.param("robot/control/max_latency_compensation_sec", max_latency_compensation_sec_,
                     max_latency_compensation_sec_);
  if (!node_handle_.getParam("robot/acceleration_max", acceleration_max_) ||
      !node_handle_.getParam("robot/acceleration_min", acceleration_min_) ||
      acceleration_max_.size() != static_cast<size_t>(num_joints_) ||
      acceleration_min_.size() != static_cast<size_t>(num_joints_)) {
    throw std::invalid_argument("robot/acceleration_{min,max} must hold one value per joint");
  }
  if (joint_names_.size() != static_cast<size_t>(num_joints_)) {
    throw std::invalid_argument("The limb does not report the expected number of joints");
  }

  if (prediction_horizon_ < 2 || control_horizon_ <= 0 ||
      control_horizon_ > prediction_horizon_ || spinner_threads_ <= 0 ||
      !std::isfinite(prediction_duration_sec_) ||
      prediction_duration_sec_ <= control_period_sec_ || !std::isfinite(state_timeout_sec_) ||
      state_timeout_sec_ <= 0.0 || !std::isfinite(cycle_deadline_sec_) ||
      cycle_deadline_sec_ <= 0.0) {
    throw std::invalid_argument("Invalid MPC horizon, timing, or thread configuration");
  }
}

void MPCControlManager::initializeGeometry() {
  node_handle_.param("robot/geometry/enabled", geometry_enabled_, false);
  if (!geometry_enabled_) return;
  if (num_joints_ != mpc_geometry::kArmJoints) {
    throw std::invalid_argument("The collision guard expects a 7-joint arm");
  }
  node_handle_.param<std::string>("robot/geometry/guard/mode", guard_mode_, guard_mode_);
  if (guard_mode_ != "reject" && guard_mode_ != "warn" && guard_mode_ != "off") {
    throw std::invalid_argument("robot/geometry/guard/mode must be reject, warn or off");
  }
  node_handle_.param("robot/geometry/other_arm/required", other_arm_required_,
                     other_arm_required_);
  node_handle_.param("robot/geometry/other_arm/timeout_sec", other_arm_timeout_sec_,
                     other_arm_timeout_sec_);
  node_handle_.param("robot/geometry/other_arm/extrapolation_sec", other_arm_extrapolation_sec_,
                     other_arm_extrapolation_sec_);
  if (!std::isfinite(other_arm_extrapolation_sec_) || other_arm_extrapolation_sec_ < 0.0) {
    throw std::invalid_argument("robot/geometry/other_arm/extrapolation_sec must be >= 0");
  }
  if (!std::isfinite(other_arm_timeout_sec_) || other_arm_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("robot/geometry/other_arm/timeout_sec must be positive");
  }
  node_handle_.param("robot/geometry/floor/activate_at_home", floor_activate_at_home_,
                     floor_activate_at_home_);
  if (floor_activate_at_home_) {
    node_handle_.param("robot/geometry/floor/home_tolerance_rad", home_tolerance_rad_,
                       home_tolerance_rad_);
    std::vector<double> home_position;
    node_handle_.getParam("robot/geometry/floor/home_position/" + limb_side_, home_position);
    if (home_position.size() != static_cast<std::size_t>(num_joints_) ||
        !std::isfinite(home_tolerance_rad_) || home_tolerance_rad_ <= 0.0) {
      throw std::invalid_argument("robot/geometry/floor/home_position/" + limb_side_ +
                                  " needs 7 joint angles and home_tolerance_rad must be positive");
    }
    home_position_ = toJointVector(home_position);
    floor_armed_ = false;
  }
  geometry_params_ = mpc_geometry::GeometryParams::fromParams(node_handle_);
  collision_model_ = mpc_geometry::ArmCollisionModel::fromParams(node_handle_);
  other_arm_listener_ = std::make_unique<mpc_geometry::OtherArmStateListener>(
      node_handle_, limb_side_ == "left" ? "right" : "left");
  linearisation_points_.assign(prediction_horizon_, mpc_geometry::JointVector{});
}

void MPCControlManager::initializeGeometryPublishers() {
  geometry_diagnostics_publisher_ = node_handle_.advertise<std_msgs::Float64MultiArray>(
      "/robot/limb/" + limb_side_ + "/mpc_geometry_diagnostics", 2);
  marker_publisher_ = std::make_unique<mpc_geometry::GeometryMarkerPublisher>(
      node_handle_, "/robot/limb/" + limb_side_ + "/collision_guard_markers",
      collision_model_->baseLink());
  mpc_geometry::ConveyorVisual conveyor;
  node_handle_.param("robot/geometry/conveyor/enabled", conveyor.enabled, conveyor.enabled);
  node_handle_.param("robot/geometry/conveyor/x", conveyor.x, conveyor.x);
  node_handle_.param("robot/geometry/conveyor/y", conveyor.y, conveyor.y);
  node_handle_.param("robot/geometry/conveyor/yaw", conveyor.yaw, conveyor.yaw);
  node_handle_.param("robot/geometry/conveyor/length", conveyor.length, conveyor.length);
  node_handle_.param("robot/geometry/conveyor/width", conveyor.width, conveyor.width);
  node_handle_.param("robot/geometry/conveyor/belt_width", conveyor.belt_width,
                     conveyor.belt_width);
  node_handle_.param("robot/geometry/conveyor/height", conveyor.height, conveyor.height);
  marker_publisher_->setConveyor(conveyor);
}

std::vector<double> MPCControlManager::makePredictionGrid() const {
  std::vector<double> durations(prediction_horizon_, control_period_sec_);
  const double remaining_duration = prediction_duration_sec_ - control_period_sec_;
  const double later_step = remaining_duration / static_cast<double>(prediction_horizon_ - 1);
  std::fill(durations.begin() + 1, durations.end(), later_step);
  return durations;
}

void MPCControlManager::onAssemblyState(const baxter_core_msgs::AssemblyState::ConstPtr& message) {
  robot_enabled_.store(message->enabled && !message->stopped && !message->error);
}

void MPCControlManager::discardPlan() {
  mpc_controller_->resetWarmStart();
  has_linearisation_trajectory_ = false;
}

bool MPCControlManager::readMeasuredState(MeasuredState& state) const {
  const JointStateSnapshot snapshot = limb_->jointStateSnapshot();
  if (snapshot.received_at.isZero()) {
    ROS_WARN_THROTTLE(1.0, "Joint state has not been received; suppressing MPC command.");
    return false;
  }
  const double state_age = (ros::WallTime::now() - snapshot.received_at).toSec();
  if (!std::isfinite(state_age) || state_age < 0.0 || state_age > state_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "Joint state is stale (%.3f s); suppressing MPC command.", state_age);
    return false;
  }
  state.stamp = snapshot.stamp;
  state.positions.clear();
  state.velocities.clear();
  state.positions.reserve(num_joints_);
  state.velocities.reserve(num_joints_);
  for (const std::string& joint_name : joint_names_) {
    const auto position = snapshot.positions.find(joint_name);
    const auto velocity = snapshot.velocities.find(joint_name);
    if (position == snapshot.positions.end() || velocity == snapshot.velocities.end() ||
        !std::isfinite(position->second) || !std::isfinite(velocity->second)) {
      ROS_WARN_THROTTLE(1.0, "Joint state is incomplete or nonfinite; suppressing MPC command.");
      return false;
    }
    state.positions.push_back(position->second);
    state.velocities.push_back(velocity->second);
  }
  return true;
}

bool MPCControlManager::passEnableAndSettleGates(const std::vector<double>& velocities) {
  if (require_enabled_ && !robot_enabled_.load()) {
    // Idle until the robot is enabled; forget any plan built meanwhile.
    if (was_enabled_) ROS_WARN("Robot disabled; MPC idling until it is enabled again.");
    was_enabled_ = false;
    discardPlan();
    std::fill(previous_applied_accelerations_.begin(), previous_applied_accelerations_.end(), 0.0);
    hold_reference_.clear();
    first_solve_converged_ = false;
    settled_after_enable_ = false;
    quiet_timer_running_ = false;
    enabled_at_ = ros::WallTime();
    ROS_INFO_THROTTLE(5.0, "Waiting for /robot/state enabled before commanding.");
    return false;
  }
  if (!settled_after_enable_ && !quiet_timer_running_ && enabled_at_.isZero()) {
    enabled_at_ = ros::WallTime::now();
  }
  if (!settled_after_enable_ && (ros::WallTime::now() - enabled_at_).toSec() < enable_delay_sec_) {
    discardPlan();
    hold_reference_.clear();
    return false;  // e.g. the simulator still places the arms at home
  }
  if (!settled_after_enable_) {
    // The simulator's position-to-torque hand-over can kick the arm 50-100 ms after it
    // reports enabled; do not take over until the joints have been quiet for
    // settle_hold_sec (the MPC would otherwise start outside its velocity bounds, stall
    // at the iteration cap and command bang-bang).
    double max_speed = 0.0;
    for (double velocity : velocities) max_speed = std::max(max_speed, std::abs(velocity));
    const ros::WallTime now = ros::WallTime::now();
    if (max_speed > settle_velocity_rad_s_) {
      quiet_timer_running_ = false;
      ROS_WARN_THROTTLE(1.0, "Arm moving at %.2f rad/s after enabling; waiting for it to come to rest.",
                        max_speed);
    } else if (!quiet_timer_running_) {
      quiet_timer_running_ = true;
      quiet_since_ = now;
    }
    if (!quiet_timer_running_ || (now - quiet_since_).toSec() < settle_hold_sec_) {
      discardPlan();
      std::fill(previous_applied_accelerations_.begin(), previous_applied_accelerations_.end(), 0.0);
      hold_reference_.clear();
      first_solve_converged_ = false;
      return false;
    }
    settled_after_enable_ = true;
  }
  if (!was_enabled_) {
    was_enabled_ = true;
    ROS_INFO("Robot enabled; MPC commanding from the current pose.");
  }
  return true;
}

bool MPCControlManager::handleOverspeedRecovery(const MeasuredState& state) {
  if (overspeed_ratio_ <= 0.0) return false;
  const std::vector<double>& velocity_lower = mpc_controller_->velocityLowerBounds();
  const std::vector<double>& velocity_upper = mpc_controller_->velocityUpperBounds();
  bool overspeed = false;
  bool all_within_bounds = true;
  for (int joint = 0; joint < num_joints_ && joint < static_cast<int>(velocity_upper.size()); ++joint) {
    const double bound = std::max(std::abs(velocity_lower[joint]), std::abs(velocity_upper[joint]));
    const double speed = std::abs(state.velocities[joint]);
    if (speed > overspeed_ratio_ * bound) overspeed = true;
    if (speed > bound) all_within_bounds = false;
  }
  if (overspeed || (recovering_ && !all_within_bounds)) {
    if (!recovering_) {
      ROS_WARN("Joint velocity beyond the MPC bounds; braking until the arm is back in range.");
    }
    recovering_ = true;
    discardPlan();
    first_solve_converged_ = false;  // converge the first solve after the recovery too
    if (braking_fallback_enabled_) publishBrakingFallback(state, state.positions);
    return true;
  }
  if (recovering_) {
    recovering_ = false;
    ROS_INFO("Joint velocities back within the MPC bounds; resuming.");
  }
  return false;
}

bool MPCControlManager::selectReference(const std::vector<double>& reference_in,
                                        const std::vector<double>& positions,
                                        std::vector<double>& reference_out) {
  // Until a client sets a reference, hold the pose measured at the first cycle (a fixed
  // set point with a restoring force; a reference that followed the measured pose would
  // have none). The reconfigure defaults are all zeros, which would otherwise swing the
  // arm straight out at start-up.
  const bool external_reference = mpc_controller_->hasExternalReference();
  if (external_reference && !had_external_reference_) {
    had_external_reference_ = true;
    ROS_INFO("First external MPC reference received; leaving the start-up hold.");
  }
  if (!external_reference && hold_reference_.empty()) {
    hold_reference_ = positions;
    ROS_INFO("No MPC reference yet: holding the start-up pose.");
  }
  reference_out = external_reference ? reference_in : hold_reference_;
  if (reference_out.size() != static_cast<size_t>(num_joints_) ||
      !std::all_of(reference_out.begin(), reference_out.end(),
                   [](double value) { return std::isfinite(value); })) {
    ROS_ERROR_THROTTLE(1.0, "MPC reference is incomplete or nonfinite; suppressing command.");
    return false;
  }
  return true;
}

void MPCControlManager::buildOtherArmTrajectory(const mpc_geometry::JointVector& other_positions,
                                                const mpc_geometry::JointVector& other_velocities) {
  // Constant-velocity extrapolation over the near horizon: a static other arm let two
  // arms closing at ~1 m/s each eat a 0.10 m margin within one cycle.
  other_arm_trajectory_.resize(static_cast<std::size_t>(prediction_horizon_ + 1));
  const mpc_geometry::GeometryLayout& layout = linearizer_->layout();
  for (int step = 0; step <= prediction_horizon_; ++step) {
    const double extrapolation_time =
        std::min(step == 0 ? 0.0 : layout.timeAt(step), other_arm_extrapolation_sec_);
    for (int joint = 0; joint < num_joints_; ++joint) {
      other_arm_trajectory_[step][joint] =
          other_positions[joint] + other_velocities[joint] * extrapolation_time;
    }
  }
}

void MPCControlManager::updateFloorArming(const std::vector<double>& positions) {
  if (floor_armed_) return;
  double deviation = 0.0;
  for (int joint = 0; joint < num_joints_; ++joint) {
    deviation = std::max(deviation, std::abs(positions[joint] - home_position_[joint]));
  }
  if (deviation <= home_tolerance_rad_) {
    floor_armed_ = true;
    linearizer_->setFloorEnabled(true);
    ROS_INFO("Home position reached (max deviation %.3f rad): floor z >= %.2f m is now enforced.",
             deviation, geometry_params_.z_min);
  }
}

void MPCControlManager::projectReference(std::vector<double>& reference_positions) {
  const mpc_geometry::JointVector projected = linearizer_->projectReference(
      toJointVector(reference_positions), other_arm_trajectory_, &last_reference_shift_,
      kReferenceProjectionIterations, kReferenceProjectionClearance, &projection_lower_bounds_,
      &projection_upper_bounds_);
  for (int joint = 0; joint < num_joints_; ++joint) reference_positions[joint] = projected[joint];
  if (last_reference_shift_ > kReportedReferenceShift) {
    ROS_INFO_THROTTLE(2.0, "Reference inside a keep-out: tracking it moved out by up to %.3f rad.",
                      last_reference_shift_);
  }
}

void MPCControlManager::buildLinearisationPoints(const std::vector<double>& positions) {
  if (has_linearisation_trajectory_) {
    // Shift last cycle's prediction by one step; repeat the last point.
    for (int step = 0; step + 1 < prediction_horizon_; ++step) {
      linearisation_points_[step] = linearisation_points_[step + 1];
    }
    return;
  }
  std::fill(linearisation_points_.begin(), linearisation_points_.end(), toJointVector(positions));
}

bool MPCControlManager::prepareGeometry(const MeasuredState& state,
                                        std::vector<double>& reference_positions) {
  mpc_geometry::JointVector other_positions{};
  mpc_geometry::JointVector other_velocities{};
  other_arm_trajectory_.clear();
  double other_arm_age_sec = 0.0;
  if (other_arm_listener_->latest(other_positions, other_velocities, other_arm_timeout_sec_,
                                  &other_arm_age_sec)) {
    buildOtherArmTrajectory(other_positions, other_velocities);
  } else if (other_arm_required_) {
    ++other_arm_stale_cycles_;
    ROS_WARN_THROTTLE(1.0, "Other arm state missing or stale (%.3f s); braking.", other_arm_age_sec);
    if (braking_fallback_enabled_) publishBrakingFallback(state, reference_positions);
    has_linearisation_trajectory_ = false;
    return false;
  }
  updateFloorArming(state.positions);
  if (project_reference_) projectReference(reference_positions);
  const auto linearize_start = std::chrono::steady_clock::now();
  buildLinearisationPoints(state.positions);
  linearizer_->linearize(linearisation_points_, other_arm_trajectory_, geometry_batch_);
  last_linearize_ms_ = elapsedMs(linearize_start, std::chrono::steady_clock::now());
  return true;
}

casadi::DM MPCControlManager::solveWithRetries(const MpcSolveInput& input) {
  casadi::DM solution = mpc_controller_->solve(input);
  if (mpc_controller_->lastSolveInaccurate() && !last_solve_was_inaccurate_) {
    // First stall after converged solves: a stale dual estimate can keep every
    // warm-started solve at the cap, so the next cycle starts from zero duals (a
    // re-solve now would double this cycle's time).
    mpc_controller_->resetDualWarmStart();
    ++dual_resets_;
    ROS_WARN_THROTTLE(2.0,
                      "Solve stalled after converged solves; next cycle starts from zero duals (%llu so far).",
                      static_cast<unsigned long long>(dual_resets_));
  }
  if (!first_solve_converged_) {
    int retries = 0;
    while (mpc_controller_->lastSolveInaccurate() && retries < first_solve_max_retries_) {
      solution = mpc_controller_->solve(input);  // continues from the previous iterate
      ++retries;
    }
    first_solve_converged_ = !mpc_controller_->lastSolveInaccurate();
    if (retries > 0) {
      ROS_INFO("First solve needed %d extra passes (%s).", retries,
               first_solve_converged_ ? "converged" : "still at the cap");
    }
  }
  last_solve_was_inaccurate_ = mpc_controller_->lastSolveInaccurate();
  return solution;
}

bool MPCControlManager::runCollisionGuard(const casadi::DM& solution, const MeasuredState& state,
                                          const std::vector<double>& reference_positions) {
  const auto guard_start = std::chrono::steady_clock::now();
  const std::vector<mpc_geometry::JointVector> predicted =
      mpc_controller_->predictedPositions(solution);
  const mpc_geometry::JointVector measured = toJointVector(state.positions);
  last_guard_report_ = linearizer_->evaluateTrajectory(measured, predicted, other_arm_trajectory_);
  last_max_geometry_slack_ = mpc_controller_->maxGeometrySlack(solution);
  last_guard_ms_ = elapsedMs(guard_start, std::chrono::steady_clock::now());
  ++geometry_sequence_;
  const mpc_geometry::JointVector* other_arm_now =
      other_arm_trajectory_.empty() ? nullptr : &other_arm_trajectory_[0];
  marker_publisher_->publish(*collision_model_, limb_side_, measured, other_arm_now,
                             geometry_params_.z_min, floor_armed_, last_guard_report_);
  // Whether accepted or rejected, the plan becomes next cycle's linearisation trajectory.
  // For a rejected plan the rows written around it are what pushes the next plan out of
  // the obstacle; linearising at the measured state again would leave every row inactive
  // (nothing is near an obstacle there) and reproduce the same rejected plan every cycle.
  linearisation_points_ = predicted;
  has_linearisation_trajectory_ = true;
  if (last_guard_report_.violated && guard_mode_ != "off") {
    if (guard_mode_ == "reject") {
      ++guard_rejections_;
      ROS_WARN_THROTTLE(1.0,
                        "Collision guard rejected the plan: %.3f m from %s (sphere %d, step %d); braking.",
                        last_guard_report_.worst_distance, last_guard_report_.worst_obstacle.c_str(),
                        last_guard_report_.worst_sphere, last_guard_report_.worst_step);
      mpc_controller_->commitSolution();  // the rejected plan stays the warm start
      publishGeometryDiagnostics();
      if (braking_fallback_enabled_) publishBrakingFallback(state, reference_positions);
      return false;
    }
    ROS_WARN_THROTTLE(1.0, "Collision guard warning: %.3f m from %s (sphere %d, step %d).",
                      last_guard_report_.worst_distance, last_guard_report_.worst_obstacle.c_str(),
                      last_guard_report_.worst_sphere, last_guard_report_.worst_step);
  }
  publishGeometryDiagnostics();
  return true;
}

void MPCControlManager::publishGeometryDiagnostics() {
  if (!geometry_enabled_) return;
  std_msgs::Float64MultiArray message;
  message.data = {static_cast<double>(geometry_sequence_),
                  last_guard_report_.min_distance[mpc_geometry::kFloorGroup],
                  last_guard_report_.min_distance[mpc_geometry::kBodyGroup],
                  last_guard_report_.min_distance[mpc_geometry::kArmGroup],
                  last_guard_report_.min_distance_now[mpc_geometry::kFloorGroup],
                  last_guard_report_.min_distance_now[mpc_geometry::kBodyGroup],
                  last_guard_report_.min_distance_now[mpc_geometry::kArmGroup],
                  static_cast<double>(geometry_batch_.active_rows),
                  last_max_geometry_slack_,
                  static_cast<double>(guard_rejections_),
                  last_linearize_ms_,
                  last_guard_ms_};
  geometry_diagnostics_publisher_.publish(message);
}

bool MPCControlManager::processControl(const std::vector<double>& reference_positions_in,
                                       const std::chrono::steady_clock::time_point& cycle_start) {
  MeasuredState state;
  if (!readMeasuredState(state)) return false;
  if (!passEnableAndSettleGates(state.velocities)) return false;
  if (handleOverspeedRecovery(state)) return false;
  std::vector<double> reference_positions;
  if (!selectReference(reference_positions_in, state.positions, reference_positions)) return false;

  const auto solve_start = std::chrono::steady_clock::now();
  last_state_preparation_ms_ = elapsedMs(cycle_start, solve_start);

  double first_jerk_dt = control_period_sec_;
  if (has_last_command_time_) {
    first_jerk_dt = (ros::Time::now() - last_command_time_).toSec();
    first_jerk_dt = std::max(1e-4, std::min(first_jerk_dt, control_period_sec_));
  }

  if (geometry_enabled_ && !prepareGeometry(state, reference_positions)) return false;

  try {
    MpcSolveInput input;
    input.positions = casadi::DM(state.positions);
    input.velocities = casadi::DM(state.velocities);
    input.reference_positions = casadi::DM(reference_positions);
    input.previous_accelerations = casadi::DM(previous_applied_accelerations_);
    input.first_jerk_dt = first_jerk_dt;
    input.geometry = geometry_enabled_ ? &geometry_batch_ : nullptr;
    const casadi::DM solution = solveWithRetries(input);
    if (geometry_enabled_ && !runCollisionGuard(solution, state, reference_positions)) {
      return false;
    }

    const casadi::DM first_acceleration = mpc_controller_->getFirstAcceleration(solution);
    std::vector<double> acceleration_command(num_joints_);
    for (int joint = 0; joint < num_joints_; ++joint) {
      acceleration_command[joint] = static_cast<double>(first_acceleration(joint));
    }
    const auto publish_time = std::chrono::steady_clock::now();
    const MpcSolveTiming timing = mpc_controller_->getLastSolveTiming();
    last_postprocessing_ms_ =
        std::max(0.0, elapsedMs(solve_start, publish_time) - timing.argument_preparation_ms -
                          timing.solver_ms - timing.validation_ms);
    const double cycle_elapsed_sec =
        std::chrono::duration<double>(publish_time - cycle_start).count();
    if (enforce_cycle_deadline_ && cycle_elapsed_sec > cycle_deadline_sec_) {
      ++deadline_misses_;
      if (!publish_on_deadline_miss_) {
        // Keep the validated solution only as a numerical seed. No command or
        // applied-acceleration state is updated on a missed deadline.
        mpc_controller_->commitSolution();
        ROS_ERROR_THROTTLE(1.0, "MPC missed %.2f ms deadline (%.2f ms); suppressing command.",
                           cycle_deadline_sec_ * 1000.0, cycle_elapsed_sec * 1000.0);
        return false;
      }
      ROS_WARN_THROTTLE(1.0, "MPC missed %.2f ms deadline (%.2f ms); commanding late.",
                        cycle_deadline_sec_ * 1000.0, cycle_elapsed_sec * 1000.0);
    }

    publishCommand(acceleration_command, state, reference_positions);
    mpc_controller_->commitSolution();
    ++successful_cycles_;
    return true;
  } catch (const std::exception& exception) {
    ++failed_cycles_;
    has_linearisation_trajectory_ = false;
    ROS_ERROR_THROTTLE(1.0, "MPC solve failed: %s", exception.what());
    if (braking_fallback_enabled_) publishBrakingFallback(state, reference_positions);
    return false;
  }
}

void MPCControlManager::publishCommand(const std::vector<double>& accelerations,
                                       const MeasuredState& state,
                                       const std::vector<double>& reference_positions) {
  // The torque loop integrates q_ref = q0 + v0 t + a t^2/2 from the moment it receives
  // the command, so (q0, v0) must describe the plant *now*, not at the time the state
  // was sampled: propagate the sample through the solve latency with the acceleration
  // that was being applied meanwhile.
  double latency = state.stamp.isZero() ? 0.0 : (ros::Time::now() - state.stamp).toSec();
  if (!std::isfinite(latency)) latency = 0.0;
  latency = std::clamp(latency, 0.0, max_latency_compensation_sec_);
  std::vector<double> base_positions(num_joints_);
  std::vector<double> base_velocities(num_joints_);
  for (int joint = 0; joint < num_joints_; ++joint) {
    const double previous_acceleration = previous_applied_accelerations_[joint];
    base_positions[joint] = state.positions[joint] + state.velocities[joint] * latency +
                            0.5 * previous_acceleration * latency * latency;
    base_velocities[joint] = state.velocities[joint] + previous_acceleration * latency;
  }
  limb_->setReferentAngles(MPCController::dmToMap(casadi::DM(reference_positions), joint_names_));
  limb_->setJointAccelerations(MPCController::dmToMap(casadi::DM(accelerations), joint_names_),
                               MPCController::dmToMap(casadi::DM(base_velocities), joint_names_),
                               MPCController::dmToMap(casadi::DM(base_positions), joint_names_));
  previous_applied_accelerations_ = accelerations;
  last_command_time_ = ros::Time::now();
  has_last_command_time_ = true;
}

void MPCControlManager::publishBrakingFallback(const MeasuredState& state,
                                               const std::vector<double>& reference_positions) {
  std::vector<double> braking(num_joints_, 0.0);
  for (int joint = 0; joint < num_joints_; ++joint) {
    // Bring the joint to rest within one control period if the limits allow.
    const double rest_acceleration = -state.velocities[joint] / control_period_sec_;
    braking[joint] =
        std::clamp(rest_acceleration, acceleration_min_[joint], acceleration_max_[joint]);
  }
  ++fallback_cycles_;
  ROS_WARN_THROTTLE(1.0, "MPC braking fallback active (%llu cycles).",
                    static_cast<unsigned long long>(fallback_cycles_));
  publishCommand(braking, state, reference_positions);
}

void MPCControlManager::run() {
  ros::AsyncSpinner spinner(spinner_threads_);
  spinner.start();
  // ros::Rate follows /clock, so the model step matches simulated time in Gazebo.
  ros::Rate rate(control_rate_hz_);
  while (ros::ok()) {
    const auto cycle_start = std::chrono::steady_clock::now();
    processControl(mpc_controller_->getReferencePositions(), cycle_start);
    const auto cycle_end = std::chrono::steady_clock::now();
    const MpcSolveTiming timing = mpc_controller_->getLastSolveTiming();
    const double min_distance =
        geometry_enabled_
            ? std::min({last_guard_report_.min_distance[mpc_geometry::kFloorGroup],
                        last_guard_report_.min_distance[mpc_geometry::kBodyGroup],
                        last_guard_report_.min_distance[mpc_geometry::kArmGroup]})
            : std::numeric_limits<double>::infinity();
    ROS_INFO_THROTTLE(2.0,
                      "MPC backend=%s cycle=%.2f ms state=%.2f ms args=%.2f ms solve=%.2f ms "
                      "validate=%.2f ms post=%.2f ms ok=%llu fail=%llu missed=%llu fallback=%llu "
                      "geo=%d rows dmin=%.3f guard_rej=%llu lin=%.2f ms",
                      mpc_controller_->solverBackendName().c_str(),
                      elapsedMs(cycle_start, cycle_end), last_state_preparation_ms_,
                      timing.argument_preparation_ms, timing.solver_ms, timing.validation_ms,
                      last_postprocessing_ms_, static_cast<unsigned long long>(successful_cycles_),
                      static_cast<unsigned long long>(failed_cycles_),
                      static_cast<unsigned long long>(deadline_misses_),
                      static_cast<unsigned long long>(fallback_cycles_),
                      geometry_batch_.active_rows, min_distance,
                      static_cast<unsigned long long>(guard_rejections_), last_linearize_ms_);
    rate.sleep();
  }
}
