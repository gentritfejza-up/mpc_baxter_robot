#include "baxter_dynamics/computed_torque_law.hpp"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace baxter {

std::vector<double> loadGainVector(ros::NodeHandle& nh, const std::string& name,
                                   std::size_t size) {
  std::vector<double> gains(size, 0.0);
  if (nh.hasParam(name) && (!nh.getParam(name, gains) || gains.size() != size)) {
    throw std::runtime_error(name + " must contain one value per joint (" +
                             std::to_string(size) + ")");
  }
  if (!std::all_of(gains.begin(), gains.end(),
                   [](double gain) { return std::isfinite(gain) && gain >= 0.0; })) {
    throw std::runtime_error(name + " must contain finite nonnegative values");
  }
  return gains;
}

ComputedTorqueSettings ComputedTorqueSettings::load(ros::NodeHandle& nh,
                                                    const std::string& parameter_namespace,
                                                    std::size_t joint_count) {
  const std::string ns = parameter_namespace + "/";
  ComputedTorqueSettings settings;
  nh.param(ns + "add_gravity_compensation", settings.add_gravity_compensation,
           settings.add_gravity_compensation);
  nh.param(ns + "compensate_viscous_damping", settings.compensate_viscous_damping,
           settings.compensate_viscous_damping);
  nh.param(ns + "extrapolate_reference", settings.extrapolate_reference,
           settings.extrapolate_reference);
  nh.param(ns + "max_reference_extrapolation_sec", settings.max_reference_extrapolation_sec,
           settings.max_reference_extrapolation_sec);
  if (!std::isfinite(settings.max_reference_extrapolation_sec) ||
      settings.max_reference_extrapolation_sec < 0.0) {
    throw std::runtime_error(ns + "max_reference_extrapolation_sec must be finite and nonnegative");
  }
  settings.inner_pd_kp = loadGainVector(nh, ns + "kp", joint_count);
  settings.inner_pd_kd = loadGainVector(nh, ns + "kd", joint_count);
  return settings;
}

ComputedTorqueLaw::ComputedTorqueLaw(const InverseDynamicsModel& model,
                                     ComputedTorqueSettings settings)
    : model_(model), settings_(std::move(settings)) {
  if (settings_.inner_pd_kp.size() != model_.jointCount() ||
      settings_.inner_pd_kd.size() != model_.jointCount()) {
    throw std::invalid_argument("inner PD gains must contain one value per joint");
  }
  dynamics_.resize(model_.jointCount());
}

double ComputedTorqueLaw::requireFiniteValue(const JointMap& values, const std::string& joint,
                                             const char* source) {
  const auto entry = values.find(joint);
  if (entry == values.end() || !std::isfinite(entry->second)) {
    throw std::runtime_error(std::string(source) + " is missing a finite value for " + joint);
  }
  return entry->second;
}

ComputedTorqueResult ComputedTorqueLaw::makeEmptyResult() const {
  const std::size_t joint_count = model_.jointCount();
  ComputedTorqueResult result;
  result.position.resize(joint_count);
  result.velocity.resize(joint_count);
  result.reference_position.resize(joint_count);
  result.reference_velocity.resize(joint_count);
  result.mpc_acceleration.resize(joint_count);
  result.pd_acceleration.resize(joint_count);
  result.commanded_acceleration.resize(joint_count);
  result.inertia_torque.resize(joint_count);
  result.coriolis_torque.resize(joint_count);
  result.gravity_torque.assign(joint_count, 0.0);
  result.damping_torque.assign(joint_count, 0.0);
  result.raw_torque.resize(joint_count);
  result.commanded_torque.resize(joint_count);
  result.torque_limit = model_.effortLimits();
  result.torques.reserve(joint_count);
  return result;
}

void ComputedTorqueLaw::readMeasuredState(const JointMap& positions, const JointMap& velocities,
                                          Eigen::VectorXd& q, Eigen::VectorXd& qd,
                                          ComputedTorqueResult& result) const {
  const std::vector<std::string>& joint_names = model_.jointNames();
  const auto n = static_cast<Eigen::Index>(joint_names.size());
  q.resize(n);
  qd.resize(n);
  for (Eigen::Index index = 0; index < n; ++index) {
    const std::string& joint = joint_names[index];
    q(index) = requireFiniteValue(positions, joint, "joint positions");
    qd(index) = requireFiniteValue(velocities, joint, "joint velocities");
    result.position[index] = q(index);
    result.velocity[index] = qd(index);
  }
}

void ComputedTorqueLaw::evaluateModelTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
                                            const Eigen::VectorXd& qdd,
                                            ComputedTorqueResult& result) const {
  model_.evaluate(q, qd, dynamics_);
  const Eigen::VectorXd inertia_torque = dynamics_.mass_matrix * qdd;
  const std::vector<double>& damping = model_.viscousDamping();
  for (Eigen::Index row = 0; row < q.size(); ++row) {
    double torque = inertia_torque(row) + dynamics_.coriolis_torque(row);
    result.inertia_torque[row] = inertia_torque(row);
    result.coriolis_torque[row] = dynamics_.coriolis_torque(row);
    if (settings_.add_gravity_compensation) {
      result.gravity_torque[row] = dynamics_.gravity_torque(row);
      torque += dynamics_.gravity_torque(row);
    }
    if (settings_.compensate_viscous_damping) {
      result.damping_torque[row] = damping[row] * qd(row);
      torque += result.damping_torque[row];
    }
    result.raw_torque[row] = torque;
  }
}

void ComputedTorqueLaw::saturateTorque(ComputedTorqueResult& result) const {
  const std::vector<std::string>& joint_names = model_.jointNames();
  const std::vector<double>& limits = model_.effortLimits();
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    const double torque = result.raw_torque[index];
    const double saturated = std::clamp(torque, -limits[index], limits[index]);
    result.commanded_torque[index] = saturated;
    result.torques[joint_names[index]] = saturated;
    if (saturated != torque) {
      ROS_WARN_STREAM_THROTTLE(1.0, joint_names[index] << " torque saturated: " << torque
                                                       << " -> " << saturated << " Nm");
    }
  }
}

ComputedTorqueResult ComputedTorqueLaw::track(const JointMap& positions,
                                              const JointMap& velocities,
                                              const JointMap& accelerations,
                                              const JointMap& reference_positions,
                                              const JointMap& reference_velocities,
                                              double reference_age_sec) const {
  const double extrapolation_sec =
      settings_.extrapolate_reference && std::isfinite(reference_age_sec)
          ? std::clamp(reference_age_sec, 0.0, settings_.max_reference_extrapolation_sec)
          : 0.0;
  ComputedTorqueResult result = makeEmptyResult();
  Eigen::VectorXd q, qd;
  readMeasuredState(positions, velocities, q, qd, result);
  Eigen::VectorXd qdd(q.size());

  const std::vector<std::string>& joint_names = model_.jointNames();
  for (Eigen::Index index = 0; index < q.size(); ++index) {
    const std::string& joint = joint_names[index];
    const double mpc_acceleration =
        requireFiniteValue(accelerations, joint, "reference accelerations");
    const double kp = settings_.inner_pd_kp[index];
    const double kd = settings_.inner_pd_kd[index];
    // Without an inner PD the reference is irrelevant (and may be absent): report
    // the measured state as the reference so the diagnostics stay well defined.
    const bool needs_reference = kp > 0.0 || kd > 0.0;
    double reference_position = q(index);
    double reference_velocity = qd(index);
    if (needs_reference) {
      const double q0 = requireFiniteValue(reference_positions, joint, "reference positions");
      const double v0 = requireFiniteValue(reference_velocities, joint, "reference velocities");
      reference_position = q0 + v0 * extrapolation_sec +
                           0.5 * mpc_acceleration * extrapolation_sec * extrapolation_sec;
      reference_velocity = v0 + mpc_acceleration * extrapolation_sec;
    }
    const double pd_acceleration =
        kp * (reference_position - q(index)) + kd * (reference_velocity - qd(index));
    qdd(index) = mpc_acceleration + pd_acceleration;
    result.reference_position[index] = reference_position;
    result.reference_velocity[index] = reference_velocity;
    result.mpc_acceleration[index] = mpc_acceleration;
    result.pd_acceleration[index] = pd_acceleration;
    result.commanded_acceleration[index] = qdd(index);
  }

  evaluateModelTorque(q, qd, qdd, result);
  saturateTorque(result);
  return result;
}

ComputedTorqueResult ComputedTorqueLaw::hold(const JointMap& positions, const JointMap& velocities,
                                             const JointMap& hold_positions,
                                             const std::vector<double>& kp,
                                             const std::vector<double>& kd) const {
  const std::vector<std::string>& joint_names = model_.jointNames();
  if (kp.size() != joint_names.size() || kd.size() != joint_names.size()) {
    throw std::invalid_argument("hold gains must contain one value per joint");
  }
  ComputedTorqueResult result = makeEmptyResult();
  Eigen::VectorXd q, qd;
  readMeasuredState(positions, velocities, q, qd, result);
  std::fill(result.mpc_acceleration.begin(), result.mpc_acceleration.end(), 0.0);
  std::fill(result.pd_acceleration.begin(), result.pd_acceleration.end(), 0.0);
  std::fill(result.commanded_acceleration.begin(), result.commanded_acceleration.end(), 0.0);
  std::fill(result.reference_velocity.begin(), result.reference_velocity.end(), 0.0);

  // Gravity, Coriolis and damping compensation at zero acceleration ...
  evaluateModelTorque(q, qd, Eigen::VectorXd::Zero(q.size()), result);
  // ... plus a torque-space PD toward the anchor pose.
  for (Eigen::Index index = 0; index < q.size(); ++index) {
    const double anchor = requireFiniteValue(hold_positions, joint_names[index], "hold positions");
    result.reference_position[index] = anchor;
    result.raw_torque[index] += kp[index] * (anchor - q(index)) - kd[index] * qd(index);
  }
  saturateTorque(result);
  return result;
}

}  // namespace baxter
