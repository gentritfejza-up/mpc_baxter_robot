#include "baxter_kdl_dynamics/kdl_inverse_dynamics.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace baxter_kdl_dynamics {
namespace {

constexpr std::size_t kArmJointCount = 7;
constexpr double kGravityMagnitude = 9.81;  ///< [m/s^2], along -z of the root link.

/// Rigid-body inertia of a KDL subtree expressed in the subtree root frame.
struct SubtreeInertia {
  KDL::RigidBodyInertia inertia{KDL::RigidBodyInertia::Zero()};
  std::size_t segment_count{0};
  std::size_t movable_joint_count{0};
};

/// Poll `/robot_description` until it is set or the configured timeout elapses.
std::string loadRobotDescription(ros::NodeHandle& nh) {
  double timeout_sec = 10.0;
  nh.param("robot/kdl_dynamics/robot_description_timeout_sec", timeout_sec, timeout_sec);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_sec);
  std::string robot_description_xml;
  while (ros::ok() && ros::WallTime::now() < deadline) {
    if (nh.getParam("/robot_description", robot_description_xml) &&
        !robot_description_xml.empty()) {
      return robot_description_xml;
    }
    ros::WallDuration(0.05).sleep();
  }
  throw std::runtime_error("Timed out waiting for /robot_description");
}

/**
 * @brief Sum the inertia of `segment_node` and all its descendants, expressed in
 *        the frame of `segment_node`, with every descendant joint at zero.
 */
SubtreeInertia aggregateSubtreeInertia(const KDL::SegmentMap::const_iterator& segment_node) {
  const auto& tree_element = segment_node->second;
  const KDL::Segment& segment = GetTreeElementSegment(tree_element);
  SubtreeInertia subtree;
  subtree.inertia = segment.getInertia();
  subtree.segment_count = 1;
  if (segment.getJoint().getType() != KDL::Joint::None) {
    ++subtree.movable_joint_count;
  }

  for (const auto& child_node : GetTreeElementChildren(tree_element)) {
    const SubtreeInertia child_subtree = aggregateSubtreeInertia(child_node);
    const KDL::Frame parent_to_child = GetTreeElementSegment(child_node->second).pose(0.0);
    subtree.inertia = subtree.inertia + parent_to_child * child_subtree.inertia;
    subtree.segment_count += child_subtree.segment_count;
    subtree.movable_joint_count += child_subtree.movable_joint_count;
  }
  return subtree;
}

/**
 * @brief Replace the inertia of the chain's terminal segment with the lumped
 *        inertia of the tip link and everything below it in the tree.
 * @return The lumped subtree inertia (for logging).
 */
SubtreeInertia lumpTipSubtreeInertia(const KDL::Tree& tree, const std::string& tip_link,
                                     KDL::Chain& chain) {
  const auto tip_node = tree.getSegment(tip_link);
  if (tip_node == tree.getSegments().end()) {
    throw std::runtime_error("KDL tree does not contain tip link " + tip_link);
  }
  if (chain.getNrOfSegments() == 0) {
    throw std::runtime_error("Cannot lump tip subtree into an empty KDL chain");
  }

  const SubtreeInertia tip_subtree = aggregateSubtreeInertia(tip_node);
  KDL::Chain lumped_chain;
  for (unsigned int segment_index = 0; segment_index < chain.getNrOfSegments(); ++segment_index) {
    KDL::Segment segment = chain.getSegment(segment_index);
    if (segment_index + 1 == chain.getNrOfSegments()) {
      if (segment.getName() != tip_link) {
        throw std::runtime_error("KDL chain terminal segment does not match tip link " +
                                 tip_link);
      }
      segment.setInertia(tip_subtree.inertia);
    }
    lumped_chain.addSegment(segment);
  }
  chain = lumped_chain;
  return tip_subtree;
}

}  // namespace

KdlInverseDynamics::KdlInverseDynamics(ros::NodeHandle& nh, const std::string& limb) {
  if (limb != "left" && limb != "right") {
    throw std::invalid_argument("limb must be 'left' or 'right'");
  }

  const std::string robot_description = loadRobotDescription(nh);
  urdf::Model urdf_model;
  if (!urdf_model.initString(robot_description)) {
    throw std::runtime_error("Failed to parse /robot_description as URDF");
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    throw std::runtime_error("Failed to construct KDL tree from /robot_description");
  }

  std::string root_link = "torso";
  std::string tip_link = limb + "_gripper_base";
  nh.param("robot/kdl_dynamics/root_link", root_link, root_link);
  nh.param("robot/kdl_dynamics/tip_link", tip_link, tip_link);
  // Private override so that two limbs launched together keep their own tip links.
  ros::NodeHandle private_nh("~");
  private_nh.param("tip_link", tip_link, tip_link);
  if (!tree.getChain(root_link, tip_link, chain_)) {
    const std::string fallback_tip_link = limb + "_hand";
    if (tip_link == limb + "_gripper_base" &&
        tree.getChain(root_link, fallback_tip_link, chain_)) {
      ROS_WARN_STREAM("KDL tip " << tip_link << " is absent; using " << fallback_tip_link);
      tip_link = fallback_tip_link;
    } else {
      throw std::runtime_error("Failed to create KDL chain from " + root_link + " to " + tip_link);
    }
  }

  bool aggregate_tip_subtree_inertia = true;
  nh.param("robot/kdl_dynamics/aggregate_tip_subtree_inertia", aggregate_tip_subtree_inertia,
           aggregate_tip_subtree_inertia);
  if (aggregate_tip_subtree_inertia) {
    if (chain_.getNrOfSegments() == 0) {
      throw std::runtime_error("Cannot lump tip subtree into an empty KDL chain");
    }
    const double original_tip_mass =
        chain_.getSegment(chain_.getNrOfSegments() - 1).getInertia().getMass();
    const SubtreeInertia tip_subtree = lumpTipSubtreeInertia(tree, tip_link, chain_);
    ROS_INFO_STREAM("Lumped " << tip_subtree.segment_count - 1 << " descendants of " << tip_link
                              << " into the arm-chain tip inertia at zero descendant-joint "
                                 "position: mass "
                              << original_tip_mass << " -> " << tip_subtree.inertia.getMass()
                              << " kg, movable descendant joints="
                              << tip_subtree.movable_joint_count);
  }

  for (unsigned int segment_index = 0; segment_index < chain_.getNrOfSegments(); ++segment_index) {
    const KDL::Joint& joint = chain_.getSegment(segment_index).getJoint();
    if (joint.getType() != KDL::Joint::None) joint_names_.push_back(joint.getName());
  }
  if (joint_names_.size() != kArmJointCount) {
    throw std::runtime_error("Expected seven Baxter arm joints in KDL chain, got " +
                             std::to_string(joint_names_.size()));
  }

  effort_limits_.reserve(joint_names_.size());
  urdf_damping_.reserve(joint_names_.size());
  for (const std::string& joint_name : joint_names_) {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model.getJoint(joint_name);
    if (!urdf_joint || !urdf_joint->limits || !std::isfinite(urdf_joint->limits->effort) ||
        urdf_joint->limits->effort <= 0.0) {
      throw std::runtime_error("Missing positive URDF effort limit for " + joint_name);
    }
    effort_limits_.push_back(urdf_joint->limits->effort);
    urdf_damping_.push_back(urdf_joint->dynamics ? urdf_joint->dynamics->damping : 0.0);
  }

  double s1_torque_limit_override_nm = -1.0;
  nh.param("robot/kdl_dynamics/s1_torque_limit_override", s1_torque_limit_override_nm,
           s1_torque_limit_override_nm);
  if (s1_torque_limit_override_nm > 0.0) {
    const auto s1_position = std::find(joint_names_.begin(), joint_names_.end(), limb + "_s1");
    if (s1_position == joint_names_.end()) {
      throw std::runtime_error("KDL chain does not contain s1");
    }
    effort_limits_[std::distance(joint_names_.begin(), s1_position)] =
        s1_torque_limit_override_nm;
  }

  nh.param("robot/kdl_dynamics/add_gravity_compensation", add_gravity_compensation_, true);
  nh.param("robot/kdl_dynamics/compensate_urdf_damping", compensate_urdf_damping_, true);
  nh.param("robot/kdl_dynamics/extrapolate_reference", extrapolate_reference_,
           extrapolate_reference_);
  nh.param("robot/kdl_dynamics/max_reference_extrapolation_sec",
           max_reference_extrapolation_sec_, max_reference_extrapolation_sec_);
  if (!std::isfinite(max_reference_extrapolation_sec_) || max_reference_extrapolation_sec_ < 0.0) {
    throw std::runtime_error("max_reference_extrapolation_sec must be finite and nonnegative");
  }
  inner_pd_kp_ = loadGainVector(nh, "robot/kdl_dynamics/kp", joint_names_.size());
  inner_pd_kd_ = loadGainVector(nh, "robot/kdl_dynamics/kd", joint_names_.size());

  dynamics_solver_ =
      std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0.0, 0.0, -kGravityMagnitude));
  ROS_INFO_STREAM("KDL inverse dynamics ready for " << limb << " arm: " << root_link << " -> "
                                                     << tip_link << ", gravity="
                                                     << (add_gravity_compensation_ ? "on" : "off")
                                                     << ", damping="
                                                     << (compensate_urdf_damping_ ? "on" : "off"));
}

std::vector<double> KdlInverseDynamics::loadGainVector(ros::NodeHandle& nh,
                                                       const std::string& name,
                                                       std::size_t size) {
  std::vector<double> gains(size, 0.0);
  if (nh.hasParam(name) && (!nh.getParam(name, gains) || gains.size() != size)) {
    throw std::runtime_error(name + " must contain one value per arm joint");
  }
  if (!std::all_of(gains.begin(), gains.end(),
                   [](double gain) { return std::isfinite(gain) && gain >= 0.0; })) {
    throw std::runtime_error(name + " must contain finite nonnegative values");
  }
  return gains;
}

double KdlInverseDynamics::requireFiniteValue(
    const std::unordered_map<std::string, double>& values, const std::string& joint,
    const std::string& source) {
  const auto entry = values.find(joint);
  if (entry == values.end() || !std::isfinite(entry->second)) {
    throw std::runtime_error(source + " is missing a finite value for " + joint);
  }
  return entry->second;
}

DynamicsResult KdlInverseDynamics::makeEmptyResult() const {
  const std::size_t joint_count = joint_names_.size();
  DynamicsResult result;
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
  result.torque_limit = effort_limits_;
  result.torques.reserve(joint_count);
  return result;
}

void KdlInverseDynamics::readMeasuredState(
    const std::unordered_map<std::string, double>& positions,
    const std::unordered_map<std::string, double>& velocities, KDL::JntArray& q,
    KDL::JntArray& qd, DynamicsResult& result) const {
  for (unsigned int joint_index = 0; joint_index < joint_names_.size(); ++joint_index) {
    const std::string& joint = joint_names_[joint_index];
    q(joint_index) = requireFiniteValue(positions, joint, "joint positions");
    qd(joint_index) = requireFiniteValue(velocities, joint, "joint velocities");
    result.position[joint_index] = q(joint_index);
    result.velocity[joint_index] = qd(joint_index);
  }
}

void KdlInverseDynamics::evaluateModelTorque(const KDL::JntArray& q, const KDL::JntArray& qd,
                                             const KDL::JntArray& qdd,
                                             DynamicsResult& result) const {
  const unsigned int joint_count = joint_names_.size();
  KDL::JntSpaceInertiaMatrix mass_matrix(joint_count);
  KDL::JntArray coriolis_torque(joint_count);
  KDL::JntArray gravity_torque(joint_count);
  if (dynamics_solver_->JntToMass(q, mass_matrix) < 0 ||
      dynamics_solver_->JntToCoriolis(q, qd, coriolis_torque) < 0 ||
      (add_gravity_compensation_ && dynamics_solver_->JntToGravity(q, gravity_torque) < 0)) {
    throw std::runtime_error("KDL inverse dynamics calculation failed");
  }

  for (unsigned int row = 0; row < joint_count; ++row) {
    double inertia_torque = 0.0;
    for (unsigned int column = 0; column < joint_count; ++column) {
      inertia_torque += mass_matrix(row, column) * qdd(column);
    }
    double torque = inertia_torque + coriolis_torque(row);
    result.inertia_torque[row] = inertia_torque;
    result.coriolis_torque[row] = coriolis_torque(row);
    if (add_gravity_compensation_) {
      result.gravity_torque[row] = gravity_torque(row);
      torque += gravity_torque(row);
    }
    if (compensate_urdf_damping_) {
      result.damping_torque[row] = urdf_damping_[row] * qd(row);
      torque += result.damping_torque[row];
    }
    result.raw_torque[row] = torque;
  }
}

void KdlInverseDynamics::saturateTorque(DynamicsResult& result) const {
  for (std::size_t joint_index = 0; joint_index < joint_names_.size(); ++joint_index) {
    const double torque = result.raw_torque[joint_index];
    const double saturated_torque =
        std::clamp(torque, -effort_limits_[joint_index], effort_limits_[joint_index]);
    result.commanded_torque[joint_index] = saturated_torque;
    result.torques[joint_names_[joint_index]] = saturated_torque;
    if (saturated_torque != torque) {
      ROS_WARN_STREAM_THROTTLE(1.0, joint_names_[joint_index] << " torque saturated: " << torque
                                                              << " -> " << saturated_torque
                                                              << " Nm");
    }
  }
}

DynamicsResult KdlInverseDynamics::compute(
    const std::unordered_map<std::string, double>& positions,
    const std::unordered_map<std::string, double>& velocities,
    const std::unordered_map<std::string, double>& accelerations,
    const std::unordered_map<std::string, double>& reference_positions,
    const std::unordered_map<std::string, double>& reference_velocities,
    double reference_age_sec) const {
  const unsigned int joint_count = joint_names_.size();
  const double extrapolation_sec =
      extrapolate_reference_ && std::isfinite(reference_age_sec)
          ? std::clamp(reference_age_sec, 0.0, max_reference_extrapolation_sec_)
          : 0.0;
  KDL::JntArray q(joint_count), qd(joint_count), qdd(joint_count);
  DynamicsResult result = makeEmptyResult();
  readMeasuredState(positions, velocities, q, qd, result);

  for (unsigned int joint_index = 0; joint_index < joint_count; ++joint_index) {
    const std::string& joint = joint_names_[joint_index];
    const double mpc_acceleration =
        requireFiniteValue(accelerations, joint, "reference accelerations");
    // Without an inner PD the reference is irrelevant (and may be absent): report
    // the measured state as the reference so the diagnostics stay well defined.
    const bool needs_reference = inner_pd_kp_[joint_index] > 0.0 || inner_pd_kd_[joint_index] > 0.0;
    const double reference_base_position =
        needs_reference ? requireFiniteValue(reference_positions, joint, "reference positions")
                        : q(joint_index);
    const double reference_base_velocity =
        needs_reference ? requireFiniteValue(reference_velocities, joint, "reference velocities")
                        : qd(joint_index);
    const double reference_position =
        needs_reference ? reference_base_position + reference_base_velocity * extrapolation_sec +
                              0.5 * mpc_acceleration * extrapolation_sec * extrapolation_sec
                        : q(joint_index);
    const double reference_velocity =
        needs_reference ? reference_base_velocity + mpc_acceleration * extrapolation_sec
                        : qd(joint_index);
    const double pd_acceleration =
        inner_pd_kp_[joint_index] * (reference_position - q(joint_index)) +
        inner_pd_kd_[joint_index] * (reference_velocity - qd(joint_index));
    qdd(joint_index) = mpc_acceleration + pd_acceleration;
    result.reference_position[joint_index] = reference_position;
    result.reference_velocity[joint_index] = reference_velocity;
    result.mpc_acceleration[joint_index] = mpc_acceleration;
    result.pd_acceleration[joint_index] = pd_acceleration;
    result.commanded_acceleration[joint_index] = qdd(joint_index);
  }

  evaluateModelTorque(q, qd, qdd, result);
  saturateTorque(result);
  return result;
}

DynamicsResult KdlInverseDynamics::computeHold(
    const std::unordered_map<std::string, double>& positions,
    const std::unordered_map<std::string, double>& velocities,
    const std::unordered_map<std::string, double>& hold_positions,
    const std::vector<double>& kp, const std::vector<double>& kd) const {
  if (kp.size() != joint_names_.size() || kd.size() != joint_names_.size()) {
    throw std::invalid_argument("hold gains must contain one value per arm joint");
  }
  const unsigned int joint_count = joint_names_.size();
  KDL::JntArray q(joint_count), qd(joint_count);
  KDL::JntArray zero_acceleration(joint_count);  // JntArray zero-initialises its data.
  DynamicsResult result = makeEmptyResult();
  readMeasuredState(positions, velocities, q, qd, result);
  std::fill(result.mpc_acceleration.begin(), result.mpc_acceleration.end(), 0.0);
  std::fill(result.pd_acceleration.begin(), result.pd_acceleration.end(), 0.0);
  std::fill(result.commanded_acceleration.begin(), result.commanded_acceleration.end(), 0.0);
  std::fill(result.reference_velocity.begin(), result.reference_velocity.end(), 0.0);

  // Gravity, Coriolis and damping compensation at zero acceleration ...
  evaluateModelTorque(q, qd, zero_acceleration, result);
  // ... plus a torque-space PD toward the anchor pose.
  for (unsigned int joint_index = 0; joint_index < joint_count; ++joint_index) {
    const double anchor_position =
        requireFiniteValue(hold_positions, joint_names_[joint_index], "hold positions");
    const double position_error = anchor_position - q(joint_index);
    result.reference_position[joint_index] = anchor_position;
    result.raw_torque[joint_index] += kp[joint_index] * position_error - kd[joint_index] * qd(joint_index);
  }
  saturateTorque(result);
  return result;
}

}  // namespace baxter_kdl_dynamics
