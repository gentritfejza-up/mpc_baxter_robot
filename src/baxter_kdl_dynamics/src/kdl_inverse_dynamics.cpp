#include "baxter_kdl_dynamics/kdl_inverse_dynamics.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

namespace baxter_kdl_dynamics {
namespace {

struct SubtreeInertia {
  KDL::RigidBodyInertia inertia{KDL::RigidBodyInertia::Zero()};
  std::size_t segment_count{0};
  std::size_t movable_joint_count{0};
};

std::string loadRobotDescription(ros::NodeHandle& nh) {
  double timeout_sec = 10.0;
  nh.param("robot/kdl_dynamics/robot_description_timeout_sec", timeout_sec, timeout_sec);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_sec);
  std::string xml;
  while (ros::ok() && ros::WallTime::now() < deadline) {
    if (nh.getParam("/robot_description", xml) && !xml.empty()) return xml;
    ros::WallDuration(0.05).sleep();
  }
  throw std::runtime_error("Timed out waiting for /robot_description");
}

SubtreeInertia aggregateSubtreeInertia(const KDL::SegmentMap::const_iterator& node) {
  const auto& element = node->second;
  const KDL::Segment& segment = GetTreeElementSegment(element);
  SubtreeInertia result;
  result.inertia = segment.getInertia();
  result.segment_count = 1;
  if (segment.getJoint().getType() != KDL::Joint::None) {
    ++result.movable_joint_count;
  }

  for (const auto& child : GetTreeElementChildren(element)) {
    const SubtreeInertia child_result = aggregateSubtreeInertia(child);
    const KDL::Frame parent_to_child = GetTreeElementSegment(child->second).pose(0.0);
    result.inertia = result.inertia + parent_to_child * child_result.inertia;
    result.segment_count += child_result.segment_count;
    result.movable_joint_count += child_result.movable_joint_count;
  }
  return result;
}

SubtreeInertia lumpTipSubtreeInertia(const KDL::Tree& tree, const std::string& tip_link,
                                     KDL::Chain& chain) {
  const auto tip = tree.getSegment(tip_link);
  if (tip == tree.getSegments().end()) {
    throw std::runtime_error("KDL tree does not contain tip link " + tip_link);
  }
  if (chain.getNrOfSegments() == 0) {
    throw std::runtime_error("Cannot lump tip subtree into an empty KDL chain");
  }

  const SubtreeInertia composite = aggregateSubtreeInertia(tip);
  KDL::Chain modified;
  for (unsigned int index = 0; index < chain.getNrOfSegments(); ++index) {
    KDL::Segment segment = chain.getSegment(index);
    if (index + 1 == chain.getNrOfSegments()) {
      if (segment.getName() != tip_link) {
        throw std::runtime_error("KDL chain terminal segment does not match tip link " +
                                 tip_link);
      }
      segment.setInertia(composite.inertia);
    }
    modified.addSegment(segment);
  }
  chain = modified;
  return composite;
}

}  // namespace

KdlInverseDynamics::KdlInverseDynamics(ros::NodeHandle& nh, const std::string& limb) {
  if (limb != "left" && limb != "right") {
    throw std::invalid_argument("limb must be 'left' or 'right'");
  }

  const std::string robot_description = loadRobotDescription(nh);
  urdf::Model model;
  if (!model.initString(robot_description)) {
    throw std::runtime_error("Failed to parse /robot_description as URDF");
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::runtime_error("Failed to construct KDL tree from /robot_description");
  }

  std::string root_link = "torso";
  std::string tip_link = limb + "_gripper_base";
  nh.param("robot/kdl_dynamics/root_link", root_link, root_link);
  nh.param("robot/kdl_dynamics/tip_link", tip_link, tip_link);
  if (!tree.getChain(root_link, tip_link, chain_)) {
    const std::string fallback_tip = limb + "_hand";
    if (tip_link == limb + "_gripper_base" && tree.getChain(root_link, fallback_tip, chain_)) {
      ROS_WARN_STREAM("KDL tip " << tip_link << " is absent; using " << fallback_tip);
      tip_link = fallback_tip;
    } else {
      throw std::runtime_error("Failed to create KDL chain from " + root_link + " to " + tip_link);
    }
  }

  bool aggregate_tip_subtree_inertia = true;
  nh.param("robot/kdl_dynamics/aggregate_tip_subtree_inertia",
           aggregate_tip_subtree_inertia, aggregate_tip_subtree_inertia);
  if (aggregate_tip_subtree_inertia) {
    if (chain_.getNrOfSegments() == 0) {
      throw std::runtime_error("Cannot lump tip subtree into an empty KDL chain");
    }
    const double original_tip_mass = chain_.getSegment(chain_.getNrOfSegments() - 1)
                                         .getInertia()
                                         .getMass();
    const SubtreeInertia composite = lumpTipSubtreeInertia(tree, tip_link, chain_);
    ROS_INFO_STREAM("Lumped " << composite.segment_count - 1 << " descendants of " << tip_link
                               << " into the arm-chain tip inertia at zero descendant-joint "
                                  "position: mass "
                               << original_tip_mass << " -> " << composite.inertia.getMass()
                               << " kg, movable descendant joints="
                               << composite.movable_joint_count);
  }

  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const auto& segment = chain_.getSegment(i);
    const auto& joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None) joint_names_.push_back(joint.getName());
  }
  if (joint_names_.size() != 7) {
    throw std::runtime_error("Expected seven Baxter arm joints in KDL chain, got " +
                             std::to_string(joint_names_.size()));
  }

  effort_limits_.reserve(joint_names_.size());
  damping_.reserve(joint_names_.size());
  for (const auto& joint_name : joint_names_) {
    const urdf::JointConstSharedPtr joint = model.getJoint(joint_name);
    if (!joint || !joint->limits || !std::isfinite(joint->limits->effort) ||
        joint->limits->effort <= 0.0) {
      throw std::runtime_error("Missing positive URDF effort limit for " + joint_name);
    }
    effort_limits_.push_back(joint->limits->effort);
    damping_.push_back(joint->dynamics ? joint->dynamics->damping : 0.0);
  }

  double s1_override = -1.0;
  nh.param("robot/kdl_dynamics/s1_torque_limit_override", s1_override, s1_override);
  if (s1_override > 0.0) {
    const auto it = std::find(joint_names_.begin(), joint_names_.end(), limb + "_s1");
    if (it == joint_names_.end()) throw std::runtime_error("KDL chain does not contain s1");
    effort_limits_[std::distance(joint_names_.begin(), it)] = s1_override;
  }

  nh.param("robot/kdl_dynamics/add_gravity_compensation", add_gravity_compensation_, true);
  nh.param("robot/kdl_dynamics/compensate_urdf_damping", compensate_urdf_damping_, true);
  kp_ = loadGainVector(nh, "robot/kdl_dynamics/kp", joint_names_.size());
  kd_ = loadGainVector(nh, "robot/kdl_dynamics/kd", joint_names_.size());

  dynamics_ = std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0.0, 0.0, -9.81));
  ROS_INFO_STREAM("KDL inverse dynamics ready for " << limb << " arm: " << root_link << " -> "
                                                     << tip_link << ", gravity="
                                                     << (add_gravity_compensation_ ? "on" : "off")
                                                     << ", damping="
                                                     << (compensate_urdf_damping_ ? "on" : "off"));
}

std::vector<double> KdlInverseDynamics::loadGainVector(ros::NodeHandle& nh,
                                                       const std::string& name,
                                                       std::size_t size) {
  std::vector<double> values(size, 0.0);
  if (nh.hasParam(name) && (!nh.getParam(name, values) || values.size() != size)) {
    throw std::runtime_error(name + " must contain one value per arm joint");
  }
  if (!std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value) && value >= 0.0; })) {
    throw std::runtime_error(name + " must contain finite nonnegative values");
  }
  return values;
}

double KdlInverseDynamics::requiredValue(const std::unordered_map<std::string, double>& values,
                                         const std::string& joint,
                                         const std::string& source) {
  const auto it = values.find(joint);
  if (it == values.end() || !std::isfinite(it->second)) {
    throw std::runtime_error(source + " is missing a finite value for " + joint);
  }
  return it->second;
}

DynamicsResult KdlInverseDynamics::compute(
    const std::unordered_map<std::string, double>& positions,
    const std::unordered_map<std::string, double>& velocities,
    const std::unordered_map<std::string, double>& accelerations,
    const std::unordered_map<std::string, double>& reference_positions,
    const std::unordered_map<std::string, double>& reference_velocities) {
  const unsigned int count = joint_names_.size();
  KDL::JntArray q(count), qd(count), qdd(count), coriolis(count), gravity(count);
  KDL::JntSpaceInertiaMatrix mass(count);
  DynamicsResult result;
  result.position.resize(count);
  result.velocity.resize(count);
  result.reference_position.resize(count);
  result.reference_velocity.resize(count);
  result.mpc_acceleration.resize(count);
  result.pd_acceleration.resize(count);
  result.commanded_acceleration.resize(count);
  result.inertia_torque.resize(count);
  result.coriolis_torque.resize(count);
  result.gravity_torque.assign(count, 0.0);
  result.damping_torque.assign(count, 0.0);
  result.raw_torque.resize(count);
  result.commanded_torque.resize(count);
  result.torque_limit = effort_limits_;

  for (unsigned int i = 0; i < count; ++i) {
    const std::string& joint = joint_names_[i];
    q(i) = requiredValue(positions, joint, "joint positions");
    qd(i) = requiredValue(velocities, joint, "joint velocities");
    const double q_ref = kp_[i] > 0.0
                             ? requiredValue(reference_positions, joint, "reference positions")
                             : q(i);
    const double qd_ref = kd_[i] > 0.0
                              ? requiredValue(reference_velocities, joint, "reference velocities")
                              : qd(i);
    const double position_error = q_ref - q(i);
    const double mpc_acceleration = requiredValue(accelerations, joint, "reference accelerations");
    const double pd_acceleration =
        kp_[i] * position_error + kd_[i] * (qd_ref - qd(i));
    qdd(i) = mpc_acceleration + pd_acceleration;
    result.position[i] = q(i);
    result.velocity[i] = qd(i);
    result.reference_position[i] = q_ref;
    result.reference_velocity[i] = qd_ref;
    result.mpc_acceleration[i] = mpc_acceleration;
    result.pd_acceleration[i] = pd_acceleration;
    result.commanded_acceleration[i] = qdd(i);
  }

  if (dynamics_->JntToMass(q, mass) < 0 || dynamics_->JntToCoriolis(q, qd, coriolis) < 0 ||
      (add_gravity_compensation_ && dynamics_->JntToGravity(q, gravity) < 0)) {
    throw std::runtime_error("KDL inverse dynamics calculation failed");
  }

  KDL::JntArray inertia_torque(count);
  for (unsigned int row = 0; row < count; ++row) {
    inertia_torque(row) = 0.0;
    for (unsigned int column = 0; column < count; ++column) {
      inertia_torque(row) += mass(row, column) * qdd(column);
    }
  }
  result.torques.reserve(count);
  for (unsigned int i = 0; i < count; ++i) {
    double torque = inertia_torque(i) + coriolis(i);
    result.inertia_torque[i] = inertia_torque(i);
    result.coriolis_torque[i] = coriolis(i);
    if (add_gravity_compensation_) {
      result.gravity_torque[i] = gravity(i);
      torque += gravity(i);
    }
    if (compensate_urdf_damping_) {
      result.damping_torque[i] = damping_[i] * qd(i);
      torque += result.damping_torque[i];
    }
    const double limited = std::clamp(torque, -effort_limits_[i], effort_limits_[i]);
    result.raw_torque[i] = torque;
    result.commanded_torque[i] = limited;
    if (limited != torque) {
      ROS_WARN_STREAM_THROTTLE(1.0, joint_names_[i] << " torque saturated: " << torque
                                                    << " -> " << limited << " Nm");
    }
    result.torques.emplace(joint_names_[i], limited);
  }
  return result;
}

}  // namespace baxter_kdl_dynamics
