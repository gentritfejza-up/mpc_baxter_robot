#include "baxter_kdl_dynamics/kdl_inverse_dynamics.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>

namespace baxter_kdl_dynamics {
namespace {

constexpr std::size_t kArmJointCount = 7;

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

  std::vector<double> gravity = {0.0, 0.0, -9.81};
  nh.param("robot/kdl_dynamics/gravity", gravity, gravity);
  if (gravity.size() != 3 || !std::all_of(gravity.begin(), gravity.end(),
                                          [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error("robot/kdl_dynamics/gravity must hold 3 finite values");
  }
  dynamics_solver_ = std::make_unique<KDL::ChainDynParam>(
      chain_, KDL::Vector(gravity[0], gravity[1], gravity[2]));

  const auto joint_count = static_cast<unsigned int>(joint_names_.size());
  q_.resize(joint_count);
  qd_.resize(joint_count);
  coriolis_.resize(joint_count);
  gravity_.resize(joint_count);
  mass_matrix_.resize(joint_count);

  std::ostringstream text;
  text << "KDL URDF model " << root_link << " -> " << tip_link << " (" << joint_count
       << " joints, tip inertia " << (aggregate_tip_subtree_inertia ? "lumped" : "as in URDF")
       << ")";
  description_ = text.str();
}

void KdlInverseDynamics::evaluate(const Eigen::VectorXd& positions,
                                  const Eigen::VectorXd& velocities,
                                  baxter::JointSpaceDynamics& dynamics) const {
  const auto joint_count = static_cast<Eigen::Index>(joint_names_.size());
  if (positions.size() != joint_count || velocities.size() != joint_count) {
    throw std::runtime_error("KDL model: expected " + std::to_string(joint_count) +
                             " joint positions and velocities");
  }
  for (Eigen::Index index = 0; index < joint_count; ++index) {
    q_(index) = positions(index);
    qd_(index) = velocities(index);
  }
  if (dynamics_solver_->JntToMass(q_, mass_matrix_) < 0 ||
      dynamics_solver_->JntToCoriolis(q_, qd_, coriolis_) < 0 ||
      dynamics_solver_->JntToGravity(q_, gravity_) < 0) {
    throw std::runtime_error("KDL inverse dynamics calculation failed");
  }
  dynamics.resize(joint_names_.size());
  for (Eigen::Index row = 0; row < joint_count; ++row) {
    dynamics.coriolis_torque(row) = coriolis_(row);
    dynamics.gravity_torque(row) = gravity_(row);
    for (Eigen::Index column = 0; column < joint_count; ++column) {
      dynamics.mass_matrix(row, column) = mass_matrix_(row, column);
    }
  }
}

}  // namespace baxter_kdl_dynamics
