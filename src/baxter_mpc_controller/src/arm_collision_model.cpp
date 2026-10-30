#include "mpc_controller/arm_collision_model.hpp"

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpc_geometry {
namespace {

/// Replaces every "{side}" in `text` with the arm side.
std::string expandSide(std::string text, const std::string& side) {
  const std::string token = "{side}";
  for (std::size_t position = text.find(token); position != std::string::npos;
       position = text.find(token)) {
    text.replace(position, token.size(), side);
  }
  return text;
}

double numberAt(XmlRpc::XmlRpcValue& value, const char* what) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
  throw std::invalid_argument(std::string("collision geometry: ") + what + " must be a number");
}

Eigen::Vector3d vectorAt(XmlRpc::XmlRpcValue& value, const char* what) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3) {
    throw std::invalid_argument(std::string("collision geometry: ") + what +
                                " must be a list of three numbers");
  }
  return {numberAt(value[0], what), numberAt(value[1], what), numberAt(value[2], what)};
}

bool boolAt(XmlRpc::XmlRpcValue& entry, const char* key, bool fallback) {
  if (!entry.hasMember(key)) return fallback;
  XmlRpc::XmlRpcValue& value = entry[key];
  if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    throw std::invalid_argument(std::string("collision geometry: ") + key + " must be a bool");
  }
  return static_cast<bool>(value);
}

Eigen::Vector3d toEigen(const KDL::Vector& vector) { return {vector.x(), vector.y(), vector.z()}; }

KDL::Vector toKdl(const Eigen::Vector3d& vector) { return {vector.x(), vector.y(), vector.z()}; }

}  // namespace

std::vector<SphereSpec> ArmCollisionModel::parseSpheres(XmlRpc::XmlRpcValue& list) {
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray || list.size() == 0) {
    throw std::invalid_argument("collision geometry: arm_spheres must be a non-empty list");
  }
  std::vector<SphereSpec> spheres;
  for (int entry_index = 0; entry_index < list.size(); ++entry_index) {
    XmlRpc::XmlRpcValue& entry = list[entry_index];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct || !entry.hasMember("link") ||
        !entry.hasMember("offset") || !entry.hasMember("radius")) {
      throw std::invalid_argument("collision geometry: each arm sphere needs link, offset, radius");
    }
    SphereSpec spec;
    spec.link = static_cast<std::string>(entry["link"]);
    if (entry.hasMember("side")) {
      spec.side = static_cast<std::string>(entry["side"]);
      if (spec.side != "left" && spec.side != "right") {
        throw std::invalid_argument("collision geometry: sphere side must be left or right");
      }
    }
    spec.offset = vectorAt(entry["offset"], "offset");
    spec.radius = numberAt(entry["radius"], "radius");
    if (!(spec.radius > 0.0)) {
      throw std::invalid_argument("collision geometry: sphere radius must be positive");
    }
    spec.checks_floor = boolAt(entry, "floor", true);
    spec.checks_body = boolAt(entry, "body", true);
    spec.checks_other_arm = boolAt(entry, "other_arm", true);
    if (entry.hasMember("body_margin")) {
      spec.body_margin_override = numberAt(entry["body_margin"], "body_margin");
    }
    if (entry.hasMember("ignore")) {
      XmlRpc::XmlRpcValue& ignore_list = entry["ignore"];
      if (ignore_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::invalid_argument("collision geometry: ignore must be a list of obstacle names");
      }
      for (int name_index = 0; name_index < ignore_list.size(); ++name_index) {
        spec.ignored_obstacles.push_back(static_cast<std::string>(ignore_list[name_index]));
      }
    }
    spheres.push_back(spec);
  }
  return spheres;
}

std::vector<Obstacle> ArmCollisionModel::parseObstacles(XmlRpc::XmlRpcValue& list) {
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::invalid_argument("collision geometry: body must be a list");
  }
  std::vector<Obstacle> obstacles;
  for (int entry_index = 0; entry_index < list.size(); ++entry_index) {
    XmlRpc::XmlRpcValue& entry = list[entry_index];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct || !entry.hasMember("name") ||
        !entry.hasMember("type")) {
      throw std::invalid_argument("collision geometry: each body obstacle needs name and type");
    }
    Obstacle obstacle;
    obstacle.name = static_cast<std::string>(entry["name"]);
    obstacle.group = ObstacleGroup::Body;
    if (!parseObstacleType(static_cast<std::string>(entry["type"]), obstacle.type)) {
      throw std::invalid_argument("collision geometry: unknown obstacle type for " + obstacle.name);
    }
    switch (obstacle.type) {
      case ObstacleType::Sphere:
        obstacle.start_point = vectorAt(entry["center"], "center");
        obstacle.radius = numberAt(entry["radius"], "radius");
        break;
      case ObstacleType::Capsule:
        obstacle.start_point = vectorAt(entry["a"], "a");
        obstacle.end_point = vectorAt(entry["b"], "b");
        obstacle.radius = numberAt(entry["radius"], "radius");
        break;
      case ObstacleType::Aabb:
        obstacle.start_point = vectorAt(entry["min"], "min");
        obstacle.end_point = vectorAt(entry["max"], "max");
        if ((obstacle.end_point.array() <= obstacle.start_point.array()).any()) {
          throw std::invalid_argument("collision geometry: aabb max must exceed min for " +
                                      obstacle.name);
        }
        break;
      case ObstacleType::ZPlane:
        obstacle.start_point = Eigen::Vector3d(0.0, 0.0, numberAt(entry["z"], "z"));
        break;
    }
    obstacles.push_back(obstacle);
  }
  return obstacles;
}

std::unique_ptr<ArmCollisionModel> ArmCollisionModel::fromParams(ros::NodeHandle& node_handle) {
  double timeout_sec = 10.0;
  node_handle.param("robot/geometry/robot_description_timeout_sec", timeout_sec, timeout_sec);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_sec);
  std::string urdf_xml;
  while (ros::ok() && ros::WallTime::now() < deadline) {
    if (node_handle.getParam("/robot_description", urdf_xml) && !urdf_xml.empty()) break;
    ros::WallDuration(0.05).sleep();
  }
  if (urdf_xml.empty()) throw std::runtime_error("Timed out waiting for /robot_description");

  std::string base_link = "base";
  node_handle.param<std::string>("robot/geometry/base_link", base_link, base_link);
  XmlRpc::XmlRpcValue geometry;
  if (!node_handle.getParam("robot/collision_geometry", geometry) ||
      geometry.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      !geometry.hasMember("arm_spheres") || !geometry.hasMember("body")) {
    throw std::runtime_error("robot/collision_geometry must define arm_spheres and body");
  }
  return std::make_unique<ArmCollisionModel>(urdf_xml, base_link,
                                             parseSpheres(geometry["arm_spheres"]),
                                             parseObstacles(geometry["body"]));
}

ArmCollisionModel::ArmCollisionModel(const std::string& urdf_xml, const std::string& base_link,
                                     const std::vector<SphereSpec>& sphere_template,
                                     const std::vector<Obstacle>& body_obstacles)
    : base_link_(base_link), body_obstacles_(body_obstacles) {
  urdf::Model model;
  if (!model.initString(urdf_xml)) throw std::runtime_error("Failed to parse the URDF");
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::runtime_error("Failed to build a KDL tree from the URDF");
  }
  buildArm(tree, "left", sphere_template, left_arm_);
  buildArm(tree, "right", sphere_template, right_arm_);
  if (left_arm_.spheres.size() != right_arm_.spheres.size()) {
    throw std::invalid_argument("collision geometry: both arms need the same number of spheres "
                                "(side-specific entries must come in left/right pairs)");
  }
}

void ArmCollisionModel::buildArm(const KDL::Tree& tree, const std::string& side,
                                 const std::vector<SphereSpec>& sphere_template,
                                 ArmChain& arm_chain) {
  arm_chain.side = side;
  const std::string tip_link = side + "_hand";
  if (!tree.getChain(base_link_, tip_link, arm_chain.chain)) {
    throw std::runtime_error("Failed to extract the KDL chain " + base_link_ + " -> " + tip_link);
  }
  // Joint index of every segment, checking the expected joint order (s0 s1 e0 e1 w0 w1 w2).
  const unsigned int segment_count = arm_chain.chain.getNrOfSegments();
  int joint_index = 0;
  arm_chain.segment_joint_index.assign(segment_count, -1);
  for (unsigned int segment = 0; segment < segment_count; ++segment) {
    const KDL::Joint& joint = arm_chain.chain.getSegment(segment).getJoint();
    if (joint.getType() == KDL::Joint::None) continue;
    if (joint_index >= kArmJoints) throw std::runtime_error("Arm chain has more than 7 joints");
    const std::string expected_name = side + "_" + kArmJointSuffixes[joint_index];
    if (joint.getName() != expected_name) {
      throw std::runtime_error("Arm chain joint order mismatch: expected " + expected_name +
                               ", got " + joint.getName());
    }
    arm_chain.segment_joint_index[segment] = joint_index++;
  }
  if (joint_index != kArmJoints) throw std::runtime_error("Arm chain does not have 7 joints");

  arm_chain.spheres.clear();
  arm_chain.sphere_segment_index.clear();
  for (const SphereSpec& spec : sphere_template) {
    if (!spec.side.empty() && spec.side != side) continue;
    SphereSpec expanded = spec;
    expanded.link = expandSide(spec.link, side);
    int carrying_segment = -1;
    for (unsigned int segment = 0; segment < segment_count; ++segment) {
      if (arm_chain.chain.getSegment(segment).getName() == expanded.link) {
        carrying_segment = static_cast<int>(segment);
        break;
      }
    }
    if (carrying_segment < 0) {
      throw std::runtime_error("Collision sphere link " + expanded.link +
                               " is not on the chain " + base_link_ + " -> " + tip_link);
    }
    arm_chain.spheres.push_back(expanded);
    arm_chain.sphere_segment_index.push_back(carrying_segment);
  }
  arm_chain.fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm_chain.chain);
  arm_chain.segment_frames.assign(segment_count, KDL::Frame::Identity());
  arm_chain.joint_scratch.resize(kArmJoints);
}

const ArmCollisionModel::ArmChain& ArmCollisionModel::armChain(const std::string& side) const {
  if (side == "left") return left_arm_;
  if (side == "right") return right_arm_;
  throw std::invalid_argument("side must be 'left' or 'right'");
}

void ArmCollisionModel::runForwardKinematics(const ArmChain& arm_chain,
                                             const JointVector& joint_positions) const {
  for (int joint = 0; joint < kArmJoints; ++joint) {
    arm_chain.joint_scratch(joint) = joint_positions[joint];
  }
  if (arm_chain.fk_solver->JntToCart(arm_chain.joint_scratch, arm_chain.segment_frames) < 0) {
    throw std::runtime_error("KDL forward kinematics failed for the " + arm_chain.side + " arm");
  }
}

void ArmCollisionModel::centersOnly(const std::string& side, const JointVector& joint_positions,
                                    std::vector<Eigen::Vector3d>& centers) const {
  const ArmChain& arm_chain = armChain(side);
  runForwardKinematics(arm_chain, joint_positions);
  centers.resize(arm_chain.spheres.size());
  for (std::size_t sphere = 0; sphere < arm_chain.spheres.size(); ++sphere) {
    const KDL::Frame& link_frame = arm_chain.segment_frames[arm_chain.sphere_segment_index[sphere]];
    centers[sphere] = toEigen(link_frame * toKdl(arm_chain.spheres[sphere].offset));
  }
}

/**
 * Point Jacobian of a sphere centre p attached to segment s: for every revolute joint j
 * at or before s, with unit axis z_j and origin o_j in the base frame,
 *     dp/dq_j = z_j x (p - o_j),
 * and zero for joints after the sphere (they do not move it).
 */
void ArmCollisionModel::evaluate(const std::string& side, const JointVector& joint_positions,
                                 std::vector<SphereState>& sphere_states) const {
  const ArmChain& arm_chain = armChain(side);
  runForwardKinematics(arm_chain, joint_positions);
  // Joint axes and origins in the base frame: the joint of segment s is expressed in
  // the frame of segment s-1 (identity for the first segment).
  std::array<Eigen::Vector3d, kArmJoints> joint_axes;
  std::array<Eigen::Vector3d, kArmJoints> joint_origins;
  std::array<int, kArmJoints> joint_segments{};
  for (unsigned int segment = 0; segment < arm_chain.chain.getNrOfSegments(); ++segment) {
    const int joint = arm_chain.segment_joint_index[segment];
    if (joint < 0) continue;
    const KDL::Frame parent_frame =
        segment == 0 ? KDL::Frame::Identity() : arm_chain.segment_frames[segment - 1];
    const KDL::Joint& kdl_joint = arm_chain.chain.getSegment(segment).getJoint();
    joint_axes[joint] = toEigen(parent_frame.M * kdl_joint.JointAxis());
    joint_origins[joint] = toEigen(parent_frame * kdl_joint.JointOrigin());
    joint_segments[joint] = static_cast<int>(segment);
  }
  sphere_states.resize(arm_chain.spheres.size());
  for (std::size_t sphere = 0; sphere < arm_chain.spheres.size(); ++sphere) {
    const int carrying_segment = arm_chain.sphere_segment_index[sphere];
    SphereState& state = sphere_states[sphere];
    state.center = toEigen(arm_chain.segment_frames[carrying_segment] *
                           toKdl(arm_chain.spheres[sphere].offset));
    state.jacobian.setZero();
    for (int joint = 0; joint < kArmJoints; ++joint) {
      if (joint_segments[joint] > carrying_segment) continue;  // a joint after the sphere does not move it
      state.jacobian.col(joint) = joint_axes[joint].cross(state.center - joint_origins[joint]);
    }
  }
}

}  // namespace mpc_geometry
