#include "baxter_dynamics/arm_description.hpp"

#include <ros/ros.h>

#include <cmath>
#include <set>
#include <stdexcept>

namespace baxter {

std::vector<std::string> ArmDescription::jointNames() const {
  std::vector<std::string> names;
  names.reserve(joints.size());
  for (const JointDescription& joint : joints) names.push_back(joint.name);
  return names;
}

void ArmDescription::validate() const {
  if (joints.empty()) {
    throw std::invalid_argument("Arm '" + name + "' has no joints");
  }
  if (!gravity.allFinite()) {
    throw std::invalid_argument("Arm '" + name + "': gravity vector is not finite");
  }
  std::set<std::string> seen_names;
  for (const JointDescription& joint : joints) {
    if (joint.name.empty() || !seen_names.insert(joint.name).second) {
      throw std::invalid_argument("Arm '" + name + "': empty or duplicate joint name '" +
                                  joint.name + "'");
    }
    const LinkInertialParameters& link = joint.link;
    const bool finite =
        std::isfinite(joint.dh.d_offset) && std::isfinite(joint.dh.theta_offset) &&
        std::isfinite(joint.dh.a) && std::isfinite(joint.dh.alpha) &&
        std::isfinite(joint.lower_limit) && std::isfinite(joint.upper_limit) &&
        std::isfinite(joint.effort_limit) && std::isfinite(joint.viscous_damping) &&
        std::isfinite(link.mass) && link.centre_of_mass.allFinite() &&
        std::isfinite(link.Ixx) && std::isfinite(link.Iyy) && std::isfinite(link.Izz) &&
        std::isfinite(link.Ixy) && std::isfinite(link.Iyz) && std::isfinite(link.Izx);
    if (!finite) {
      throw std::invalid_argument("Joint '" + joint.name + "': a parameter is not finite");
    }
    if (joint.lower_limit > joint.upper_limit) {
      throw std::invalid_argument("Joint '" + joint.name + "': inverted position limits");
    }
    if (joint.effort_limit <= 0.0) {
      throw std::invalid_argument("Joint '" + joint.name + "': effort limit must be positive");
    }
    if (joint.viscous_damping < 0.0) {
      throw std::invalid_argument("Joint '" + joint.name + "': negative viscous damping");
    }
    if (link.mass < 0.0) {
      throw std::invalid_argument("Joint '" + joint.name + "': negative link mass");
    }
  }
}

LinkInertialParameters combineBodies(const LinkInertialParameters& first,
                                     const LinkInertialParameters& second) {
  const auto tensor = [](const LinkInertialParameters& body) {
    Eigen::Matrix3d I;
    I << body.Ixx, body.Ixy, body.Izx, body.Ixy, body.Iyy, body.Iyz, body.Izx, body.Iyz, body.Izz;
    return I;
  };
  LinkInertialParameters combined;
  combined.mass = first.mass + second.mass;
  combined.centre_of_mass = Eigen::Vector3d::Zero();
  if (combined.mass > 0.0) {
    combined.centre_of_mass =
        (first.mass * first.centre_of_mass + second.mass * second.centre_of_mass) / combined.mass;
  }
  Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
  for (const LinkInertialParameters* body : {&first, &second}) {
    const Eigen::Vector3d d = body->centre_of_mass - combined.centre_of_mass;
    I += tensor(*body) + body->mass * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
  }
  combined.Ixx = I(0, 0);
  combined.Iyy = I(1, 1);
  combined.Izz = I(2, 2);
  combined.Ixy = I(0, 1);
  combined.Iyz = I(1, 2);
  combined.Izx = I(2, 0);
  return combined;
}

void ArmDescription::addTipPayload(const LinkInertialParameters& payload) {
  if (joints.empty()) {
    throw std::invalid_argument("Arm '" + name + "' has no joints to attach a payload to");
  }
  if (!(payload.mass >= 0.0) || !payload.centre_of_mass.allFinite()) {
    throw std::invalid_argument("Arm '" + name + "': invalid tip payload");
  }
  joints.back().link = combineBodies(joints.back().link, payload);
}

namespace {

/// Per-joint tables are keyed by the joint name without the `<arm>_` prefix
/// (Baxter: `left_s0` -> `s0`); the full name is accepted as well.
std::vector<std::string> jointKeys(const std::string& joint_name, const std::string& arm) {
  const std::string prefix = arm + "_";
  std::vector<std::string> keys;
  if (joint_name.compare(0, prefix.size(), prefix) == 0) {
    keys.push_back(joint_name.substr(prefix.size()));
  }
  keys.push_back(joint_name);
  return keys;
}

template <typename T>
bool getPerJoint(ros::NodeHandle& nh, const std::string& table, const std::string& joint_name,
                 const std::string& arm, T& value) {
  for (const std::string& key : jointKeys(joint_name, arm)) {
    if (nh.getParam(table + "/" + key, value)) return true;
  }
  return false;
}

std::vector<double> requireList(ros::NodeHandle& nh, const std::string& key,
                                std::size_t expected_size) {
  std::vector<double> values;
  if (!nh.getParam(key, values) || values.size() != expected_size) {
    throw std::runtime_error("Parameter " + key + " must hold " +
                             std::to_string(expected_size) + " values");
  }
  return values;
}

}  // namespace

ArmDescription loadArmDescription(ros::NodeHandle& nh, const std::string& parameter_namespace,
                                  const std::string& arm) {
  const std::string ns = parameter_namespace + "/";
  ArmDescription description;
  description.name = arm;

  std::vector<std::string> joint_names;
  if (!nh.getParam(ns + "joints/" + arm, joint_names) || joint_names.empty()) {
    throw std::runtime_error("Parameter " + ns + "joints/" + arm +
                             " must list the joint names of the arm");
  }
  const std::size_t joint_count = joint_names.size();

  std::vector<double> gravity;
  if (nh.getParam(ns + "gravity", gravity)) {
    if (gravity.size() != 3) {
      throw std::runtime_error("Parameter " + ns + "gravity must hold 3 values");
    }
    description.gravity = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]);
  }

  const std::string links = ns + "links/";
  const std::vector<double> mass = requireList(nh, links + "mass", joint_count);
  const std::vector<double> com_x = requireList(nh, links + "com_x", joint_count);
  const std::vector<double> com_y = requireList(nh, links + "com_y", joint_count);
  const std::vector<double> com_z = requireList(nh, links + "com_z", joint_count);
  const std::vector<double> Ixx = requireList(nh, links + "Ixx", joint_count);
  const std::vector<double> Iyy = requireList(nh, links + "Iyy", joint_count);
  const std::vector<double> Izz = requireList(nh, links + "Izz", joint_count);
  const std::vector<double> Ixy = requireList(nh, links + "Ixy", joint_count);
  const std::vector<double> Iyz = requireList(nh, links + "Iyz", joint_count);
  const std::vector<double> Izx = requireList(nh, links + "Izx", joint_count);

  description.joints.reserve(joint_count);
  for (std::size_t index = 0; index < joint_count; ++index) {
    JointDescription joint;
    joint.name = joint_names[index];

    std::string type = "revolute";
    getPerJoint(nh, ns + "joint_types", joint.name, arm, type);
    if (type == "revolute") {
      joint.type = JointType::kRevolute;
    } else if (type == "prismatic") {
      joint.type = JointType::kPrismatic;
    } else {
      throw std::runtime_error("Joint " + joint.name + ": unknown joint type '" + type + "'");
    }

    std::vector<double> dh_row;
    if (!getPerJoint(nh, ns + "dh_parameters", joint.name, arm, dh_row) || dh_row.size() != 4) {
      throw std::runtime_error("Joint " + joint.name +
                               ": dh_parameters must hold [d, theta, a, alpha]");
    }
    joint.dh = DhParameters{dh_row[0], dh_row[1], dh_row[2], dh_row[3]};

    std::vector<double> limits;
    if (!getPerJoint(nh, ns + "angle_limits", joint.name, arm, limits) || limits.size() != 2) {
      throw std::runtime_error("Joint " + joint.name + ": angle_limits must hold [lower, upper]");
    }
    joint.lower_limit = limits[0];
    joint.upper_limit = limits[1];

    if (!getPerJoint(nh, ns + "effort_limits", joint.name, arm, joint.effort_limit)) {
      throw std::runtime_error("Joint " + joint.name + ": effort_limits entry is missing");
    }
    getPerJoint(nh, ns + "viscous_damping", joint.name, arm, joint.viscous_damping);

    joint.link.mass = mass[index];
    joint.link.centre_of_mass = Eigen::Vector3d(com_x[index], com_y[index], com_z[index]);
    joint.link.Ixx = Ixx[index];
    joint.link.Iyy = Iyy[index];
    joint.link.Izz = Izz[index];
    joint.link.Ixy = Ixy[index];
    joint.link.Iyz = Iyz[index];
    joint.link.Izx = Izx[index];
    description.joints.push_back(joint);
  }

  bool payload_enabled = false;
  nh.param(ns + "tip_payload/enabled", payload_enabled, payload_enabled);
  if (payload_enabled) {
    LinkInertialParameters payload;
    std::vector<double> com, inertia;
    if (!nh.getParam(ns + "tip_payload/mass", payload.mass) ||
        !nh.getParam(ns + "tip_payload/com", com) || com.size() != 3 ||
        !nh.getParam(ns + "tip_payload/inertia", inertia) || inertia.size() != 6) {
      throw std::runtime_error("Parameter " + ns +
                               "tip_payload needs mass, com [x, y, z] and inertia "
                               "[Ixx, Iyy, Izz, Ixy, Iyz, Izx]");
    }
    payload.centre_of_mass = Eigen::Vector3d(com[0], com[1], com[2]);
    payload.Ixx = inertia[0];
    payload.Iyy = inertia[1];
    payload.Izz = inertia[2];
    payload.Ixy = inertia[3];
    payload.Iyz = inertia[4];
    payload.Izx = inertia[5];
    description.addTipPayload(payload);
    ROS_INFO_STREAM("Arm '" << arm << "': lumped a " << payload.mass << " kg tip payload into "
                            << description.joints.back().name << " (link mass now "
                            << description.joints.back().link.mass << " kg)");
  }

  description.validate();
  return description;
}

}  // namespace baxter
