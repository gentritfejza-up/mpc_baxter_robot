#include "baxter_dynamics/lagrange_euler.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace baxter {

LagrangeEuler::LagrangeEuler(
    const std::string &limb, ros::NodeHandle &nh,
    std::shared_ptr<ForwardKinematics> forward_kinematics)
    : limb_(limb), forward_kinematics_(std::move(forward_kinematics)) {
  loadJointNames(nh);
  loadLinkInertialParameters(nh);
  loadTorqueLimits(nh);
  pseudo_inertia_ = buildPseudoInertiaMatrices();
}

LagrangeEuler::LagrangeEuler(const std::string &limb, ros::NodeHandle &nh)
    : LagrangeEuler(limb, nh, std::make_shared<ForwardKinematics>(limb, nh)) {}

// ---------------------------------------------------------------------------
// Parameter loading
// ---------------------------------------------------------------------------

void LagrangeEuler::loadJointNames(ros::NodeHandle &nh) {
  if (!nh.getParam("robot/limb/" + limb_, joint_names_) ||
      static_cast<int>(joint_names_.size()) != kNumLinks) {
    ROS_ERROR_STREAM("robot/limb/" << limb_ << " must list exactly "
                                   << kNumLinks << " joint names.");
    throw std::runtime_error("Invalid joint name list for limb: " + limb_);
  }
}

void LagrangeEuler::loadTorqueLimits(ros::NodeHandle &nh) {
  nh.param("robot/torque_limits/shoulder_elbow_max",
           torque_limit_shoulder_elbow_max_, 50.0);
  nh.param("robot/torque_limits/shoulder_elbow_min",
           torque_limit_shoulder_elbow_min_, -50.0);
  nh.param("robot/torque_limits/s1_max", torque_limit_s1_max_, 50.0);
  nh.param("robot/torque_limits/s1_min", torque_limit_s1_min_, -50.0);
  nh.param("robot/torque_limits/wrist_max", torque_limit_wrist_max_, 20.0);
  nh.param("robot/torque_limits/wrist_min", torque_limit_wrist_min_, -20.0);
}

void LagrangeEuler::loadLinkInertialParameters(ros::NodeHandle &nh) {
  // Every per-link table in the YAML is a flat list with one entry per link.
  const auto load_per_link = [&nh](const std::string &key) {
    std::vector<double> values;
    if (!nh.getParam(key, values) ||
        static_cast<int>(values.size()) != kNumLinks) {
      ROS_ERROR_STREAM("Parameter " << key << " must hold " << kNumLinks
                                    << " values.");
      throw std::runtime_error("Invalid link parameter: " + key);
    }
    return values;
  };

  const std::vector<double> Ixx = load_per_link("robot/limb/inertia/Ixx");
  const std::vector<double> Iyy = load_per_link("robot/limb/inertia/Iyy");
  const std::vector<double> Izz = load_per_link("robot/limb/inertia/Izz");
  const std::vector<double> Ixy = load_per_link("robot/limb/inertia/Ixy");
  const std::vector<double> Iyz = load_per_link("robot/limb/inertia/Iyz");
  const std::vector<double> Izx = load_per_link("robot/limb/inertia/Izx");
  const std::vector<double> masses = load_per_link("robot/limb/link_masses");
  const std::vector<double> com_x = load_per_link("robot/limb/com_x");
  const std::vector<double> com_y = load_per_link("robot/limb/com_y");
  const std::vector<double> com_z = load_per_link("robot/limb/com_z");
  const std::vector<double> com_w = load_per_link("robot/limb/com_w");

  for (int link = 0; link < kNumLinks; ++link) {
    LinkInertialParameters &params = links_[link];
    params.mass = masses[link];
    params.centre_of_mass << com_x[link], com_y[link], com_z[link], com_w[link];
    params.Ixx = Ixx[link];
    params.Iyy = Iyy[link];
    params.Izz = Izz[link];
    params.Ixy = Ixy[link];
    params.Iyz = Iyz[link];
    params.Izx = Izx[link];
  }

  std::vector<double> Q_j_row_major;
  if (!nh.getParam("robot/limb/Q_j", Q_j_row_major) ||
      Q_j_row_major.size() != 16) {
    ROS_ERROR("Parameter robot/limb/Q_j must hold 16 values (row major).");
    throw std::runtime_error("Invalid Q_j matrix parameter.");
  }
  Q_j_ = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
      Q_j_row_major.data());
}

// ---------------------------------------------------------------------------
// Model matrices
// ---------------------------------------------------------------------------

LagrangeEuler::LinkMatrices LagrangeEuler::buildPseudoInertiaMatrices() const {
  // J_i = int r r^T dm about the DH frame origin. The YAML tensor is about the
  // centre of mass with the URDF sign convention (I_xy = -int xy dm), hence
  // the sign flip on the products of inertia and the parallel-axis shift
  // m c c^T (see lagrange_euler.hpp for the derivation). Without both
  // corrections M is up to 30 % low on the wrist and ~6 % low on the
  // shoulder (verified against KDL on the Baxter URDF).
  LinkMatrices pseudo_inertia;
  pseudo_inertia.reserve(kNumLinks);
  for (const LinkInertialParameters &link : links_) {
    const double m = link.mass;
    const Eigen::Vector3d c = link.centre_of_mass.head<3>();

    Eigen::Matrix3d second_moment;
    second_moment << (-link.Ixx + link.Iyy + link.Izz) / 2, -link.Ixy, -link.Izx,
        -link.Ixy, (link.Ixx - link.Iyy + link.Izz) / 2, -link.Iyz,
        -link.Izx, -link.Iyz, (link.Ixx + link.Iyy - link.Izz) / 2;
    second_moment += m * c * c.transpose();

    Eigen::Matrix4d J_i = Eigen::Matrix4d::Zero();
    J_i.topLeftCorner<3, 3>() = second_moment;
    J_i.topRightCorner<3, 1>() = m * c;
    J_i.bottomLeftCorner<1, 3>() = m * c.transpose();
    J_i(3, 3) = m;
    pseudo_inertia.push_back(J_i);
  }
  return pseudo_inertia;
}

LagrangeEuler::LinkMatrices LagrangeEuler::computeJointDerivativeOperators(
    const LinkMatrices &link_transforms) const {
  // D_j = ^0T_{j-1} Q_j (^0T_{j-1})^{-1}; link_transforms[j] is ^0T_j with
  // the zero-based joint index j, i.e. the frame the joint axis is fixed in.
  LinkMatrices joint_operators;
  joint_operators.reserve(kNumLinks);
  for (int joint = 0; joint < kNumLinks; ++joint) {
    const Eigen::Matrix4d &base_to_joint_frame = link_transforms[joint];
    joint_operators.push_back(base_to_joint_frame * Q_j_ *
                              base_to_joint_frame.inverse());
  }
  return joint_operators;
}

LagrangeEuler::LinkMatrices
LagrangeEuler::computeUij(const LinkMatrices &link_transforms,
                          const LinkMatrices &joint_operators) const {
  LinkMatrices U_ij(kNumLinks * kNumLinks, Eigen::Matrix4d::Zero());
  for (int i = 0; i < kNumLinks; ++i) {
    const Eigen::Matrix4d &base_to_link = link_transforms[i + 1]; // ^0T_i
    for (int j = 0; j <= i; ++j) {
      U_ij[indexUij(i, j)] = joint_operators[j] * base_to_link;
    }
  }
  return U_ij;
}

LagrangeEuler::LinkMatrices
LagrangeEuler::computeUijk(const LinkMatrices &U_ij,
                           const LinkMatrices &joint_operators) const {
  // U_ijk = D_j U_ik for k >= j and D_k U_ij for j >= k (both require i >=
  // max(j, k)); the two agree for j == k.
  LinkMatrices U_ijk(kNumLinks * kNumLinks * kNumLinks,
                     Eigen::Matrix4d::Zero());
  for (int i = 0; i < kNumLinks; ++i) {
    for (int j = 0; j <= i; ++j) {
      for (int k = 0; k <= i; ++k) {
        U_ijk[indexUijk(i, j, k)] =
            k >= j ? joint_operators[j] * U_ij[indexUij(i, k)]
                   : joint_operators[k] * U_ij[indexUij(i, j)];
      }
    }
  }
  return U_ijk;
}

LagrangeEuler::LinkMatrices
LagrangeEuler::computeInertiaWeightedUij(const LinkMatrices &U_ij) const {
  LinkMatrices weighted_U_ij(kNumLinks * kNumLinks, Eigen::Matrix4d::Zero());
  for (int j = 0; j < kNumLinks; ++j) {
    for (int i = 0; i <= j; ++i) {
      weighted_U_ij[indexUij(j, i)] =
          pseudo_inertia_[j] * U_ij[indexUij(j, i)].transpose();
    }
  }
  return weighted_U_ij;
}

LagrangeEuler::JointMatrix
LagrangeEuler::computeMassMatrix(const LinkMatrices &U_ij,
                                 const LinkMatrices &weighted_U_ij) const {
  JointMatrix mass_matrix = JointMatrix::Zero();
  for (int i = 0; i < kNumLinks; ++i) {
    for (int k = 0; k < kNumLinks; ++k) {
      double M_ik = 0.0;
      for (int j = std::max(i, k); j < kNumLinks; ++j) {
        M_ik += (U_ij[indexUij(j, k)] * weighted_U_ij[indexUij(j, i)]).trace();
      }
      mass_matrix(i, k) = M_ik;
    }
  }
  return mass_matrix;
}

LagrangeEuler::JointVector LagrangeEuler::computeCoriolisVector(
    const LinkMatrices &U_ijk, const LinkMatrices &weighted_U_ij,
    const JointVector &joint_velocities) const {
  JointVector coriolis = JointVector::Zero();
  for (int i = 0; i < kNumLinks; ++i) {
    for (int k = 0; k < kNumLinks; ++k) {
      for (int m = 0; m < kNumLinks; ++m) {
        double h_ikm = 0.0;
        for (int j = std::max({i, k, m}); j < kNumLinks; ++j) {
          h_ikm += (U_ijk[indexUijk(j, k, m)] * weighted_U_ij[indexUij(j, i)])
                       .trace();
        }
        coriolis(i) += h_ikm * joint_velocities(k) * joint_velocities(m);
      }
    }
  }
  return coriolis;
}

LagrangeEuler::JointVector
LagrangeEuler::computeGravityVector(const LinkMatrices &U_ij) const {
  const Eigen::Vector4d gravity(0.0, 0.0, -kGravity, 0.0);
  JointVector gravity_torque = JointVector::Zero();
  for (int i = 0; i < kNumLinks; ++i) {
    double G_i = 0.0;
    for (int j = i; j < kNumLinks; ++j) {
      const Eigen::Vector4d com_sensitivity =
          U_ij[indexUij(j, i)] * links_[j].centre_of_mass; // U_ji r_j
      G_i += links_[j].mass * gravity.dot(com_sensitivity);
    }
    gravity_torque(i) = -G_i;
  }
  return gravity_torque;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

LagrangeEuler::JointVector LagrangeEuler::mapToJointVector(
    const std::unordered_map<std::string, double> &values,
    const char *quantity_name) const {
  JointVector ordered;
  for (int i = 0; i < kNumLinks; ++i) {
    const auto value_it = values.find(joint_names_[i]);
    if (value_it == values.end()) {
      throw std::runtime_error("Joint " + joint_names_[i] + " missing from " +
                               quantity_name + ".");
    }
    ordered(i) = value_it->second;
  }
  return ordered;
}

std::unordered_map<std::string, double>
LagrangeEuler::clampToTorqueLimits(const JointVector &torques) const {
  // Written as max(min(.)) rather than std::clamp so that an inverted limit
  // pair degrades to the lower bound instead of undefined behaviour.
  const auto clamp_between = [](double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
  };

  std::unordered_map<std::string, double> clamped;
  clamped.reserve(joint_names_.size());

  for (int i = 0; i < kNumLinks; ++i) {
    const std::string &joint_name = joint_names_[i];
    double torque = torques(i);

    if (joint_name == limb_ + "_s1") {
      torque = clamp_between(torque, torque_limit_s1_min_, torque_limit_s1_max_);
    } else if (joint_name == limb_ + "_s0" || joint_name == limb_ + "_e0" ||
               joint_name == limb_ + "_e1") {
      torque = clamp_between(torque, torque_limit_shoulder_elbow_min_,
                             torque_limit_shoulder_elbow_max_);
    } else if (joint_name == limb_ + "_w0" || joint_name == limb_ + "_w1" ||
               joint_name == limb_ + "_w2") {
      torque = clamp_between(torque, torque_limit_wrist_min_,
                             torque_limit_wrist_max_);
    }

    clamped.emplace(joint_name, torque);
  }

  return clamped;
}

std::unordered_map<std::string, double> LagrangeEuler::computeTorques(
    const std::unordered_map<std::string, double> &joint_angles,
    const std::unordered_map<std::string, double> &joint_velocities,
    const std::unordered_map<std::string, double> &joint_accelerations) const {
  const JointVector qd = mapToJointVector(joint_velocities, "joint_velocities");
  const JointVector qdd =
      mapToJointVector(joint_accelerations, "joint_accelerations");

  const LinkMatrices link_transforms =
      forward_kinematics_->solveIntermediateFK(joint_angles);
  const LinkMatrices joint_operators =
      computeJointDerivativeOperators(link_transforms);
  const LinkMatrices U_ij = computeUij(link_transforms, joint_operators);
  const LinkMatrices U_ijk = computeUijk(U_ij, joint_operators);
  const LinkMatrices weighted_U_ij = computeInertiaWeightedUij(U_ij);

  const JointMatrix M = computeMassMatrix(U_ij, weighted_U_ij);
  const JointVector C = computeCoriolisVector(U_ijk, weighted_U_ij, qd);
  const JointVector g = computeGravityVector(U_ij);

  const JointVector tau = M * qdd + C + g;
  return clampToTorqueLimits(tau);
}

} // namespace baxter
