#include "baxter_dynamics/lagrange_euler.hpp"

#include <ros/console.h>

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace baxter {

Eigen::Matrix4d LagrangeEuler::jointOperator(JointType type) {
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  if (type == JointType::kRevolute) {
    Q(0, 1) = -1.0;  // d/dtheta of Rz(theta) at the frame origin.
    Q(1, 0) = 1.0;
  } else {
    Q(2, 3) = 1.0;   // d/dd of Tz(d).
  }
  return Q;
}

LagrangeEuler::LagrangeEuler(ArmDescription arm)
    : arm_(std::move(arm)),
      joint_count_(arm_.jointCount()),
      forward_kinematics_(arm_) {
  arm_.validate();
  joint_names_ = arm_.jointNames();
  gravity_ << arm_.gravity, 0.0;
  effort_limits_.reserve(joint_count_);
  viscous_damping_.reserve(joint_count_);
  joint_Q_.reserve(joint_count_);
  centre_of_mass_.reserve(joint_count_);
  for (const JointDescription& joint : arm_.joints) {
    effort_limits_.push_back(joint.effort_limit);
    viscous_damping_.push_back(joint.viscous_damping);
    joint_Q_.push_back(jointOperator(joint.type));
    centre_of_mass_.emplace_back(joint.link.centre_of_mass.x(), joint.link.centre_of_mass.y(),
                                 joint.link.centre_of_mass.z(), 1.0);
  }
  pseudo_inertia_ = buildPseudoInertiaMatrices();

  workspace_.transforms.assign(joint_count_ + 1, Eigen::Matrix4d::Identity());
  workspace_.joint_operators.assign(joint_count_, Eigen::Matrix4d::Zero());
  workspace_.U_ij.assign(joint_count_ * joint_count_, Eigen::Matrix4d::Zero());
  workspace_.U_ijk.assign(joint_count_ * joint_count_ * joint_count_, Eigen::Matrix4d::Zero());
  workspace_.weighted_U_ij.assign(joint_count_ * joint_count_, Eigen::Matrix4d::Zero());
}

std::string LagrangeEuler::description() const {
  std::ostringstream text;
  text << "Lagrange-Euler DH model of arm '" << arm_.name << "': " << joint_count_
       << " joints (" << joint_names_.front() << " .. " << joint_names_.back()
       << "), gravity (" << arm_.gravity.transpose() << ") m/s^2";
  return text.str();
}

// ---------------------------------------------------------------------------
// Constant model matrices
// ---------------------------------------------------------------------------

LagrangeEuler::LinkMatrices LagrangeEuler::buildPseudoInertiaMatrices() const {
  // J_i = int r r^T dm about the DH frame origin. The description holds the
  // tensor about the centre of mass with the URDF sign convention
  // (I_xy = -int xy dm), hence the sign flip on the products of inertia and the
  // parallel-axis shift m c c^T (see lagrange_euler.hpp). Without both
  // corrections M is up to 30 % low on the Baxter wrist and ~6 % low on the
  // shoulder (verified against KDL on the URDF).
  LinkMatrices pseudo_inertia;
  pseudo_inertia.reserve(joint_count_);
  for (const JointDescription& joint : arm_.joints) {
    const LinkInertialParameters& link = joint.link;
    const double m = link.mass;
    const Eigen::Vector3d& c = link.centre_of_mass;

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

// ---------------------------------------------------------------------------
// Per-evaluation tables
// ---------------------------------------------------------------------------

void LagrangeEuler::computeJointDerivativeOperators(const LinkMatrices& transforms,
                                                    LinkMatrices& joint_operators) const {
  // D_j = ^0T_{j-1} Q_j (^0T_{j-1})^{-1}; transforms[j] is ^0T_j with the
  // zero-based joint index j, i.e. the frame the joint axis is fixed in. A
  // homogeneous transform is inverted in closed form.
  for (std::size_t joint = 0; joint < joint_count_; ++joint) {
    const Eigen::Matrix4d& base_to_joint_frame = transforms[joint];
    Eigen::Matrix4d inverse = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d rotation_transposed =
        base_to_joint_frame.topLeftCorner<3, 3>().transpose();
    inverse.topLeftCorner<3, 3>() = rotation_transposed;
    inverse.topRightCorner<3, 1>() =
        -rotation_transposed * base_to_joint_frame.topRightCorner<3, 1>();
    joint_operators[joint] = base_to_joint_frame * joint_Q_[joint] * inverse;
  }
}

void LagrangeEuler::computeUij(const LinkMatrices& transforms,
                               const LinkMatrices& joint_operators, LinkMatrices& U_ij) const {
  for (std::size_t i = 0; i < joint_count_; ++i) {
    const Eigen::Matrix4d& base_to_link = transforms[i + 1];  // ^0T_i
    for (std::size_t j = 0; j <= i; ++j) {
      U_ij[indexUij(i, j)] = joint_operators[j] * base_to_link;
    }
  }
}

void LagrangeEuler::computeUijk(const LinkMatrices& U_ij, const LinkMatrices& joint_operators,
                                LinkMatrices& U_ijk) const {
  // U_ijk = D_j U_ik for k >= j; U_ikj is the same matrix (the second
  // derivative is symmetric), so only the upper triangle is computed.
  for (std::size_t i = 0; i < joint_count_; ++i) {
    for (std::size_t j = 0; j <= i; ++j) {
      for (std::size_t k = j; k <= i; ++k) {
        const Eigen::Matrix4d& value = U_ijk[indexUijk(i, j, k)] =
            joint_operators[j] * U_ij[indexUij(i, k)];
        if (k != j) U_ijk[indexUijk(i, k, j)] = value;
      }
    }
  }
}

void LagrangeEuler::computeInertiaWeightedUij(const LinkMatrices& U_ij,
                                              LinkMatrices& weighted_U_ij) const {
  for (std::size_t j = 0; j < joint_count_; ++j) {
    for (std::size_t i = 0; i <= j; ++i) {
      weighted_U_ij[indexUij(j, i)] = pseudo_inertia_[j] * U_ij[indexUij(j, i)].transpose();
    }
  }
}

void LagrangeEuler::computeMassMatrix(const LinkMatrices& U_ij, const LinkMatrices& weighted_U_ij,
                                      Eigen::MatrixXd& mass_matrix) const {
  // M is symmetric: evaluate i <= k and mirror.
  for (std::size_t i = 0; i < joint_count_; ++i) {
    for (std::size_t k = i; k < joint_count_; ++k) {
      double M_ik = 0.0;
      for (std::size_t j = k; j < joint_count_; ++j) {
        M_ik += (U_ij[indexUij(j, k)] * weighted_U_ij[indexUij(j, i)]).trace();
      }
      mass_matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) = M_ik;
      mass_matrix(static_cast<Eigen::Index>(k), static_cast<Eigen::Index>(i)) = M_ik;
    }
  }
}

void LagrangeEuler::computeCoriolisVector(const LinkMatrices& U_ijk,
                                          const LinkMatrices& weighted_U_ij,
                                          const Eigen::VectorXd& velocities,
                                          Eigen::VectorXd& coriolis) const {
  coriolis.setZero();
  for (std::size_t i = 0; i < joint_count_; ++i) {
    double C_i = 0.0;
    for (std::size_t k = 0; k < joint_count_; ++k) {
      for (std::size_t m = k; m < joint_count_; ++m) {
        double h_ikm = 0.0;
        for (std::size_t j = std::max({i, k, m}); j < joint_count_; ++j) {
          h_ikm += (U_ijk[indexUijk(j, k, m)] * weighted_U_ij[indexUij(j, i)]).trace();
        }
        const double velocity_product = velocities(static_cast<Eigen::Index>(k)) *
                                        velocities(static_cast<Eigen::Index>(m));
        // h_ikm = h_imk: the off-diagonal pairs appear twice in the double sum.
        C_i += (k == m ? 1.0 : 2.0) * h_ikm * velocity_product;
      }
    }
    coriolis(static_cast<Eigen::Index>(i)) = C_i;
  }
}

void LagrangeEuler::computeGravityVector(const LinkMatrices& U_ij,
                                         Eigen::VectorXd& gravity) const {
  for (std::size_t i = 0; i < joint_count_; ++i) {
    double G_i = 0.0;
    for (std::size_t j = i; j < joint_count_; ++j) {
      const Eigen::Vector4d com_sensitivity = U_ij[indexUij(j, i)] * centre_of_mass_[j];  // U_ji r_j
      G_i += arm_.joints[j].link.mass * gravity_.dot(com_sensitivity);
    }
    gravity(static_cast<Eigen::Index>(i)) = -G_i;
  }
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void LagrangeEuler::evaluate(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                             JointSpaceDynamics& dynamics) const {
  const auto n = static_cast<Eigen::Index>(joint_count_);
  if (positions.size() != n || velocities.size() != n) {
    throw std::runtime_error("Lagrange-Euler: expected " + std::to_string(joint_count_) +
                             " joint positions and velocities, got " +
                             std::to_string(positions.size()) + " and " +
                             std::to_string(velocities.size()));
  }
  if (!positions.allFinite() || !velocities.allFinite()) {
    throw std::runtime_error("Lagrange-Euler: joint state is not finite");
  }
  const std::size_t limit_violations = forward_kinematics_.countLimitViolations(positions);
  if (limit_violations > 0) {
    ROS_WARN_STREAM_THROTTLE(1.0, "Arm '" << arm_.name << "': " << limit_violations
                                          << " joint(s) outside the configured limits");
  }

  Workspace& ws = workspace_;
  forward_kinematics_.compute(positions, ws.transforms);
  computeJointDerivativeOperators(ws.transforms, ws.joint_operators);
  computeUij(ws.transforms, ws.joint_operators, ws.U_ij);
  computeUijk(ws.U_ij, ws.joint_operators, ws.U_ijk);
  computeInertiaWeightedUij(ws.U_ij, ws.weighted_U_ij);

  dynamics.resize(joint_count_);
  computeMassMatrix(ws.U_ij, ws.weighted_U_ij, dynamics.mass_matrix);
  computeCoriolisVector(ws.U_ijk, ws.weighted_U_ij, velocities, dynamics.coriolis_torque);
  computeGravityVector(ws.U_ij, dynamics.gravity_torque);
}

}  // namespace baxter
