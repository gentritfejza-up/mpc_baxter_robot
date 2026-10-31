// Closed-form checks of the generic Lagrange-Euler model on synthetic arms that
// are not Baxter: a planar two-link RR arm with point masses (textbook M, C, g),
// a single prismatic joint lifting a mass, and a three-joint RPR chain whose M
// is checked against the kinetic-energy definition. No ROS master is needed.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <random>

#include "baxter_dynamics/lagrange_euler.hpp"

namespace {

constexpr double kGravity = 9.81;

baxter::JointDescription pointMassJoint(const std::string& name, baxter::JointType type,
                                        double d, double theta, double a, double alpha,
                                        double mass) {
  baxter::JointDescription joint;
  joint.name = name;
  joint.type = type;
  joint.dh = baxter::DhParameters{d, theta, a, alpha};
  joint.lower_limit = -10.0;
  joint.upper_limit = 10.0;
  joint.effort_limit = 100.0;
  joint.link.mass = mass;  // Point mass at the link frame origin (distal end).
  return joint;
}

/// Planar RR arm in the x-y plane of the base frame, gravity along -y.
baxter::ArmDescription planarArm(double l1, double l2, double m1, double m2) {
  baxter::ArmDescription arm;
  arm.name = "planar";
  arm.gravity = Eigen::Vector3d(0.0, -kGravity, 0.0);
  arm.joints.push_back(pointMassJoint("planar_j1", baxter::JointType::kRevolute, 0, 0, l1, 0, m1));
  arm.joints.push_back(pointMassJoint("planar_j2", baxter::JointType::kRevolute, 0, 0, l2, 0, m2));
  return arm;
}

}  // namespace

TEST(PlanarArm, MassCoriolisGravityMatchTextbook) {
  const double l1 = 0.8, l2 = 0.5, m1 = 2.0, m2 = 1.5;
  baxter::LagrangeEuler model(planarArm(l1, l2, m1, m2));
  ASSERT_EQ(model.jointCount(), 2u);

  std::mt19937 rng(7);
  std::uniform_real_distribution<double> angle(-3.0, 3.0);
  std::uniform_real_distribution<double> rate(-2.0, 2.0);
  baxter::JointSpaceDynamics dynamics;
  for (int sample = 0; sample < 50; ++sample) {
    Eigen::VectorXd q(2), qd(2);
    q << angle(rng), angle(rng);
    qd << rate(rng), rate(rng);
    model.evaluate(q, qd, dynamics);

    const double c2 = std::cos(q(1)), s2 = std::sin(q(1));
    // Point masses at the link ends: Spong, Hutchinson, Vidyasagar, "Robot
    // Modeling and Control", example of the two-link planar arm.
    Eigen::Matrix2d M_expected;
    M_expected << m1 * l1 * l1 + m2 * (l1 * l1 + l2 * l2 + 2 * l1 * l2 * c2),
        m2 * (l2 * l2 + l1 * l2 * c2), m2 * (l2 * l2 + l1 * l2 * c2), m2 * l2 * l2;
    Eigen::Vector2d C_expected;
    C_expected << -m2 * l1 * l2 * s2 * (2 * qd(0) * qd(1) + qd(1) * qd(1)),
        m2 * l1 * l2 * s2 * qd(0) * qd(0);
    Eigen::Vector2d g_expected;
    g_expected << (m1 + m2) * kGravity * l1 * std::cos(q(0)) +
                      m2 * kGravity * l2 * std::cos(q(0) + q(1)),
        m2 * kGravity * l2 * std::cos(q(0) + q(1));

    EXPECT_LT((dynamics.mass_matrix - M_expected).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((dynamics.coriolis_torque - C_expected).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((dynamics.gravity_torque - g_expected).cwiseAbs().maxCoeff(), 1e-9);
  }
}

TEST(PrismaticJoint, LiftingAMassAlongGravity) {
  baxter::ArmDescription arm;
  arm.name = "lift";
  arm.joints.push_back(pointMassJoint("lift_z", baxter::JointType::kPrismatic, 0.1, 0, 0, 0, 3.0));
  baxter::LagrangeEuler model(arm);

  baxter::JointSpaceDynamics dynamics;
  Eigen::VectorXd q(1), qd(1);
  q << 0.4;
  qd << 1.3;
  model.evaluate(q, qd, dynamics);
  // tau = m qdd + m g: the mass matrix is the mass, no Coriolis term, and the
  // gravity term is the weight (positive: pushing up along +z).
  EXPECT_NEAR(dynamics.mass_matrix(0, 0), 3.0, 1e-12);
  EXPECT_NEAR(dynamics.coriolis_torque(0), 0.0, 1e-12);
  EXPECT_NEAR(dynamics.gravity_torque(0), 3.0 * kGravity, 1e-12);
}

TEST(MixedChain, MassMatrixMatchesKineticEnergyAndCoriolisIsConsistent) {
  // R-P-R chain with distributed inertia: M must be symmetric positive definite,
  // the Coriolis term must satisfy the energy identity qd^T (Mdot - 2 h) qd = 0
  // written as qd^T C(q, qd) qd = 1/2 qd^T Mdot(q, qd) qd, with Mdot from finite
  // differences, and g must be the gradient of the potential energy.
  baxter::ArmDescription arm;
  arm.name = "rpr";
  arm.gravity = Eigen::Vector3d(0.0, 0.0, -kGravity);
  arm.joints.push_back(pointMassJoint("rpr_1", baxter::JointType::kRevolute, 0.3, 0.2, 0.1, 1.2, 2.0));
  arm.joints.push_back(pointMassJoint("rpr_2", baxter::JointType::kPrismatic, 0.2, 0.4, 0.05, -0.7, 1.0));
  arm.joints.push_back(pointMassJoint("rpr_3", baxter::JointType::kRevolute, 0.1, -0.3, 0.25, 0.5, 0.8));
  for (baxter::JointDescription& joint : arm.joints) {
    joint.link.centre_of_mass = Eigen::Vector3d(0.05, -0.02, 0.03);
    joint.link.Ixx = 0.02;
    joint.link.Iyy = 0.015;
    joint.link.Izz = 0.01;
    joint.link.Ixy = 0.001;
    joint.link.Iyz = -0.002;
    joint.link.Izx = 0.0005;
  }
  baxter::LagrangeEuler model(arm);

  std::mt19937 rng(11);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  baxter::JointSpaceDynamics at, plus, minus;
  const double step = 1e-6;
  for (int sample = 0; sample < 30; ++sample) {
    Eigen::VectorXd q(3), qd(3);
    for (int i = 0; i < 3; ++i) {
      q(i) = unit(rng);
      qd(i) = unit(rng);
    }
    model.evaluate(q, qd, at);
    EXPECT_LT((at.mass_matrix - at.mass_matrix.transpose()).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_GT(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(at.mass_matrix).eigenvalues().minCoeff(), 0.0);

    // Mdot = sum_k dM/dq_k qd_k by central differences.
    Eigen::MatrixXd M_dot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd potential_gradient(3);
    for (int k = 0; k < 3; ++k) {
      Eigen::VectorXd q_plus = q, q_minus = q;
      q_plus(k) += step;
      q_minus(k) -= step;
      model.evaluate(q_plus, qd, plus);
      model.evaluate(q_minus, qd, minus);
      M_dot += (plus.mass_matrix - minus.mass_matrix) / (2 * step) * qd(k);
    }
    const double power_identity = qd.dot(at.coriolis_torque) - 0.5 * qd.dot(M_dot * qd);
    EXPECT_NEAR(power_identity, 0.0, 1e-6);

    // g(q) = dV/dq with the potential energy V = -sum_j m_j g^T p_j(q), p_j the
    // centre of mass of link j in the base frame from the forward kinematics.
    const baxter::ForwardKinematics kinematics(model.arm());
    const auto potential_energy = [&](const Eigen::VectorXd& positions) {
      baxter::ForwardKinematics::Transforms transforms;
      kinematics.compute(positions, transforms);
      double energy = 0.0;
      for (std::size_t j = 0; j < model.arm().jointCount(); ++j) {
        const baxter::LinkInertialParameters& link = model.arm().joints[j].link;
        const Eigen::Vector4d com_homogeneous(link.centre_of_mass.x(), link.centre_of_mass.y(),
                                              link.centre_of_mass.z(), 1.0);
        const Eigen::Vector3d com_in_base = (transforms[j + 1] * com_homogeneous).head<3>();
        energy -= link.mass * model.arm().gravity.dot(com_in_base);
      }
      return energy;
    };
    for (int k = 0; k < 3; ++k) {
      Eigen::VectorXd q_plus = q, q_minus = q;
      q_plus(k) += step;
      q_minus(k) -= step;
      potential_gradient(k) = (potential_energy(q_plus) - potential_energy(q_minus)) / (2 * step);
    }
    EXPECT_LT((potential_gradient - at.gravity_torque).cwiseAbs().maxCoeff(), 1e-6);

    // The gravity term does not depend on qd and the Coriolis term vanishes at rest.
    model.evaluate(q, Eigen::VectorXd::Zero(3), plus);
    EXPECT_LT((plus.gravity_torque - at.gravity_torque).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT(plus.coriolis_torque.cwiseAbs().maxCoeff(), 1e-12);
  }
}

TEST(ArmDescription, ValidationRejectsInconsistentArms) {
  baxter::ArmDescription arm = planarArm(1.0, 1.0, 1.0, 1.0);
  EXPECT_NO_THROW(arm.validate());

  baxter::ArmDescription duplicate = arm;
  duplicate.joints[1].name = duplicate.joints[0].name;
  EXPECT_THROW(duplicate.validate(), std::invalid_argument);

  baxter::ArmDescription inverted = arm;
  inverted.joints[0].lower_limit = 1.0;
  inverted.joints[0].upper_limit = -1.0;
  EXPECT_THROW(inverted.validate(), std::invalid_argument);

  baxter::ArmDescription no_effort = arm;
  no_effort.joints[0].effort_limit = 0.0;
  EXPECT_THROW(no_effort.validate(), std::invalid_argument);

  baxter::ArmDescription empty;
  EXPECT_THROW(empty.validate(), std::invalid_argument);

  baxter::LagrangeEuler model(arm);
  baxter::JointSpaceDynamics dynamics;
  EXPECT_THROW(model.evaluate(Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), dynamics),
               std::runtime_error);
}

TEST(ArmDescription, TipPayloadFollowsTheParallelAxisTheorem) {
  // Two point masses 1 m apart along x: common COM at the weighted mean, inertia
  // about it m1 m2 / (m1 + m2) d^2 on the axes perpendicular to the separation.
  baxter::LinkInertialParameters first, second;
  first.mass = 2.0;
  first.centre_of_mass = Eigen::Vector3d(0.0, 0.0, 0.0);
  second.mass = 1.0;
  second.centre_of_mass = Eigen::Vector3d(1.0, 0.0, 0.0);
  const baxter::LinkInertialParameters combined = baxter::combineBodies(first, second);
  EXPECT_NEAR(combined.mass, 3.0, 1e-12);
  EXPECT_NEAR(combined.centre_of_mass.x(), 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(combined.Ixx, 0.0, 1e-12);
  EXPECT_NEAR(combined.Iyy, 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(combined.Izz, 2.0 / 3.0, 1e-12);

  // Lumping the payload into the planar arm's last link changes the model exactly
  // like a heavier second link with the shifted centre of mass.
  baxter::ArmDescription arm = planarArm(0.8, 0.5, 2.0, 1.5);
  baxter::LinkInertialParameters payload;
  payload.mass = 0.5;
  payload.centre_of_mass = Eigen::Vector3d(0.0, 0.0, 0.0);  // at the tip frame origin
  arm.addTipPayload(payload);
  EXPECT_NEAR(arm.joints.back().link.mass, 2.0, 1e-12);
  baxter::LagrangeEuler lumped(arm);
  baxter::LagrangeEuler heavier(planarArm(0.8, 0.5, 2.0, 2.0));
  baxter::JointSpaceDynamics a, b;
  Eigen::VectorXd q(2), qd(2);
  q << 0.3, -1.1;
  qd << 0.7, 0.2;
  lumped.evaluate(q, qd, a);
  heavier.evaluate(q, qd, b);
  EXPECT_LT((a.mass_matrix - b.mass_matrix).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((a.coriolis_torque - b.coriolis_torque).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((a.gravity_torque - b.gravity_torque).cwiseAbs().maxCoeff(), 1e-12);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
