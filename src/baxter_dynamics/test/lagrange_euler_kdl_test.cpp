// Compares the Lagrange-Euler model (DH description from dynamics_params.yaml)
// against KDL's recursive Newton-Euler dynamics built from the live
// /robot_description URDF. Both must agree on the mass matrix, the
// Coriolis/centrifugal vector and the gravity vector up to the DH-rounding
// error, and the model must evaluate well within the 800 Hz loop period.

#include <gtest/gtest.h>
#include <kdl/chaindynparam.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <string>
#include <vector>

#include "baxter_dynamics/arm_description.hpp"
#include "baxter_dynamics/lagrange_euler.hpp"

namespace {

constexpr int kN = 7;

// Lumps `node` and every descendant reachable through fixed joints into one
// rigid body expressed in node's frame. This mirrors the YAML's link 7, which is
// the wrist + hand + gripper base; the gripper fingers are separate moving
// bodies and are left out on both sides.
KDL::RigidBodyInertia subtreeInertia(const KDL::SegmentMap::const_iterator& node) {
  const auto& element = node->second;
  KDL::RigidBodyInertia total = GetTreeElementSegment(element).getInertia();
  for (const auto& child : GetTreeElementChildren(element)) {
    const KDL::Segment& child_segment = GetTreeElementSegment(child->second);
    if (child_segment.getJoint().getType() != KDL::Joint::None) continue;
    total = total + child_segment.pose(0.0) * subtreeInertia(child);
  }
  return total;
}

KDL::Chain buildLumpedChain(const std::string& root, const std::string& tip) {
  std::string xml;
  if (!ros::param::get("/robot_description", xml) || xml.empty()) {
    throw std::runtime_error("/robot_description is not set");
  }
  urdf::Model model;
  if (!model.initString(xml)) throw std::runtime_error("URDF parse failed");
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) throw std::runtime_error("KDL tree failed");
  KDL::Chain chain;
  if (!tree.getChain(root, tip, chain)) throw std::runtime_error("KDL chain failed");
  const auto tip_node = tree.getSegment(tip);
  KDL::Chain lumped;
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    KDL::Segment segment = chain.getSegment(i);
    if (i + 1 == chain.getNrOfSegments()) {
      const KDL::RigidBodyInertia composite = subtreeInertia(tip_node);
      const KDL::Vector cog = composite.getCOG();
      ROS_INFO("lumped tip: mass %.5f cog (%.5f %.5f %.5f)", composite.getMass(), cog.x(),
               cog.y(), cog.z());
      segment.setInertia(composite);
    }
    lumped.addSegment(segment);
  }
  return lumped;
}

struct Terms {
  Eigen::MatrixXd M{Eigen::MatrixXd::Zero(kN, kN)};
  Eigen::VectorXd C{Eigen::VectorXd::Zero(kN)};
  Eigen::VectorXd G{Eigen::VectorXd::Zero(kN)};
};

Terms kdlTerms(KDL::ChainDynParam& dyn, const Eigen::VectorXd& q, const Eigen::VectorXd& qd) {
  KDL::JntArray jq(kN), jqd(kN), c(kN), g(kN);
  KDL::JntSpaceInertiaMatrix m(kN);
  for (int i = 0; i < kN; ++i) {
    jq(i) = q(i);
    jqd(i) = qd(i);
  }
  EXPECT_GE(dyn.JntToMass(jq, m), 0);
  EXPECT_GE(dyn.JntToCoriolis(jq, jqd, c), 0);
  EXPECT_GE(dyn.JntToGravity(jq, g), 0);
  Terms t;
  for (int i = 0; i < kN; ++i) {
    t.C(i) = c(i);
    t.G(i) = g(i);
    for (int j = 0; j < kN; ++j) t.M(i, j) = m(i, j);
  }
  return t;
}

}  // namespace

TEST(LagrangeEulerKdl, MassCoriolisGravityMatchUrdf) {
  ros::NodeHandle nh;
  const baxter::ArmDescription arm = baxter::loadArmDescription(nh, "robot/arm_model", "left");
  ASSERT_EQ(arm.jointCount(), static_cast<std::size_t>(kN));
  baxter::LagrangeEuler le(arm);
  EXPECT_EQ(le.jointNames().front(), "left_s0");
  EXPECT_EQ(le.jointNames().back(), "left_w2");

  KDL::Chain chain = buildLumpedChain("torso", "left_wrist");
  ASSERT_EQ(chain.getNrOfJoints(), 7u);
  KDL::ChainDynParam dyn(chain, KDL::Vector(0.0, 0.0, -9.81));

  std::mt19937 rng(20260911);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);

  double max_m = 0.0, max_c = 0.0, max_g = 0.0;
  baxter::JointSpaceDynamics dynamics;
  for (int sample = 0; sample < 40; ++sample) {
    Eigen::VectorXd q(kN), qd(kN);
    for (int i = 0; i < kN; ++i) {
      const double u = 0.5 * (unit(rng) + 1.0);
      const baxter::JointDescription& joint = arm.joints[i];
      q(i) = joint.lower_limit + u * (joint.upper_limit - joint.lower_limit);
      qd(i) = unit(rng);
    }
    le.evaluate(q, qd, dynamics);
    const Terms b = kdlTerms(dyn, q, qd);
    max_m = std::max(max_m, (dynamics.mass_matrix - b.M).cwiseAbs().maxCoeff());
    max_c = std::max(max_c, (dynamics.coriolis_torque - b.C).cwiseAbs().maxCoeff());
    max_g = std::max(max_g, (dynamics.gravity_torque - b.G).cwiseAbs().maxCoeff());
    EXPECT_LT((dynamics.mass_matrix - dynamics.mass_matrix.transpose()).cwiseAbs().maxCoeff(),
              1e-9)
        << "M not symmetric";
  }
  ROS_INFO("max |M| diff %.2e kg m^2, max |C| diff %.2e Nm, max |G| diff %.2e Nm",
           max_m, max_c, max_g);
  // The two 7e-5 m z-offsets in the DH table bound the gravity residual.
  EXPECT_LT(max_m, 2e-3);
  EXPECT_LT(max_c, 2e-3);
  EXPECT_LT(max_g, 2e-2);
}

TEST(LagrangeEulerKdl, EvaluationFitsTheTorqueLoopPeriod) {
  ros::NodeHandle nh;
  baxter::LagrangeEuler le(baxter::loadArmDescription(nh, "robot/arm_model", "right"));
  baxter::JointSpaceDynamics dynamics;
  Eigen::VectorXd q = Eigen::VectorXd::LinSpaced(kN, -0.5, 0.5);
  Eigen::VectorXd qd = Eigen::VectorXd::LinSpaced(kN, -1.0, 1.0);
  le.evaluate(q, qd, dynamics);  // warm up (allocates the scratch tables)

  constexpr int kIterations = 2000;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIterations; ++i) {
    q(i % kN) += 1e-4;
    le.evaluate(q, qd, dynamics);
  }
  const double mean_us = std::chrono::duration<double, std::micro>(
                             std::chrono::steady_clock::now() - start).count() / kIterations;
  ROS_INFO("Lagrange-Euler evaluate: %.1f us per call", mean_us);
  EXPECT_LT(mean_us, 300.0);  // 800 Hz leaves 1250 us per loop iteration.
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "lagrange_euler_kdl_test");
  return RUN_ALL_TESTS();
}
