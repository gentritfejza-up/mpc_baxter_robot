// Compares the Lagrange-Euler inverse dynamics (DH model from
// dynamics_params.yaml) against KDL's recursive Newton-Euler dynamics built
// from the live /robot_description URDF. Both must agree on the mass matrix,
// Coriolis/centrifugal vector and gravity vector up to the DH-rounding error.

#include <gtest/gtest.h>
#include <kdl/chaindynparam.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "baxter_dynamics/lagrange_euler.hpp"

namespace {

const std::vector<std::string> kJoints = {"left_s0", "left_s1", "left_e0", "left_e1",
                                          "left_w0", "left_w1", "left_w2"};
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

std::unordered_map<std::string, double> toMap(const Eigen::VectorXd& v) {
  std::unordered_map<std::string, double> out;
  for (int i = 0; i < kN; ++i) out[kJoints[i]] = v(i);
  return out;
}

Eigen::VectorXd toVec(const std::unordered_map<std::string, double>& m) {
  Eigen::VectorXd out(kN);
  for (int i = 0; i < kN; ++i) out(i) = m.at(kJoints[i]);
  return out;
}

struct Terms {
  Eigen::MatrixXd M{Eigen::MatrixXd::Zero(kN, kN)};
  Eigen::VectorXd C{Eigen::VectorXd::Zero(kN)};
  Eigen::VectorXd G{Eigen::VectorXd::Zero(kN)};
};

// Recovers M, C and G from the torque interface: G = tau(q,0,0),
// C = tau(q,qd,0) - G, M(:,i) = tau(q,0,e_i) - G.
Terms lagrangeEulerTerms(baxter::LagrangeEuler& le, const Eigen::VectorXd& q,
                         const Eigen::VectorXd& qd) {
  Terms t;
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(kN);
  t.G = toVec(le.computeTorques(toMap(q), toMap(zero), toMap(zero)));
  t.C = toVec(le.computeTorques(toMap(q), toMap(qd), toMap(zero))) - t.G;
  for (int i = 0; i < kN; ++i) {
    Eigen::VectorXd unit = Eigen::VectorXd::Zero(kN);
    unit(i) = 1.0;
    t.M.col(i) = toVec(le.computeTorques(toMap(q), toMap(zero), toMap(unit))) - t.G;
  }
  return t;
}

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
  // Compare raw model torques: disable the torque clamping applied by computeTorques.
  for (const char* key : {"shoulder_elbow", "s1", "wrist"}) {
    nh.setParam(std::string("robot/torque_limits/") + key + "_max", 1.0e6);
    nh.setParam(std::string("robot/torque_limits/") + key + "_min", -1.0e6);
  }
  baxter::LagrangeEuler le("left", nh);
  KDL::Chain chain = buildLumpedChain("torso", "left_wrist");
  ASSERT_EQ(chain.getNrOfJoints(), 7u);
  KDL::ChainDynParam dyn(chain, KDL::Vector(0.0, 0.0, -9.81));

  const std::vector<std::pair<double, double>> limits = {
      {-1.70, 1.70}, {-2.14, 1.04}, {-3.05, 3.05}, {-0.04, 2.61},
      {-3.05, 3.05}, {-1.57, 2.09}, {-3.05, 3.05}};
  std::mt19937 rng(20260911);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);

  double max_m = 0.0, max_c = 0.0, max_g = 0.0;
  for (int sample = 0; sample < 40; ++sample) {
    Eigen::VectorXd q(kN), qd(kN);
    for (int i = 0; i < kN; ++i) {
      const double u = 0.5 * (unit(rng) + 1.0);
      q(i) = limits[i].first + u * (limits[i].second - limits[i].first);
      qd(i) = unit(rng);
    }
    const Terms a = lagrangeEulerTerms(le, q, qd);
    const Terms b = kdlTerms(dyn, q, qd);
    max_m = std::max(max_m, (a.M - b.M).cwiseAbs().maxCoeff());
    max_c = std::max(max_c, (a.C - b.C).cwiseAbs().maxCoeff());
    max_g = std::max(max_g, (a.G - b.G).cwiseAbs().maxCoeff());
    EXPECT_TRUE(((a.M - a.M.transpose()).cwiseAbs().maxCoeff()) < 1e-9) << "M not symmetric";
  }
  ROS_INFO("max |M| diff %.2e kg m^2, max |C| diff %.2e Nm, max |G| diff %.2e Nm",
           max_m, max_c, max_g);
  // The two 7e-5 m z-offsets in the DH table bound the gravity residual.
  EXPECT_LT(max_m, 2e-3);
  EXPECT_LT(max_c, 2e-3);
  EXPECT_LT(max_g, 2e-2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "lagrange_euler_kdl_test");
  return RUN_ALL_TESTS();
}
