// Checks the sphere collision model against KDL's own FK/Jacobian solvers and
// finite differences, on the live URDF.

#include <gtest/gtest.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <ros/ros.h>

#include <algorithm>
#include <random>

#include "mpc_controller/arm_collision_model.hpp"

using mpc_geometry::ArmCollisionModel;
using mpc_geometry::JointVector;
using mpc_geometry::SphereState;

namespace {

JointVector randomJoints(std::mt19937& rng) {
  static const double lo[7] = {-1.70, -2.14, -3.05, -0.04, -3.05, -1.57, -3.05};
  static const double hi[7] = {1.70, 1.04, 3.05, 2.61, 3.05, 2.09, 3.05};
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  JointVector q;
  for (int i = 0; i < 7; ++i) q[i] = lo[i] + unit(rng) * (hi[i] - lo[i]);
  return q;
}

KDL::JntArray toKdl(const JointVector& q) {
  KDL::JntArray out(7);
  for (int i = 0; i < 7; ++i) out(i) = q[i];
  return out;
}

}  // namespace

class ArmCollisionModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::NodeHandle nh;
    model_ = ArmCollisionModel::fromParams(nh);
  }
  std::unique_ptr<ArmCollisionModel> model_;
};

TEST_F(ArmCollisionModelTest, BuildsBothArms) {
  EXPECT_EQ(model_->sphereCount(), 12);
  EXPECT_EQ(model_->spheres("left").size(), 12u);
  EXPECT_EQ(model_->spheres("right").size(), 12u);
  EXPECT_EQ(model_->chain("left").getNrOfJoints(), 7u);
  EXPECT_EQ(model_->chain("right").getNrOfJoints(), 7u);
  EXPECT_EQ(model_->bodyObstacles().size(), 4u);
  EXPECT_EQ(model_->spheres("left")[0].link, "left_upper_elbow");
  EXPECT_EQ(model_->spheres("right")[11].link, "right_hand");
  EXPECT_EQ(model_->spheres("left")[0].ignored_obstacles.size(), 1u);
}

TEST_F(ArmCollisionModelTest, CentersMatchKdlForwardKinematics) {
  std::mt19937 rng(7);
  for (const std::string side : {"left", "right"}) {
    const KDL::Chain& chain = model_->chain(side);
    KDL::ChainFkSolverPos_recursive fk(chain);
    std::vector<SphereState> states;
    std::vector<Eigen::Vector3d> centers;
    for (int sample = 0; sample < 20; ++sample) {
      const JointVector q = randomJoints(rng);
      model_->evaluate(side, q, states);
      model_->centersOnly(side, q, centers);
      ASSERT_EQ(states.size(), 12u);
      for (int i = 0; i < 12; ++i) {
        const int segment = model_->sphereSegment(side, i);
        KDL::Frame frame;
        ASSERT_GE(fk.JntToCart(toKdl(q), frame, segment + 1), 0);
        const auto& offset = model_->spheres(side)[i].offset;
        const KDL::Vector p = frame * KDL::Vector(offset.x(), offset.y(), offset.z());
        EXPECT_NEAR((states[i].center - Eigen::Vector3d(p.x(), p.y(), p.z())).norm(), 0.0, 1e-9);
        EXPECT_NEAR((states[i].center - centers[i]).norm(), 0.0, 1e-12);
      }
    }
  }
}

TEST_F(ArmCollisionModelTest, JacobiansMatchKdlAndFiniteDifferences) {
  std::mt19937 rng(11);
  for (const std::string side : {"left", "right"}) {
    const KDL::Chain& chain = model_->chain(side);
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::ChainJntToJacSolver jac_solver(chain);
    std::vector<SphereState> states, plus, minus;
    for (int sample = 0; sample < 10; ++sample) {
      const JointVector q = randomJoints(rng);
      model_->evaluate(side, q, states);
      for (int i = 0; i < 12; ++i) {
        const int segment = model_->sphereSegment(side, i);
        // KDL Jacobian of the segment tip frame, shifted to the sphere centre.
        KDL::Jacobian jac(7);
        ASSERT_GE(jac_solver.JntToJac(toKdl(q), jac, segment + 1), 0);
        KDL::Frame frame;
        ASSERT_GE(fk.JntToCart(toKdl(q), frame, segment + 1), 0);
        const Eigen::Vector3d origin(frame.p.x(), frame.p.y(), frame.p.z());
        const Eigen::Vector3d r = states[i].center - origin;
        for (int j = 0; j < 7; ++j) {
          const Eigen::Vector3d lin(jac(0, j), jac(1, j), jac(2, j));
          const Eigen::Vector3d rot(jac(3, j), jac(4, j), jac(5, j));
          const Eigen::Vector3d expected = lin + rot.cross(r);
          EXPECT_NEAR((states[i].jacobian.col(j) - expected).norm(), 0.0, 1e-9)
              << side << " sphere " << i << " joint " << j;
        }
        // Finite differences.
        const double h = 1e-6;
        for (int j = 0; j < 7; ++j) {
          JointVector qp = q, qm = q;
          qp[j] += h; qm[j] -= h;
          model_->evaluate(side, qp, plus);
          model_->evaluate(side, qm, minus);
          const Eigen::Vector3d numeric = (plus[i].center - minus[i].center) / (2 * h);
          EXPECT_NEAR((states[i].jacobian.col(j) - numeric).norm(), 0.0, 1e-5)
              << side << " sphere " << i << " joint " << j;
        }
      }
    }
  }
}

TEST_F(ArmCollisionModelTest, DistancesAtKnownPosesMakeSense) {
  // Gazebo start pose: clear of the body and above the pedestal.
  const JointVector start{0.192483, 1.047, 0.000806, 0.491094, -0.178079, -0.061033, -0.012471};
  std::vector<Eigen::Vector3d> centers;
  model_->centersOnly("left", start, centers);
  const auto& spheres = model_->spheres("left");
  for (int i = 0; i < 12; ++i) {
    for (const auto& obstacle : model_->bodyObstacles()) {
      if (std::find(spheres[i].ignored_obstacles.begin(), spheres[i].ignored_obstacles.end(), obstacle.name) != spheres[i].ignored_obstacles.end()) continue;
      const auto d = mpc_geometry::distanceToObstacle(centers[i], spheres[i].radius, obstacle);
      EXPECT_GT(d.distance, 0.0) << "sphere " << i << " vs " << obstacle.name;
    }
  }
  // Arm hanging straight down (s1 = 1.0, e1 ~ 0): the hand reaches z ~ -0.47, well
  // below a conveyor floor at z = -0.20, while staying beside the pedestal column.
  const JointVector down{0.0, 1.0, 0.0, 0.05, 0.0, 0.0, 0.0};
  model_->centersOnly("left", down, centers);
  mpc_geometry::Obstacle floor;
  floor.type = mpc_geometry::ObstacleType::ZPlane;
  floor.start_point = Eigen::Vector3d(0.0, 0.0, -0.20);
  double min_floor = 1.0;
  double min_pedestal = 1.0;
  for (int i = 0; i < 12; ++i) {
    min_floor = std::min(min_floor, mpc_geometry::distanceToObstacle(centers[i], spheres[i].radius, floor).distance);
    for (const auto& obstacle : model_->bodyObstacles()) {
      if (obstacle.name != "pedestal") continue;
      min_pedestal = std::min(min_pedestal,
                              mpc_geometry::distanceToObstacle(centers[i], spheres[i].radius, obstacle).distance);
    }
  }
  EXPECT_LT(min_floor, -0.15);
  EXPECT_GT(min_pedestal, 0.0);
  // The two arms at the start pose do not touch each other.
  std::vector<Eigen::Vector3d> right_centers;
  const JointVector right_start{-0.272659, 1.04701, -0.00123203, 0.49262, -0.0806423, -0.0620532, 0.0265941};
  model_->centersOnly("right", right_start, right_centers);
  model_->centersOnly("left", start, centers);
  for (int i = 0; i < 12; ++i) {
    for (int k = 0; k < 12; ++k) {
      EXPECT_GT(mpc_geometry::distanceBetweenSpheres(centers[i], spheres[i].radius, right_centers[k],
                                                     model_->spheres("right")[k].radius).distance, 0.0);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "arm_collision_model_test");
  return RUN_ALL_TESTS();
}
