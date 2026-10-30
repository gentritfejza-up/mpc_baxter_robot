// Row linearisation and guard evaluation against the live URDF model.

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <limits>
#include <random>

#include "mpc_controller/geometry_rows.hpp"

using mpc_geometry::ArmCollisionModel;
using mpc_geometry::GeometryLayout;
using mpc_geometry::GeometryParams;
using mpc_geometry::GeometryRowBatch;
using mpc_geometry::GeometryRowLinearizer;
using mpc_geometry::JointVector;
using mpc_geometry::kArmJoints;

namespace {

constexpr int kN = 6;
const JointVector kStart{0.192483, 1.047, 0.000806, 0.491094, -0.178079, -0.061033, -0.012471};
// Arm raised in front and to the side: clear of the body, the head and the floor.
const JointVector kClear{0.3, -0.3, 0.0, 0.8, 0.0, 0.5, 0.0};
// Arm swung across the chest (s0 negative) with the elbow bent: close to the torso.
const JointVector kNearTorso{-1.2, -0.3, -0.2, 1.9, 0.0, 0.5, 0.0};
// Arm hanging down: below the floor at z_min = -0.2.
const JointVector kDown{0.0, 1.0, 0.0, 0.05, 0.0, 0.0, 0.0};

GeometryLayout makeLayout(const ArmCollisionModel& model, const GeometryParams& p) {
  GeometryLayout layout;
  layout.prediction_horizon = kN;
  layout.sphere_count = model.sphereCount();
  layout.rows_floor = p.rows_floor;
  layout.rows_body = p.rows_body;
  layout.rows_other_arm = p.rows_other_arm;
  return layout;
}

}  // namespace

class GeometryRowsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::NodeHandle nh;
    model_ = ArmCollisionModel::fromParams(nh);
    params_ = GeometryParams::fromParams(nh);
    layout_ = makeLayout(*model_, params_);
    lin_ = std::make_unique<GeometryRowLinearizer>(*model_, "left", params_, layout_);
  }
  std::unique_ptr<ArmCollisionModel> model_;
  GeometryParams params_;
  GeometryLayout layout_;
  std::unique_ptr<GeometryRowLinearizer> lin_;
};

TEST_F(GeometryRowsTest, LayoutAndInactiveRows) {
  EXPECT_EQ(layout_.slotsPerSphere(), 3);
  EXPECT_EQ(layout_.slotsPerStep(), 36);
  EXPECT_EQ(layout_.rowCount(), kN * 36);
  EXPECT_EQ(layout_.rowIndex(1, 0, 0), 0);
  EXPECT_EQ(layout_.rowIndex(2, 0, 0), 36);
  EXPECT_EQ(layout_.rowIndex(1, 1, 2), 5);

  // Raised pose with activation distance 0: everything is clear (positive distances),
  // so no row is active, bounds are -inf and coefficients 0. (The Gazebo rest pose
  // instead hangs within 0.15 m of the torso and below a floor at -0.20 m, so rows
  // activate there, which is intended.)
  GeometryParams strict = params_;
  strict.activation_distance = 0.0;
  GeometryRowLinearizer strict_lin(*model_, "left", strict, layout_);
  std::vector<JointVector> qbar(kN, kClear);
  GeometryRowBatch batch;
  strict_lin.linearize(qbar, nullptr, batch);
  EXPECT_EQ(batch.active_rows, 0);
  EXPECT_EQ(batch.coefficients.size(), static_cast<std::size_t>(layout_.coefficientCount()));
  EXPECT_EQ(batch.lower_bounds.size(), static_cast<std::size_t>(layout_.rowCount()));
  for (double lb : batch.lower_bounds) EXPECT_TRUE(std::isinf(lb) && lb < 0);
  for (double c : batch.coefficients) EXPECT_EQ(c, 0.0);
  EXPECT_GT(batch.min_linearised_distance[mpc_geometry::kFloorGroup], 0.0);
  EXPECT_GT(batch.min_linearised_distance[mpc_geometry::kBodyGroup], 0.0);
  EXPECT_TRUE(std::isinf(batch.min_linearised_distance[mpc_geometry::kArmGroup]));  // no other arm
  // At the Gazebo rest pose the default activation distance activates rows (arm beside the torso).
  GeometryRowBatch near;
  lin_->linearize(std::vector<JointVector>(kN, kStart), nullptr, near);
  EXPECT_GT(near.active_rows, 0);
}

TEST_F(GeometryRowsTest, FloorRowsActivateAndAreConsistent) {
  std::vector<JointVector> qbar(kN, kDown);
  GeometryRowBatch batch;
  lin_->linearize(qbar, nullptr, batch);
  EXPECT_GT(batch.active_rows, 0);
  EXPECT_LT(batch.min_linearised_distance[mpc_geometry::kFloorGroup], 0.0);

  // Check every active floor slot against a direct linearisation and the true distance.
  std::vector<mpc_geometry::SphereState> states;
  model_->evaluate("left", kDown, states);
  mpc_geometry::Obstacle floor;
  floor.type = mpc_geometry::ObstacleType::ZPlane;
  floor.start_point = Eigen::Vector3d(0, 0, params_.z_min);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> unit(-0.05, 0.05);
  const auto& spheres = model_->spheres("left");
  int checked = 0;
  for (int i = 0; i < layout_.sphere_count; ++i) {
    const int row = layout_.rowIndex(1, i, 0);
    if (std::isinf(batch.lower_bounds[row])) continue;
    const auto d = mpc_geometry::distanceToObstacle(states[i].center, spheres[i].radius, floor);
    Eigen::Matrix<double, 1, kArmJoints> g = d.normal.transpose() * states[i].jacobian;
    double dot = 0.0;
    for (int j = 0; j < kArmJoints; ++j) {
      EXPECT_NEAR(batch.coefficients[row * kArmJoints + j], g(j), 1e-12);
      dot += g(j) * kDown[j];
    }
    // The requested recovery is capped per horizon (max_recovery_per_horizon).
    const double requested = std::min(params_.floor_margin - d.distance, params_.max_recovery_per_horizon);
    EXPECT_NEAR(batch.lower_bounds[row], requested + dot, 1e-12);
    // Linearisation accuracy for a small step.
    JointVector delta_q = kDown;
    double predicted = d.distance;
    for (int j = 0; j < kArmJoints; ++j) {
      const double delta = unit(rng);
      delta_q[j] += delta;
      predicted += g(j) * delta;
    }
    std::vector<Eigen::Vector3d> centers;
    model_->centersOnly("left", delta_q, centers);
    const double truth = mpc_geometry::distanceToObstacle(centers[i], spheres[i].radius, floor).distance;
    EXPECT_NEAR(truth, predicted, 5e-3);
    ++checked;
  }
  EXPECT_GT(checked, 0);
}

TEST_F(GeometryRowsTest, BodyRowsNearTorsoAndOtherArmSlots) {
  std::vector<JointVector> qbar(kN, kNearTorso);
  GeometryRowBatch batch;
  lin_->linearize(qbar, nullptr, batch);
  const double body_min = batch.min_linearised_distance[mpc_geometry::kBodyGroup];
  EXPECT_LT(body_min, params_.activation_distance) << "pose should be near the torso";
  // Body slot (slot index 1) active for the sphere closest to the body; arm slots inactive.
  bool any_body = false;
  for (int i = 0; i < layout_.sphere_count; ++i) {
    any_body = any_body || !std::isinf(batch.lower_bounds[layout_.rowIndex(1, i, 1)]);
    EXPECT_TRUE(std::isinf(batch.lower_bounds[layout_.rowIndex(1, i, 2)]));
  }
  EXPECT_TRUE(any_body);

  // With the other arm given at the start pose, its slots may activate only if close.
  GeometryRowBatch with_arm;
  const JointVector right_start{-0.272659, 1.04701, -0.00123203, 0.49262, -0.0806423, -0.0620532, 0.0265941};
  lin_->linearize(qbar, &right_start, with_arm);
  EXPECT_FALSE(std::isinf(with_arm.min_linearised_distance[mpc_geometry::kArmGroup]));
  EXPECT_GE(with_arm.active_rows, batch.active_rows);
}

TEST_F(GeometryRowsTest, StepStrideLeavesSkippedStepsInactive) {
  GeometryParams strided = params_;
  strided.step_stride = 3;
  GeometryRowLinearizer lin(*model_, "left", strided, layout_);
  std::vector<JointVector> qbar(kN, kDown);
  GeometryRowBatch batch;
  lin.linearize(qbar, nullptr, batch);
  for (int step = 1; step <= kN; ++step) {
    bool active = false;
    for (int i = 0; i < layout_.sphere_count; ++i) {
      active = active || !std::isinf(batch.lower_bounds[layout_.rowIndex(step, i, 0)]);
    }
    const bool expected = (step - 1) % 3 == 0 || step == kN;
    EXPECT_EQ(active, expected) << "step " << step;
  }
}

TEST_F(GeometryRowsTest, GuardFlagsWorseningOnly) {
  // From a clear pose, a predicted trajectory that ends below the floor is a violation.
  std::vector<JointVector> predicted(kN, kClear);
  predicted.back() = kDown;
  auto report = lin_->evaluateTrajectory(kClear, predicted, nullptr);
  EXPECT_TRUE(report.violated);
  EXPECT_EQ(report.worst_group, mpc_geometry::kFloorGroup);
  EXPECT_EQ(report.worst_step, kN);
  EXPECT_LT(report.min_distance[mpc_geometry::kFloorGroup], 0.0);
  EXPECT_GT(report.min_distance_now[mpc_geometry::kFloorGroup], 0.0);
  // Already below the floor and staying there (not worsening): no violation.
  std::vector<JointVector> stay(kN, kDown);
  report = lin_->evaluateTrajectory(kDown, stay, nullptr);
  EXPECT_FALSE(report.violated);
  // Staying clear everywhere: no violation and finite minima.
  std::vector<JointVector> clear(kN, kClear);
  report = lin_->evaluateTrajectory(kClear, clear, nullptr);
  EXPECT_FALSE(report.violated);
  EXPECT_TRUE(std::isfinite(report.min_distance[mpc_geometry::kBodyGroup]));
}

TEST(GeometryParams, Validation) {
  GeometryParams p;
  EXPECT_NO_THROW(p.validate());
  p.body_margin = -0.01;
  EXPECT_THROW(p.validate(), std::invalid_argument);
  p = GeometryParams();
  p.rows_floor = 2;
  EXPECT_THROW(p.validate(), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "geometry_rows_test");
  return RUN_ALL_TESTS();
}
