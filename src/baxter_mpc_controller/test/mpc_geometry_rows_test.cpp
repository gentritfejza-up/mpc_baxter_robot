// The collision-guard block of the quadratic-cost QP, exercised with synthetic rows
// (no URDF needed): inactive rows change nothing, an active row is respected,
// an unsatisfiable row is absorbed by the slack, bad batches are rejected.

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "mpc_controller/mpc_controller.hpp"

namespace {

constexpr int kJoints = 7;
constexpr int kHorizon = 10;
constexpr int kControl = 6;
constexpr double kPeriod = 0.02;
constexpr double kInf = std::numeric_limits<double>::infinity();

std::vector<double> grid() {
  std::vector<double> durations(kHorizon, kPeriod);
  std::fill(durations.begin() + 1, durations.end(), (0.6 - kPeriod) / (kHorizon - 1));
  return durations;
}

MpcSolveInput baseInput() {
  MpcSolveInput input;
  input.positions = casadi::DM::zeros(kJoints, 1);
  input.velocities = casadi::DM::zeros(kJoints, 1);
  input.reference_positions = casadi::DM(std::vector<double>{-0.8, 0.3, 0.0, 0.5, 0.0, 0.0, 0.0});
  input.previous_accelerations = casadi::DM::zeros(kJoints, 1);
  input.first_jerk_dt = kPeriod;
  return input;
}

std::vector<double> toVector(const casadi::DM& value) { return value.nonzeros(); }

}  // namespace

TEST(MpcGeometryRows, InactiveRowsMatchNoGeometry) {
  ros::NodeHandle nh;
  std::vector<double> without;
  {
    MPCController plain(nh, kJoints, kHorizon, kControl, grid(), kPeriod, 0);
    EXPECT_FALSE(plain.geometryEnabled());
    without = toVector(plain.getFirstAcceleration(plain.solve(baseInput())));
  }
  MPCController geometry(nh, kJoints, kHorizon, kControl, grid(), kPeriod, 2);
  EXPECT_TRUE(geometry.geometryEnabled());
  const auto layout = geometry.geometryLayout();
  EXPECT_EQ(layout.sphere_count, 2);
  EXPECT_EQ(layout.slotsPerSphere(), 3);
  EXPECT_EQ(layout.rowCount(), kHorizon * 6);

  // nullptr batch: all rows inactive.
  const casadi::DM solution = geometry.solve(baseInput());
  const std::vector<double> with = toVector(geometry.getFirstAcceleration(solution));
  ASSERT_EQ(with.size(), without.size());
  for (int j = 0; j < kJoints; ++j) EXPECT_NEAR(with[j], without[j], 1e-5) << "joint " << j;
  EXPECT_NEAR(geometry.maxGeometrySlack(solution), 0.0, 1e-9);
  const auto predicted = geometry.predictedPositions(solution);
  EXPECT_EQ(predicted.size(), static_cast<std::size_t>(kHorizon));
  EXPECT_LT(predicted.back()[0], 0.0);  // joint 0 heads towards its -0.8 reference

  // An explicit batch with every slot inactive gives the same answer.
  mpc_geometry::GeometryRowBatch batch;
  batch.reset(layout);
  MpcSolveInput input = baseInput();
  input.geometry = &batch;
  geometry.commitSolution();
  const std::vector<double> again = toVector(geometry.getFirstAcceleration(geometry.solve(input)));
  for (int j = 0; j < kJoints; ++j) EXPECT_NEAR(again[j], without[j], 1e-5) << "joint " << j;
}

TEST(MpcGeometryRows, ActiveFloorRowIsRespected) {
  ros::NodeHandle nh;
  MPCController geometry(nh, kJoints, kHorizon, kControl, grid(), kPeriod, 2);
  const auto layout = geometry.geometryLayout();
  // Row "X_k[0] >= -0.3" on sphere 0's floor slot at every step; the reference is -0.8.
  mpc_geometry::GeometryRowBatch batch;
  batch.reset(layout);
  for (int step = 1; step <= kHorizon; ++step) {
    const int row = layout.rowIndex(step, 0, 0);
    batch.coefficients[row * kJoints + 0] = 1.0;
    batch.lower_bounds[row] = -0.3;
    ++batch.active_rows;
  }
  MpcSolveInput input = baseInput();
  input.geometry = &batch;
  casadi::DM solution;
  for (int cycle = 0; cycle < 3; ++cycle) {  // warm start settles the active set
    solution = geometry.solve(input);
    geometry.commitSolution();
  }
  const auto predicted = geometry.predictedPositions(solution);
  // Quadratic slack: the bound is a few millimetres soft under the reference's pull.
  for (int step = 0; step < kHorizon; ++step) {
    EXPECT_GE(predicted[step][0], -0.3 - 1e-2) << "step " << step + 1;
  }
  // The bound is active at the end of the horizon (the reference lies beyond it).
  EXPECT_NEAR(predicted.back()[0], -0.3, 2e-2);
  EXPECT_LT(geometry.maxGeometrySlack(solution), 1e-2);
  // Other joints are untouched by the row.
  EXPECT_GT(predicted.back()[1], 0.1);
}

TEST(MpcGeometryRows, UnsatisfiableRowUsesSlack) {
  ros::NodeHandle nh;
  MPCController geometry(nh, kJoints, kHorizon, kControl, grid(), kPeriod, 2);
  const auto layout = geometry.geometryLayout();
  // X_1[0] >= 0.1 cannot be reached from rest within 20 ms: the slack absorbs it.
  mpc_geometry::GeometryRowBatch batch;
  batch.reset(layout);
  const int row = layout.rowIndex(1, 1, 1);
  batch.coefficients[row * kJoints + 0] = 1.0;
  batch.lower_bounds[row] = 0.1;
  batch.active_rows = 1;
  MpcSolveInput input = baseInput();
  input.geometry = &batch;
  casadi::DM solution;
  try {
    solution = geometry.solve(input);
  } catch (const std::exception& exception) {
    FAIL() << "solve threw: " << exception.what();
  }
  EXPECT_GT(geometry.maxGeometrySlack(solution), 0.09);
  EXPECT_LT(geometry.maxGeometrySlack(solution), 0.11);
}

TEST(MpcGeometryRows, BadBatchesAreRejected) {
  ros::NodeHandle nh;
  MPCController geometry(nh, kJoints, kHorizon, kControl, grid(), kPeriod, 2);
  mpc_geometry::GeometryRowBatch batch;
  batch.reset(geometry.geometryLayout());
  batch.lower_bounds.pop_back();
  MpcSolveInput input = baseInput();
  input.geometry = &batch;
  EXPECT_THROW(geometry.solve(input), std::invalid_argument);
  batch.reset(geometry.geometryLayout());
  batch.coefficients[0] = std::nan("");
  EXPECT_THROW(geometry.solve(input), std::invalid_argument);
  batch.reset(geometry.geometryLayout());
  batch.lower_bounds[0] = kInf;
  EXPECT_THROW(geometry.solve(input), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mpc_geometry_rows_test");
  return RUN_ALL_TESTS();
}
