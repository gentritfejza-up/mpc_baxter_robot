// Distance primitives: values at known configurations, gradient against finite
// differences, and the degenerate fallbacks.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "mpc_controller/collision_primitives.hpp"

using mpc_geometry::DistanceResult;
using mpc_geometry::Obstacle;
using mpc_geometry::ObstacleType;

namespace {

Obstacle sphere(const Eigen::Vector3d& center, double radius) {
  Obstacle obstacle;
  obstacle.type = ObstacleType::Sphere;
  obstacle.start_point = center;
  obstacle.radius = radius;
  return obstacle;
}
Obstacle capsule(const Eigen::Vector3d& segment_start, const Eigen::Vector3d& segment_end,
                 double radius) {
  Obstacle obstacle;
  obstacle.type = ObstacleType::Capsule;
  obstacle.start_point = segment_start;
  obstacle.end_point = segment_end;
  obstacle.radius = radius;
  return obstacle;
}
Obstacle aabb(const Eigen::Vector3d& min_corner, const Eigen::Vector3d& max_corner) {
  Obstacle obstacle;
  obstacle.type = ObstacleType::Aabb;
  obstacle.start_point = min_corner;
  obstacle.end_point = max_corner;
  return obstacle;
}
Obstacle zplane(double height) {
  Obstacle obstacle;
  obstacle.type = ObstacleType::ZPlane;
  obstacle.start_point = Eigen::Vector3d(0, 0, height);
  return obstacle;
}

// Central finite-difference gradient of the distance with respect to the centre.
Eigen::Vector3d numericGradient(const Eigen::Vector3d& center, double radius,
                                const Obstacle& obstacle) {
  const double step = 1e-6;
  Eigen::Vector3d gradient;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = center, minus = center;
    plus[axis] += step;
    minus[axis] -= step;
    gradient[axis] = (mpc_geometry::distanceToObstacle(plus, radius, obstacle).distance -
                      mpc_geometry::distanceToObstacle(minus, radius, obstacle).distance) /
                     (2 * step);
  }
  return gradient;
}

void expectGradient(const Eigen::Vector3d& center, double radius, const Obstacle& obstacle) {
  const DistanceResult result = mpc_geometry::distanceToObstacle(center, radius, obstacle);
  const Eigen::Vector3d gradient = numericGradient(center, radius, obstacle);
  EXPECT_NEAR((result.normal - gradient).norm(), 0.0, 1e-5)
      << "analytic " << result.normal.transpose() << " numeric " << gradient.transpose();
  EXPECT_NEAR(result.normal.norm(), 1.0, 1e-12);
}

}  // namespace

TEST(CollisionPrimitives, ZPlane) {
  const DistanceResult result =
      mpc_geometry::distanceToObstacle({0.3, -0.2, 0.05}, 0.07, zplane(-0.1));
  EXPECT_NEAR(result.distance, 0.05 - 0.07 + 0.1, 1e-12);
  EXPECT_TRUE(result.normal.isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_FALSE(result.degenerate);
  expectGradient({0.3, -0.2, 0.05}, 0.07, zplane(-0.1));
}

TEST(CollisionPrimitives, Sphere) {
  const Obstacle obstacle = sphere({1, 0, 0}, 0.2);
  const DistanceResult result = mpc_geometry::distanceToObstacle({1, 0.5, 0}, 0.1, obstacle);
  EXPECT_NEAR(result.distance, 0.5 - 0.3, 1e-12);
  EXPECT_TRUE(result.normal.isApprox(Eigen::Vector3d::UnitY()));
  expectGradient({0.4, 0.7, -0.2}, 0.1, obstacle);
  // Penetration is negative, coincident centres give a finite fallback normal.
  EXPECT_LT(mpc_geometry::distanceToObstacle({1.05, 0, 0}, 0.1, obstacle).distance, 0.0);
  const DistanceResult same = mpc_geometry::distanceToObstacle({1, 0, 0}, 0.1, obstacle);
  EXPECT_TRUE(same.degenerate);
  EXPECT_NEAR(same.normal.norm(), 1.0, 1e-12);
  EXPECT_NEAR(same.distance, -0.3, 1e-12);
}

TEST(CollisionPrimitives, Capsule) {
  const Obstacle obstacle = capsule({0, 0, 0}, {0, 0, 1}, 0.2);
  // Beside the segment interior.
  DistanceResult result = mpc_geometry::distanceToObstacle({0.5, 0, 0.5}, 0.1, obstacle);
  EXPECT_NEAR(result.distance, 0.5 - 0.3, 1e-12);
  EXPECT_TRUE(result.normal.isApprox(Eigen::Vector3d::UnitX()));
  // Beyond the top end cap.
  result = mpc_geometry::distanceToObstacle({0, 0, 1.6}, 0.1, obstacle);
  EXPECT_NEAR(result.distance, 0.6 - 0.3, 1e-12);
  EXPECT_TRUE(result.normal.isApprox(Eigen::Vector3d::UnitZ()));
  // Below the bottom end cap, off axis.
  expectGradient({0.3, -0.4, -0.5}, 0.1, obstacle);
  expectGradient({-0.2, 0.6, 0.3}, 0.05, obstacle);
  // On the axis: degenerate but finite and orthogonal to the axis.
  result = mpc_geometry::distanceToObstacle({0, 0, 0.4}, 0.1, obstacle);
  EXPECT_TRUE(result.degenerate);
  EXPECT_NEAR(result.normal.norm(), 1.0, 1e-12);
  EXPECT_NEAR(result.normal.z(), 0.0, 1e-12);
  EXPECT_NEAR(result.distance, -0.3, 1e-12);
}

TEST(CollisionPrimitives, AabbOutside) {
  const Obstacle obstacle = aabb({-1, -1, -1}, {1, 1, 1});
  // Face region.
  DistanceResult result = mpc_geometry::distanceToObstacle({1.5, 0.2, -0.3}, 0.1, obstacle);
  EXPECT_NEAR(result.distance, 0.5 - 0.1, 1e-12);
  EXPECT_TRUE(result.normal.isApprox(Eigen::Vector3d::UnitX()));
  // Edge and corner regions.
  expectGradient({1.5, 1.5, 0.0}, 0.1, obstacle);
  expectGradient({-1.4, 1.3, 1.2}, 0.1, obstacle);
  result = mpc_geometry::distanceToObstacle({2, 2, 2}, 0.0, obstacle);
  EXPECT_NEAR(result.distance, std::sqrt(3.0), 1e-12);
}

TEST(CollisionPrimitives, AabbInside) {
  const Obstacle obstacle = aabb({-1, -1, -1}, {1, 1, 1});
  // Nearest face is +x (0.2 away): distance is minus that penetration minus the radius.
  const DistanceResult near_x = mpc_geometry::distanceToObstacle({0.8, 0.1, -0.3}, 0.1, obstacle);
  EXPECT_NEAR(near_x.distance, -0.2 - 0.1, 1e-12);
  EXPECT_TRUE(near_x.normal.isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(near_x.degenerate);
  // Nearest face is -z.
  const DistanceResult near_z = mpc_geometry::distanceToObstacle({0.0, 0.0, -0.9}, 0.0, obstacle);
  EXPECT_NEAR(near_z.distance, -0.1, 1e-12);
  EXPECT_TRUE(near_z.normal.isApprox(-Eigen::Vector3d::UnitZ()));
  // Moving along the normal increases the distance (consistent gradient sign).
  const DistanceResult nearer_z =
      mpc_geometry::distanceToObstacle({0.0, 0.0, -0.95}, 0.0, obstacle);
  EXPECT_GT(nearer_z.distance, near_z.distance);
}

TEST(CollisionPrimitives, SphereSphere) {
  const DistanceResult result =
      mpc_geometry::distanceBetweenSpheres({0, 0, 1}, 0.1, {0, 0, 0}, 0.2);
  EXPECT_NEAR(result.distance, 1.0 - 0.3, 1e-12);
  EXPECT_TRUE(result.normal.isApprox(Eigen::Vector3d::UnitZ()));
  const DistanceResult same =
      mpc_geometry::distanceBetweenSpheres({0, 0, 0}, 0.1, {0, 0, 0}, 0.2);
  EXPECT_TRUE(same.degenerate);
  EXPECT_NEAR(same.distance, -0.3, 1e-12);
}

TEST(CollisionPrimitives, TypeNames) {
  ObstacleType type;
  EXPECT_TRUE(mpc_geometry::parseObstacleType("capsule", type));
  EXPECT_EQ(type, ObstacleType::Capsule);
  EXPECT_FALSE(mpc_geometry::parseObstacleType("cylinder", type));
  EXPECT_STREQ(mpc_geometry::obstacleTypeName(ObstacleType::Aabb), "aabb");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
