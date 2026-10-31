#include "mpc_controller/collision_primitives.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mpc_geometry {
namespace {

/// Offsets shorter than this have no usable direction: the normal falls back to a default.
constexpr double kDegenerateNorm = 1e-9;

/// A unit vector orthogonal to `axis` (used when the sphere centre lies on a capsule axis).
Eigen::Vector3d anyPerpendicular(const Eigen::Vector3d& axis) {
  const Eigen::Vector3d reference =
      std::abs(axis.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d candidate = axis.cross(reference);
  const double norm = candidate.norm();
  return norm > kDegenerateNorm ? Eigen::Vector3d(candidate / norm) : Eigen::Vector3d::UnitZ();
}

/**
 * @brief Distance result from the offset between the sphere centre and the closest core point.
 * @param offset Sphere centre minus the closest point of the primitive core [m].
 * @param surface_gap Sum of the radii that separate the two surfaces along the offset [m].
 * @param fallback_normal Normal reported when the offset has no direction.
 */
DistanceResult fromOffset(const Eigen::Vector3d& offset, double surface_gap,
                          const Eigen::Vector3d& fallback_normal) {
  DistanceResult result;
  const double norm = offset.norm();
  if (norm > kDegenerateNorm) {
    result.normal = offset / norm;
  } else {
    result.normal = fallback_normal;
    result.degenerate = true;
  }
  result.distance = norm - surface_gap;
  return result;
}

}  // namespace

DistanceResult distanceToObstacle(const Eigen::Vector3d& center, double radius,
                                  const Obstacle& obstacle) {
  switch (obstacle.type) {
    case ObstacleType::ZPlane: {
      DistanceResult result;
      result.distance = center.z() - radius - obstacle.start_point.z();
      result.normal = Eigen::Vector3d::UnitZ();
      return result;
    }
    case ObstacleType::Sphere:
      return fromOffset(center - obstacle.start_point, radius + obstacle.radius,
                        Eigen::Vector3d::UnitZ());
    case ObstacleType::Capsule: {
      const Eigen::Vector3d axis = obstacle.end_point - obstacle.start_point;
      const double length_squared = axis.squaredNorm();
      double axis_fraction = 0.0;  // position of the closest core point along the segment, 0..1
      if (length_squared > kDegenerateNorm * kDegenerateNorm) {
        axis_fraction =
            std::clamp((center - obstacle.start_point).dot(axis) / length_squared, 0.0, 1.0);
      }
      const Eigen::Vector3d closest = obstacle.start_point + axis_fraction * axis;
      return fromOffset(center - closest, radius + obstacle.radius, anyPerpendicular(axis));
    }
    case ObstacleType::Aabb: {
      const Eigen::Vector3d closest =
          center.cwiseMax(obstacle.start_point).cwiseMin(obstacle.end_point);
      const Eigen::Vector3d offset = center - closest;
      if (offset.norm() > kDegenerateNorm) {
        return fromOffset(offset, radius, Eigen::Vector3d::UnitZ());
      }
      // Inside the box: the signed distance is minus the smallest face penetration
      // and the gradient is that face's outward normal.
      DistanceResult result;
      double smallest_penetration = std::numeric_limits<double>::infinity();
      for (int axis_index = 0; axis_index < 3; ++axis_index) {
        const double to_min_face = center[axis_index] - obstacle.start_point[axis_index];
        const double to_max_face = obstacle.end_point[axis_index] - center[axis_index];
        if (to_min_face < smallest_penetration) {
          smallest_penetration = to_min_face;
          result.normal = Eigen::Vector3d::Zero();
          result.normal[axis_index] = -1.0;
        }
        if (to_max_face < smallest_penetration) {
          smallest_penetration = to_max_face;
          result.normal = Eigen::Vector3d::Zero();
          result.normal[axis_index] = 1.0;
        }
      }
      result.distance = -smallest_penetration - radius;
      result.degenerate = true;
      return result;
    }
  }
  return DistanceResult{};
}

DistanceResult distanceBetweenSpheres(const Eigen::Vector3d& center_a, double radius_a,
                                      const Eigen::Vector3d& center_b, double radius_b) {
  return fromOffset(center_a - center_b, radius_a + radius_b, Eigen::Vector3d::UnitZ());
}

const char* obstacleTypeName(ObstacleType type) {
  switch (type) {
    case ObstacleType::ZPlane: return "zplane";
    case ObstacleType::Sphere: return "sphere";
    case ObstacleType::Capsule: return "capsule";
    case ObstacleType::Aabb: return "aabb";
  }
  return "unknown";
}

bool parseObstacleType(const std::string& text, ObstacleType& type) {
  if (text == "zplane") type = ObstacleType::ZPlane;
  else if (text == "sphere") type = ObstacleType::Sphere;
  else if (text == "capsule") type = ObstacleType::Capsule;
  else if (text == "aabb") type = ObstacleType::Aabb;
  else return false;
  return true;
}

}  // namespace mpc_geometry
