#pragma once

/**
 * @file collision_primitives.hpp
 * @brief Signed surface-to-surface distances between a sphere and the keep-out
 *        primitives of the MPC collision guard (floor plane, sphere, capsule, box).
 *
 * Every distance function also returns the gradient of the distance with respect
 * to the sphere centre (a unit normal), which is what the per-cycle linearisation
 * of the collision rows needs.
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace mpc_geometry {

/** @brief Shape of a keep-out primitive. */
enum class ObstacleType { ZPlane, Sphere, Capsule, Aabb };

/** @brief Which constraint group an obstacle belongs to (floor, robot body, other arm). */
enum class ObstacleGroup { Floor, Body, OtherArm };

/**
 * @brief One keep-out primitive in the robot base frame.
 *
 * The meaning of the two points depends on the type:
 * - ZPlane: `start_point.z()` is the floor height, `end_point` unused.
 * - Sphere: `start_point` is the centre, `end_point` unused.
 * - Capsule: the core segment runs from `start_point` to `end_point`.
 * - Aabb: `start_point` is the minimum corner, `end_point` the maximum corner.
 */
struct Obstacle {
  std::string name;                               ///< Name used in diagnostics and ignore lists.
  ObstacleType type{ObstacleType::Sphere};        ///< Primitive shape.
  ObstacleGroup group{ObstacleGroup::Body};       ///< Constraint group.
  Eigen::Vector3d start_point{Eigen::Vector3d::Zero()};  ///< See the struct description [m].
  Eigen::Vector3d end_point{Eigen::Vector3d::Zero()};    ///< See the struct description [m].
  double radius{0.0};                             ///< Sphere / Capsule radius [m]; unused otherwise.
};

/** @brief Distance query result between a sphere and a primitive. */
struct DistanceResult {
  double distance{0.0};                   ///< Surface-to-surface distance, negative when penetrating [m].
  Eigen::Vector3d normal{0.0, 0.0, 1.0};  ///< d(distance)/d(sphere centre), unit length [-].
  bool degenerate{false};  ///< Centre coincided with the primitive core (or lies inside a box); normal is a fallback.
};

/**
 * @brief Signed distance from a sphere to an obstacle.
 * @param center Sphere centre in the base frame [m].
 * @param radius Sphere radius [m].
 * @param obstacle Keep-out primitive.
 * @return Distance, gradient with respect to `center` and the degeneracy flag.
 */
DistanceResult distanceToObstacle(const Eigen::Vector3d& center, double radius,
                                  const Obstacle& obstacle);

/**
 * @brief Signed distance between two spheres.
 * @param center_a Centre of the first sphere [m]; the returned normal is the gradient with respect to it.
 * @param radius_a Radius of the first sphere [m].
 * @param center_b Centre of the second sphere [m].
 * @param radius_b Radius of the second sphere [m].
 * @return Distance, gradient with respect to `center_a` and the degeneracy flag.
 */
DistanceResult distanceBetweenSpheres(const Eigen::Vector3d& center_a, double radius_a,
                                      const Eigen::Vector3d& center_b, double radius_b);

/**
 * @brief Lower-case name of an obstacle type as used in the YAML configuration.
 * @param type Obstacle type.
 * @return "zplane", "sphere", "capsule" or "aabb".
 */
const char* obstacleTypeName(ObstacleType type);

/**
 * @brief Parses a YAML obstacle type name.
 * @param text Type name ("zplane", "sphere", "capsule", "aabb").
 * @param[out] type Parsed type; untouched when the name is unknown.
 * @return True when the name was recognised.
 */
bool parseObstacleType(const std::string& text, ObstacleType& type);

}  // namespace mpc_geometry
