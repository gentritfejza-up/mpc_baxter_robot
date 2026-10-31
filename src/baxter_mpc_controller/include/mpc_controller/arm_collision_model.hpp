#pragma once

/**
 * @file arm_collision_model.hpp
 * @brief Collision model of Baxter's two arms as chains of spheres, built from the
 *        URDF with KDL.
 *
 * For a joint vector the model yields every sphere centre in the base frame together
 * with the 3x7 point Jacobian of that centre, which the MPC uses to linearise the
 * distance constraints once per cycle. The model is not thread-safe: the KDL scratch
 * buffers are members.
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include "mpc_controller/collision_primitives.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace mpc_geometry {

/** @brief Number of joints of one Baxter arm. */
constexpr int kArmJoints = 7;

/** @brief Joint name suffixes of one arm in kinematic order; the full name is "<side>_<suffix>". */
inline constexpr const char* const kArmJointSuffixes[kArmJoints] = {"s0", "s1", "e0", "e1",
                                                                     "w0", "w1", "w2"};

/** @brief Joint positions of one arm in kinematic order (s0 s1 e0 e1 w0 w1 w2) [rad]. */
using JointVector = std::array<double, kArmJoints>;

/** @brief Linear velocity Jacobian of a point on the arm: d(point)/d(joints) [m/rad]. */
using PointJacobian = Eigen::Matrix<double, 3, kArmJoints>;

/** @brief One collision sphere attached to an arm link, as configured in YAML. */
struct SphereSpec {
  std::string link;        ///< URDF link name; "{side}" expands to left/right.
  std::string side;        ///< "" (both arms), "left" or "right": arm this entry is for.
  Eigen::Vector3d offset{Eigen::Vector3d::Zero()};  ///< Centre in the link frame [m].
  double radius{0.05};     ///< Sphere radius [m].
  bool checks_floor{true};      ///< Participates in the z-floor constraint.
  bool checks_body{true};       ///< Participates in the torso/head/pedestal constraints.
  bool checks_other_arm{true};  ///< Participates in the other-arm constraints.
  std::vector<std::string> ignored_obstacles;  ///< Body obstacle names this sphere never checks.
  double body_margin_override{-1.0};  ///< Body margin for this sphere [m]; negative: use the global one.
};

/** @brief Pose-dependent state of one collision sphere. */
struct SphereState {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};  ///< Centre in the base frame [m].
  PointJacobian jacobian{PointJacobian::Zero()};    ///< d(center)/d(joints) [m/rad].
};

/**
 * @brief Sphere collision model of both arms with forward kinematics and point Jacobians.
 */
class ArmCollisionModel {
 public:
  /**
   * @brief Builds both arm chains from the URDF and attaches the spheres.
   * @param urdf_xml Robot description (URDF XML).
   * @param base_link Link whose frame every result is expressed in.
   * @param sphere_template Sphere list applying to both arms ("{side}" is expanded;
   *        entries with a `side` only go to that arm).
   * @param body_obstacles Keep-out primitives of the robot body in the base frame.
   */
  ArmCollisionModel(const std::string& urdf_xml, const std::string& base_link,
                    const std::vector<SphereSpec>& sphere_template,
                    const std::vector<Obstacle>& body_obstacles);

  /// The KDL solvers keep references into this object, so it is neither copyable nor movable.
  ArmCollisionModel(const ArmCollisionModel&) = delete;
  ArmCollisionModel& operator=(const ArmCollisionModel&) = delete;

  /**
   * @brief Builds the model from the parameter server.
   *
   * Reads /robot_description (polling up to robot/geometry/robot_description_timeout_sec),
   * robot/geometry/base_link and robot/collision_geometry/{arm_spheres,body}.
   * @param node_handle Node handle used for the parameter lookups.
   * @return The model.
   */
  static std::unique_ptr<ArmCollisionModel> fromParams(ros::NodeHandle& node_handle);

  /**
   * @brief Parses the robot/collision_geometry/arm_spheres list.
   * @param list YAML list of sphere entries.
   * @return Parsed sphere template.
   */
  static std::vector<SphereSpec> parseSpheres(XmlRpc::XmlRpcValue& list);

  /**
   * @brief Parses the robot/collision_geometry/body list.
   * @param list YAML list of obstacle entries.
   * @return Parsed body obstacles.
   */
  static std::vector<Obstacle> parseObstacles(XmlRpc::XmlRpcValue& list);

  /** @brief Number of spheres per arm (both arms have the same count). */
  int sphereCount() const { return static_cast<int>(left_arm_.spheres.size()); }

  /**
   * @brief Sphere specifications of one arm.
   * @param side "left" or "right".
   */
  const std::vector<SphereSpec>& spheres(const std::string& side) const {
    return armChain(side).spheres;
  }

  /** @brief Keep-out primitives of the robot body. */
  const std::vector<Obstacle>& bodyObstacles() const { return body_obstacles_; }

  /** @brief Frame every centre and Jacobian is expressed in. */
  const std::string& baseLink() const { return base_link_; }

  /**
   * @brief KDL chain from the base link to the hand of one arm.
   * @param side "left" or "right".
   */
  const KDL::Chain& chain(const std::string& side) const { return armChain(side).chain; }

  /**
   * @brief Index of the chain segment a sphere is attached to.
   * @param side "left" or "right".
   * @param sphere Sphere index, 0..sphereCount()-1.
   */
  int sphereSegment(const std::string& side, int sphere) const {
    return armChain(side).sphere_segment_index.at(sphere);
  }

  /**
   * @brief Centres and point Jacobians of all spheres of one arm.
   * @param side "left" or "right".
   * @param joint_positions Arm joint positions [rad].
   * @param[out] sphere_states Resized to sphereCount().
   */
  void evaluate(const std::string& side, const JointVector& joint_positions,
                std::vector<SphereState>& sphere_states) const;

  /**
   * @brief Centres only (guard path, no Jacobians).
   * @param side "left" or "right".
   * @param joint_positions Arm joint positions [rad].
   * @param[out] centers Resized to sphereCount(); base frame [m].
   */
  void centersOnly(const std::string& side, const JointVector& joint_positions,
                   std::vector<Eigen::Vector3d>& centers) const;

 private:
  /** @brief One arm: KDL chain, attached spheres and the forward-kinematics scratch buffers. */
  struct ArmChain {
    std::string side;
    KDL::Chain chain;
    std::vector<SphereSpec> spheres;
    std::vector<int> sphere_segment_index;  ///< Chain segment carrying each sphere.
    std::vector<int> segment_joint_index;   ///< Arm joint (0..6) moving each segment, -1 if fixed.
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    mutable std::vector<KDL::Frame> segment_frames;  ///< Scratch: pose of every segment tip.
    mutable KDL::JntArray joint_scratch;             ///< Scratch: joint positions for KDL.
  };

  const ArmChain& armChain(const std::string& side) const;
  void buildArm(const KDL::Tree& tree, const std::string& side,
                const std::vector<SphereSpec>& sphere_template, ArmChain& arm_chain);
  /// Fills `arm_chain.segment_frames` for `joint_positions`.
  void runForwardKinematics(const ArmChain& arm_chain, const JointVector& joint_positions) const;

  std::string base_link_;
  ArmChain left_arm_;
  ArmChain right_arm_;
  std::vector<Obstacle> body_obstacles_;
};

}  // namespace mpc_geometry
