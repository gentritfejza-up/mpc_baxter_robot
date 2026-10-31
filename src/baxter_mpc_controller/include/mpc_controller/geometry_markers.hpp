#pragma once

/**
 * @file geometry_markers.hpp
 * @brief RViz visualisation of the collision guard: this arm's spheres (coloured by
 *        their current clearance), the other arm's spheres, the body keep-out shapes,
 *        the floor plane and the conveyor, all in the base frame.
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include "mpc_controller/arm_collision_model.hpp"
#include "mpc_controller/geometry_rows.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace mpc_geometry {

/**
 * @brief The conveyor as drawn in RViz (and spawned in Gazebo by baxter_mpc.launch):
 *        footprint centre and belt-axis yaw in the base frame; its top surface is z_min.
 */
struct ConveyorVisual {
  bool enabled{false};     ///< Draw the conveyor at all.
  double x{0.70};          ///< Footprint centre x in the base frame [m].
  double y{0.0};           ///< Footprint centre y in the base frame [m].
  double yaw{1.5708};      ///< Belt axis yaw about z [rad].
  double length{1.50};     ///< Along the belt axis [m].
  double width{0.25};      ///< Frame width across the belt axis [m].
  double belt_width{0.19}; ///< Belt width across the belt axis [m].
  double height{0.75};     ///< Frame height below the belt surface [m].
};

/** @brief Rate-limited MarkerArray publisher for the collision-guard geometry. */
class GeometryMarkerPublisher {
 public:
  /**
   * @param node_handle Node handle used to advertise the topic.
   * @param topic MarkerArray topic.
   * @param frame_id Frame the markers are expressed in (the model's base link).
   * @param rate_hz Maximum publish rate [Hz].
   */
  GeometryMarkerPublisher(ros::NodeHandle& node_handle, const std::string& topic,
                          const std::string& frame_id, double rate_hz = 10.0);

  /** @brief Sets the conveyor drawn below the floor plane. */
  void setConveyor(const ConveyorVisual& conveyor) { conveyor_ = conveyor; }

  /**
   * @brief Publishes the markers at most `rate_hz` times per second (call every cycle).
   * @param model Sphere collision model.
   * @param side Arm this controller drives, "left" or "right".
   * @param joint_positions Measured joint positions of this arm [rad].
   * @param other_arm_positions Measured joint positions of the other arm [rad]; may be nullptr.
   * @param floor_height Floor plane height in the base frame [m].
   * @param floor_active Draws the floor orange when the floor constraint is armed, grey otherwise.
   * @param guard Latest guard report (clearance colouring and the violating sphere).
   */
  void publish(const ArmCollisionModel& model, const std::string& side,
               const JointVector& joint_positions, const JointVector* other_arm_positions,
               double floor_height, bool floor_active, const GuardReport& guard);

 private:
  ros::Publisher publisher_;
  std::string frame_id_;
  ros::WallDuration period_;
  ros::WallTime last_publish_;
  ConveyorVisual conveyor_;
  std::vector<Eigen::Vector3d> sphere_centers_;     ///< Scratch: this arm.
  std::vector<Eigen::Vector3d> other_arm_centers_;  ///< Scratch: the other arm.
};

}  // namespace mpc_geometry
