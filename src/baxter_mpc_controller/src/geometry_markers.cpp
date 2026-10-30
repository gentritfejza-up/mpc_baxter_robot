#include "mpc_controller/geometry_markers.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_geometry {
namespace {

visualization_msgs::Marker baseMarker(const std::string& frame_id, const std::string& marker_namespace,
                                      int id, int type, double red, double green, double blue,
                                      double alpha) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;
  marker.lifetime = ros::Duration(0.5);
  return marker;
}

void setPosition(visualization_msgs::Marker& marker, const Eigen::Vector3d& position) {
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();
}

geometry_msgs::Point toPoint(const Eigen::Vector3d& position) {
  geometry_msgs::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  return point;
}

/// Clearance colour: red at or below zero, yellow at the margin, green when clear.
void setClearanceColor(double distance, double margin, visualization_msgs::Marker& marker) {
  const double clearance_ratio = std::clamp(distance / std::max(margin, 1e-3), 0.0, 2.0) / 2.0;  // 0..1
  marker.color.r = static_cast<float>(1.0 - std::max(0.0, 2.0 * clearance_ratio - 1.0));
  marker.color.g = static_cast<float>(std::min(1.0, 2.0 * clearance_ratio));
  marker.color.b = 0.0f;
}

}  // namespace

GeometryMarkerPublisher::GeometryMarkerPublisher(ros::NodeHandle& node_handle,
                                                 const std::string& topic,
                                                 const std::string& frame_id, double rate_hz)
    : frame_id_(frame_id), period_(rate_hz > 0.0 ? 1.0 / rate_hz : 0.1) {
  publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(topic, 1);
}

void GeometryMarkerPublisher::publish(const ArmCollisionModel& model, const std::string& side,
                                      const JointVector& joint_positions,
                                      const JointVector* other_arm_positions, double floor_height,
                                      bool floor_active, const GuardReport& guard) {
  const ros::WallTime now = ros::WallTime::now();
  if (now - last_publish_ < period_) return;
  last_publish_ = now;
  if (publisher_.getNumSubscribers() == 0) return;

  visualization_msgs::MarkerArray array;
  const std::string other_side = side == "left" ? "right" : "left";
  const double min_distance_now =
      std::min({guard.min_distance_now[kFloorGroup], guard.min_distance_now[kBodyGroup],
                guard.min_distance_now[kArmGroup]});

  // This arm's spheres, coloured by the arm's overall clearance now.
  model.centersOnly(side, joint_positions, sphere_centers_);
  const std::vector<SphereSpec>& spheres = model.spheres(side);
  for (std::size_t sphere = 0; sphere < spheres.size(); ++sphere) {
    visualization_msgs::Marker marker =
        baseMarker(frame_id_, side + "_arm_spheres", static_cast<int>(sphere),
                   visualization_msgs::Marker::SPHERE, 0.0, 1.0, 0.0, 0.55);
    setPosition(marker, sphere_centers_[sphere]);
    marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * spheres[sphere].radius;
    setClearanceColor(std::isfinite(min_distance_now) ? min_distance_now : 1.0, 0.05, marker);
    if (guard.violated && guard.worst_sphere == static_cast<int>(sphere)) {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.9f;
    }
    array.markers.push_back(marker);
  }

  // The other arm as seen by this controller (measured joints, blue).
  if (other_arm_positions) {
    model.centersOnly(other_side, *other_arm_positions, other_arm_centers_);
    const std::vector<SphereSpec>& other_spheres = model.spheres(other_side);
    for (std::size_t sphere = 0; sphere < other_spheres.size(); ++sphere) {
      visualization_msgs::Marker marker =
          baseMarker(frame_id_, side + "_sees_" + other_side, static_cast<int>(sphere),
                     visualization_msgs::Marker::SPHERE, 0.2, 0.4, 1.0, 0.35);
      setPosition(marker, other_arm_centers_[sphere]);
      marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * other_spheres[sphere].radius;
      array.markers.push_back(marker);
    }
  }

  // Body keep-out shapes (grey).
  int obstacle_id = 0;
  for (const Obstacle& obstacle : model.bodyObstacles()) {
    visualization_msgs::Marker marker =
        baseMarker(frame_id_, side + "_body_obstacles", obstacle_id++,
                   visualization_msgs::Marker::SPHERE, 0.5, 0.5, 0.5, 0.25);
    switch (obstacle.type) {
      case ObstacleType::Sphere:
        setPosition(marker, obstacle.start_point);
        marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * obstacle.radius;
        break;
      case ObstacleType::Aabb:
        marker.type = visualization_msgs::Marker::CUBE;
        setPosition(marker, 0.5 * (obstacle.start_point + obstacle.end_point));
        marker.scale.x = obstacle.end_point.x() - obstacle.start_point.x();
        marker.scale.y = obstacle.end_point.y() - obstacle.start_point.y();
        marker.scale.z = obstacle.end_point.z() - obstacle.start_point.z();
        break;
      case ObstacleType::Capsule:
        // Drawn as a line strip of the axis with the capsule radius as line width.
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 2.0 * obstacle.radius;
        marker.points = {toPoint(obstacle.start_point), toPoint(obstacle.end_point)};
        break;
      case ObstacleType::ZPlane:
        continue;
    }
    array.markers.push_back(marker);
  }

  // The floor plane: orange once the floor constraint is armed, grey before.
  visualization_msgs::Marker floor =
      floor_active ? baseMarker(frame_id_, side + "_floor", 0, visualization_msgs::Marker::CUBE,
                                1.0, 0.5, 0.0, 0.3)
                   : baseMarker(frame_id_, side + "_floor", 0, visualization_msgs::Marker::CUBE,
                                0.5, 0.5, 0.5, 0.15);
  setPosition(floor, Eigen::Vector3d(0.3, 0.0, floor_height - 0.005));
  floor.scale.x = 2.0;
  floor.scale.y = 2.4;
  floor.scale.z = 0.01;
  array.markers.push_back(floor);

  // The conveyor body (steel frame, dark belt) with its top surface at the floor height.
  if (conveyor_.enabled) {
    const double half_yaw = 0.5 * conveyor_.yaw;
    visualization_msgs::Marker frame =
        baseMarker(frame_id_, side + "_conveyor", 0, visualization_msgs::Marker::CUBE, 0.78, 0.78,
                   0.80, 0.5);
    setPosition(frame, Eigen::Vector3d(conveyor_.x, conveyor_.y,
                                       floor_height - 0.5 * conveyor_.height));
    frame.pose.orientation.z = std::sin(half_yaw);
    frame.pose.orientation.w = std::cos(half_yaw);
    frame.scale.x = conveyor_.length;
    frame.scale.y = conveyor_.width;
    frame.scale.z = conveyor_.height;
    array.markers.push_back(frame);
    visualization_msgs::Marker belt =
        baseMarker(frame_id_, side + "_conveyor", 1, visualization_msgs::Marker::CUBE, 0.08, 0.08,
                   0.08, 0.9);
    setPosition(belt, Eigen::Vector3d(conveyor_.x, conveyor_.y, floor_height + 0.001));
    belt.pose.orientation = frame.pose.orientation;
    belt.scale.x = conveyor_.length;
    belt.scale.y = conveyor_.belt_width;
    belt.scale.z = 0.002;
    array.markers.push_back(belt);
  }

  publisher_.publish(array);
}

}  // namespace mpc_geometry
