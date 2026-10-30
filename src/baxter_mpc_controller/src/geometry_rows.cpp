#include "mpc_controller/geometry_rows.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpc_geometry {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

bool ignoresObstacle(const SphereSpec& spec, const std::string& obstacle_name) {
  return std::find(spec.ignored_obstacles.begin(), spec.ignored_obstacles.end(), obstacle_name) !=
         spec.ignored_obstacles.end();
}

}  // namespace

GeometryParams GeometryParams::fromParams(ros::NodeHandle& node_handle) {
  GeometryParams params;
  node_handle.param("robot/geometry/z_min", params.z_min, params.z_min);
  node_handle.param("robot/geometry/floor_margin", params.floor_margin, params.floor_margin);
  node_handle.param("robot/geometry/body_margin", params.body_margin, params.body_margin);
  node_handle.param("robot/geometry/arm_margin", params.arm_margin, params.arm_margin);
  node_handle.param("robot/geometry/activation_distance", params.activation_distance,
                    params.activation_distance);
  node_handle.param("robot/geometry/margin_growth_per_sec", params.margin_growth_per_sec,
                    params.margin_growth_per_sec);
  node_handle.param("robot/geometry/max_recovery_per_horizon", params.max_recovery_per_horizon,
                    params.max_recovery_per_horizon);
  node_handle.param("robot/geometry/rows_per_sphere/floor", params.rows_floor, params.rows_floor);
  node_handle.param("robot/geometry/rows_per_sphere/body", params.rows_body, params.rows_body);
  node_handle.param("robot/geometry/rows_per_sphere/other_arm", params.rows_other_arm,
                    params.rows_other_arm);
  node_handle.param("robot/geometry/step_stride", params.step_stride, params.step_stride);
  node_handle.param("robot/geometry/guard/margin", params.guard_margin, params.guard_margin);
  node_handle.param("robot/geometry/guard/worsening_tolerance", params.guard_worsening_tolerance,
                    params.guard_worsening_tolerance);
  params.validate();
  return params;
}

void GeometryParams::validate() const {
  const auto finite = [](double value) { return std::isfinite(value); };
  if (!finite(z_min) || !finite(floor_margin) || !finite(body_margin) || !finite(arm_margin) ||
      !finite(activation_distance) || !finite(guard_margin) || !finite(guard_worsening_tolerance)) {
    throw std::invalid_argument("robot/geometry: all distances must be finite");
  }
  if (!std::isfinite(margin_growth_per_sec) || margin_growth_per_sec < 0.0) {
    throw std::invalid_argument("robot/geometry/margin_growth_per_sec must be finite and >= 0");
  }
  if (floor_margin < 0.0 || body_margin < 0.0 || arm_margin < 0.0 || activation_distance < 0.0 ||
      guard_worsening_tolerance < 0.0 || !std::isfinite(max_recovery_per_horizon) ||
      max_recovery_per_horizon <= 0.0) {
    throw std::invalid_argument(
        "robot/geometry: margins and activation distance must be nonnegative");
  }
  if (rows_floor < 0 || rows_floor > 1 || rows_body < 0 || rows_other_arm < 0 || step_stride < 1) {
    throw std::invalid_argument(
        "robot/geometry: rows_per_sphere.floor in {0,1}, other row counts >= 0, step_stride >= 1");
  }
  const double guard_floor = guard_margin + guard_worsening_tolerance;
  if ((rows_floor > 0 && floor_margin <= guard_floor) ||
      (rows_body > 0 && body_margin <= guard_floor) ||
      (rows_other_arm > 0 && arm_margin <= guard_floor)) {
    throw std::invalid_argument(
        "robot/geometry: every active row margin (floor/body/arm) must exceed guard.margin + "
        "guard.worsening_tolerance, otherwise plans that rest at the row margin are rejected forever");
  }
}

void GeometryRowBatch::reset(const GeometryLayout& layout) {
  coefficients.assign(layout.coefficientCount(), 0.0);
  lower_bounds.assign(layout.rowCount(), -kInfinity);
  active_rows = 0;
  degenerate_normals = 0;
  min_linearised_distance = infiniteGroupDistances();
}

GeometryRowLinearizer::GeometryRowLinearizer(const ArmCollisionModel& model,
                                             const std::string& side,
                                             const GeometryParams& params,
                                             const GeometryLayout& layout)
    : model_(model),
      side_(side),
      other_side_(side == "left" ? "right" : "left"),
      params_(params),
      layout_(layout) {
  if (side != "left" && side != "right") throw std::invalid_argument("side must be left or right");
  if (layout_.sphere_count != model_.sphereCount() || layout_.rows_floor != params_.rows_floor ||
      layout_.rows_body != params_.rows_body || layout_.rows_other_arm != params_.rows_other_arm ||
      layout_.prediction_horizon <= 0) {
    throw std::invalid_argument("GeometryLayout does not match the model and parameters");
  }
  floor_.name = "floor";
  floor_.type = ObstacleType::ZPlane;
  floor_.group = ObstacleGroup::Floor;
  floor_.start_point = Eigen::Vector3d(0.0, 0.0, params_.z_min);
}

bool GeometryRowLinearizer::stepActive(int step) const {
  return params_.step_stride <= 1 || (step - 1) % params_.step_stride == 0 ||
         step == layout_.prediction_horizon;
}

double GeometryRowLinearizer::bodyMargin(const SphereSpec& sphere) const {
  return sphere.body_margin_override >= 0.0 ? sphere.body_margin_override : params_.body_margin;
}

void GeometryRowLinearizer::computeOtherArmCenters(const OtherArmTrajectory& other_arm) const {
  has_other_arm_centers_ = !other_arm.empty();
  if (!has_other_arm_centers_) return;
  if (static_cast<int>(other_arm.size()) != layout_.prediction_horizon + 1) {
    throw std::invalid_argument(
        "other arm trajectory needs prediction_horizon + 1 entries (now + each step)");
  }
  other_arm_centers_.resize(other_arm.size());
  for (std::size_t step = 0; step < other_arm.size(); ++step) {
    model_.centersOnly(other_side_, other_arm[step], other_arm_centers_[step]);
  }
}

OtherArmTrajectory GeometryRowLinearizer::staticOtherArm(
    const JointVector* other_arm_positions) const {
  if (!other_arm_positions) return {};
  return OtherArmTrajectory(static_cast<std::size_t>(layout_.prediction_horizon + 1),
                            *other_arm_positions);
}

void GeometryRowLinearizer::linearize(const std::vector<JointVector>& linearisation_points,
                                      const JointVector* other_arm_positions,
                                      GeometryRowBatch& batch) const {
  linearize(linearisation_points, staticOtherArm(other_arm_positions), batch);
}

GuardReport GeometryRowLinearizer::evaluateTrajectory(const JointVector& measured_positions,
                                                      const std::vector<JointVector>& predicted,
                                                      const JointVector* other_arm_positions) const {
  return evaluateTrajectory(measured_positions, predicted, staticOtherArm(other_arm_positions));
}

void GeometryRowLinearizer::bodyCandidates(const SphereSpec& spec, const Eigen::Vector3d& center,
                                           std::vector<Candidate>& candidates) const {
  candidates.clear();
  if (!spec.checks_body) return;
  const std::vector<Obstacle>& obstacles = model_.bodyObstacles();
  for (int obstacle_index = 0; obstacle_index < static_cast<int>(obstacles.size());
       ++obstacle_index) {
    if (ignoresObstacle(spec, obstacles[obstacle_index].name)) continue;
    const DistanceResult result = distanceToObstacle(center, spec.radius, obstacles[obstacle_index]);
    candidates.push_back({result.distance, obstacle_index, result.normal, result.degenerate});
  }
}

void GeometryRowLinearizer::armCandidates(const SphereSpec& spec, const Eigen::Vector3d& center,
                                          int step, std::vector<Candidate>& candidates) const {
  candidates.clear();
  if (!spec.checks_other_arm || !has_other_arm_centers_) return;
  const std::vector<SphereSpec>& other_spheres = model_.spheres(other_side_);
  const std::vector<Eigen::Vector3d>& other_centers = other_arm_centers_[static_cast<std::size_t>(step)];
  for (int other_index = 0; other_index < static_cast<int>(other_spheres.size()); ++other_index) {
    const DistanceResult result = distanceBetweenSpheres(
        center, spec.radius, other_centers[other_index], other_spheres[other_index].radius);
    candidates.push_back({result.distance, other_index, result.normal, result.degenerate});
  }
}

void GeometryRowLinearizer::keepNearest(std::vector<Candidate>& candidates, int slot_count) {
  const auto by_distance = [](const Candidate& left, const Candidate& right) {
    return left.distance < right.distance;
  };
  if (static_cast<int>(candidates.size()) > slot_count) {
    std::partial_sort(candidates.begin(), candidates.begin() + slot_count, candidates.end(),
                      by_distance);
    candidates.resize(slot_count);
  } else {
    std::sort(candidates.begin(), candidates.end(), by_distance);
  }
  // Stable slot assignment across cycles: ascending obstacle index.
  std::sort(candidates.begin(), candidates.end(), [](const Candidate& left, const Candidate& right) {
    return left.obstacle_index < right.obstacle_index;
  });
}

/**
 * Writes one row  g^T X_k >= requested + g^T qbar  with g = n^T J (see the header), where
 * the requested distance change  margin - d(qbar)  is capped by max_recovery_per_horizon.
 */
void GeometryRowLinearizer::writeRow(const SphereState& state,
                                     const JointVector& linearisation_point,
                                     const Candidate& candidate, double margin, int row,
                                     GeometryRowBatch& batch) const {
  double gradient_dot_point = 0.0;  // g^T qbar
  for (int joint = 0; joint < kArmJoints; ++joint) {
    const double gradient = candidate.normal.dot(state.jacobian.col(joint));
    batch.coefficients[row * kArmJoints + joint] = gradient;
    gradient_dot_point += gradient * linearisation_point[joint];
  }
  const double requested_change =
      std::min(margin - candidate.distance, params_.max_recovery_per_horizon);
  batch.lower_bounds[row] = requested_change + gradient_dot_point;
  ++batch.active_rows;
  if (candidate.degenerate) ++batch.degenerate_normals;
}

void GeometryRowLinearizer::linearize(const std::vector<JointVector>& linearisation_points,
                                      const OtherArmTrajectory& other_arm,
                                      GeometryRowBatch& batch) const {
  if (static_cast<int>(linearisation_points.size()) != layout_.prediction_horizon) {
    throw std::invalid_argument("linearize: one joint vector per prediction step is required");
  }
  batch.reset(layout_);
  computeOtherArmCenters(other_arm);
  const std::vector<SphereSpec>& spheres = model_.spheres(side_);

  for (int step = 1; step <= layout_.prediction_horizon; ++step) {
    if (!stepActive(step)) continue;
    const JointVector& point = linearisation_points[step - 1];
    model_.evaluate(side_, point, sphere_states_);
    for (int sphere = 0; sphere < layout_.sphere_count; ++sphere) {
      const SphereSpec& spec = spheres[sphere];
      const SphereState& state = sphere_states_[sphere];
      int slot = 0;
      // Floor slot.
      if (layout_.rows_floor > 0) {
        if (spec.checks_floor && floor_enabled_) {
          const DistanceResult result = distanceToObstacle(state.center, spec.radius, floor_);
          batch.min_linearised_distance[kFloorGroup] =
              std::min(batch.min_linearised_distance[kFloorGroup], result.distance);
          if (result.distance < params_.activation_distance) {
            writeRow(state, point, {result.distance, 0, result.normal, result.degenerate},
                     stepMargin(params_.floor_margin, step), layout_.rowIndex(step, sphere, slot),
                     batch);
          }
        }
        slot += layout_.rows_floor;
      }
      // Body slots: the nearest obstacles within the activation distance.
      if (layout_.rows_body > 0) {
        bodyCandidates(spec, state.center, candidates_);
        for (const Candidate& candidate : candidates_) {
          batch.min_linearised_distance[kBodyGroup] =
              std::min(batch.min_linearised_distance[kBodyGroup], candidate.distance);
        }
        keepNearest(candidates_, layout_.rows_body);
        int used_slots = 0;
        for (const Candidate& candidate : candidates_) {
          if (candidate.distance >= params_.activation_distance) continue;
          writeRow(state, point, candidate, stepMargin(bodyMargin(spec), step),
                   layout_.rowIndex(step, sphere, slot + used_slots), batch);
          ++used_slots;
        }
        slot += layout_.rows_body;
      }
      // Other-arm slots: the nearest other-arm spheres within the activation distance.
      if (layout_.rows_other_arm > 0) {
        armCandidates(spec, state.center, step, candidates_);
        for (const Candidate& candidate : candidates_) {
          batch.min_linearised_distance[kArmGroup] =
              std::min(batch.min_linearised_distance[kArmGroup], candidate.distance);
        }
        keepNearest(candidates_, layout_.rows_other_arm);
        int used_slots = 0;
        for (const Candidate& candidate : candidates_) {
          if (candidate.distance >= params_.activation_distance) continue;
          writeRow(state, point, candidate, stepMargin(params_.arm_margin, step),
                   layout_.rowIndex(step, sphere, slot + used_slots), batch);
          ++used_slots;
        }
        slot += layout_.rows_other_arm;
      }
    }
  }
}

/**
 * Gauss-Newton projection: at every iteration the sphere/obstacle pair with the most
 * negative  d - margin - extra_clearance  is found and the minimum-norm joint step
 *     dq = g^T (-(d - margin - extra)) / |g|^2,   g = n^T J,
 * raises its linearised distance to the margin. Degenerate normals are skipped.
 */
JointVector GeometryRowLinearizer::projectReference(const JointVector& reference_positions,
                                                    const OtherArmTrajectory& other_arm,
                                                    double* max_shift, int max_iterations,
                                                    double extra_clearance,
                                                    const std::vector<double>* lower_bounds,
                                                    const std::vector<double>* upper_bounds) const {
  JointVector projected = reference_positions;
  const bool has_lower = lower_bounds && static_cast<int>(lower_bounds->size()) == kArmJoints;
  const bool has_upper = upper_bounds && static_cast<int>(upper_bounds->size()) == kArmJoints;
  const auto clamp_to_joint_limits = [&]() {
    for (int joint = 0; joint < kArmJoints; ++joint) {
      if (has_lower) projected[joint] = std::max(projected[joint], (*lower_bounds)[joint]);
      if (has_upper) projected[joint] = std::min(projected[joint], (*upper_bounds)[joint]);
    }
  };
  clamp_to_joint_limits();
  if (max_shift) *max_shift = 0.0;
  computeOtherArmCenters(other_arm);
  const std::vector<SphereSpec>& spheres = model_.spheres(side_);
  const std::vector<Obstacle>& obstacles = model_.bodyObstacles();
  const std::vector<SphereSpec>& other_spheres = model_.spheres(other_side_);
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    model_.evaluate(side_, projected, sphere_states_);
    double worst_violation = 0.0;  // most negative (distance - margin - extra_clearance)
    Eigen::Matrix<double, 1, kArmJoints> worst_gradient;
    bool found_violation = false;
    for (int sphere = 0; sphere < layout_.sphere_count; ++sphere) {
      const SphereSpec& spec = spheres[sphere];
      const SphereState& state = sphere_states_[sphere];
      const auto consider = [&](const DistanceResult& result, double margin) {
        if (result.degenerate) return;
        const double violation = result.distance - margin - extra_clearance;
        if (violation < worst_violation) {
          worst_violation = violation;
          worst_gradient = result.normal.transpose() * state.jacobian;
          found_violation = true;
        }
      };
      if (spec.checks_floor && floor_enabled_) {
        consider(distanceToObstacle(state.center, spec.radius, floor_), params_.floor_margin);
      }
      if (spec.checks_body) {
        for (const Obstacle& obstacle : obstacles) {
          if (ignoresObstacle(spec, obstacle.name)) continue;
          consider(distanceToObstacle(state.center, spec.radius, obstacle), bodyMargin(spec));
        }
      }
      if (spec.checks_other_arm && has_other_arm_centers_) {
        const std::vector<Eigen::Vector3d>& other_centers_now = other_arm_centers_[0];
        for (std::size_t other_index = 0; other_index < other_spheres.size(); ++other_index) {
          consider(distanceBetweenSpheres(state.center, spec.radius, other_centers_now[other_index],
                                          other_spheres[other_index].radius),
                   params_.arm_margin);
        }
      }
    }
    if (!found_violation) break;
    const double gradient_norm_squared = worst_gradient.squaredNorm();
    if (gradient_norm_squared < 1e-8) break;
    const Eigen::Matrix<double, kArmJoints, 1> joint_step =
        worst_gradient.transpose() * (-worst_violation / gradient_norm_squared);
    for (int joint = 0; joint < kArmJoints; ++joint) projected[joint] += joint_step(joint);
    clamp_to_joint_limits();
  }
  if (max_shift) {
    for (int joint = 0; joint < kArmJoints; ++joint) {
      *max_shift = std::max(*max_shift, std::abs(projected[joint] - reference_positions[joint]));
    }
  }
  return projected;
}

/**
 * Guard rule. Obstacles are indexed per sphere as [floor | body obstacles | other-arm
 * spheres]. The distance of the arm to each obstacle now (minimum over the spheres) is
 * the no-worsening reference: while the arm climbs out of an obstacle one sphere may get
 * slightly closer than it is now, which must not reject the plan, but no obstacle may be
 * entered deeper than the arm is now (minus guard_worsening_tolerance), and never below
 * guard_margin.
 */
GuardReport GeometryRowLinearizer::evaluateTrajectory(const JointVector& measured_positions,
                                                      const std::vector<JointVector>& predicted,
                                                      const OtherArmTrajectory& other_arm) const {
  GuardReport report;
  computeOtherArmCenters(other_arm);
  if (has_other_arm_centers_ && static_cast<int>(predicted.size()) > layout_.prediction_horizon) {
    throw std::invalid_argument("evaluateTrajectory: more predicted steps than other-arm entries");
  }
  const std::vector<SphereSpec>& spheres = model_.spheres(side_);
  const std::vector<Obstacle>& obstacles = model_.bodyObstacles();
  const std::vector<SphereSpec>& other_spheres = model_.spheres(other_side_);
  const int body_count = static_cast<int>(obstacles.size());
  const int arm_count = has_other_arm_centers_ ? static_cast<int>(other_spheres.size()) : 0;
  const int obstacles_per_sphere = 1 + body_count + arm_count;  // floor, bodies, other-arm spheres

  current_distance_per_obstacle_.assign(static_cast<std::size_t>(obstacles_per_sphere), kInfinity);
  const auto obstacle_name = [&](int obstacle_index) -> std::string {
    if (obstacle_index == 0) return floor_.name;
    if (obstacle_index <= body_count) return obstacles[obstacle_index - 1].name;
    return other_side_ + " sphere " + std::to_string(obstacle_index - 1 - body_count);
  };

  // is_measured_state: record the per-obstacle reference distances; otherwise check the
  // predicted pose at `step` against them.
  const auto evaluate_pose = [&](const JointVector& positions, bool is_measured_state, int step) {
    model_.centersOnly(side_, positions, sphere_centers_);
    for (int sphere = 0; sphere < layout_.sphere_count; ++sphere) {
      const SphereSpec& spec = spheres[sphere];
      const auto consider = [&](int obstacle_index, int group, double distance) {
        if (is_measured_state) {
          double& current = current_distance_per_obstacle_[static_cast<std::size_t>(obstacle_index)];
          current = std::min(current, distance);
          report.min_distance_now[group] = std::min(report.min_distance_now[group], distance);
          return;
        }
        report.min_distance[group] = std::min(report.min_distance[group], distance);
        const double reference =
            current_distance_per_obstacle_[static_cast<std::size_t>(obstacle_index)];
        if (distance < params_.guard_margin &&
            distance < reference - params_.guard_worsening_tolerance &&
            distance < report.worst_distance) {
          report.violated = true;
          report.worst_distance = distance;
          report.worst_step = step;
          report.worst_sphere = sphere;
          report.worst_group = group;
          report.worst_obstacle = obstacle_name(obstacle_index);
        }
      };
      if (spec.checks_floor && floor_enabled_) {
        consider(0, kFloorGroup,
                 distanceToObstacle(sphere_centers_[sphere], spec.radius, floor_).distance);
      }
      if (spec.checks_body) {
        for (int body_index = 0; body_index < body_count; ++body_index) {
          if (ignoresObstacle(spec, obstacles[body_index].name)) continue;
          consider(1 + body_index, kBodyGroup,
                   distanceToObstacle(sphere_centers_[sphere], spec.radius, obstacles[body_index])
                       .distance);
        }
      }
      if (spec.checks_other_arm && arm_count > 0) {
        const std::vector<Eigen::Vector3d>& other_centers =
            other_arm_centers_[static_cast<std::size_t>(step)];
        for (int other_index = 0; other_index < arm_count; ++other_index) {
          consider(1 + body_count + other_index, kArmGroup,
                   distanceBetweenSpheres(sphere_centers_[sphere], spec.radius,
                                          other_centers[other_index],
                                          other_spheres[other_index].radius)
                       .distance);
        }
      }
    }
  };
  evaluate_pose(measured_positions, true, 0);
  for (int step = 1; step <= static_cast<int>(predicted.size()); ++step) {
    evaluate_pose(predicted[step - 1], false, step);
  }
  return report;
}

}  // namespace mpc_geometry
