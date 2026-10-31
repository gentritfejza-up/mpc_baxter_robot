/**
 * @file mpc_solver_benchmark.cpp
 * @brief Offline timing benchmark of the MPC QP (and optionally the collision-guard
 *        linearisation and guard) on a fixed or simulated scenario, without a robot.
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "mpc_controller/arm_collision_model.hpp"
#include "mpc_controller/geometry_rows.hpp"
#include "mpc_controller/mpc_controller.hpp"

namespace {

constexpr int kJoints = mpc_geometry::kArmJoints;

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const size_t index = static_cast<size_t>(fraction * static_cast<double>(values.size() - 1));
  return values[index];
}

double elapsedMs(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

/// Same grid as the node: one control period, then equal shares of the remaining duration.
std::vector<double> predictionGrid(double rate_hz, double duration_sec, int horizon) {
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0 || horizon < 2 || !std::isfinite(duration_sec) ||
      duration_sec <= 1.0 / rate_hz) {
    throw std::invalid_argument("Invalid benchmark timing configuration");
  }
  std::vector<double> grid(horizon, 1.0 / rate_hz);
  std::fill(grid.begin() + 1, grid.end(), (duration_sec - grid.front()) / (horizon - 1));
  return grid;
}

mpc_geometry::JointVector toJointVector(const casadi::DM& value) {
  const std::vector<double> values = casadi::DM::densify(value).nonzeros();
  mpc_geometry::JointVector joint_vector{};
  for (int joint = 0; joint < kJoints; ++joint) joint_vector[joint] = values[joint];
  return joint_vector;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpc_solver_benchmark");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  int iterations = 10000;
  int warmup_iterations = 100;
  int prediction_horizon = 20;
  int control_horizon = 15;
  double rate_hz = 50.0;
  double prediction_duration_sec = 2.0;
  private_node_handle.param("iterations", iterations, iterations);
  private_node_handle.param("warmup_iterations", warmup_iterations, warmup_iterations);
  node_handle.param("robot/control/prediction_horizon", prediction_horizon, prediction_horizon);
  node_handle.param("robot/control/control_horizon", control_horizon, control_horizon);
  node_handle.param("robot/control/mpc_rate_hz", rate_hz, rate_hz);
  node_handle.param("robot/control/prediction_duration_sec", prediction_duration_sec,
                    prediction_duration_sec);
  if (iterations <= 0 || warmup_iterations < 0) {
    ROS_FATAL("Benchmark iteration counts are invalid.");
    return EXIT_FAILURE;
  }

  try {
    const double control_period_sec = 1.0 / rate_hz;
    // geometry: linearise the collision-guard rows around the shifted previous prediction
    // each iteration (needs /robot_description and robot/collision_geometry on the server).
    bool geometry_enabled = false;
    std::string limb_side = "left";
    std::vector<double> other_arm_values;
    private_node_handle.param("geometry", geometry_enabled, false);
    private_node_handle.param<std::string>("limb", limb_side, limb_side);
    private_node_handle.param("other_arm_positions", other_arm_values, other_arm_values);
    std::unique_ptr<mpc_geometry::ArmCollisionModel> collision_model;
    std::unique_ptr<mpc_geometry::GeometryRowLinearizer> linearizer;
    mpc_geometry::GeometryRowBatch geometry_batch;
    mpc_geometry::JointVector other_arm_positions{};
    const mpc_geometry::JointVector* other_arm = nullptr;  // nullptr: unknown other arm
    int sphere_count = 0;
    if (geometry_enabled) {
      collision_model = mpc_geometry::ArmCollisionModel::fromParams(node_handle);
      sphere_count = collision_model->sphereCount();
      if (other_arm_values.size() == static_cast<std::size_t>(kJoints)) {
        for (int joint = 0; joint < kJoints; ++joint) other_arm_positions[joint] = other_arm_values[joint];
        other_arm = &other_arm_positions;
      }
    }
    MPCController controller(node_handle, kJoints, prediction_horizon, control_horizon,
                             predictionGrid(rate_hz, prediction_duration_sec, prediction_horizon),
                             control_period_sec, sphere_count);
    // Scenario can be overridden (e.g. replay a problematic instance from a Gazebo log):
    //   _positions:="[...]" _velocities:="[...]" _reference:="[...]" _previous_acceleration:="[...]"
    std::vector<double> position_values{0.192483, 1.047000, 0.000806, 0.491094, -0.178079,
                                        -0.061033, -0.012471};
    std::vector<double> velocity_values(kJoints, 0.0);
    std::vector<double> reference_values{0.20, 0.90, 0.00, 0.60, -0.20, 0.00, 0.00};
    std::vector<double> previous_acceleration_values(kJoints, 0.0);
    private_node_handle.param("positions", position_values, position_values);
    private_node_handle.param("velocities", velocity_values, velocity_values);
    private_node_handle.param("reference", reference_values, reference_values);
    private_node_handle.param("previous_acceleration", previous_acceleration_values,
                              previous_acceleration_values);
    for (const std::vector<double>* values :
         {&position_values, &velocity_values, &reference_values, &previous_acceleration_values}) {
      if (values->size() != static_cast<std::size_t>(kJoints)) {
        throw std::invalid_argument("Scenario vectors must have 7 entries");
      }
    }
    casadi::DM positions(position_values);
    casadi::DM velocities(velocity_values);
    const casadi::DM reference(reference_values);
    casadi::DM previous_acceleration(previous_acceleration_values);
    bool hold_previous_acceleration = false;
    private_node_handle.param("hold_previous_acceleration", hold_previous_acceleration, false);
    // simulate: propagate the state with the first acceleration every cycle (as the plant
    // would), and every stop_every cycles zero the velocity (as the simulator does when
    // the torque loop drops to position mode). This exercises realistic warm starts
    // instead of re-solving a fixed problem.
    bool simulate = false;
    int stop_every = 0;
    private_node_handle.param("simulate", simulate, false);
    private_node_handle.param("stop_every", stop_every, 0);
    const auto propagate = [&](const casadi::DM& acceleration, int iteration) {
      if (!simulate) return;
      const double step = control_period_sec;
      positions = positions + step * velocities + 0.5 * step * step * acceleration;
      velocities = velocities + step * acceleration;
      if (stop_every > 0 && iteration % stop_every == 0) velocities = casadi::DM::zeros(kJoints, 1);
    };

    std::vector<mpc_geometry::JointVector> linearisation_points(prediction_horizon);
    bool has_prediction = false;
    std::vector<double> linearize_times_ms;
    std::vector<double> guard_times_ms;
    int guard_violations = 0;
    double min_guard_distance = std::numeric_limits<double>::infinity();
    mpc_geometry::GroupDistances min_distance_now = mpc_geometry::infiniteGroupDistances();
    mpc_geometry::GroupDistances min_distance_predicted = mpc_geometry::infiniteGroupDistances();
    if (geometry_enabled) {
      const mpc_geometry::GeometryParams params = mpc_geometry::GeometryParams::fromParams(node_handle);
      linearizer = std::make_unique<mpc_geometry::GeometryRowLinearizer>(
          *collision_model, limb_side, params, controller.geometryLayout());
      geometry_batch.reset(controller.geometryLayout());
    }
    // One cycle of the node's geometry path: linearise (shifted prediction or measured
    // state), solve, evaluate the guard, keep the prediction for the next cycle.
    const auto solve_cycle = [&](bool record_timing) {
      MpcSolveInput input;
      input.positions = positions;
      input.velocities = velocities;
      input.reference_positions = reference;
      input.previous_accelerations = previous_acceleration;
      input.first_jerk_dt = control_period_sec;
      if (geometry_enabled) {
        const mpc_geometry::JointVector measured = toJointVector(positions);
        if (has_prediction) {
          for (int step = 0; step + 1 < prediction_horizon; ++step) {
            linearisation_points[step] = linearisation_points[step + 1];
          }
        } else {
          std::fill(linearisation_points.begin(), linearisation_points.end(), measured);
        }
        const auto linearize_start = std::chrono::steady_clock::now();
        linearizer->linearize(linearisation_points, other_arm, geometry_batch);
        if (record_timing) linearize_times_ms.push_back(elapsedMs(linearize_start));
        input.geometry = &geometry_batch;
      }
      const casadi::DM solution = controller.solve(input);
      if (geometry_enabled) {
        const auto guard_start = std::chrono::steady_clock::now();
        const mpc_geometry::JointVector measured = toJointVector(positions);
        linearisation_points = controller.predictedPositions(solution);
        has_prediction = true;
        const mpc_geometry::GuardReport report =
            linearizer->evaluateTrajectory(measured, linearisation_points, other_arm);
        if (record_timing) guard_times_ms.push_back(elapsedMs(guard_start));
        if (report.violated) ++guard_violations;
        for (int group = 0; group < mpc_geometry::kGroupCount; ++group) {
          min_guard_distance = std::min(min_guard_distance, report.min_distance[group]);
          min_distance_now[group] = std::min(min_distance_now[group], report.min_distance_now[group]);
          min_distance_predicted[group] =
              std::min(min_distance_predicted[group], report.min_distance[group]);
        }
      }
      return solution;
    };

    for (int iteration = 0; iteration < warmup_iterations; ++iteration) {
      try {
        const casadi::DM solution = solve_cycle(false);
        const casadi::DM first_acceleration = controller.getFirstAcceleration(solution);
        if (!hold_previous_acceleration) previous_acceleration = first_acceleration;
        controller.commitSolution();
        propagate(first_acceleration, iteration + 1);
      } catch (const std::exception& exception) {
        ROS_WARN("Warm-up solve failed: %s", exception.what());
      }
    }

    std::vector<double> solver_times_ms;
    std::vector<double> total_times_ms;
    solver_times_ms.reserve(iterations);
    total_times_ms.reserve(iterations);
    int failures = 0;
    for (int iteration = 0; iteration < iterations; ++iteration) {
      const auto cycle_start = std::chrono::steady_clock::now();
      try {
        const casadi::DM solution = solve_cycle(true);
        const casadi::DM first_acceleration = controller.getFirstAcceleration(solution);
        if (!hold_previous_acceleration) previous_acceleration = first_acceleration;
        controller.commitSolution();
        solver_times_ms.push_back(controller.getLastSolveTiming().solver_ms);
        propagate(first_acceleration, iteration + 1);
      } catch (const std::exception& exception) {
        ++failures;
        solver_times_ms.push_back(controller.getLastSolveTiming().solver_ms);
        ROS_ERROR_THROTTLE(1.0, "Benchmark solve failed: %s", exception.what());
      }
      total_times_ms.push_back(elapsedMs(cycle_start));
    }

    std::cout << std::fixed << std::setprecision(3)
              << "backend=" << controller.solverBackendName() << " iterations=" << iterations
              << " failures=" << failures << '\n'
              << "solver_ms median=" << percentile(solver_times_ms, 0.50)
              << " p95=" << percentile(solver_times_ms, 0.95)
              << " p99=" << percentile(solver_times_ms, 0.99)
              << " max=" << percentile(solver_times_ms, 1.0) << '\n'
              << "total_ms  median=" << percentile(total_times_ms, 0.50)
              << " p95=" << percentile(total_times_ms, 0.95)
              << " p99=" << percentile(total_times_ms, 0.99)
              << " max=" << percentile(total_times_ms, 1.0) << '\n';
    if (geometry_enabled) {
      std::cout << "geometry  rows=" << controller.geometryLayout().rowCount()
                << " active=" << geometry_batch.active_rows
                << " linearize_ms median=" << percentile(linearize_times_ms, 0.50)
                << " p95=" << percentile(linearize_times_ms, 0.95)
                << " guard_ms median=" << percentile(guard_times_ms, 0.50)
                << " p95=" << percentile(guard_times_ms, 0.95)
                << " guard_violations=" << guard_violations
                << " min_distance=" << min_guard_distance << '\n'
                << "distances (floor/body/arm) now=" << min_distance_now[mpc_geometry::kFloorGroup]
                << '/' << min_distance_now[mpc_geometry::kBodyGroup] << '/'
                << min_distance_now[mpc_geometry::kArmGroup]
                << " predicted=" << min_distance_predicted[mpc_geometry::kFloorGroup] << '/'
                << min_distance_predicted[mpc_geometry::kBodyGroup] << '/'
                << min_distance_predicted[mpc_geometry::kArmGroup] << '\n';
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception& exception) {
    ROS_FATAL("Benchmark initialization failed: %s", exception.what());
    return EXIT_FAILURE;
  }
}
