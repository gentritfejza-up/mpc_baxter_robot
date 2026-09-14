#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "mpc_controller/mpc_controller.hpp"

namespace {

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const size_t index = static_cast<size_t>(fraction * static_cast<double>(values.size() - 1));
  return values[index];
}

std::vector<double> predictionGrid(double rate_hz, double duration, int horizon) {
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0 || horizon < 2 ||
      !std::isfinite(duration) || duration <= 1.0 / rate_hz) {
    throw std::invalid_argument("Invalid benchmark timing configuration");
  }
  std::vector<double> grid(horizon, 1.0 / rate_hz);
  std::fill(grid.begin() + 1, grid.end(), (duration - grid.front()) / (horizon - 1));
  return grid;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpc_solver_benchmark");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int iterations = 10000;
  int warmup_iterations = 100;
  int prediction_horizon = 20;
  int control_horizon = 15;
  double rate_hz = 50.0;
  double prediction_duration = 2.0;
  private_nh.param("iterations", iterations, iterations);
  private_nh.param("warmup_iterations", warmup_iterations, warmup_iterations);
  nh.param("robot/control/prediction_horizon", prediction_horizon, prediction_horizon);
  nh.param("robot/control/control_horizon", control_horizon, control_horizon);
  nh.param("robot/control/mpc_rate_hz", rate_hz, rate_hz);
  nh.param("robot/control/prediction_duration_sec", prediction_duration, prediction_duration);
  if (iterations <= 0 || warmup_iterations < 0) {
    ROS_FATAL("Benchmark iteration counts are invalid.");
    return EXIT_FAILURE;
  }

  try {
    const double control_period = 1.0 / rate_hz;
    MPCController controller(nh, 7, prediction_horizon, control_horizon,
                             predictionGrid(rate_hz, prediction_duration, prediction_horizon),
                             control_period);
    casadi::DM positions = casadi::DM(std::vector<double>{0.192483, 1.047000, 0.000806,
                                                          0.491094, -0.178079, -0.061033,
                                                          -0.012471});
    casadi::DM velocities = casadi::DM::zeros(7, 1);
    casadi::DM reference = casadi::DM(std::vector<double>{0.20, 0.90, 0.00, 0.60,
                                                          -0.20, 0.00, 0.00});
    casadi::DM previous_acceleration = casadi::DM::zeros(7, 1);

    for (int iteration = 0; iteration < warmup_iterations; ++iteration) {
      const casadi::DM solution = controller.solve(positions, velocities, reference,
                                                   previous_acceleration, control_period);
      previous_acceleration = controller.getFirstAcceleration(solution);
      controller.commitSolution();
    }

    std::vector<double> solver_times;
    std::vector<double> total_times;
    solver_times.reserve(iterations);
    total_times.reserve(iterations);
    int failures = 0;
    for (int iteration = 0; iteration < iterations; ++iteration) {
      const auto start = std::chrono::steady_clock::now();
      try {
        const casadi::DM solution = controller.solve(positions, velocities, reference,
                                                     previous_acceleration, control_period);
        previous_acceleration = controller.getFirstAcceleration(solution);
        controller.commitSolution();
        solver_times.push_back(controller.getLastSolveTiming().solver_ms);
      } catch (const std::exception& exception) {
        ++failures;
        controller.resetWarmStart();
        ROS_ERROR_THROTTLE(1.0, "Benchmark solve failed: %s", exception.what());
      }
      const auto end = std::chrono::steady_clock::now();
      total_times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }

    std::cout << std::fixed << std::setprecision(3)
              << "backend=" << controller.solverBackendName() << " iterations=" << iterations
              << " failures=" << failures << '\n'
              << "solver_ms median=" << percentile(solver_times, 0.50)
              << " p95=" << percentile(solver_times, 0.95)
              << " p99=" << percentile(solver_times, 0.99)
              << " max=" << percentile(solver_times, 1.0) << '\n'
              << "total_ms  median=" << percentile(total_times, 0.50)
              << " p95=" << percentile(total_times, 0.95)
              << " p99=" << percentile(total_times, 0.99)
              << " max=" << percentile(total_times, 1.0) << '\n';
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception& exception) {
    ROS_FATAL("Benchmark initialization failed: %s", exception.what());
    return EXIT_FAILURE;
  }
}
