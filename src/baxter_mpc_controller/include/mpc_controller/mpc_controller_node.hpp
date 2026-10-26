#pragma once

#include <ros/ros.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mpc_controller.hpp"

class MPCControlManager {
 public:
  MPCControlManager(ros::NodeHandle& nh, const std::string& limb_side);
  void run();

 private:
  void loadAndValidateConfiguration();
  std::vector<double> makePredictionGrid() const;
  bool processControl(const std::vector<double>& x_ref,
                      const std::chrono::steady_clock::time_point& cycle_start);
  // Commands a braking acceleration (decelerate every joint to rest within the
  // acceleration limits) when no valid MPC solution is available, so the torque
  // loop keeps a fresh reference instead of timing out into position mode.
  void publishBrakingFallback(const std::vector<double>& positions,
                              const std::vector<double>& velocities,
                              const std::vector<double>& x_ref, const ros::Time& state_stamp);
  // `state_stamp` is when `positions`/`velocities` were measured; the base state
  // carried with the command is propagated to publish time with the acceleration
  // that was active meanwhile, so the torque loop's reference does not lag the
  // plant by the solve latency (which otherwise makes its PD fight the feedforward).
  void publishCommand(const std::vector<double>& accelerations,
                      const std::vector<double>& positions,
                      const std::vector<double>& velocities,
                      const std::vector<double>& x_ref, const ros::Time& state_stamp);

  ros::NodeHandle nh_;
  std::string limb_side_;
  std::unique_ptr<Limb> limb_;
  std::unique_ptr<MPCController> mpc_;

  int num_joints_{7};
  int prediction_horizon_{20};
  int control_horizon_{15};
  int spinner_threads_{10};
  double control_rate_hz_{50.0};
  double prediction_duration_sec_{2.0};
  double control_period_sec_{0.02};
  double state_timeout_sec_{0.04};
  double cycle_deadline_sec_{0.02};
  bool enforce_cycle_deadline_{true};
  // true: a solution that misses the cycle deadline is still commanded (late is
  // safer than letting the torque loop time out into position mode mid-motion);
  // false: the late solution is only kept as the next warm start.
  bool publish_on_deadline_miss_{true};
  double max_latency_compensation_sec_{0.15};

  std::vector<double> previous_applied_accelerations_;
  std::vector<double> acceleration_min_;
  std::vector<double> acceleration_max_;
  bool braking_fallback_enabled_{true};

  bool has_last_command_time_{false};
  ros::Time last_command_time_;
  double last_state_preparation_ms_{0.0};
  double last_postprocessing_ms_{0.0};
  uint64_t successful_cycles_{0};
  uint64_t failed_cycles_{0};
  uint64_t deadline_misses_{0};
  uint64_t fallback_cycles_{0};
};
