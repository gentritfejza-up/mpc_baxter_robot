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

  std::vector<double> previous_applied_accelerations_;

  bool has_last_command_time_{false};
  std::chrono::steady_clock::time_point last_command_time_;
  double last_state_preparation_ms_{0.0};
  double last_postprocessing_ms_{0.0};
  uint64_t successful_cycles_{0};
  uint64_t failed_cycles_{0};
  uint64_t deadline_misses_{0};
};
