/**
 * @file main.cpp
 * @brief Entry point of the run_mpc node: `run_mpc <left|right>` runs the MPC of one arm.
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include <ros/ros.h>

#include <cstdlib>
#include <memory>
#include <string>

#include "mpc_controller/mpc_controller_node.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    ROS_ERROR("Usage: run_mpc <limb_side>");
    ROS_ERROR("Please specify the limb ('left' or 'right') for MPC controller.");
    return EXIT_FAILURE;
  }

  const std::string limb_side = argv[1];
  if (limb_side != "right" && limb_side != "left") {
    ROS_ERROR("Invalid limb specified: '%s'. Please choose either 'left' or 'right'.",
              limb_side.c_str());
    return EXIT_FAILURE;
  }
  const std::string node_name = "mpc_controller_node_" + limb_side;

  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;

  try {
    MPCControlManager control_manager(node_handle, limb_side);
    control_manager.run();
  } catch (const std::exception& exception) {
    ROS_FATAL("MPC controller initialization failed: %s", exception.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
