#include <ros/ros.h>

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <vector>

#include "mpc_controller/mpc_controller_node.hpp"

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		ROS_ERROR("Usage: run_mpc <limb_side>");
		ROS_ERROR("Please specify the limb ('left' or 'right') for MPC controller.");
		return EXIT_FAILURE;
	}

	std::string limb_side = argv[1];
	if(limb_side != "right" && limb_side != "left")
	{
		ROS_ERROR("Invalid limb specified: '%s'. Please choose either 'left' or 'right'.", limb_side.c_str());
		return EXIT_FAILURE;
	}
	std::string node_name = "mpc_controller_node_" + limb_side;

	ros::init(argc, argv, node_name.c_str());
	ros::NodeHandle nh;

	try {
		std::unique_ptr<MPCControlManager> mpcManager =
			std::make_unique<MPCControlManager>(nh, limb_side);
		mpcManager->run();
	} catch (const std::exception& ex) {
		ROS_FATAL("MPC controller initialization failed: %s", ex.what());
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
