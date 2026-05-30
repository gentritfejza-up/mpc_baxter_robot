#include "baxter_dynamics/lagrange_euler_node.hpp"

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>

int main(int argc, char **argv) {
  if (argc < 2) {
    ROS_ERROR("Please specify the limb ('left' or 'right') as the first argument.");
    return EXIT_FAILURE;
  }

  const std::string limb_side = argv[1];
  if (limb_side != "left" && limb_side != "right") {
    ROS_ERROR("Invalid limb specified. Choose either 'left' or 'right'.");
    return EXIT_FAILURE;
  }

  Eigen::initParallel();

  const std::string node_name = "lagrange_euler_node_" + limb_side;
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle nh;

  auto node = std::make_unique<LagrangeEulerNode>(nh, limb_side);
  node->runNode();
  return 0;
}