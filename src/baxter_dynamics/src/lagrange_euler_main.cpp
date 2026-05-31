#include "baxter_dynamics/lagrange_euler_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "lagrange_euler_node");

  const std::vector<std::string> args = ros::remove_ros_args(argc, argv);
  if (args.size() < 2) {
    ROS_ERROR("Please specify the limb ('left' or 'right') as the first argument.");
    return EXIT_FAILURE;
  }

  const std::string limb_side = args[1];
  if (limb_side != "left" && limb_side != "right") {
    ROS_ERROR("Invalid limb specified. Choose either 'left' or 'right'.");
    return EXIT_FAILURE;
  }

  Eigen::initParallel();
  ros::NodeHandle nh;

  auto node = std::make_unique<LagrangeEulerNode>(nh, limb_side);
  node->runNode();
  return 0;
}