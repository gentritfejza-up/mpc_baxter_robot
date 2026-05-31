#pragma once

#include "robot_model.hpp"

#include <Eigen/Dense>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace baxter {

class ForwardKinematics {
public:
  ForwardKinematics(const std::string &limb, ros::NodeHandle &nh);
  ForwardKinematics(ros::NodeHandle &nh, std::shared_ptr<Robot> baxter);

  std::vector<Eigen::Matrix4d> solveIntermediateFK(
      const std::unordered_map<std::string, double> &joint_angles);

private:
  std::shared_ptr<Robot> robot_{nullptr};

  Eigen::Matrix4d DH2T(const double &d, const double &theta, const double &a,
                       const double &alpha);

  bool checkJointLimits(
      const std::unordered_map<std::string, double> &joint_angles) const;
};

} // namespace baxter