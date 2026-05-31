#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace baxter {

class Robot {
public:
  Robot(const std::string &limb, ros::NodeHandle &nh);

  const std::unordered_map<std::string, Eigen::Vector4d> &getDhParams() const;
  const std::unordered_map<std::string, std::pair<double, double>> &
  getAngleLimits() const;
  const std::vector<std::string> &getJointNames() const;

private:
  void computeDHref(ros::NodeHandle &nh);
  void setJointNames(ros::NodeHandle &nh);
  void setAngleLimits(ros::NodeHandle &nh);

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::pair<double, double>> angle_limits_;
  std::vector<std::string> segment_names_;
  std::unordered_map<std::string, Eigen::Vector4d> dh_parameters_;
  std::unordered_map<std::string, double> lengths_;
  std::string limb_;
};

} // namespace baxter