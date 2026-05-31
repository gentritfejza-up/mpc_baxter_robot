#include "baxter_dynamics/robot_model.hpp"

#include <cmath>
#include <stdexcept>

namespace baxter {

Robot::Robot(const std::string &limb, ros::NodeHandle &nh) : limb_(limb) {
  setJointNames(nh);
  setAngleLimits(nh);
  computeDHref(nh);
}

void Robot::setJointNames(ros::NodeHandle &nh) {
  if (!nh.getParam("robot/limb/segment_names", segment_names_) ||
      !nh.getParam(std::string("robot/limb/" + limb_), joint_names_)) {
    ROS_ERROR("Failed to load limb segment names or joint names.");
    throw std::runtime_error("Missing limb segment names or joint names.");
  }

  for (const auto &key : segment_names_) {
    double length = 0.0;
    if (!nh.getParam("robot/limb/segment_lengths/" + key, length)) {
      ROS_ERROR_STREAM("Failed to load segment length for key: " << key);
      throw std::runtime_error("Missing segment length: " + key);
    }
    lengths_[key] = length;
  }
}

void Robot::setAngleLimits(ros::NodeHandle &nh) {
  for (const auto &joint : joint_names_) {
    std::vector<double> limits;
    if (!nh.getParam(
            "robot/limb/angle_limits/" + joint.substr(joint.size() - 2),
            limits) ||
        limits.size() != 2) {
      ROS_ERROR_STREAM("Failed to load angle limits for joint: " << joint);
      throw std::runtime_error("Invalid angle limits for joint: " + joint);
    }
    angle_limits_[joint.substr(joint.size() - 2)] = {limits[0], limits[1]};
  }
}

void Robot::computeDHref(ros::NodeHandle &nh) {
  for (const auto &joint : joint_names_) {
    std::vector<double> dh_params;
    if (nh.getParam(
            "robot/limb/dh_parameters/" + joint.substr(joint.size() - 2),
            dh_params) &&
        dh_params.size() == 4) {
      dh_parameters_[joint] = Eigen::Vector4d(dh_params[0], dh_params[1],
                                              dh_params[2], dh_params[3]);
    } else {
      ROS_ERROR_STREAM("Failed to load DH parameters for joint: " << joint);
      throw std::runtime_error("Invalid DH parameters for joint: " + joint);
    }
  }
}

const std::unordered_map<std::string, std::pair<double, double>> &
Robot::getAngleLimits() const {
  return angle_limits_;
}

const std::vector<std::string> &Robot::getJointNames() const {
  return joint_names_;
}

const std::unordered_map<std::string, Eigen::Vector4d> &
Robot::getDhParams() const {
  return dh_parameters_;
}

} // namespace baxter