#include "baxter_dynamics/robot_model.hpp"

#include <stdexcept>

namespace baxter {

Robot::Robot(const std::string &limb, ros::NodeHandle &nh) : limb_(limb) {
  loadJointNames(nh);
  loadAngleLimits(nh);
  loadDhParameters(nh);
}

std::string Robot::jointSuffix(const std::string &joint_name) {
  // Baxter joint names end in a two-letter joint id: left_s0 -> s0.
  if (joint_name.size() < 2) {
    throw std::runtime_error("Joint name too short to carry a joint id: " +
                             joint_name);
  }
  return joint_name.substr(joint_name.size() - 2);
}

void Robot::loadJointNames(ros::NodeHandle &nh) {
  if (!nh.getParam("robot/limb/" + limb_, joint_names_) ||
      joint_names_.empty()) {
    ROS_ERROR_STREAM("Failed to load joint names for limb: " << limb_);
    throw std::runtime_error("Missing joint names for limb: " + limb_);
  }
}

void Robot::loadAngleLimits(ros::NodeHandle &nh) {
  for (const auto &joint : joint_names_) {
    std::vector<double> limits;
    if (!nh.getParam("robot/limb/angle_limits/" + jointSuffix(joint),
                     limits) ||
        limits.size() != 2) {
      ROS_ERROR_STREAM("Failed to load angle limits for joint: " << joint);
      throw std::runtime_error("Invalid angle limits for joint: " + joint);
    }
    angle_limits_[joint] = {limits[0], limits[1]};
  }
}

void Robot::loadDhParameters(ros::NodeHandle &nh) {
  for (const auto &joint : joint_names_) {
    std::vector<double> dh_row;
    if (!nh.getParam("robot/limb/dh_parameters/" + jointSuffix(joint),
                     dh_row) ||
        dh_row.size() != 4) {
      ROS_ERROR_STREAM("Failed to load DH parameters for joint: " << joint);
      throw std::runtime_error("Invalid DH parameters for joint: " + joint);
    }
    // YAML row order: [d, theta_offset, a, alpha].
    dh_parameters_[joint] = DhParameters{dh_row[0], dh_row[1], dh_row[2],
                                         dh_row[3]};
  }
}

const std::unordered_map<std::string, std::pair<double, double>> &
Robot::getAngleLimits() const {
  return angle_limits_;
}

const std::vector<std::string> &Robot::getJointNames() const {
  return joint_names_;
}

const std::unordered_map<std::string, DhParameters> &
Robot::getDhParams() const {
  return dh_parameters_;
}

} // namespace baxter
