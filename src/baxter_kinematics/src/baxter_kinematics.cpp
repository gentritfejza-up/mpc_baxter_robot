#include <baxter_kinematics/baxter_kinematics.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <urdf/model.h>

#include <limits>
#include <stdexcept>

namespace {
constexpr unsigned int kIkMaxIterations = 200;
constexpr double kIkToleranceM = 1e-5;
}  // namespace

BaxterKinematicsInterface::BaxterKinematicsInterface(const std::string& base_link,
                                                     const std::string& end_link) {
  if (!buildChain(base_link, end_link)) {
    throw std::runtime_error("BaxterKinematicsInterface: Failed to build KDL chain.");
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ik_velocity_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);

  KDL::JntArray lower_limits(joint_count_), upper_limits(joint_count_);
  for (size_t joint_index = 0; joint_index < joint_count_; ++joint_index) {
    lower_limits(joint_index) = joint_limits_lower_[joint_index];
    upper_limits(joint_index) = joint_limits_upper_[joint_index];
  }
  ik_position_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      chain_, lower_limits, upper_limits, *fk_solver_, *ik_velocity_solver_, kIkMaxIterations,
      kIkToleranceM);

  ROS_INFO_STREAM("BaxterKinematicsInterface created. Joints in chain: " << joint_count_);
}

bool BaxterKinematicsInterface::buildChain(const std::string& base_link,
                                           const std::string& end_link) {
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description")) {
    ROS_ERROR("Failed to load URDF from param: robot_description");
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    ROS_ERROR("Failed to parse URDF into KDL Tree.");
    return false;
  }

  if (!tree.getChain(base_link, end_link, chain_)) {
    ROS_ERROR_STREAM("Failed to get KDL chain from " << base_link << " to " << end_link);
    return false;
  }

  joint_count_ = 0;
  joint_names_.clear();
  for (unsigned int segment_index = 0; segment_index < chain_.getNrOfSegments(); ++segment_index) {
    const KDL::Joint& joint = chain_.getSegment(segment_index).getJoint();
    if (joint.getType() != KDL::Joint::None) {
      ++joint_count_;
      joint_names_.push_back(joint.getName());
    }
  }

  joint_limits_lower_.assign(joint_count_, -std::numeric_limits<double>::infinity());
  joint_limits_upper_.assign(joint_count_, std::numeric_limits<double>::infinity());
  parseJointLimits(urdf_model);
  return true;
}

void BaxterKinematicsInterface::parseJointLimits(const urdf::Model& urdf_model) {
  for (size_t joint_index = 0; joint_index < joint_names_.size(); ++joint_index) {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model.getJoint(joint_names_[joint_index]);
    if (urdf_joint && urdf_joint->limits) {
      joint_limits_lower_[joint_index] = urdf_joint->limits->lower;
      joint_limits_upper_[joint_index] = urdf_joint->limits->upper;
    }
  }
}

KDL::Frame BaxterKinematicsInterface::forward(const std::vector<double>& q) const {
  if (q.size() != joint_count_) {
    ROS_ERROR_STREAM("forward: q.size()=" << q.size() << ", expected " << joint_count_);
    return KDL::Frame::Identity();
  }
  KDL::Frame tip_pose;
  const int solver_status = fk_solver_->JntToCart(toJntArray(q), tip_pose);
  if (solver_status < 0) {
    ROS_ERROR_STREAM("forward: solver failed with code=" << solver_status);
    return KDL::Frame::Identity();
  }
  return tip_pose;
}

std::vector<double> BaxterKinematicsInterface::inverse(const KDL::Frame& desired_pose,
                                                       const std::vector<double>& seed) const {
  if (seed.size() != joint_count_) {
    ROS_ERROR_STREAM("inverse: seed.size()=" << seed.size() << ", expected " << joint_count_);
    return {};
  }
  KDL::JntArray solution(joint_count_);
  const int solver_status = ik_position_solver_->CartToJnt(toJntArray(seed), desired_pose, solution);
  if (solver_status < 0) {
    ROS_ERROR_STREAM("inverse: solver failed with code=" << solver_status);
    return {};
  }
  return toStdVector(solution);
}

KDL::Jacobian BaxterKinematicsInterface::jacobian(const std::vector<double>& q) const {
  if (q.size() != joint_count_) {
    ROS_ERROR_STREAM("jacobian: q.size()=" << q.size() << ", expected " << joint_count_);
    return KDL::Jacobian(0);
  }
  KDL::Jacobian tip_jacobian(joint_count_);
  const int solver_status = jacobian_solver_->JntToJac(toJntArray(q), tip_jacobian);
  if (solver_status < 0) {
    ROS_ERROR_STREAM("jacobian: solver failed with code=" << solver_status);
    return KDL::Jacobian(0);
  }
  return tip_jacobian;
}

KDL::JntArray BaxterKinematicsInterface::toJntArray(const std::vector<double>& values) const {
  KDL::JntArray joint_array(joint_count_);
  for (size_t joint_index = 0; joint_index < joint_count_; ++joint_index) {
    joint_array(joint_index) = values[joint_index];
  }
  return joint_array;
}

std::vector<double> BaxterKinematicsInterface::toStdVector(const KDL::JntArray& joint_array) const {
  std::vector<double> values(joint_array.rows(), 0.0);
  for (unsigned int joint_index = 0; joint_index < joint_array.rows(); ++joint_index) {
    values[joint_index] = joint_array(joint_index);
  }
  return values;
}
