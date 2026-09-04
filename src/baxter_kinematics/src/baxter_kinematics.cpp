#include <baxter_kinematics/baxter_kinematics.hpp>

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <limits>
#include <stdexcept>

BaxterKinematicsInterface::BaxterKinematicsInterface(const std::string& base_link,
                                                     const std::string& end_link) {
  if (!buildChain(base_link, end_link)) {
    throw std::runtime_error("BaxterKinematicsInterface: Failed to build KDL chain.");
  }

  // Create basic solvers
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);

  // Build the joint-limited IK solver
  KDL::JntArray q_min(num_joints_), q_max(num_joints_);
  for (size_t i = 0; i < num_joints_; i++) {
    q_min(i) = joint_limits_lower_[i];
    q_max(i) = joint_limits_upper_[i];
  }

  // 200 iterations, tolerance = 1e-5
  ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      chain_, q_min, q_max, *fk_solver_, *ik_vel_solver_, 200, 1e-5);

  ROS_INFO_STREAM("BaxterKinematicsInterface created. Joints in chain: " << num_joints_);
}

bool BaxterKinematicsInterface::buildChain(const std::string& base_link,
                                           const std::string& end_link) {
  // 1) Load URDF from "robot_description"
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description")) {
    ROS_ERROR("Failed to load URDF from param: robot_description");
    return false;
  }

  // 2) Create KDL Tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    ROS_ERROR("Failed to parse URDF into KDL Tree.");
    return false;
  }

  // 3) Extract chain
  if (!tree.getChain(base_link, end_link, chain_)) {
    ROS_ERROR_STREAM("Failed to get KDL chain from " << base_link << " to " << end_link);
    return false;
  }

  // 4) Count joints and gather names
  num_joints_ = 0;
  joint_names_.clear();

  for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
    const KDL::Segment& seg = chain_.getSegment(i);
    const KDL::Joint& jnt = seg.getJoint();
    if (jnt.getType() != KDL::Joint::None) {
      num_joints_++;
      joint_names_.push_back(jnt.getName());
    }
  }

  // 5) Prepare limit arrays
  joint_limits_lower_.resize(num_joints_, -std::numeric_limits<double>::infinity());
  joint_limits_upper_.resize(num_joints_, std::numeric_limits<double>::infinity());

  // 6) Parse actual joint limits from URDF
  parseJointLimits(urdf_model);

  return true;
}

void BaxterKinematicsInterface::parseJointLimits(const urdf::Model& urdf) {
  // For each chain joint, read the URDF joint's limits if available
  for (size_t i = 0; i < joint_names_.size(); i++) {
    const std::string& jname = joint_names_[i];
    urdf::JointConstSharedPtr jnt = urdf.getJoint(jname);
    if (!jnt) continue;  // skip if not found
    if (jnt->limits) {
      joint_limits_lower_[i] = jnt->limits->lower;
      joint_limits_upper_[i] = jnt->limits->upper;
    }
  }
}

KDL::Frame BaxterKinematicsInterface::forward(const std::vector<double>& q) const {
  if (q.size() != num_joints_) {
    ROS_ERROR_STREAM("forward: q.size()=" << q.size() << ", expected " << num_joints_);
    return KDL::Frame::Identity();
  }
  KDL::JntArray jnt_arr = toKDL(q);

  KDL::Frame frame_out;
  int ret = fk_solver_->JntToCart(jnt_arr, frame_out);
  if (ret < 0) {
    ROS_ERROR_STREAM("forward: solver failed with code=" << ret);
    return KDL::Frame::Identity();
  }
  return frame_out;
}

std::vector<double> BaxterKinematicsInterface::inverse(const KDL::Frame& desired_pose,
                                                       const std::vector<double>& seed) const {
  if (seed.size() != num_joints_) {
    ROS_ERROR_STREAM("inverse: seed.size()=" << seed.size() << ", expected " << num_joints_);
    return {};
  }

  KDL::JntArray q_in = toKDL(seed);
  KDL::JntArray q_out(num_joints_);

  int ret = ik_pos_solver_->CartToJnt(q_in, desired_pose, q_out);
  if (ret < 0) {
    ROS_ERROR_STREAM("inverse: solver failed with code=" << ret);
    return {};
  }
  return toStd(q_out);
}

KDL::Jacobian BaxterKinematicsInterface::jacobian(const std::vector<double>& q) const {
  if (q.size() != num_joints_) {
    ROS_ERROR_STREAM("jacobian: q.size()=" << q.size() << ", expected " << num_joints_);
    return KDL::Jacobian(0);
  }

  KDL::JntArray jnt_arr = toKDL(q);
  KDL::Jacobian jac(num_joints_);
  int ret = jac_solver_->JntToJac(jnt_arr, jac);
  if (ret < 0) {
    ROS_ERROR_STREAM("jacobian: solver failed with code=" << ret);
    return KDL::Jacobian(0);
  }
  return jac;
}

KDL::JntArray BaxterKinematicsInterface::toKDL(const std::vector<double>& v) const {
  KDL::JntArray arr(num_joints_);
  for (size_t i = 0; i < num_joints_; i++) arr(i) = v[i];
  return arr;
}

std::vector<double> BaxterKinematicsInterface::toStd(const KDL::JntArray& arr) const {
  std::vector<double> v(arr.rows(), 0.0);
  for (unsigned int i = 0; i < arr.rows(); i++) v[i] = arr(i);
  return v;
}
