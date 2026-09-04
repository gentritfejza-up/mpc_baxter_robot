#include "baxter_kinematics/baxter_ik_client.hpp"

BaxterIKClient::BaxterIKClient(const std::string& arm_side) {
  service_name_ = "/ExternalTools/" + arm_side + "/PositionKinematicsNode/IKService";
  ik_client_ = nh_.serviceClient<baxter_core_msgs::SolvePositionIK>(service_name_);

  if (!ik_client_.waitForExistence(ros::Duration(5.0))) {
    ROS_WARN_STREAM("IK service [" << service_name_ << "] is not available.");
  }
}

bool BaxterIKClient::computeIK(const geometry_msgs::PoseStamped& desired_pose,
                               sensor_msgs::JointState& joint_solution, uint8_t seed_mode,
                               const sensor_msgs::JointState& user_seed) {
  baxter_core_msgs::SolvePositionIK srv;

  srv.request.pose_stamp.push_back(desired_pose);
  srv.request.seed_mode = seed_mode;

  if (seed_mode == baxter_core_msgs::SolvePositionIKRequest::SEED_USER) {
    if (user_seed.name.empty()) {
      ROS_WARN("SEED_USER selected but user_seed is empty.");
    } else {
      srv.request.seed_angles.push_back(user_seed);
    }
  }

  if (!ik_client_.call(srv)) {
    ROS_ERROR_STREAM("Failed to call IK service: " << service_name_);
    return false;
  }

  if (srv.response.isValid.empty() || !srv.response.isValid[0]) {
    ROS_WARN("IK service returned no valid solution.");
    return false;
  }

  joint_solution = srv.response.joints[0];
  return true;
}
