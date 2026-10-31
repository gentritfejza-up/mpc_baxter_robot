// Manual check of BaxterIKClient: solves a fixed reachable pose for one or both
// arms with a user seed and prints the solution.
//   rosrun baxter_kinematics test_baxter_kinematics_node [left|right|both]
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>

#include "baxter_kinematics/baxter_ik_client.hpp"

namespace {

geometry_msgs::PoseStamped makeTestPose(const std::string& arm) {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "base";
  if (arm == "left") {
    pose.pose.position.x = 0.57389;
    pose.pose.position.y = 0.4890132;
    pose.pose.position.z = -0.233944;
    pose.pose.orientation.x = -0.69505;
    pose.pose.orientation.y = 0.719020;
    pose.pose.orientation.z = -0.014003;
    pose.pose.orientation.w = -0.00351899;
  } else {
    pose.pose.position.x = 0.634511;
    pose.pose.position.y = -0.2194136;
    pose.pose.position.z = -0.285653337;
    pose.pose.orientation.x = 0.73322649;
    pose.pose.orientation.y = -0.679479;
    pose.pose.orientation.z = 0.0127264562;
    pose.pose.orientation.w = 0.02894003;
  }
  return pose;
}

sensor_msgs::JointState makeUserSeed(const std::string& arm) {
  sensor_msgs::JointState seed;
  seed.name = {arm + "_s0", arm + "_s1", arm + "_e0", arm + "_e1",
               arm + "_w0", arm + "_w1", arm + "_w2"};
  if (arm == "left") {
    seed.position = {-0.4598107, -0.39615, 0.0958737, 1.39093, -0.20555342, 0.583296, -1.00130987};
  } else {
    seed.position = {0.55632, -0.07823, -0.12490, 1.26733, -0.01420, 0.56318, 0.98528};
  }
  return seed;
}

void solveArm(const std::string& arm) {
  BaxterIKClient ik_client(arm);
  const geometry_msgs::PoseStamped target_pose = makeTestPose(arm);
  const sensor_msgs::JointState seed = makeUserSeed(arm);
  sensor_msgs::JointState solution;
  if (ik_client.computeIK(target_pose, solution, seed)) {
    ROS_INFO_STREAM("\n[" << arm << " arm] IK solution:");
    for (size_t joint_index = 0; joint_index < solution.name.size(); ++joint_index) {
      ROS_INFO_STREAM("  " << solution.name[joint_index] << " = "
                           << solution.position[joint_index]);
    }
  } else {
    ROS_WARN_STREAM("[" << arm << " arm] IK failed.");
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "baxter_ik_demo");
  const std::string arm_selection = (argc > 1) ? argv[1] : "left";
  if (arm_selection == "both") {
    solveArm("left");
    solveArm("right");
  } else if (arm_selection == "left" || arm_selection == "right") {
    solveArm(arm_selection);
  } else {
    ROS_ERROR("Usage: rosrun baxter_kinematics test_baxter_kinematics_node [left|right|both]");
    return 1;
  }
  return 0;
}
