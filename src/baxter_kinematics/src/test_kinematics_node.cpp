#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include "baxter_kinematics/baxter_ik_client.hpp"

static geometry_msgs::PoseStamped makeTestPose(const std::string& arm) {
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = "base";

  if (arm == "left") {
    ps.pose.position.x = 0.57389;
    ps.pose.position.y = 0.4890132;
    ps.pose.position.z = -0.233944;
    ps.pose.orientation.x = -0.69505;
    ps.pose.orientation.y = 0.719020;
    ps.pose.orientation.z = -0.014003;
    ps.pose.orientation.w = -0.00351899;
  } else {  // right
    ps.pose.position.x = 0.634511;
    ps.pose.position.y = -0.2194136;
    ps.pose.position.z = -0.285653337;
    ps.pose.orientation.x = 0.73322649;
    ps.pose.orientation.y = -0.679479;
    ps.pose.orientation.z = 0.0127264562;
    ps.pose.orientation.w = 0.02894003;
  }
  return ps;
}

static sensor_msgs::JointState makeUserSeed(const std::string& arm) {
  sensor_msgs::JointState seed;
  seed.name = {
      arm + "_s0", arm + "_s1", arm + "_e0", arm + "_e1", arm + "_w0", arm + "_w1", arm + "_w2"};

  // Example positions – replace with your preferred seed.
  if (arm == "left") {
    seed.position = {-0.4598107, -0.39615, 0.0958737, 1.39093, -0.20555342, 0.583296, -1.00130987};
  } else {  // right
    seed.position = {0.55632, -0.07823, -0.12490, 1.26733, -0.01420, 0.56318, 0.98528};
  }
  return seed;
}

static void solveArm(const std::string& arm) {
  BaxterIKClient ik(arm);
  geometry_msgs::PoseStamped target = makeTestPose(arm);
  sensor_msgs::JointState seed = makeUserSeed(arm);
  sensor_msgs::JointState sol;

  if (ik.computeIK(target, sol, seed)) {
    ROS_INFO_STREAM("\n[" << arm << " arm] IK solution:");
    for (size_t i = 0; i < sol.name.size(); ++i)
      ROS_INFO_STREAM("  " << sol.name[i] << " = " << sol.position[i]);
  } else {
    ROS_WARN_STREAM("[" << arm << " arm] IK failed.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "baxter_ik_demo");

  std::string mode = (argc > 1) ? argv[1] : "left";
  if (mode == "both") {
    solveArm("left");
    solveArm("right");
  } else if (mode == "left" || mode == "right") {
    solveArm(mode);
  } else {
    ROS_ERROR("Usage: rosrun <pkg> ik_demo [left|right|both]");
    return 1;
  }
  return 0;
}
