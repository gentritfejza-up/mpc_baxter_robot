#!/usr/bin/env python3

import argparse
import time

import rospy
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def parse_args():
    parser = argparse.ArgumentParser(
        description="Enable Baxter only after its simulated effort controller is loaded."
    )
    parser.add_argument("--limb", choices=("left", "right"), default="left")
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args(rospy.myargv()[1:])


def wait_for_effort_controller(limb, timeout_sec):
    service_name = "/robot/controller_manager/list_controllers"
    controller_name = "{}_joint_effort_controller".format(limb)
    deadline = time.monotonic() + timeout_sec

    rospy.wait_for_service(service_name, timeout=timeout_sec)
    list_controllers = rospy.ServiceProxy(service_name, ListControllers)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        response = list_controllers()
        if any(controller.name == controller_name for controller in response.controller):
            rospy.wait_for_message("/robot/joint_states", JointState, timeout=5.0)
            return
        rospy.rostime.wallsleep(0.1)

    raise RuntimeError("{} was not loaded within {:.0f} s".format(controller_name, timeout_sec))


def main():
    rospy.init_node("enable_simulated_baxter")
    args = parse_args()
    if args.timeout <= 0.0:
        raise RuntimeError("--timeout must be positive")

    wait_for_effort_controller(args.limb, args.timeout)
    publisher = rospy.Publisher("/robot/set_super_enable", Bool, queue_size=1, latch=True)
    deadline = time.monotonic() + args.timeout
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        rospy.rostime.wallsleep(0.1)
    if publisher.get_num_connections() == 0:
        raise RuntimeError("Baxter simulator enable subscriber is not available")

    publisher.publish(Bool(data=True))
    rospy.loginfo("Enabled Baxter after %s was loaded", args.limb + "_joint_effort_controller")
    rospy.rostime.wallsleep(0.5)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exception:
        rospy.logfatal("Simulator enable failed: %s", exception)
        raise SystemExit(1)
