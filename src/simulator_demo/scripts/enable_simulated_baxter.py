#!/usr/bin/env python3
"""Enable the simulated Baxter once its effort controller for the given limb is loaded.

Enabling earlier makes the simulator hand the joints over before the controller that
will drive them exists. The node waits for ``<limb>_joint_effort_controller`` in the
controller manager and for joint states, then publishes a latched enable.
"""

import argparse
import time

import rospy
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

CONTROLLER_MANAGER_SERVICE = "/robot/controller_manager/list_controllers"
ENABLE_TOPIC = "/robot/set_super_enable"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Enable Baxter only after its simulated effort controller is loaded."
    )
    parser.add_argument("--limb", choices=("left", "right"), default="left")
    parser.add_argument("--timeout", type=float, default=60.0, help="wait limit [s] for the controller and the enable")
    return parser.parse_args(rospy.myargv()[1:])


def wait_for_effort_controller(limb: str, timeout_sec: float) -> None:
    """Blocks until ``<limb>_joint_effort_controller`` is listed and joint states arrive."""
    controller_name = "{}_joint_effort_controller".format(limb)
    deadline = time.monotonic() + timeout_sec

    rospy.wait_for_service(CONTROLLER_MANAGER_SERVICE, timeout=timeout_sec)
    list_controllers = rospy.ServiceProxy(CONTROLLER_MANAGER_SERVICE, ListControllers)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        response = list_controllers()
        if any(controller.name == controller_name for controller in response.controller):
            rospy.wait_for_message("/robot/joint_states", JointState, timeout=5.0)
            return
        rospy.rostime.wallsleep(0.1)

    raise RuntimeError("{} was not loaded within {:.0f} s".format(controller_name, timeout_sec))


def publish_enable(timeout_sec: float) -> None:
    """Publishes a latched enable once the simulator's enable subscriber is connected."""
    publisher = rospy.Publisher(ENABLE_TOPIC, Bool, queue_size=1, latch=True)
    deadline = time.monotonic() + timeout_sec
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        rospy.rostime.wallsleep(0.1)
    if publisher.get_num_connections() == 0:
        raise RuntimeError("Baxter simulator enable subscriber is not available")
    publisher.publish(Bool(data=True))
    rospy.rostime.wallsleep(0.5)   # let the latched message go out before the node exits


def main() -> None:
    rospy.init_node("enable_simulated_baxter")
    args = parse_args()
    if args.timeout <= 0.0:
        raise RuntimeError("--timeout must be positive")

    wait_for_effort_controller(args.limb, args.timeout)
    publish_enable(args.timeout)
    rospy.loginfo("Enabled Baxter after %s was loaded", args.limb + "_joint_effort_controller")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exception:
        rospy.logfatal("Simulator enable failed: %s", exception)
        raise SystemExit(1)
