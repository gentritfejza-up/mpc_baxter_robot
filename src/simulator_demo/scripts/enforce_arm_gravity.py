#!/usr/bin/env python3
"""Keep gravity enabled on the simulated arm links.

baxter_sim_kinematics (started by the simulator's baxter_emulator) calls
/gazebo/set_link_properties with gravity_mode=false on every arm link to emulate the
real robot's built-in gravity compensation. It races with the robot spawn: when it runs
first the calls fail silently and the arms keep their weight; when it runs after the
spawn the arms become weightless while the KDL torque loop still adds gravity
compensation, and both arms drift up at a steady rate until the elbows hit their stops.
This node re-enables gravity on the 14 arm links whenever it finds it off during the
first ``~duration_sec`` seconds (ROS time) of a run, so the simulation always matches
the torque loop's model.
"""
import sys

import rospy
from gazebo_msgs.srv import GetLinkProperties, SetLinkProperties

#: The seven links of each arm that baxter_sim_kinematics makes weightless.
ARM_LINKS = ["%s_%s" % (side, link) for side in ("left", "right")
             for link in ("upper_shoulder", "lower_shoulder", "upper_elbow", "lower_elbow",
                          "upper_forearm", "lower_forearm", "wrist")]
#: Default watch duration [s] (~duration_sec).
DEFAULT_DURATION_SEC = 40.0
#: Time between sweeps over the links [s].
CHECK_PERIOD_SEC = 0.5


def restore_gravity(link_name: str, get_properties: rospy.ServiceProxy,
                    set_properties: rospy.ServiceProxy) -> bool:
    """Turns gravity back on for ``link_name`` if it is off; True when a fix was applied."""
    try:
        properties = get_properties(link_name)
    except rospy.ServiceException:
        return False
    if not properties.success or properties.gravity_mode:
        return False
    try:
        result = set_properties(link_name=link_name, com=properties.com, gravity_mode=True,
                                mass=properties.mass,
                                ixx=properties.ixx, ixy=properties.ixy, ixz=properties.ixz,
                                iyy=properties.iyy, iyz=properties.iyz, izz=properties.izz)
    except rospy.ServiceException as error:
        rospy.logwarn("set_link_properties failed for %s: %s", link_name, error)
        return False
    if result.success:
        rospy.logwarn("Gravity was off on %s (simulator gravity emulation); turned it back on", link_name)
    return result.success


def main() -> None:
    rospy.init_node("enforce_arm_gravity")
    sys.stdout.reconfigure(line_buffering=True)
    duration_sec = rospy.get_param("~duration_sec", DEFAULT_DURATION_SEC)
    rospy.wait_for_service("/gazebo/get_link_properties", 60.0)
    rospy.wait_for_service("/gazebo/set_link_properties", 60.0)
    get_properties = rospy.ServiceProxy("/gazebo/get_link_properties", GetLinkProperties)
    set_properties = rospy.ServiceProxy("/gazebo/set_link_properties", SetLinkProperties)
    started_at = rospy.Time.now()
    fixed_count = 0
    while not rospy.is_shutdown() and (rospy.Time.now() - started_at).to_sec() < duration_sec:
        for link_name in ARM_LINKS:
            if restore_gravity(link_name, get_properties, set_properties):
                fixed_count += 1
        rospy.sleep(CHECK_PERIOD_SEC)
    rospy.loginfo("Arm link gravity check finished (%d links re-enabled)", fixed_count)


if __name__ == "__main__":
    main()
