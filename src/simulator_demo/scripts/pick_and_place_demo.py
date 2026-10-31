#!/usr/bin/env python3
"""Two-arm pick-and-place demo on the (moving) conveyor belt.

Cuboids are spawned on the belt and driven along it at the belt speed (the belt
model is static, so the objects are moved kinematically through Gazebo's model
state interface). Each arm claims the next object entering its zone, hovers above
the object's predicted position (constant-velocity lead), descends, closes the
gripper, lifts, carries the object through a via point beside the shoulder to its
drop spot above the box and releases it. Placed objects (and objects that reach the
end of the belt unpicked) re-enter the belt at its start as soon as the spacing
allows, so the supply never stops.

Joint references come from a local KDL IK and go to the MPC through dynamic
reconfigure, exactly like send_sim_references.py (rate-limited to 10 Hz and sent
only when the reference changed). Carry and drop poses are chosen among several
gripper orientations by their clearance from the body keep-outs, evaluated with
the same spheres and primitives the MPC guard uses (/robot/collision_geometry).

Frames: Gazebo's world origin is on the ground; Baxter's base link is spawned
BASE_Z_WORLD above it with the same x/y, so base-frame x/y equal world x/y and only
z differs. Everything the arms reason about is in the base frame.
"""

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

import PyKDL

import rospy
from baxter_core_msgs.msg import AssemblyState
from dynamic_reconfigure.client import Client
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelConfiguration, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion

import baxter_interface

JointAngles = Sequence[float]

#: Baxter joint name suffixes in MPC reference order (ref_0 ... ref_6).
JOINT_SUFFIXES = ("s0", "s1", "e0", "e1", "w0", "w1", "w2")
#: Baxter's "untucked" home pose per limb [rad]; the real robot starts here and the
#: MPC arms its conveyor floor once the arm has reached it.
HOME_JOINT_ANGLES = {"left": (-0.08, -1.0, -1.19, 1.94, 0.67, 1.03, -0.5),
                     "right": (0.08, -1.0, 1.19, 1.94, -0.67, 1.03, 0.5)}
#: Gripper pointing down, fingers across the belt (from baxter_sim_examples).
OVERHEAD_ORIENTATION = Quaternion(x=-0.0249590815779, y=0.999649402929,
                                  z=0.00737916180073, w=0.00486450832011)


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Returns the geometry_msgs quaternion for fixed-axis roll/pitch/yaw [rad]."""
    x, y, z, w = PyKDL.Rotation.RPY(roll, pitch, yaw).GetQuaternion()
    return Quaternion(x=x, y=y, z=z, w=w)


#: Orientations tried for the carry and drop above the boxes behind the robot: down
#: first, then yawed and tilted variants that stay reachable beside the shoulders.
DROP_ORIENTATIONS = [OVERHEAD_ORIENTATION,
                     _quaternion_from_rpy(math.pi, 0.0, math.pi / 2),
                     _quaternion_from_rpy(math.pi, 0.0, -math.pi / 2),
                     _quaternion_from_rpy(math.pi, 0.6, math.pi / 2)]

#: Height of Baxter's base link above the Gazebo ground plane [m].
BASE_Z_WORLD = 0.93
#: CT6030 belt surface height above the ground [m].
BELT_TOP_WORLD = 0.75
#: Belt surface height in the base frame [m] (-0.18).
BELT_TOP_BASE = BELT_TOP_WORLD - BASE_Z_WORLD
#: Cuboids are driven 1 mm above the belt so they never intersect it.
OBJECT_Z_WORLD = BELT_TOP_WORLD + 0.001

#: Dynamic-reconfigure sends are expensive (every parameter is written to the ROS
#: master): send at most this often ...
REFERENCE_MIN_PERIOD_SEC = 0.1
#: ... and only when some joint of the reference moved by more than this [rad].
REFERENCE_MIN_CHANGE_RAD = 0.002
#: A placed object rests in its box this long before it may re-enter the belt [s].
BELT_REENTRY_DELAY_SEC = 1.5
#: Belt integration step cap [s]: after a stall the objects must not jump.
BELT_MAX_TICK_SEC = 0.1
#: Consecutive IK failures in HOVER before the pick is abandoned (1 s at 20 Hz).
MAX_HOVER_IK_FAILURES = 20
#: Lift/carry/drop moves give up waiting for the tolerance after this long [s].
MOVE_TIMEOUT_SEC = 8.0
#: Demo loop rate [Hz]; the state machines and the belt tick at this rate.
LOOP_RATE_HZ = 20.0
#: Belt extent along +y in the base frame [m].
BELT_Y_START = -0.80
BELT_Y_END = 0.80


class KdlIk:
    """Levenberg-Marquardt IK on the base -> <limb>_gripper chain built from the URDF.

    The simulator's IK service rejected every overhead hover pose over the belt, so the
    demo solves its own, seeded with the current joints (continuous solutions).

    Attributes:
        joint_names: Names of the movable joints along the chain, base to gripper.
        lower_limits: Per-joint lower position limits [rad].
        upper_limits: Per-joint upper position limits [rad].
    """

    #: Solutions are clamped this far inside the joint limits [rad].
    LIMIT_MARGIN_RAD = 0.02
    #: A clamped solution is rejected when its hand position is off by more than this [m].
    MAX_POSITION_ERROR_M = 0.01

    def __init__(self, urdf_xml: str, limb: str):
        """Builds the KDL chain for ``limb`` from the URDF text.

        Args:
            urdf_xml: Contents of /robot_description.
            limb: "left" or "right".
        """
        root = ET.fromstring(urdf_xml)
        joint_by_child_link = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
        chain_joints = []
        link = "%s_gripper" % limb
        while link != "base":
            joint = joint_by_child_link[link]
            chain_joints.append(joint)
            link = joint.find("parent").get("link")
        chain_joints.reverse()

        self.chain = PyKDL.Chain()
        self.joint_names = []  # type: List[str]
        self.lower_limits = []  # type: List[float]
        self.upper_limits = []  # type: List[float]
        for joint in chain_joints:
            origin = joint.find("origin")
            xyz = self._vector_attribute(origin, "xyz")
            rpy = self._vector_attribute(origin, "rpy")
            joint_frame = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))
            joint_name = joint.get("name")
            if joint.get("type") in ("revolute", "continuous"):
                axis = [float(value) for value in joint.find("axis").get("xyz").split()]
                kdl_joint = PyKDL.Joint(joint_name, joint_frame.p, joint_frame.M * PyKDL.Vector(*axis),
                                        PyKDL.Joint.RotAxis)
                limit = joint.find("limit")
                self.joint_names.append(joint_name)
                self.lower_limits.append(float(limit.get("lower")) if limit is not None else -math.pi)
                self.upper_limits.append(float(limit.get("upper")) if limit is not None else math.pi)
            else:
                # PyKDL names the fixed joint type "Fixed" (KDL >= 1.4) or "None" (older).
                fixed_type = getattr(PyKDL.Joint, "Fixed", getattr(PyKDL.Joint, "None", None))
                kdl_joint = PyKDL.Joint(joint_name, fixed_type)
            self.chain.addSegment(PyKDL.Segment(joint.find("child").get("link"), kdl_joint, joint_frame))
        self.segment_links = [joint.find("child").get("link") for joint in chain_joints]
        self.forward_kinematics = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.inverse_kinematics = PyKDL.ChainIkSolverPos_LMA(self.chain, 1e-5, 500)

    @staticmethod
    def _vector_attribute(origin, attribute: str) -> List[float]:
        """Parses a 3-vector attribute of an <origin> element, defaulting to zeros."""
        text = origin.get(attribute) if origin is not None else None
        return [float(value) for value in (text or "0 0 0").split()]

    @staticmethod
    def _joint_array(values: JointAngles) -> PyKDL.JntArray:
        joint_array = PyKDL.JntArray(len(values))
        for index, value in enumerate(values):
            joint_array[index] = value
        return joint_array

    def link_frame(self, joint_angles: JointAngles, link: str) -> PyKDL.Frame:
        """Returns the base-frame pose of a chain link for the given joint angles [rad]."""
        frame = PyKDL.Frame()
        self.forward_kinematics.JntToCart(self._joint_array(joint_angles), frame,
                                          self.segment_links.index(link) + 1)
        return frame

    def solve(self, seed: JointAngles, x: float, y: float, z: float,
              orientation: Quaternion) -> Optional[List[float]]:
        """Solves for the gripper at base-frame (x, y, z) [m] with ``orientation``.

        Args:
            seed: Joint angles to start the iteration from [rad].

        Returns:
            Joint angles clamped inside the limits, or None when the solver failed or
            the clamped solution misses the position by more than MAX_POSITION_ERROR_M.
        """
        rotation = PyKDL.Rotation.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        target = PyKDL.Frame(rotation, PyKDL.Vector(x, y, z))
        raw_solution = PyKDL.JntArray(len(seed))
        if self.inverse_kinematics.CartToJnt(self._joint_array(seed), target, raw_solution) < 0:
            return None
        solution = [min(max(raw_solution[index], self.lower_limits[index] + self.LIMIT_MARGIN_RAD),
                        self.upper_limits[index] - self.LIMIT_MARGIN_RAD)
                    for index in range(len(seed))]
        reached = PyKDL.Frame()
        self.forward_kinematics.JntToCart(self._joint_array(solution), reached)
        if (reached.p - target.p).Norm() > self.MAX_POSITION_ERROR_M:
            return None
        return solution


@dataclass
class ArmSphere:
    """One collision sphere of the arm, as in collision_geometry.yaml."""
    link: str
    offset: PyKDL.Vector            # in the link frame [m]
    radius: float                   # [m]
    ignored_bodies: Set[str] = field(default_factory=set)


@dataclass
class BodyPrimitive:
    """One body keep-out: an axis-aligned box or a sphere in the base frame [m]."""
    name: str
    kind: str                       # "aabb" or "sphere"
    box_min: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    box_max: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: float = 0.0


class BodyClearance:
    """Surface clearance between the arm's collision spheres and the body keep-outs.

    Uses the same spheres and primitives as the MPC guard (/robot/collision_geometry).
    A reference inside the guard margin is projected out by the controller, so the arm
    parks short of it: candidate carry/drop poses are ranked by this clearance.
    """

    def __init__(self, ik: KdlIk, limb: str, geometry: dict):
        """Reads the sphere and body specs for ``limb`` from the guard's parameter dict."""
        self.ik = ik
        self.spheres = []  # type: List[ArmSphere]
        for spec in geometry.get("arm_spheres", []):
            if spec.get("side", limb) != limb or not spec.get("body", True):
                continue
            link = spec["link"].replace("{side}", limb)
            if link not in ik.segment_links:
                rospy.logwarn("%s arm: collision sphere on %s is not on the IK chain; ignored", limb, link)
                continue
            self.spheres.append(ArmSphere(link, PyKDL.Vector(*spec["offset"]), float(spec["radius"]),
                                          set(spec.get("ignore", []))))
        self.bodies = []  # type: List[BodyPrimitive]
        for body in geometry.get("body", []):
            if body["type"] == "aabb":
                self.bodies.append(BodyPrimitive(body["name"], "aabb",
                                                 box_min=tuple(body["min"]), box_max=tuple(body["max"])))
            elif body["type"] == "sphere":
                self.bodies.append(BodyPrimitive(body["name"], "sphere",
                                                 center=tuple(body["center"]), radius=float(body["radius"])))

    @staticmethod
    def _surface_distance(body: BodyPrimitive, center: PyKDL.Vector, radius: float) -> float:
        """Signed surface-to-surface distance [m] between a sphere and a body (negative inside)."""
        point = (center.x(), center.y(), center.z())
        if body.kind == "sphere":
            return math.sqrt(sum((p - c) ** 2 for p, c in zip(point, body.center))) - body.radius - radius
        outside = [max(low - value, 0.0, value - high)
                   for value, low, high in zip(point, body.box_min, body.box_max)]
        if any(outside):
            return math.sqrt(sum(gap * gap for gap in outside)) - radius
        penetration = min(min(value - low, high - value)
                          for value, low, high in zip(point, body.box_min, body.box_max))
        return -penetration - radius

    def evaluate(self, joint_angles: JointAngles) -> Tuple[float, Optional[str]]:
        """Returns (minimum clearance [m], obstacle name) over all spheres and bodies."""
        worst_distance, worst_body = float("inf"), None
        for sphere in self.spheres:
            center = self.ik.link_frame(joint_angles, sphere.link) * sphere.offset
            for body in self.bodies:
                if body.name in sphere.ignored_bodies:
                    continue
                distance = self._surface_distance(body, center, sphere.radius)
                if distance < worst_distance:
                    worst_distance, worst_body = distance, body.name
        return worst_distance, worst_body


class Belt:
    """Kinematic conveyor: moves the free objects along +y at the belt speed.

    Objects are teleported with ModelState poses only (no twist: Gazebo would
    integrate a twist on top of the teleport steps and double the belt speed).
    The belt runs on wall-clock time (time.monotonic).

    Attributes:
        free: Names of the objects currently driven along the belt.
        reentry_queue: Objects waiting to re-enter at the start -> monotonic time they
            were queued (0.0 for objects that left the belt unpicked: eligible at once).
        driven_y: Commanded world y per object [m].
        world_poses: Latest world poses from /gazebo/model_states per object.
    """

    def __init__(self, speed: float, belt_x: float, y_start: float, y_end: float,
                 object_count: int, spacing: float):
        """
        Args:
            speed: Belt speed along +y [m/s].
            belt_x: World/base x of the belt centre line [m].
            y_start: Where objects enter the belt [m].
            y_end: Where unpicked objects leave the belt [m].
            object_count: Number of cuboids to spawn and cycle.
            spacing: Initial and minimum re-entry gap between objects [m].
        """
        self.speed = speed
        self.belt_x = belt_x
        self.y_start = y_start
        self.y_end = y_end
        self.spacing = spacing
        self.object_names = ["pick_cuboid_%d" % index for index in range(object_count)]
        self.free = set(self.object_names)  # type: Set[str]
        self.reentry_queue = {}  # type: Dict[str, float]
        self.driven_y = {name: y_start + index * spacing for index, name in enumerate(self.object_names)}
        self.world_poses = {}  # type: Dict[str, Pose]
        self.last_tick_time = None  # type: Optional[float]
        self._state_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=20)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=1)

    def _on_model_states(self, message: ModelStates) -> None:
        for name, pose in zip(message.name, message.pose):
            if name in self.driven_y:
                self.world_poses[name] = pose

    def _belt_pose(self, name: str) -> Pose:
        return Pose(position=Point(x=self.belt_x, y=self.driven_y[name], z=OBJECT_Z_WORLD),
                    orientation=Quaternion(0, 0, 0, 1))

    def spawn(self, sdf_xml: str) -> None:
        """Spawns every object on the belt at its initial position."""
        rospy.wait_for_service("/gazebo/spawn_sdf_model", 30.0)
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        for name in self.object_names:
            result = spawn_model(name, sdf_xml, "/", self._belt_pose(name), "world")
            if not result.success:
                rospy.logwarn("Spawning %s failed: %s", name, result.status_message)

    def tick(self) -> None:
        """Advances the free objects by one wall-clock step and feeds the entrance."""
        now = time.monotonic()
        step_sec = 0.0 if self.last_tick_time is None else min(now - self.last_tick_time, BELT_MAX_TICK_SEC)
        self.last_tick_time = now
        for name in list(self.free):
            self.driven_y[name] += self.speed * step_sec
            if self.driven_y[name] > self.y_end:
                # Left the belt unpicked: back to the start, first in line and without delay.
                self.free.discard(name)
                self.reentry_queue[name] = 0.0
                continue
            state = ModelState()
            state.model_name = name
            state.pose = self._belt_pose(name)
            state.reference_frame = "world"
            self._state_publisher.publish(state)
        self._feed_entrance(now)

    def _feed_entrance(self, now: float) -> None:
        """Puts the longest-waiting queued object on the belt start once the entrance is
        clear by one spacing and the object has waited its re-entry delay."""
        if not self.reentry_queue:
            return
        if any(self.driven_y[name] < self.y_start + self.spacing for name in self.free):
            return
        name = min(self.reentry_queue, key=self.reentry_queue.get)
        if now - self.reentry_queue[name] < BELT_REENTRY_DELAY_SEC:
            return
        del self.reentry_queue[name]
        self.driven_y[name] = self.y_start
        self.free.add(name)

    def recycle(self, name: str) -> None:
        """Object was placed in a box: send it back to the belt start when there is room."""
        self.reentry_queue[name] = time.monotonic()

    def release(self, name: str) -> None:
        """Object left the belt (held by a gripper): stop driving it."""
        self.free.discard(name)

    def put_back(self, name: str, belt_y: Optional[float] = None) -> None:
        """Object dropped back on the belt: drive it again, from ``belt_y`` when known [m]."""
        if belt_y is not None:
            self.driven_y[name] = belt_y
        self.free.add(name)

    def object_base_xyz(self, name: str) -> Optional[Tuple[float, float, float]]:
        """Measured object position in the base frame [m], or None before the first model state."""
        pose = self.world_poses.get(name)
        if pose is None:
            return None
        return (pose.position.x, pose.position.y, pose.position.z - BASE_Z_WORLD)


class Arm:
    """Pick-and-place state machine for one arm, stepped at the demo loop rate.

    States: HOME -> IDLE -> HOVER -> DESCEND -> CLOSE -> LIFT -> CARRY -> DROP -> RELEASE
    -> HOME. A failed grasp (LIFT) or an object escaping the zone (HOVER, DESCEND) goes
    back to HOME. Cartesian targets are in the base frame [m].
    """

    HOME, IDLE, HOVER, DESCEND, CLOSE, LIFT, CARRY, DROP, RELEASE = (
        "HOME", "IDLE", "HOVER", "DESCEND", "CLOSE", "LIFT", "CARRY", "DROP", "RELEASE")

    #: Fingertip height for the grasp [m]: grip the top 4.5 cm of the 100 mm cuboid.
    GRASP_Z = BELT_TOP_BASE + 0.055
    #: Hover height above the grasp [m].
    HOVER_Z = GRASP_Z + 0.12
    #: Lift height after the grasp [m].
    CARRY_Z = 0.12
    #: Drop height [m]: fingertips 13 cm above the box rim; the held cuboid clears it.
    DROP_Z = 0.25
    #: Carry via point beside the shoulder [m]: swinging straight back from the belt
    #: drags the upper arm along the head keep-out.
    VIA_X = 0.35
    VIA_Z = 0.28
    #: Pick candidates: objects from this far upstream of the zone ...
    CANDIDATE_UPSTREAM_M = 0.30
    #: ... up to this far before the zone end are considered.
    CANDIDATE_END_MARGIN_M = 0.15
    #: Hover waits for an upstream object this far before the zone start [m].
    HOVER_WAIT_OFFSET_M = 0.03
    #: Descent is abandoned once the object is this far past the zone end [m].
    DESCEND_OVERRUN_M = 0.08
    #: Time the gripper gets to close before the lift [s].
    GRIPPER_CLOSE_SEC = 0.6
    #: Time the gripper gets to open before the object counts as placed [s].
    GRIPPER_OPEN_SEC = 0.8
    #: An object still this close to the belt after the lift was not grasped [m].
    GRASP_CHECK_HEIGHT_M = 0.08
    #: Home is reached when every joint is within this of the home pose [rad].
    HOME_TOLERANCE_RAD = 0.08

    def __init__(self, limb: str, limb_interface: baxter_interface.Limb, belt: Belt,
                 zone: Tuple[float, float], drop_xy: Tuple[float, float], lead_sec: float,
                 reach_tolerance: float, claimed_objects: Set[str]):
        """
        Args:
            limb: "left" or "right".
            limb_interface: baxter_interface.Limb for the measured joint state and endpoint.
            belt: The shared conveyor model.
            zone: (y_min, y_max) of this arm's pick zone in the base frame [m].
            drop_xy: Base-frame x/y of the drop spot above the box [m].
            lead_sec: Constant-velocity prediction lead for the moving object [s].
            reach_tolerance: Default Cartesian tolerance for "reached" [m].
            claimed_objects: Set shared by both arms of object names currently claimed.
        """
        self.limb = limb
        self.limb_interface = limb_interface
        self.belt = belt
        self.zone_y_min, self.zone_y_max = zone
        self.drop_xy = drop_xy
        self.lead_sec = lead_sec
        self.reach_tolerance = reach_tolerance
        self.claimed_objects = claimed_objects

        self.state = self.HOME
        self.state_entered_at = time.monotonic()
        self.target_object = None  # type: Optional[str]
        self.lift_xy = (belt.belt_x, 0.0)
        self.placed_count = 0
        self.missed_count = 0
        self.ik_failures = 0
        self.last_reference = HOME_JOINT_ANGLES[limb]  # type: Tuple[float, ...]
        self.sent_reference = None  # type: Optional[Tuple[float, ...]]
        self.last_send_time = 0.0
        self.locked_orientation = None  # type: Optional[Quaternion]

        self.reconfigure_client = Client("/mpc_controller_node_%s" % limb, timeout=30.0)
        self.gripper = baxter_interface.Gripper(limb)
        self.joint_names = ["%s_%s" % (limb, suffix) for suffix in JOINT_SUFFIXES]
        self.ik = KdlIk(rospy.get_param("/robot_description"), limb)
        assert self.ik.joint_names == self.joint_names, self.ik.joint_names
        self.clearance = BodyClearance(self.ik, limb, rospy.get_param("/robot/collision_geometry", {}))
        self.body_margin = float(rospy.get_param("/robot/geometry/body_margin", 0.04))

        self._state_handlers = {
            self.HOME: self._step_home,
            self.IDLE: self._step_idle,
            self.HOVER: self._step_hover,
            self.DESCEND: self._step_descend,
            self.CLOSE: self._step_close,
            self.LIFT: self._step_lift,
            self.CARRY: self._step_carry,
            self.DROP: self._step_drop,
            self.RELEASE: self._step_release,
        }

    # ---- reference output and sensing

    def send_reference(self, reference: JointAngles) -> None:
        """Sends a joint reference [rad] to the MPC, rate-limited and change-gated.

        Dynamic-reconfigure updates write every parameter to the ROS master, so the
        reference goes out at most every REFERENCE_MIN_PERIOD_SEC and only when some
        joint moved by more than REFERENCE_MIN_CHANGE_RAD since the last send.
        """
        reference = tuple(reference)
        now = time.monotonic()
        changed = (self.sent_reference is None or
                   max(abs(new - old) for new, old in zip(reference, self.sent_reference))
                   > REFERENCE_MIN_CHANGE_RAD)
        if not changed or now - self.last_send_time < REFERENCE_MIN_PERIOD_SEC:
            self.last_reference = reference
            return
        self.reconfigure_client.update_configuration(
            {"ref_%d" % index: value for index, value in enumerate(reference)})
        self.sent_reference = reference
        self.last_reference = reference
        self.last_send_time = now

    def current_joint_angles(self) -> List[float]:
        """Measured joint angles [rad] in JOINT_SUFFIXES order (0.0 for a missing joint)."""
        angles = self.limb_interface.joint_angles()
        return [angles.get(name, 0.0) for name in self.joint_names]

    def endpoint_xyz(self) -> Optional[Tuple[float, float, float]]:
        """Measured gripper position in the base frame [m], or None before the first endpoint state."""
        pose = self.limb_interface.endpoint_pose()
        if not pose:
            return None
        position = pose["position"]
        return (position.x, position.y, position.z)

    def solve_ik(self, x: float, y: float, z: float,
                 orientations: Sequence[Quaternion] = (OVERHEAD_ORIENTATION,),
                 by_clearance: bool = False) -> Optional[List[float]]:
        """Joint angles [rad] for the gripper at base-frame (x, y, z) [m].

        Returns the first solvable orientation or, with ``by_clearance``, the one whose
        arm pose keeps the most clearance from the body keep-outs (a reference inside
        the guard margin is projected out by the controller and the arm parks short of
        the target). The chosen orientation is locked for the rest of the current state
        so the reference does not flip between orientations.
        """
        seed = self.current_joint_angles()
        if self.locked_orientation is not None:
            orientations = (self.locked_orientation,)
            by_clearance = False
        best = None  # (clearance, obstacle, orientation, solution)
        for orientation in orientations:
            solution = self.ik.solve(seed, x, y, z, orientation)
            if solution is None:
                continue
            if not by_clearance:
                return solution
            clearance, obstacle = self.clearance.evaluate(solution)
            if best is None or clearance > best[0]:
                best = (clearance, obstacle, orientation, solution)
        if best is not None:
            clearance, obstacle, orientation, solution = best
            self.locked_orientation = orientation
            rospy.loginfo("%s arm: %s pose (%.2f, %.2f, %.2f) with %.3f m clearance from %s%s",
                          self.limb, self.state, x, y, z, clearance, obstacle,
                          "" if clearance >= self.body_margin else " (inside the guard margin)")
            return solution
        rospy.logwarn_throttle(2.0, "%s: no IK solution for (%.2f, %.2f, %.2f)", self.limb, x, y, z)
        return None

    def go_cartesian(self, x: float, y: float, z: float,
                     orientations: Sequence[Quaternion] = (OVERHEAD_ORIENTATION,),
                     by_clearance: bool = False) -> bool:
        """Sends the IK solution for a base-frame target [m]; returns False when IK failed."""
        joint_angles = self.solve_ik(x, y, z, orientations, by_clearance)
        if joint_angles is not None:
            self.send_reference(joint_angles)
            self.ik_failures = 0
            return True
        self.ik_failures += 1
        return False

    def reached(self, x: float, y: float, z: float, tolerance: Optional[float] = None) -> bool:
        """True when the measured gripper position is within ``tolerance`` [m] of the target."""
        tip = self.endpoint_xyz()
        if tip is None:
            return False
        tolerance = self.reach_tolerance if tolerance is None else tolerance
        return math.sqrt((tip[0] - x) ** 2 + (tip[1] - y) ** 2 + (tip[2] - z) ** 2) < tolerance

    def reached_joints(self, reference: JointAngles, tolerance: float) -> bool:
        """True when every joint is within ``tolerance`` [rad] of ``reference``."""
        return max(abs(current - desired)
                   for current, desired in zip(self.current_joint_angles(), reference)) < tolerance

    # ---- state bookkeeping

    def set_state(self, state: str) -> None:
        """Enters ``state``: logs the transition and unlocks the orientation choice."""
        rospy.loginfo("%s arm: %s -> %s%s", self.limb, self.state, state,
                      (" (%s)" % self.target_object) if self.target_object else "")
        self.state = state
        self.state_entered_at = time.monotonic()
        self.locked_orientation = None

    def time_in_state(self) -> float:
        """Seconds since the current state was entered."""
        return time.monotonic() - self.state_entered_at

    def predicted_target_xy(self) -> Optional[Tuple[float, float]]:
        """Target object position [m] extrapolated along the belt by lead_sec, or None."""
        xyz = self.belt.object_base_xyz(self.target_object)
        if xyz is None:
            return None
        return (xyz[0], xyz[1] + self.belt.speed * self.lead_sec)

    def _release_claim(self) -> None:
        """Forgets the target object and frees its claim for the other arm."""
        self.claimed_objects.discard(self.target_object)
        self.target_object = None

    def abort_pick(self, reason: str) -> None:
        """Gives up the current pick (object still on the belt) and returns home."""
        rospy.logwarn("%s arm: %s (%s)", self.limb, reason, self.target_object)
        self.missed_count += 1
        self.gripper.open()
        self._release_claim()
        self.set_state(self.HOME)

    def pick_candidate(self) -> Optional[str]:
        """The most downstream free, unclaimed object that is about to enter this zone."""
        best_name, best_y = None, None
        for name in sorted(self.belt.free):
            if name in self.claimed_objects:
                continue
            xyz = self.belt.object_base_xyz(name)
            if xyz is None:
                continue
            object_y = xyz[1]
            if self.zone_y_min - self.CANDIDATE_UPSTREAM_M <= object_y <= self.zone_y_max - self.CANDIDATE_END_MARGIN_M:
                if best_y is None or object_y > best_y:
                    best_name, best_y = name, object_y
        return best_name

    # ---- state machine, called at the loop rate

    def step(self) -> None:
        """Runs one iteration of the current state's handler."""
        self._state_handlers[self.state]()

    def _step_home(self) -> None:
        home = HOME_JOINT_ANGLES[self.limb]
        self.send_reference(home)
        if self.time_in_state() > 1.0 and self.reached_joints(home, self.HOME_TOLERANCE_RAD):
            self.set_state(self.IDLE)

    def _step_idle(self) -> None:
        candidate = self.pick_candidate()
        if candidate:
            self.target_object = candidate
            self.claimed_objects.add(candidate)
            self.ik_failures = 0
            self.set_state(self.HOVER)

    def _step_hover(self) -> None:
        predicted_xy = self.predicted_target_xy()
        actual_xyz = self.belt.object_base_xyz(self.target_object)
        if predicted_xy is None or actual_xyz is None or actual_xyz[1] > self.zone_y_max:
            self.abort_pick("object left the zone before the grasp")
            return
        if self.ik_failures > MAX_HOVER_IK_FAILURES:
            self.abort_pick("no reachable hover pose")
            return
        # Wait at the zone entrance for an object still upstream of it.
        hover_x = predicted_xy[0]
        hover_y = min(max(predicted_xy[1], self.zone_y_min - self.HOVER_WAIT_OFFSET_M), self.zone_y_max)
        self.go_cartesian(hover_x, hover_y, self.HOVER_Z)
        if self.reached(hover_x, hover_y, self.HOVER_Z) and actual_xyz[1] >= self.zone_y_min:
            self.set_state(self.DESCEND)

    def _step_descend(self) -> None:
        predicted_xy = self.predicted_target_xy()
        actual_xyz = self.belt.object_base_xyz(self.target_object)
        if predicted_xy is None or actual_xyz is None or actual_xyz[1] > self.zone_y_max + self.DESCEND_OVERRUN_M:
            self.abort_pick("object left the zone during the descent")
            return
        self.go_cartesian(predicted_xy[0], predicted_xy[1], self.GRASP_Z)
        if self.reached(predicted_xy[0], predicted_xy[1], self.GRASP_Z, 0.025):
            self.gripper.close()
            self.set_state(self.CLOSE)

    def _step_close(self) -> None:
        predicted_xy = self.predicted_target_xy()
        if predicted_xy is not None:
            self.go_cartesian(predicted_xy[0], predicted_xy[1], self.GRASP_Z)   # keep moving with the belt
        if self.time_in_state() > self.GRIPPER_CLOSE_SEC:
            self.belt.release(self.target_object)                               # the gripper carries it now
            tip = self.endpoint_xyz()
            if tip is not None:
                self.lift_xy = (tip[0], tip[1])
            else:
                self.lift_xy = (self.belt.belt_x, predicted_xy[1] if predicted_xy else 0.0)
            self.set_state(self.LIFT)

    def _step_lift(self) -> None:
        lift_x, lift_y = self.lift_xy
        self.go_cartesian(lift_x, lift_y, self.CARRY_Z)
        if not (self.reached(lift_x, lift_y, self.CARRY_Z, 0.03) or self.time_in_state() > MOVE_TIMEOUT_SEC):
            return
        object_xyz = self.belt.object_base_xyz(self.target_object)
        if object_xyz is None or object_xyz[2] < BELT_TOP_BASE + self.GRASP_CHECK_HEIGHT_M:
            self.gripper.open()
            self.missed_count += 1
            rospy.logwarn("%s arm: grasp of %s failed (object still on the belt)", self.limb, self.target_object)
            self.belt.put_back(self.target_object, object_xyz[1] if object_xyz else None)
            self._release_claim()
            self.set_state(self.HOME)
        else:
            self.set_state(self.CARRY)

    def _step_carry(self) -> None:
        via_x, via_y, via_z = self.VIA_X, self.drop_xy[1], self.VIA_Z
        self.go_cartesian(via_x, via_y, via_z, DROP_ORIENTATIONS, by_clearance=True)
        if self.reached(via_x, via_y, via_z, 0.06) or self.time_in_state() > MOVE_TIMEOUT_SEC:
            self.set_state(self.DROP)

    def _step_drop(self) -> None:
        drop_x, drop_y = self.drop_xy
        self.go_cartesian(drop_x, drop_y, self.DROP_Z, DROP_ORIENTATIONS, by_clearance=True)
        if self.reached(drop_x, drop_y, self.DROP_Z, 0.04) or self.time_in_state() > MOVE_TIMEOUT_SEC:
            self.gripper.open()
            self.set_state(self.RELEASE)

    def _step_release(self) -> None:
        if self.time_in_state() > self.GRIPPER_OPEN_SEC:
            self.placed_count += 1
            rospy.loginfo("%s arm: placed %s (%d so far)", self.limb, self.target_object, self.placed_count)
            self.belt.recycle(self.target_object)
            self._release_claim()
            self.set_state(self.HOME)


def wait_for_rest(limb_interfaces: Dict[str, baxter_interface.Limb], max_speed: float = 0.2,
                  hold_sec: float = 0.5, timeout_sec: float = 10.0) -> bool:
    """True once every joint of every limb moved slower than ``max_speed`` [rad/s] for ``hold_sec``."""
    deadline = time.monotonic() + timeout_sec
    quiet_since = None
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        peak_speed = max(abs(velocity) for limb in limb_interfaces.values()
                         for velocity in limb.joint_velocities().values())
        if peak_speed > max_speed:
            quiet_since = None
        elif quiet_since is None:
            quiet_since = time.monotonic()
        elif time.monotonic() - quiet_since >= hold_sec:
            return True
        rospy.sleep(0.05)
    return False


def _max_home_error(limb_interfaces: Dict[str, baxter_interface.Limb]) -> float:
    """Largest joint distance of either arm from its home pose [rad]."""
    return max(abs(limb_interfaces[limb].joint_angle("%s_%s" % (limb, suffix)) - HOME_JOINT_ANGLES[limb][index])
               for limb in ("left", "right") for index, suffix in enumerate(JOINT_SUFFIXES))


def set_simulated_home(limb_interfaces: Dict[str, baxter_interface.Limb], attempts: int = 5) -> bool:
    """Teleports both simulated arms to the home pose and verifies that they stay there.

    The teleport keeps the joint velocities, so an arm still swinging from the enable
    hand-over drifts away before the torque loop's hold can catch it: wait for rest
    first and verify the pose afterwards. Opt-in through --teleport-home.
    """
    rospy.wait_for_service("/gazebo/set_model_configuration", 10.0)
    set_configuration = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
    joint_names = ["%s_%s" % (limb, suffix) for limb in ("left", "right") for suffix in JOINT_SUFFIXES]
    joint_values = [value for limb in ("left", "right") for value in HOME_JOINT_ANGLES[limb]]
    for attempt in range(1, attempts + 1):
        if not wait_for_rest(limb_interfaces):
            rospy.logwarn("Arms still moving before the teleport; teleporting anyway")
        home_error = _max_home_error(limb_interfaces)
        if home_error < 0.1:                      # already at home: nothing to do
            rospy.loginfo("Simulated arms already at home (max error %.3f rad)", home_error)
            return True
        result = set_configuration("baxter", "robot_description", joint_names, joint_values)
        if not result.success:
            rospy.logwarn("Setting the simulated arms to home failed: %s", result.status_message)
        rospy.sleep(1.0)
        home_error = _max_home_error(limb_interfaces)
        if home_error < 0.1:
            rospy.loginfo("Simulated arms set to home (attempt %d, max error %.3f rad)", attempt, home_error)
            return True
        rospy.logwarn("Arms drifted %.2f rad from home after the teleport (attempt %d); retrying",
                      home_error, attempt)
    return False


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--belt-speed", type=float, default=0.05, help="belt speed [m/s] (+y)")
    parser.add_argument("--objects", type=int, default=6)
    parser.add_argument("--spacing", type=float, default=0.22)
    parser.add_argument("--duration", type=float, default=120.0, help="demo duration [s]")
    parser.add_argument("--lead", type=float, default=0.5, help="prediction lead for the moving object [s]")
    parser.add_argument("--reach-tol", type=float, default=0.025)
    parser.add_argument("--cuboid-sdf", required=True)
    parser.add_argument("--belt-x", type=float, default=0.70)
    parser.add_argument("--teleport-home", action="store_true",
                        help="teleport the simulated arms to the home pose after the enable instead of "
                             "letting the MPC drive them there (Gazebo's set_model_configuration throws "
                             "the joints to their velocity limits; off by default)")
    return parser.parse_args(argv)


def wait_for_robot_enabled(timeout_sec: float = 90.0) -> None:
    """Blocks until /robot/state reports the robot enabled (the launch enables it)."""
    deadline = time.monotonic() + timeout_sec
    while not rospy.is_shutdown():
        try:
            if rospy.wait_for_message("/robot/state", AssemblyState, timeout=2.0).enabled:
                return
        except rospy.ROSException:
            pass
        if time.monotonic() > deadline:
            raise RuntimeError("robot not enabled")


def main() -> None:
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("pick_and_place_demo")
    sys.stdout.reconfigure(line_buffering=True)   # roslaunch pipes stdout: show the log live
    belt = Belt(args.belt_speed, args.belt_x, BELT_Y_START, BELT_Y_END, args.objects, args.spacing)
    with open(args.cuboid_sdf) as sdf_file:
        belt.spawn(sdf_file.read())

    wait_for_robot_enabled()
    limb_interfaces = {limb: baxter_interface.Limb(limb) for limb in ("left", "right")}
    if args.teleport_home:
        # The real robot always starts in its home pose; optionally put the simulated
        # one there too instead of letting the MPC drive it there from the rest pose.
        try:
            if not set_simulated_home(limb_interfaces):
                rospy.logwarn("Could not keep the simulated arms at home; continuing anyway")
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn("Could not set the simulated arms to home: %s", error)

    # Pick zones and drop spots in the base frame [m]. The boxes sit behind the shoulder
    # line, far enough out that the raised upper arm clears the head keep-out (same
    # positions as pick_and_place_demo.launch and collision_geometry_demo.yaml).
    arm_layout = (("right", (-0.45, -0.15), (-0.10, -0.90)),
                  ("left", (0.15, 0.45), (-0.10, 0.90)))
    claimed_objects = set()  # type: Set[str]
    arms = {}  # type: Dict[str, Arm]
    for limb, zone, drop_xy in arm_layout:
        arm = Arm(limb, limb_interfaces[limb], belt, zone, drop_xy, args.lead, args.reach_tol, claimed_objects)
        if not arm.gripper.calibrated():
            arm.gripper.calibrate()
        arm.gripper.open()
        arms[limb] = arm
    rospy.loginfo("Pick-and-place demo: %d cuboids at %.2f m/s, %.0f s", args.objects, args.belt_speed, args.duration)

    loop_rate = rospy.Rate(LOOP_RATE_HZ)
    started_at = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic() - started_at < args.duration:
        belt.tick()
        for arm in arms.values():
            try:
                arm.step()
            except Exception as error:   # keep the other arm running
                rospy.logerr_throttle(2.0, "%s arm step failed: %s", arm.limb, error)
        loop_rate.sleep()
    for arm in arms.values():
        rospy.loginfo("DEMO RESULT %s arm: placed=%d missed=%d", arm.limb, arm.placed_count, arm.missed_count)
    rospy.loginfo("DEMO RESULT total placed=%d missed=%d",
                  sum(arm.placed_count for arm in arms.values()),
                  sum(arm.missed_count for arm in arms.values()))


if __name__ == "__main__":
    main()
