#!/usr/bin/env python3
"""Send deterministic, body-clearance-checked joint references to the Baxter MPC.

Used by baxter_mpc.launch for the Gazebo validation runs. Either a seeded sequence
of random poses inside SAFE_RANGES (each pose and the predicted transit to it kept
clear of the torso, the parked other arm and the floor by ArmClearance) or one of
the collision-guard scenarios in SCENARIO_POSES is sent through dynamic reconfigure,
one pose per --interval, with TRACK log lines every --report-interval. Every run
first sends the home (untucked) pose unless --no-home-first is given: the real robot
starts there and the MPC arms its conveyor floor once the arm has reached it.

The TARGET and TRACK log lines are parsed by summarize_control_diagnostics.py from
the recorder's event stream, so their format is part of the interface.
"""

import argparse
import math
import random
import threading
import time
import xml.etree.ElementTree as ET
from typing import Callable, List, Optional, Sequence, Tuple

import rospy
from baxter_core_msgs.msg import AssemblyState
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

JointAngles = Sequence[float]
Matrix4 = List[List[float]]

#: Baxter joint name suffixes in MPC reference order (ref_0 ... ref_6).
JOINT_SUFFIXES = ("s0", "s1", "e0", "e1", "w0", "w1", "w2")
#: URDF joint position limits [rad], used to validate every pose.
JOINT_LIMITS = (
    (-1.701679, 1.701679),
    (-2.146754, 1.0472),
    (-3.054326, 3.054326),
    (-0.049868, 2.617994),
    (-3.059, 3.059),
    (-1.5708, 2.0944),
    (-3.059, 3.059),
)
#: Random poses are drawn inside these ranges [rad], away from the hard joint limits:
#: limit-recovery behaviour is evaluated separately from nominal tracking.
SAFE_RANGES = (
    (-1.30, 1.30),
    (-1.80, 0.70),
    (-2.20, 2.20),
    (0.35, 2.30),
    (-2.20, 2.20),
    (-1.10, 1.60),
    (-2.20, 2.20),
)
REFERENCE_POSE_COUNT = 100
REFERENCE_RANDOM_SEED = 20260911

#: Collision-guard validation scenarios: joint targets [rad] for the LEFT arm (the
#: right arm uses the mirrored pose, see mirror_pose). The targets are deliberately
#: unsafe; the MPC guard must stop the arm short of them.
SCENARIO_POSES = {
    # Rest-height pose 0.55 m below the CT6030 belt top (z_min -0.18): the arm must hold at the belt.
    "floor": [(0.0, 1.0, 0.0, 0.05, 0.0, 0.0, 0.0)],
    # Hand into the chest/head keep-out, 0.21 m above the belt.
    "torso": [(-0.5, -1.2, -1.2, 2.5, 0.0, 0.5, 0.0)],
    # Both arms reach for the same spot in front of the chest, 0.14 m above the belt.
    "crossing": [(-1.3, -0.6, 0.2, 0.9, 0.0, 0.9, 0.0)],
    # A pick-like pose with the fingertips 0.059 m above the belt (just outside the
    # 0.05 m floor margin), then one 0.55 m below it: settle at the limit without chatter.
    "belt": [(0.3, -0.2, -1.0, 1.8, 0.5, 1.2, -0.3), (0.0, 1.0, 0.0, 0.05, 0.0, 0.0, 0.0)],
}
#: Per-joint signs that turn a left-arm pose into the mirrored right-arm pose.
MIRROR_SIGNS = (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0)
#: Baxter's "untucked" home pose for the left arm [rad]; the right arm is the mirror.
HOME_POSE = (-0.08, -1.0, -1.19, 1.94, 0.67, 1.03, -0.5)
#: Consecutive poses must differ by at least this RMS of the SAFE_RANGES-normalised steps ...
MIN_NORMALIZED_STEP_RMS = 0.45
#: ... with at least this many joints stepping by >= 30 % of their range.
MIN_LARGE_JOINT_STEPS = 4
LARGE_JOINT_STEP_FRACTION = 0.30


def mirror_pose(pose: JointAngles) -> Tuple[float, ...]:
    """Mirrors a left-arm pose onto the right arm (s0, e0, w0, w2 negated)."""
    return tuple(value * sign for value, sign in zip(pose, MIRROR_SIGNS))


# ---- minimal homogeneous-transform helpers (no numpy dependency)

def _rpy_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """3x3 rotation matrix for URDF fixed-axis roll/pitch/yaw [rad]."""
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return [
        [cos_y * cos_p, cos_y * sin_p * sin_r - sin_y * cos_r, cos_y * sin_p * cos_r + sin_y * sin_r],
        [sin_y * cos_p, sin_y * sin_p * sin_r + cos_y * cos_r, sin_y * sin_p * cos_r - cos_y * sin_r],
        [-sin_p, cos_p * sin_r, cos_p * cos_r],
    ]


def _homogeneous_transform(xyz: Sequence[float], rpy: Sequence[float]) -> Matrix4:
    """4x4 transform from a translation [m] and fixed-axis roll/pitch/yaw [rad]."""
    rotation = _rpy_matrix(*rpy)
    return [rotation[0] + [xyz[0]], rotation[1] + [xyz[1]], rotation[2] + [xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def _matmul(left: Matrix4, right: Matrix4) -> Matrix4:
    return [[sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4)] for row in range(4)]


def _rotation_z(angle: float) -> Matrix4:
    return _homogeneous_transform((0.0, 0.0, 0.0), (0.0, 0.0, angle))


def _transform_point(transform: Matrix4, point: Tuple[float, float, float]) -> List[float]:
    homogeneous_point = point + (1.0,)
    return [sum(transform[row][k] * homogeneous_point[k] for k in range(4)) for row in range(3)]


class ArmClearance:
    """Rejects arm poses (and transits) whose links would run into Baxter's own body.

    A coarse model for pose generation only: sample points along the arm against
    axis-aligned keep-out boxes in the torso frame (~6 cm link-radius margin) and a
    floor plane. The MPC's own guard uses the sphere model in collision_geometry.yaml.
    """

    #: Torso + head + pedestal column, extended backwards (nothing reachable behind the robot) [m].
    TORSO_BOX = ((-0.60, 0.33), (-0.30, 0.30), (-1.3, 1.05))
    #: The other arm parked in the simulator start pose (hanging forward-down) [m].
    OTHER_ARM_BOX = ((0.0, 0.65), (-0.75, -0.25), (-0.6, 0.25))
    #: Default floor [m] in the torso frame: the torso origin is 0.93 m above the ground (no belt).
    DEFAULT_FLOOR_Z = -0.80
    #: Sample points are link centre lines, so keep them this far above the floor/belt [m].
    FLOOR_CLEARANCE = 0.08
    #: Per-joint limits used to predict the transit path (MPC defaults) [rad/s^2], [rad/s].
    DEFAULT_PATH_ACCELERATION = (2.0, 5.0, 2.0, 5.0, 6.0, 6.0, 6.0)
    PATH_VELOCITY = (1.2, 1.2, 1.2, 1.2, 2.8, 2.8, 2.8)

    def __init__(self, urdf_xml: str, limb: str, z_min: Optional[float] = None,
                 acceleration_limits: Optional[Sequence[float]] = None):
        """
        Args:
            urdf_xml: Contents of /robot_description (joint origins are read from it).
            limb: "left" or "right".
            z_min: Conveyor surface height in the base frame [m]; None keeps the ground.
            acceleration_limits: Per-joint acceleration limits [rad/s^2] for the transit
                prediction (e.g. /robot/acceleration_max); None uses the MPC defaults.
        """
        self.floor_z = self.DEFAULT_FLOOR_Z if z_min is None else z_min + self.FLOOR_CLEARANCE
        self.path_acceleration = (list(self.DEFAULT_PATH_ACCELERATION) if acceleration_limits is None
                                  else [abs(float(value)) for value in acceleration_limits])
        root = ET.fromstring(urdf_xml)
        joints = {joint.get("name"): joint for joint in root.findall("joint")}

        def origin_transform(joint_name: str) -> Matrix4:
            origin = joints[joint_name].find("origin")
            xyz = tuple(float(value) for value in origin.get("xyz", "0 0 0").split())
            rpy = tuple(float(value) for value in origin.get("rpy", "0 0 0").split())
            return _homogeneous_transform(xyz, rpy)

        self._limb = limb
        self._mount_transform = origin_transform("{}_torso_arm_mount".format(limb))
        self._joint_origins = [origin_transform("{}_{}".format(limb, suffix)) for suffix in JOINT_SUFFIXES]
        self._hand_transform = origin_transform("{}_hand".format(limb))
        self._mirror_sign = 1.0 if limb == "left" else -1.0

    def sample_points(self, joint_angles: JointAngles) -> List[List[float]]:
        """Points along the arm in the torso frame [m]: joint origins from the elbow
        roll on, link midpoints, the hand and the gripper tip."""
        transform = self._mount_transform
        joint_frames = []
        for origin, angle in zip(self._joint_origins, joint_angles):
            transform = _matmul(_matmul(transform, origin), _rotation_z(angle))
            joint_frames.append(transform)
        upper_elbow, lower_elbow, upper_forearm, lower_forearm, wrist = (
            joint_frames[2], joint_frames[3], joint_frames[4], joint_frames[5], joint_frames[6])
        hand = _matmul(wrist, self._hand_transform)
        return [
            _transform_point(upper_elbow, (0.0, 0.0, 0.13)),     # upper arm midpoint
            _transform_point(lower_elbow, (0.0, 0.0, 0.0)),      # elbow
            _transform_point(upper_forearm, (0.0, 0.0, 0.135)),  # forearm midpoint
            _transform_point(lower_forearm, (0.0, 0.0, 0.0)),    # wrist pitch
            _transform_point(wrist, (0.0, 0.0, 0.0)),            # wrist roll
            _transform_point(hand, (0.0, 0.0, 0.0)),             # hand
            _transform_point(hand, (0.0, 0.0, 0.12)),            # gripper tip
        ]

    @staticmethod
    def _inside(box, point) -> bool:
        return all(low < value < high for (low, high), value in zip(box, point))

    def reason(self, joint_angles: JointAngles) -> Optional[str]:
        """None when the pose is clear, otherwise a short description of the hit."""
        for x, y, z in self.sample_points(joint_angles):
            mirrored_point = (x, self._mirror_sign * y, z)  # the right arm uses the same boxes
            if self._inside(self.TORSO_BOX, mirrored_point):
                return "torso/head at (%.2f, %.2f, %.2f)" % (x, y, z)
            if self._inside(self.OTHER_ARM_BOX, mirrored_point):
                return "other arm at (%.2f, %.2f, %.2f)" % (x, y, z)
            if z < self.floor_z:
                return "floor at (%.2f, %.2f, %.2f)" % (x, y, z)
        return None

    def is_clear(self, joint_angles: JointAngles) -> bool:
        return self.reason(joint_angles) is None

    @staticmethod
    def _time_optimal_profile(distance: float, acceleration: float,
                              velocity: float) -> Tuple[Callable[[float], float], float]:
        """1-D bang-bang (trapezoidal or triangular) move.

        Returns:
            (position(t) [rad], duration [s]) for a move of ``distance`` [rad] within
            the given acceleration [rad/s^2] and velocity [rad/s] limits.
        """
        span = abs(distance)
        sign = 1.0 if distance >= 0.0 else -1.0
        if span < velocity * velocity / acceleration:
            peak_time = math.sqrt(span / acceleration)
            duration = 2.0 * peak_time

            def position(elapsed: float) -> float:
                if elapsed <= peak_time:
                    travelled = 0.5 * acceleration * elapsed * elapsed
                elif elapsed <= duration:
                    travelled = span - 0.5 * acceleration * (duration - elapsed) ** 2
                else:
                    travelled = span
                return sign * travelled
        else:
            ramp_time = velocity / acceleration
            duration = 2.0 * ramp_time + (span - velocity * velocity / acceleration) / velocity

            def position(elapsed: float) -> float:
                if elapsed <= ramp_time:
                    travelled = 0.5 * acceleration * elapsed * elapsed
                elif elapsed <= duration - ramp_time:
                    travelled = 0.5 * velocity * ramp_time + velocity * (elapsed - ramp_time)
                elif elapsed <= duration:
                    travelled = span - 0.5 * acceleration * (duration - elapsed) ** 2
                else:
                    travelled = span
                return sign * travelled
        return position, duration

    def path_reason(self, start: JointAngles, goal: JointAngles, samples: int = 30) -> Optional[str]:
        """Checks the transit between two poses.

        Each joint is assumed to move time-optimally within its own limits (what the
        MPC does with a large position weight), so fast wrist joints finish long before
        the slow shoulder/elbow joints; a straight joint-space line misses the sweeps
        this produces, and those sweeps are what hit the torso.
        """
        profiles = []
        duration = 0.0
        for index, (start_angle, goal_angle) in enumerate(zip(start, goal)):
            position, joint_duration = self._time_optimal_profile(
                goal_angle - start_angle, self.path_acceleration[index], self.PATH_VELOCITY[index])
            profiles.append(position)
            duration = max(duration, joint_duration)
        for sample in range(samples + 1):
            elapsed = duration * sample / float(samples)
            joint_angles = [start_angle + profiles[index](elapsed) for index, start_angle in enumerate(start)]
            hit = self.reason(joint_angles)
            if hit:
                return "%.1f s into the %.1f s move: %s" % (elapsed, duration, hit)
        return None


def normalized_step_rms(previous: JointAngles, current: JointAngles) -> Tuple[float, List[float]]:
    """RMS of the per-joint steps normalised by the SAFE_RANGES width, and the steps."""
    normalized_steps = [
        abs(new - old) / (limits[1] - limits[0])
        for new, old, limits in zip(current, previous, SAFE_RANGES)
    ]
    return math.sqrt(sum(step * step for step in normalized_steps) / len(normalized_steps)), normalized_steps


def generate_reference_poses(count: int = REFERENCE_POSE_COUNT, clearance: Optional[ArmClearance] = None,
                             seed: int = REFERENCE_RANDOM_SEED) -> Tuple[Tuple[float, ...], ...]:
    """Deterministic, well-separated random poses inside SAFE_RANGES that keep the
    arm (and the transit from the previous pose) clear of the robot body when a
    clearance checker is given."""
    generator = random.Random(seed)
    poses = []  # type: List[Tuple[float, ...]]
    attempts = 0
    while len(poses) < count:
        attempts += 1
        if attempts > 200000:
            raise RuntimeError("Could not generate sufficiently separated reference poses")
        candidate = tuple(generator.uniform(*limits) for limits in SAFE_RANGES)
        if clearance is not None and not clearance.is_clear(candidate):
            continue
        if not poses:
            poses.append(candidate)
            continue
        if clearance is not None and clearance.path_reason(poses[-1], candidate) is not None:
            continue
        step_rms, normalized_steps = normalized_step_rms(poses[-1], candidate)
        large_joint_steps = sum(step >= LARGE_JOINT_STEP_FRACTION for step in normalized_steps)
        if step_rms >= MIN_NORMALIZED_STEP_RMS and large_joint_steps >= MIN_LARGE_JOINT_STEPS:
            poses.append(candidate)
    return tuple(poses)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send deterministic, body-clearance-checked joint references to the Baxter MPC in Gazebo."
    )
    parser.add_argument("--limb", choices=("left", "right"), default="left")
    parser.add_argument("--interval", type=float, default=10.0,
                        help="dwell per pose [s]")
    parser.add_argument("--report-interval", type=float, default=1.0,
                        help="seconds between TRACK log lines")
    parser.add_argument(
        "--enable-simulator",
        action="store_true",
        help="Enable Baxter after Gazebo and the MPC dynamic-reconfigure server are ready.",
    )
    parser.add_argument(
        "--repeat", action="store_true", help="Repeat the pose sequence until shutdown."
    )
    parser.add_argument("--poses", type=int, default=REFERENCE_POSE_COUNT,
                        help="Number of poses to send (first N of the deterministic sequence).")
    parser.add_argument("--seed", type=int, default=REFERENCE_RANDOM_SEED)
    parser.add_argument("--no-clearance-filter", action="store_true",
                        help="Do not reject poses that would run the arm into the body.")
    parser.add_argument("--no-home-first", action="store_true",
                        help="do not send the home (untucked) pose before the test poses")
    parser.add_argument("--z-min", type=float, default=None,
                        help="conveyor surface height in the base frame [m]; generated poses keep "
                             "the arm above it (default: the ground)")
    parser.add_argument("--scenario", default="",
                        help="Collision-guard scenario instead of random poses: "
                             "floor, torso, crossing or belt (empty: random poses).")
    return parser.parse_args(rospy.myargv()[1:])


def wait_for_simulator(timeout_sec: float = 60.0) -> None:
    """Blocks until Gazebo publishes model states and the robot publishes joint states."""
    deadline = time.monotonic() + timeout_sec
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if rospy.get_param("/use_sim_time", False):
            topics = dict(rospy.get_published_topics("/"))
            if "/gazebo/model_states" in topics and "/robot/joint_states" in topics:
                rospy.wait_for_message("/robot/joint_states", JointState, timeout=5.0)
                return
        rospy.rostime.wallsleep(0.25)
    raise RuntimeError("Gazebo and /robot/joint_states were not ready within {:.0f} s".format(timeout_sec))


def wait_for_enabled(timeout_sec: float = 60.0) -> None:
    """Blocks until /robot/state reports the robot enabled, so the first target is
    not spent while the arm is still disabled and held by the simulator."""
    deadline = time.monotonic() + timeout_sec
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            state = rospy.wait_for_message("/robot/state", AssemblyState, timeout=2.0)
        except rospy.ROSException:
            continue
        if state.enabled:
            return
        rospy.rostime.wallsleep(0.2)
    raise RuntimeError("Robot was not enabled within {:.0f} s".format(timeout_sec))


def enable_simulator() -> rospy.Publisher:
    """Publishes the latched enable; the returned publisher must stay alive."""
    publisher = rospy.Publisher("/robot/set_super_enable", Bool, queue_size=1, latch=True)
    deadline = time.monotonic() + 10.0
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        rospy.rostime.wallsleep(0.1)
    if publisher.get_num_connections() == 0:
        raise RuntimeError("Baxter simulator enable subscriber is not available")
    publisher.publish(Bool(data=True))
    rospy.loginfo("Enabled the simulated Baxter robot")
    return publisher


def validate_poses(poses: Sequence[JointAngles]) -> None:
    """Raises RuntimeError unless the poses are unique, within JOINT_LIMITS and well separated."""
    if len(set(poses)) != len(poses):
        raise RuntimeError("Reference poses must be unique")
    for pose_index, pose in enumerate(poses, start=1):
        if len(pose) != len(JOINT_LIMITS):
            raise RuntimeError("Pose {} has the wrong joint count".format(pose_index))
        for joint_index, (value, limits) in enumerate(zip(pose, JOINT_LIMITS)):
            if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
                raise RuntimeError(
                    "Pose {} joint {} is outside [{}, {}]".format(
                        pose_index, joint_index, limits[0], limits[1]
                    )
                )
    for previous, current in zip(poses, poses[1:]):
        step_rms, _ = normalized_step_rms(previous, current)
        if step_rms < MIN_NORMALIZED_STEP_RMS:
            raise RuntimeError("Adjacent reference poses are not sufficiently separated")


class JointStateCache:
    """Keeps the latest /robot/joint_states message (rospy.wait_for_message can
    block far longer than its timeout under a 1 kHz publisher)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._state = None  # type: Optional[JointState]
        self._received_at = None  # type: Optional[float]
        self._subscriber = rospy.Subscriber("/robot/joint_states", JointState, self._on_joint_state,
                                            queue_size=1)

    def _on_joint_state(self, message: JointState) -> None:
        with self._lock:
            self._state = message
            self._received_at = time.monotonic()

    def latest(self, max_age_sec: float = 1.0) -> Optional[JointState]:
        """The last message if it is younger than ``max_age_sec``, else None."""
        with self._lock:
            if self._state is None or time.monotonic() - self._received_at > max_age_sec:
                return None
            return self._state


def report_tracking(joint_states: JointStateCache, limb: str, target: JointAngles,
                    pose_index: int, sample_index: int, elapsed_sec: float) -> None:
    """Logs one TRACK line with the per-joint reference, actual, error and velocity."""
    state = joint_states.latest()
    if state is None:
        rospy.logwarn("TRACK pose=%03d sample=%03d no_joint_state", pose_index, sample_index)
        return

    positions = dict(zip(state.name, state.position))
    velocities = dict(zip(state.name, state.velocity))
    joint_names = ["{}_{}".format(limb, suffix) for suffix in JOINT_SUFFIXES]
    if any(name not in positions for name in joint_names):
        rospy.logwarn("TRACK pose=%03d sample=%03d incomplete_joint_state", pose_index, sample_index)
        return

    signed_errors = [desired - positions[name] for name, desired in zip(joint_names, target)]
    rms_error = math.sqrt(sum(error * error for error in signed_errors) / len(signed_errors))
    worst_index = max(range(len(signed_errors)), key=lambda index: abs(signed_errors[index]))
    details = " ".join(
        "{}_ref={:.4f} {}_actual={:.4f} {}_error={:.4f} {}_vel={:.4f}".format(
            suffix,
            desired,
            suffix,
            positions[name],
            suffix,
            desired - positions[name],
            suffix,
            velocities.get(name, float("nan")),
        )
        for suffix, name, desired in zip(JOINT_SUFFIXES, joint_names, target)
    )
    rospy.loginfo(
        "TRACK pose=%03d sample=%03d elapsed=%.2f rms_error=%.5f max_error=%.5f "
        "max_joint=%s %s",
        pose_index,
        sample_index,
        elapsed_sec,
        rms_error,
        abs(signed_errors[worst_index]),
        JOINT_SUFFIXES[worst_index],
        details,
    )


def monitor_pose(joint_states: JointStateCache, limb: str, target: JointAngles, pose_index: int,
                 dwell_sec: float, report_interval_sec: float) -> None:
    """Reports the tracking of ``target`` every ``report_interval_sec`` for ``dwell_sec``."""
    started_at = time.monotonic()
    deadline = started_at + dwell_sec
    sample_index = 1
    while not rospy.is_shutdown():
        now = time.monotonic()
        if now >= deadline:
            return
        report_tracking(joint_states, limb, target, pose_index, sample_index, now - started_at)
        sample_index += 1
        remaining = max(0.0, deadline - time.monotonic())
        rospy.rostime.wallsleep(min(report_interval_sec, remaining))


def build_pose_sequence(args: argparse.Namespace, scenario: str) -> Tuple[Tuple[Tuple[float, ...], ...], bool]:
    """Returns (poses to send, whether the clearance filter was used)."""
    if scenario:
        poses = tuple(mirror_pose(pose) if args.limb == "right" else pose for pose in SCENARIO_POSES[scenario])
        rospy.logwarn("SCENARIO %s: %d deliberately unsafe target(s); the MPC guard must stop the arm",
                      scenario, len(poses))
        return poses, False

    clearance = None
    if not args.no_clearance_filter:
        urdf_xml = rospy.get_param("/robot_description", "")
        if not urdf_xml:
            raise RuntimeError("/robot_description is required for the clearance filter")
        acceleration_limits = rospy.get_param("/robot/acceleration_max", None)
        if not (isinstance(acceleration_limits, list) and len(acceleration_limits) == len(JOINT_SUFFIXES)):
            acceleration_limits = None
        clearance = ArmClearance(urdf_xml, args.limb, args.z_min, acceleration_limits)
    poses = generate_reference_poses(args.poses, clearance, args.seed)
    validate_poses(poses)
    return poses, clearance is not None


def main() -> None:
    rospy.init_node("mpc_sim_reference_sender", anonymous=True)
    args = parse_args()
    if not math.isfinite(args.interval) or args.interval <= 0.0:
        raise RuntimeError("--interval must be finite and positive")
    if not math.isfinite(args.report_interval) or args.report_interval <= 0.0:
        raise RuntimeError("--report-interval must be finite and positive")
    if args.report_interval > args.interval:
        raise RuntimeError("--report-interval must not exceed --interval")
    if args.poses <= 0:
        raise RuntimeError("--poses must be positive")
    scenario = args.scenario.strip()
    if scenario and scenario not in SCENARIO_POSES:
        raise RuntimeError("Unknown scenario '{}' (expected floor, torso, crossing or belt)".format(scenario))
    wait_for_simulator()
    joint_states = JointStateCache()
    poses, clearance_filtered = build_pose_sequence(args, scenario)

    if not args.no_home_first:
        home = mirror_pose(HOME_POSE) if args.limb == "right" else HOME_POSE
        poses = (home,) + tuple(poses)

    server = "/mpc_controller_node_{}".format(args.limb)
    client = Client(server, timeout=30.0)
    # The latched enable stays valid only while its publisher lives: keep it until main returns.
    enable_publisher = enable_simulator() if args.enable_simulator else None  # noqa: F841
    wait_for_enabled()
    rospy.rostime.wallsleep(1.0)  # let the torque loop take over before the first target
    rospy.loginfo(
        "TUNING_RUN poses=%d seed=%d clearance_filter=%s scenario=%s limb=%s server=%s dwell=%.1f report_interval=%.1f",
        len(poses),
        args.seed,
        clearance_filtered,
        scenario or "none",
        args.limb,
        server,
        args.interval,
        args.report_interval,
    )

    while not rospy.is_shutdown():
        for pose_index, pose in enumerate(poses, start=1):
            if rospy.is_shutdown():
                return
            configuration = {"ref_{}".format(index): value for index, value in enumerate(pose)}
            client.update_configuration(configuration)
            # Parsed by summarize_control_diagnostics.py: keep "values=<tuple repr>".
            rospy.loginfo("TARGET pose=%03d/%03d values=%s", pose_index, len(poses), pose)
            monitor_pose(joint_states, args.limb, pose, pose_index, args.interval, args.report_interval)
        if not args.repeat:
            return


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException) as exception:
        rospy.logfatal("Reference sender stopped: %s", exception)
        raise SystemExit(1)
