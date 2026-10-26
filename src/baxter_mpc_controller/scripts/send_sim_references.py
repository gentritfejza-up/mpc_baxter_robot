#!/usr/bin/env python3

import argparse
import math
import random
import threading
import time
import xml.etree.ElementTree as ET

import rospy
from baxter_core_msgs.msg import AssemblyState
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


JOINT_SUFFIXES = ("s0", "s1", "e0", "e1", "w0", "w1", "w2")
JOINT_LIMITS = (
    (-1.701679, 1.701679),
    (-2.146754, 1.0472),
    (-3.054326, 3.054326),
    (-0.049868, 2.617994),
    (-3.059, 3.059),
    (-1.5708, 2.0944),
    (-3.059, 3.059),
)

# Keep the tuning workload away from hard joint limits. Limit-recovery behavior
# must be evaluated separately from nominal tracking performance.
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
MIN_NORMALIZED_STEP_RMS = 0.45
MIN_LARGE_JOINT_STEPS = 4


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _transform(xyz, rpy):
    rot = _rpy_matrix(*rpy)
    return [rot[0] + [xyz[0]], rot[1] + [xyz[1]], rot[2] + [xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _rot_z(angle):
    return _transform((0.0, 0.0, 0.0), (0.0, 0.0, angle))


def _apply(t, point):
    return [sum(t[i][k] * (point + (1.0,))[k] for k in range(4)) for i in range(3)]


class ArmClearance:
    """Rejects arm poses whose links would run into Baxter's own body."""

    # Axis-aligned keep-out boxes in the torso frame (metres), ~6 cm link-radius margin.
    # Torso + head + pedestal column, extended backwards (nothing reachable behind the robot).
    TORSO_BOX = ((-0.60, 0.33), (-0.30, 0.30), (-1.3, 1.05))
    # The other arm parked in the simulator start pose (hanging forward-down).
    OTHER_ARM_BOX = ((0.0, 0.65), (-0.75, -0.25), (-0.6, 0.25))
    FLOOR_Z = -0.80          # torso origin is 0.93 m above the ground plane

    def __init__(self, urdf_xml, limb):
        root = ET.fromstring(urdf_xml)
        joints = {j.get("name"): j for j in root.findall("joint")}

        def origin(joint_name):
            el = joints[joint_name].find("origin")
            xyz = tuple(float(v) for v in (el.get("xyz", "0 0 0")).split())
            rpy = tuple(float(v) for v in (el.get("rpy", "0 0 0")).split())
            return _transform(xyz, rpy)

        self._limb = limb
        self._mount = origin("{}_torso_arm_mount".format(limb))
        self._joint_origins = [origin("{}_{}".format(limb, s)) for s in JOINT_SUFFIXES]
        self._hand = origin("{}_hand".format(limb))
        self._sign = 1.0 if limb == "left" else -1.0

    def sample_points(self, angles):
        """Points along the arm in the torso frame: joint origins from the elbow
        roll on, link midpoints, the hand and the gripper tip."""
        t = self._mount
        frames = []
        for origin_t, angle in zip(self._joint_origins, angles):
            t = _matmul(_matmul(t, origin_t), _rot_z(angle))
            frames.append(t)
        upper_elbow, lower_elbow, upper_forearm, lower_forearm, wrist = (
            frames[2], frames[3], frames[4], frames[5], frames[6])
        hand = _matmul(wrist, self._hand)
        points = [
            _apply(upper_elbow, (0.0, 0.0, 0.13)),   # upper arm midpoint
            _apply(lower_elbow, (0.0, 0.0, 0.0)),    # elbow
            _apply(upper_forearm, (0.0, 0.0, 0.135)),  # forearm midpoint
            _apply(lower_forearm, (0.0, 0.0, 0.0)),  # wrist pitch
            _apply(wrist, (0.0, 0.0, 0.0)),          # wrist roll
            _apply(hand, (0.0, 0.0, 0.0)),           # hand
            _apply(hand, (0.0, 0.0, 0.12)),          # gripper tip
        ]
        return points

    @staticmethod
    def _inside(box, point):
        return all(low < value < high for (low, high), value in zip(box, point))

    def reason(self, angles):
        """None when the pose is clear, otherwise a short description of the hit."""
        for x, y, z in self.sample_points(angles):
            point = (x, self._sign * y, z)  # mirror so the right arm uses the same boxes
            if self._inside(self.TORSO_BOX, point):
                return "torso/head at (%.2f, %.2f, %.2f)" % (x, y, z)
            if self._inside(self.OTHER_ARM_BOX, point):
                return "other arm at (%.2f, %.2f, %.2f)" % (x, y, z)
            if z < self.FLOOR_Z:
                return "floor at (%.2f, %.2f, %.2f)" % (x, y, z)
        return None

    def is_clear(self, angles):
        return self.reason(angles) is None

    # Per-joint limits used to predict the transit path (MPC defaults; the
    # acceleration limits are overridden from /robot/acceleration_max when set).
    PATH_ACCELERATION = [2.0, 5.0, 2.0, 5.0, 6.0, 6.0, 6.0]
    PATH_VELOCITY = [1.2, 1.2, 1.2, 1.2, 2.8, 2.8, 2.8]

    @staticmethod
    def _time_optimal_profile(distance, acceleration, velocity):
        """1-D bang-bang (trapezoidal / triangular) move: returns (position(t), duration)."""
        span = abs(distance)
        sign = 1.0 if distance >= 0.0 else -1.0
        if span < velocity * velocity / acceleration:
            peak = math.sqrt(span / acceleration)
            duration = 2.0 * peak

            def position(t):
                if t <= peak:
                    x = 0.5 * acceleration * t * t
                elif t <= duration:
                    x = span - 0.5 * acceleration * (duration - t) ** 2
                else:
                    x = span
                return sign * x
        else:
            ramp = velocity / acceleration
            duration = 2.0 * ramp + (span - velocity * velocity / acceleration) / velocity

            def position(t):
                if t <= ramp:
                    x = 0.5 * acceleration * t * t
                elif t <= duration - ramp:
                    x = 0.5 * velocity * ramp + velocity * (t - ramp)
                elif t <= duration:
                    x = span - 0.5 * acceleration * (duration - t) ** 2
                else:
                    x = span
                return sign * x
        return position, duration

    def path_reason(self, start, goal, samples=30):
        """Checks the transit between two poses. Each joint is assumed to move
        time-optimally within its own limits (what the MPC does with a large
        position weight), so fast wrist joints finish long before the slow
        shoulder/elbow joints; a straight joint-space line misses the sweeps
        this produces, and those sweeps are what hit the torso."""
        profiles = []
        duration = 0.0
        for index, (a, b) in enumerate(zip(start, goal)):
            position, joint_duration = self._time_optimal_profile(
                b - a, self.PATH_ACCELERATION[index], self.PATH_VELOCITY[index])
            profiles.append(position)
            duration = max(duration, joint_duration)
        for k in range(samples + 1):
            t = duration * k / float(samples)
            angles = [a + profiles[index](t) for index, a in enumerate(start)]
            why = self.reason(angles)
            if why:
                return "%.1f s into the %.1f s move: %s" % (t, duration, why)
        return None


def generate_reference_poses(count=REFERENCE_POSE_COUNT, clearance=None, seed=REFERENCE_RANDOM_SEED):
    """Deterministic, well-separated random poses inside SAFE_RANGES that keep the
    arm clear of the robot body (when a clearance checker is given)."""
    generator = random.Random(seed)
    poses = []
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

        normalized_steps = [
            abs(current - previous) / (limits[1] - limits[0])
            for current, previous, limits in zip(candidate, poses[-1], SAFE_RANGES)
        ]
        step_rms = math.sqrt(
            sum(step * step for step in normalized_steps) / len(normalized_steps)
        )
        large_joint_steps = sum(step >= 0.30 for step in normalized_steps)
        if step_rms >= MIN_NORMALIZED_STEP_RMS and large_joint_steps >= MIN_LARGE_JOINT_STEPS:
            poses.append(candidate)
    return tuple(poses)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send deterministic, body-clearance-checked joint references to the Baxter MPC in Gazebo."
    )
    parser.add_argument("--limb", choices=("left", "right"), default="left")
    parser.add_argument("--interval", type=float, default=10.0)
    parser.add_argument("--report-interval", type=float, default=1.0)
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
    return parser.parse_args(rospy.myargv()[1:])


def wait_for_simulator(timeout_sec=60.0):
    deadline = time.monotonic() + timeout_sec
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if rospy.get_param("/use_sim_time", False):
            topics = dict(rospy.get_published_topics("/"))
            if "/gazebo/model_states" in topics and "/robot/joint_states" in topics:
                rospy.wait_for_message("/robot/joint_states", JointState, timeout=5.0)
                return
        rospy.rostime.wallsleep(0.25)
    raise RuntimeError("Gazebo and /robot/joint_states were not ready within {:.0f} s".format(timeout_sec))


def wait_for_enabled(timeout_sec=60.0):
    """Block until /robot/state reports the robot enabled, so the first target is
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


def enable_simulator():
    publisher = rospy.Publisher("/robot/set_super_enable", Bool, queue_size=1, latch=True)
    deadline = time.monotonic() + 10.0
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        rospy.rostime.wallsleep(0.1)
    if publisher.get_num_connections() == 0:
        raise RuntimeError("Baxter simulator enable subscriber is not available")
    publisher.publish(Bool(data=True))
    rospy.loginfo("Enabled the simulated Baxter robot")
    return publisher


def validate_poses(poses):
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
        normalized_steps = [
            abs(value - old_value) / (limits[1] - limits[0])
            for value, old_value, limits in zip(current, previous, SAFE_RANGES)
        ]
        step_rms = math.sqrt(
            sum(step * step for step in normalized_steps) / len(normalized_steps)
        )
        if step_rms < MIN_NORMALIZED_STEP_RMS:
            raise RuntimeError("Adjacent reference poses are not sufficiently separated")


class JointStateCache:
    """Keeps the latest /robot/joint_states message (rospy.wait_for_message can
    block far longer than its timeout under a 1 kHz publisher)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._state = None
        self._received_at = None
        self._subscriber = rospy.Subscriber("/robot/joint_states", JointState, self._callback,
                                            queue_size=1)

    def _callback(self, message):
        with self._lock:
            self._state = message
            self._received_at = time.monotonic()

    def latest(self, max_age_sec=1.0):
        with self._lock:
            if self._state is None or time.monotonic() - self._received_at > max_age_sec:
                return None
            return self._state


JOINT_STATE_CACHE = None


def report_tracking(limb, target, pose_index, sample_index, elapsed_sec):
    state = JOINT_STATE_CACHE.latest() if JOINT_STATE_CACHE is not None else None
    if state is None:
        rospy.logwarn("TRACK pose=%03d sample=%03d no_joint_state", pose_index, sample_index)
        return

    positions = dict(zip(state.name, state.position))
    velocities = dict(zip(state.name, state.velocity))
    names = ["{}_{}".format(limb, suffix) for suffix in JOINT_SUFFIXES]
    if any(name not in positions for name in names):
        rospy.logwarn("TRACK pose=%03d sample=%03d incomplete_joint_state", pose_index, sample_index)
        return

    signed_errors = [desired - positions[name] for name, desired in zip(names, target)]
    rms_error = math.sqrt(sum(error * error for error in signed_errors) / len(signed_errors))
    max_index = max(range(len(signed_errors)), key=lambda index: abs(signed_errors[index]))
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
        for suffix, name, desired in zip(JOINT_SUFFIXES, names, target)
    )
    rospy.loginfo(
        "TRACK pose=%03d sample=%03d elapsed=%.2f rms_error=%.5f max_error=%.5f "
        "max_joint=%s %s",
        pose_index,
        sample_index,
        elapsed_sec,
        rms_error,
        abs(signed_errors[max_index]),
        JOINT_SUFFIXES[max_index],
        details,
    )


def monitor_pose(limb, target, pose_index, dwell_sec, report_interval_sec):
    started_at = time.monotonic()
    deadline = started_at + dwell_sec
    sample_index = 1
    while not rospy.is_shutdown():
        now = time.monotonic()
        if now >= deadline:
            return
        report_tracking(limb, target, pose_index, sample_index, now - started_at)
        sample_index += 1
        remaining = max(0.0, deadline - time.monotonic())
        rospy.rostime.wallsleep(min(report_interval_sec, remaining))


def main():
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
    wait_for_simulator()
    global JOINT_STATE_CACHE
    JOINT_STATE_CACHE = JointStateCache()
    clearance = None
    if not args.no_clearance_filter:
        urdf_xml = rospy.get_param("/robot_description", "")
        if not urdf_xml:
            raise RuntimeError("/robot_description is required for the clearance filter")
        clearance = ArmClearance(urdf_xml, args.limb)
        limits = rospy.get_param("/robot/acceleration_max", None)
        if isinstance(limits, list) and len(limits) == len(JOINT_SUFFIXES):
            clearance.PATH_ACCELERATION = [abs(float(v)) for v in limits]
    poses = generate_reference_poses(args.poses, clearance, args.seed)
    validate_poses(poses)

    server = "/mpc_controller_node_{}".format(args.limb)
    client = Client(server, timeout=30.0)
    enable_publisher = enable_simulator() if args.enable_simulator else None
    wait_for_enabled()
    rospy.rostime.wallsleep(1.0)  # let the torque loop take over before the first target
    rospy.loginfo(
        "TUNING_RUN poses=%d seed=%d clearance_filter=%s limb=%s server=%s dwell=%.1f report_interval=%.1f",
        len(poses),
        args.seed,
        clearance is not None,
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
            rospy.loginfo("TARGET pose=%03d/%03d values=%s", pose_index, len(poses), pose)
            monitor_pose(args.limb, pose, pose_index, args.interval, args.report_interval)
        if not args.repeat:
            _ = enable_publisher
            return


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException) as exception:
        rospy.logfatal("Reference sender stopped: %s", exception)
        raise SystemExit(1)
