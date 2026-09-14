#!/usr/bin/env python3

import argparse
import math
import random
import time

import rospy
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


def generate_reference_poses():
    generator = random.Random(REFERENCE_RANDOM_SEED)
    poses = []
    attempts = 0
    while len(poses) < REFERENCE_POSE_COUNT:
        attempts += 1
        if attempts > 100000:
            raise RuntimeError("Could not generate sufficiently separated reference poses")
        candidate = tuple(generator.uniform(*limits) for limits in SAFE_RANGES)
        if not poses:
            poses.append(candidate)
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


REFERENCE_POSES = generate_reference_poses()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send 100 deterministic joint references to the Baxter MPC in Gazebo."
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
        "--repeat", action="store_true", help="Repeat the 100-pose sequence until shutdown."
    )
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


def validate_poses():
    if len(REFERENCE_POSES) != REFERENCE_POSE_COUNT:
        raise RuntimeError("Expected {} reference poses".format(REFERENCE_POSE_COUNT))
    if len(set(REFERENCE_POSES)) != REFERENCE_POSE_COUNT:
        raise RuntimeError("Reference poses must be unique")
    for pose_index, pose in enumerate(REFERENCE_POSES, start=1):
        if len(pose) != len(JOINT_LIMITS):
            raise RuntimeError("Pose {} has the wrong joint count".format(pose_index))
        for joint_index, (value, limits) in enumerate(zip(pose, JOINT_LIMITS)):
            if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
                raise RuntimeError(
                    "Pose {} joint {} is outside [{}, {}]".format(
                        pose_index, joint_index, limits[0], limits[1]
                    )
                )
    for previous, current in zip(REFERENCE_POSES, REFERENCE_POSES[1:]):
        normalized_steps = [
            abs(value - old_value) / (limits[1] - limits[0])
            for value, old_value, limits in zip(current, previous, SAFE_RANGES)
        ]
        step_rms = math.sqrt(
            sum(step * step for step in normalized_steps) / len(normalized_steps)
        )
        if step_rms < MIN_NORMALIZED_STEP_RMS:
            raise RuntimeError("Adjacent reference poses are not sufficiently separated")


def report_tracking(limb, target, pose_index, sample_index, elapsed_sec):
    try:
        state = rospy.wait_for_message("/robot/joint_states", JointState, timeout=2.0)
    except rospy.ROSException:
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
    wait_for_simulator()
    validate_poses()

    server = "/mpc_controller_node_{}".format(args.limb)
    client = Client(server, timeout=30.0)
    enable_publisher = enable_simulator() if args.enable_simulator else None
    rospy.loginfo(
        "TUNING_RUN poses=%d seed=%d limb=%s server=%s dwell=%.1f report_interval=%.1f",
        len(REFERENCE_POSES),
        REFERENCE_RANDOM_SEED,
        args.limb,
        server,
        args.interval,
        args.report_interval,
    )

    while not rospy.is_shutdown():
        for pose_index, pose in enumerate(REFERENCE_POSES, start=1):
            if rospy.is_shutdown():
                return
            configuration = {"ref_{}".format(index): value for index, value in enumerate(pose)}
            client.update_configuration(configuration)
            rospy.loginfo("TARGET pose=%03d/%03d values=%s", pose_index, len(REFERENCE_POSES), pose)
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
