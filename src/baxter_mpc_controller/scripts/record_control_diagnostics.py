#!/usr/bin/env python3

import argparse
import copy
import json
import math
import os
import threading
import time

import rospy
from baxter_core_msgs.msg import AccelerationCommand, JointCommand, ReferentJointAngles, SEAJointState
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_SUFFIXES = ("s0", "s1", "e0", "e1", "w0", "w1", "w2")
KDL_FIELDS = (
    "position",
    "velocity",
    "reference_position",
    "reference_velocity",
    "mpc_acceleration",
    "pd_acceleration",
    "commanded_acceleration",
    "inertia_torque",
    "coriolis_torque",
    "gravity_torque",
    "damping_torque",
    "raw_torque",
    "commanded_torque",
    "torque_limit",
)


def parse_args():
    parser = argparse.ArgumentParser(description="Record synchronized Baxter MPC/KDL diagnostics")
    parser.add_argument("--limb", choices=("left", "right"), default="left")
    parser.add_argument("--output", required=True)
    parser.add_argument("--rate", type=float, default=20.0)
    return parser.parse_args(rospy.myargv()[1:])


def indexed(names, values):
    return {name: values[index] for index, name in enumerate(names) if index < len(values)}


class Recorder:
    def __init__(self, limb, output_path, sample_rate):
        if not math.isfinite(sample_rate) or sample_rate <= 0.0 or sample_rate > 200.0:
            raise RuntimeError("diagnostics rate must be in (0, 200] Hz")
        self._limb = limb
        self._joints = ["{}_{}".format(limb, suffix) for suffix in JOINT_SUFFIXES]
        self._lock = threading.Lock()
        self._latest = {}
        self._received_at = {}
        self._counts = {}
        self._rates = {}
        self._rate_started = time.monotonic()
        self._previous_joint_velocity = None
        self._previous_joint_stamp = None
        parent = os.path.dirname(os.path.abspath(output_path))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        self._file = open(output_path, "w", buffering=1)
        self._write({
            "type": "metadata",
            "format": "baxter_control_diagnostics_v1",
            "wall_time": time.time(),
            "limb": limb,
            "joint_order": self._joints,
            "sample_rate_hz": sample_rate,
            "parameters": {
                "control": rospy.get_param("/robot/control", {}),
                "kdl_dynamics": rospy.get_param("/robot/kdl_dynamics", {}),
                "solver": rospy.get_param("/robot/solver", {}),
                "solver_mode": rospy.get_param("/robot/solver_mode", None),
                "solver_backend": rospy.get_param("/robot/solver_backend", None),
                "acceleration_max": rospy.get_param("/robot/acceleration_max", []),
                "acceleration_min": rospy.get_param("/robot/acceleration_min", []),
                "jerk_max": rospy.get_param("/robot/jerk_max", []),
                "jerk_min": rospy.get_param("/robot/jerk_min", []),
            },
        })

        prefix = "/robot/limb/{}/".format(limb)
        rospy.Subscriber("/robot/joint_states", JointState, self._joint_state, queue_size=10)
        rospy.Subscriber(prefix + "acceleration_command", AccelerationCommand,
                         self._mpc_command, queue_size=10)
        rospy.Subscriber(prefix + "joint_command", JointCommand, self._torque_command,
                         queue_size=100)
        rospy.Subscriber(prefix + "referent_joint_angles", ReferentJointAngles,
                         self._target, queue_size=10)
        rospy.Subscriber(prefix + "gravity_compensation_torques", SEAJointState,
                         self._sea_state, queue_size=10)
        rospy.Subscriber(prefix + "kdl_dynamics_diagnostics", Float64MultiArray,
                         self._kdl, queue_size=10)
        rospy.Subscriber("/rosout", Log, self._rosout, queue_size=100)
        self._timer = rospy.Timer(rospy.Duration(1.0 / sample_rate), self._sample)

    def _write(self, record):
        self._file.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")

    def _store(self, source, value):
        now = time.monotonic()
        with self._lock:
            self._latest[source] = value
            self._received_at[source] = now
            self._counts[source] = self._counts.get(source, 0) + 1

    def _joint_state(self, message):
        position = indexed(message.name, message.position)
        velocity = indexed(message.name, message.velocity)
        effort = indexed(message.name, message.effort)
        stamp = message.header.stamp.to_sec()
        estimated_acceleration = {}
        if self._previous_joint_velocity is not None and self._previous_joint_stamp is not None:
            dt = stamp - self._previous_joint_stamp
            if dt > 0.0:
                estimated_acceleration = {
                    joint: (velocity[joint] - self._previous_joint_velocity[joint]) / dt
                    for joint in self._joints
                    if joint in velocity and joint in self._previous_joint_velocity
                }
        self._previous_joint_velocity = velocity
        self._previous_joint_stamp = stamp
        self._store("joint_state", {
            "position": {joint: position.get(joint) for joint in self._joints},
            "velocity": {joint: velocity.get(joint) for joint in self._joints},
            "estimated_acceleration": estimated_acceleration,
            "effort": {joint: effort.get(joint) for joint in self._joints},
            "stamp": stamp,
        })

    def _mpc_command(self, message):
        self._store("mpc_command", {
            "acceleration": indexed(message.joint_names, message.accelerations),
            "predicted_velocity": indexed(message.joint_names, message.velocities),
            "predicted_position": indexed(message.joint_names, message.angles),
        })

    def _torque_command(self, message):
        self._store("joint_command", {
            "mode": message.mode,
            "command": indexed(message.names, message.command),
        })

    def _target(self, message):
        self._store("target", indexed(message.joint_names, message.angles))

    def _sea_state(self, message):
        self._store("sea_state", {
            "commanded_effort": indexed(message.name, message.commanded_effort),
            "actual_effort": indexed(message.name, message.actual_effort),
            "gravity_model_effort": indexed(message.name, message.gravity_model_effort),
            "gravity_only": indexed(message.name, message.gravity_only),
            "stamp": message.header.stamp.to_sec(),
        })

    def _kdl(self, message):
        count = len(self._joints)
        metadata_count = 5
        expected = metadata_count + len(KDL_FIELDS) * count
        if len(message.data) != expected:
            rospy.logwarn_throttle(1.0, "Ignoring malformed KDL diagnostics: %d != %d",
                                   len(message.data), expected)
            return
        values = message.data
        result = {
            "sequence": int(values[0]),
            "loop_dt_ms": values[1],
            "compute_ms": values[2],
            "acceleration_reference_age_sec": values[3],
            "torque_control_active": bool(values[4]),
        }
        offset = metadata_count
        for field in KDL_FIELDS:
            result[field] = dict(zip(self._joints, values[offset:offset + count]))
            offset += count
        self._store("kdl", result)

    def _rosout(self, message):
        name = message.name.lower()
        text = message.msg.lower()
        relevant = ("mpc" in name or "kdl" in name or "solver" in text or
                    "stale" in text or "saturat" in text or "deadline" in text)
        if not relevant:
            return
        with self._lock:
            self._write({
                "type": "event",
                "wall_time": time.time(),
                "ros_time": rospy.Time.now().to_sec(),
                "node": message.name,
                "level": message.level,
                "message": message.msg,
            })

    def _sample(self, _event):
        monotonic_now = time.monotonic()
        with self._lock:
            elapsed = monotonic_now - self._rate_started
            if elapsed >= 1.0:
                for source, count in self._counts.items():
                    self._rates[source] = count / elapsed
                self._counts = {}
                self._rate_started = monotonic_now
            ages = {source: monotonic_now - stamp for source, stamp in self._received_at.items()}
            self._write({
                "type": "sample",
                "wall_time": time.time(),
                "ros_time": rospy.Time.now().to_sec(),
                "rates_hz": copy.deepcopy(self._rates),
                "ages_sec": ages,
                "data": copy.deepcopy(self._latest),
            })

    def close(self):
        with self._lock:
            if not self._file.closed:
                self._write({"type": "end", "wall_time": time.time()})
                self._file.close()


def main():
    rospy.init_node("baxter_control_diagnostics_recorder")
    args = parse_args()
    recorder = Recorder(args.limb, args.output, args.rate)
    rospy.on_shutdown(recorder.close)
    rospy.loginfo("Recording synchronized control diagnostics to %s at %.1f Hz",
                  args.output, args.rate)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, rospy.ROSException) as exception:
        rospy.logfatal("Diagnostics recorder stopped: %s", exception)
        raise SystemExit(1)
