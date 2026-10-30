#!/usr/bin/env python3
"""Record synchronized Baxter MPC/KDL diagnostics to a JSON-lines file.

Format ``baxter_control_diagnostics_v1``, one JSON object per line:

* ``metadata`` (first line): limb, joint order, sample rate and the controller
  parameters on the parameter server at start.
* ``sample`` at ``--rate`` Hz (a ROS timer, i.e. simulation time when
  /use_sim_time is set): the latest message of every subscribed source under
  ``data``, each source's arrival rate over the last second under ``rates_hz``
  (wall clock) and the wall-clock age of each source under ``ages_sec``.
* ``event``: controller-related /rosout lines (MPC, KDL, solver, guard, stale,
  saturation, deadline), including the reference sender's TARGET/TRACK lines.
* ``end`` on shutdown.

Every record carries ``wall_time`` (time.time) and the samples and events also
``ros_time`` (rospy.Time.now). Non-finite numbers (an inactive guard group reports
+inf) are stored as null, since JSON cannot carry them. Read the result with
summarize_control_diagnostics.py.
"""

import argparse
import copy
import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional, Sequence

import rospy
from baxter_core_msgs.msg import AccelerationCommand, JointCommand, ReferentJointAngles, SEAJointState
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


#: Baxter joint name suffixes in controller order.
JOINT_SUFFIXES = ("s0", "s1", "e0", "e1", "w0", "w1", "w2")
#: Layout of /robot/limb/<limb>/mpc_geometry_diagnostics (mpc_controller_node.cpp).
GEOMETRY_FIELDS = (
    "sequence",
    "min_predicted_floor",
    "min_predicted_body",
    "min_predicted_arm",
    "min_now_floor",
    "min_now_body",
    "min_now_arm",
    "active_rows",
    "max_slack",
    "guard_rejections",
    "linearize_ms",
    "guard_ms",
)
#: Per-joint blocks of /robot/limb/<limb>/kdl_dynamics_diagnostics after the
#: KDL_METADATA_COUNT scalars (kdl_inverse_dynamics_node.cpp, layout "v1").
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
KDL_METADATA_COUNT = 5
#: Sample rates above this are refused [Hz].
MAX_SAMPLE_RATE_HZ = 200.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record synchronized Baxter MPC/KDL diagnostics")
    parser.add_argument("--limb", choices=("left", "right"), default="left")
    parser.add_argument("--output", required=True, help="output JSON-lines file")
    parser.add_argument("--rate", type=float, default=20.0, help="sample rate [Hz]")
    return parser.parse_args(rospy.myargv()[1:])


def indexed(names: Sequence[str], values: Sequence[float]) -> Dict[str, float]:
    """Maps parallel name/value arrays to a dict, ignoring names without a value."""
    return {name: values[index] for index, name in enumerate(names) if index < len(values)}


def _json_safe(value: Any) -> Any:
    """Recursively replaces non-finite floats with None (JSON has no NaN/inf)."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class Recorder:
    """Subscribes to the control topics of one limb and writes the JSON-lines log.

    Subscriber callbacks store the latest message per source; a ROS timer writes a
    sample at the requested rate. A lock serialises the sample writer, the /rosout
    event writer and the shutdown close so nothing is written to a closed file.
    """

    def __init__(self, limb: str, output_path: str, sample_rate_hz: float):
        if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0 or sample_rate_hz > MAX_SAMPLE_RATE_HZ:
            raise RuntimeError("diagnostics rate must be in (0, %g] Hz" % MAX_SAMPLE_RATE_HZ)
        self._limb = limb
        self._joint_names = ["{}_{}".format(limb, suffix) for suffix in JOINT_SUFFIXES]
        self._lock = threading.Lock()
        self._latest = {}  # type: Dict[str, Any]          # source -> latest decoded message
        self._received_at = {}  # type: Dict[str, float]   # source -> monotonic arrival time
        self._counts = {}  # type: Dict[str, int]          # source -> messages in the current rate window
        self._rates_hz = {}  # type: Dict[str, float]      # source -> rate over the last window
        self._rate_window_started = time.monotonic()
        self._previous_joint_velocity = None  # type: Optional[Dict[str, float]]
        self._previous_joint_stamp = None  # type: Optional[float]

        parent_directory = os.path.dirname(os.path.abspath(output_path))
        if parent_directory and not os.path.isdir(parent_directory):
            os.makedirs(parent_directory)
        self._file = open(output_path, "w", buffering=1)
        self._write({
            "type": "metadata",
            "format": "baxter_control_diagnostics_v1",
            "wall_time": time.time(),
            "limb": limb,
            "joint_order": self._joint_names,
            "sample_rate_hz": sample_rate_hz,
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
                "geometry": rospy.get_param("/robot/geometry", {}),
            },
        })

        topic_prefix = "/robot/limb/{}/".format(limb)
        rospy.Subscriber("/robot/joint_states", JointState, self._on_joint_state, queue_size=10)
        rospy.Subscriber(topic_prefix + "acceleration_command", AccelerationCommand,
                         self._on_mpc_command, queue_size=10)
        rospy.Subscriber(topic_prefix + "joint_command", JointCommand, self._on_torque_command,
                         queue_size=100)
        rospy.Subscriber(topic_prefix + "referent_joint_angles", ReferentJointAngles,
                         self._on_target, queue_size=10)
        rospy.Subscriber(topic_prefix + "gravity_compensation_torques", SEAJointState,
                         self._on_sea_state, queue_size=10)
        rospy.Subscriber(topic_prefix + "kdl_dynamics_diagnostics", Float64MultiArray,
                         self._on_kdl_diagnostics, queue_size=10)
        rospy.Subscriber(topic_prefix + "mpc_geometry_diagnostics", Float64MultiArray,
                         self._on_geometry_diagnostics, queue_size=10)
        rospy.Subscriber("/rosout", Log, self._on_rosout, queue_size=100)
        self._timer = rospy.Timer(rospy.Duration(1.0 / sample_rate_hz), self._write_sample)

    # ---- output

    def _write(self, record: dict) -> None:
        """Appends one record; callers hold the lock (except the metadata write)."""
        if self._file.closed:  # callbacks still fire while roslaunch tears the node down
            return
        self._file.write(json.dumps(_json_safe(record), separators=(",", ":"), allow_nan=False) + "\n")

    def _store(self, source: str, value: Any) -> None:
        now = time.monotonic()
        with self._lock:
            self._latest[source] = value
            self._received_at[source] = now
            self._counts[source] = self._counts.get(source, 0) + 1

    # ---- subscribers

    def _on_joint_state(self, message: JointState) -> None:
        position = indexed(message.name, message.position)
        velocity = indexed(message.name, message.velocity)
        effort = indexed(message.name, message.effort)
        stamp = message.header.stamp.to_sec()
        estimated_acceleration = {}
        if self._previous_joint_velocity is not None and self._previous_joint_stamp is not None:
            step_sec = stamp - self._previous_joint_stamp
            if step_sec > 0.0:
                estimated_acceleration = {
                    joint: (velocity[joint] - self._previous_joint_velocity[joint]) / step_sec
                    for joint in self._joint_names
                    if joint in velocity and joint in self._previous_joint_velocity
                }
        self._previous_joint_velocity = velocity
        self._previous_joint_stamp = stamp
        self._store("joint_state", {
            "position": {joint: position.get(joint) for joint in self._joint_names},
            "velocity": {joint: velocity.get(joint) for joint in self._joint_names},
            "estimated_acceleration": estimated_acceleration,
            "effort": {joint: effort.get(joint) for joint in self._joint_names},
            "stamp": stamp,
        })

    def _on_mpc_command(self, message: AccelerationCommand) -> None:
        self._store("mpc_command", {
            "acceleration": indexed(message.joint_names, message.accelerations),
            "predicted_velocity": indexed(message.joint_names, message.velocities),
            "predicted_position": indexed(message.joint_names, message.angles),
        })

    def _on_torque_command(self, message: JointCommand) -> None:
        self._store("joint_command", {
            "mode": message.mode,
            "command": indexed(message.names, message.command),
        })

    def _on_target(self, message: ReferentJointAngles) -> None:
        self._store("target", indexed(message.joint_names, message.angles))

    def _on_sea_state(self, message: SEAJointState) -> None:
        self._store("sea_state", {
            "commanded_effort": indexed(message.name, message.commanded_effort),
            "actual_effort": indexed(message.name, message.actual_effort),
            "gravity_model_effort": indexed(message.name, message.gravity_model_effort),
            "gravity_only": indexed(message.name, message.gravity_only),
            "stamp": message.header.stamp.to_sec(),
        })

    def _on_kdl_diagnostics(self, message: Float64MultiArray) -> None:
        joint_count = len(self._joint_names)
        expected_length = KDL_METADATA_COUNT + len(KDL_FIELDS) * joint_count
        if len(message.data) != expected_length:
            rospy.logwarn_throttle(1.0, "Ignoring malformed KDL diagnostics: %d != %d",
                                   len(message.data), expected_length)
            return
        values = message.data
        decoded = {
            "sequence": int(values[0]),
            "loop_dt_ms": values[1],
            "compute_ms": values[2],
            "acceleration_reference_age_sec": values[3],
            "torque_control_active": bool(values[4]),
        }
        offset = KDL_METADATA_COUNT
        for field_name in KDL_FIELDS:
            decoded[field_name] = dict(zip(self._joint_names, values[offset:offset + joint_count]))
            offset += joint_count
        self._store("kdl", decoded)

    def _on_geometry_diagnostics(self, message: Float64MultiArray) -> None:
        if len(message.data) != len(GEOMETRY_FIELDS):
            rospy.logwarn_throttle(1.0, "Ignoring malformed geometry diagnostics: %d != %d",
                                   len(message.data), len(GEOMETRY_FIELDS))
            return
        # An inactive group (e.g. the floor before the arm reached home) reports +inf: stored as None.
        self._store("geometry", {name: (value if math.isfinite(value) else None)
                                 for name, value in zip(GEOMETRY_FIELDS, message.data)})

    def _on_rosout(self, message: Log) -> None:
        node_name = message.name.lower()
        text = message.msg.lower()
        relevant = ("mpc" in node_name or "kdl" in node_name or "solver" in text or
                    "stale" in text or "saturat" in text or "deadline" in text or
                    "guard" in text)
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

    # ---- periodic sample

    def _write_sample(self, _event) -> None:
        now = time.monotonic()
        with self._lock:
            window_sec = now - self._rate_window_started
            if window_sec >= 1.0:
                for source, count in self._counts.items():
                    self._rates_hz[source] = count / window_sec
                self._counts = {}
                self._rate_window_started = now
            ages_sec = {source: now - received_at for source, received_at in self._received_at.items()}
            self._write({
                "type": "sample",
                "wall_time": time.time(),
                "ros_time": rospy.Time.now().to_sec(),
                "rates_hz": copy.deepcopy(self._rates_hz),
                "ages_sec": ages_sec,
                "data": copy.deepcopy(self._latest),
            })

    def close(self) -> None:
        """Writes the end record and closes the file (idempotent; called on shutdown)."""
        with self._lock:
            if not self._file.closed:
                self._write({"type": "end", "wall_time": time.time()})
                self._file.close()


def main() -> None:
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
