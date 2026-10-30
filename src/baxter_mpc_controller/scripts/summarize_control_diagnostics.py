#!/usr/bin/env python3
"""Summarise a baxter_control_diagnostics_v1 JSON-lines file written by
record_control_diagnostics.py: per-pose settling, overshoot and final error,
solver failures, fallbacks, torque saturation, guard clearances and loop timing.

    rosrun baxter_mpc_controller summarize_control_diagnostics.py run.txt [--settle-tol 0.02]

Poses are delimited by the reference sender's TARGET events; a pose's window runs
from its TARGET until the next one (or the last sample). All times are the
recorder's ros_time [s], angles are [rad], clearances [m].
"""
import argparse
import json
import math
import statistics
from collections import defaultdict
from typing import Dict, List, Optional, Sequence, Tuple

#: A joint slower than this at the end of the window counts as at rest [rad/s].
REST_VELOCITY_RAD_S = 0.05
#: raw vs commanded torque differing by more than this means the limit clipped [Nm].
SATURATION_EPSILON_NM = 1e-6
GUARD_GROUPS = ("floor", "body", "arm")


def _finite(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("path")
    parser.add_argument("--settle-tol", type=float, default=0.02,
                        help="|error| threshold in rad for 'settled' [default 0.02]")
    parser.add_argument("--per-pose", action="store_true", help="print one line per pose")
    parser.add_argument("--expect-blocked", action="store_true",
                        help="poses are expected to stop short of the target (collision guard); "
                             "report whether the arm came to rest with positive clearance instead "
                             "of whether it converged")
    parser.add_argument("--clearance-tol", type=float, default=-0.005,
                        help="minimum acceptable measured clearance in metres [default -0.005]")
    return parser.parse_args()


def load(path: str) -> Tuple[List[str], List[dict], List[dict]]:
    """Returns (joint order, sample records, event records) from a JSON-lines file."""
    joint_names, samples, events = None, [], []
    with open(path) as handle:
        for line in handle:
            try:
                record = json.loads(line)
            except ValueError:
                continue
            kind = record.get("type")
            if kind == "metadata":
                joint_names = record["joint_order"]
            elif kind == "sample":
                samples.append(record)
            elif kind == "event":
                events.append(record)
    if joint_names is None:
        raise SystemExit("no metadata record found")
    return joint_names, samples, events


def field(sample: dict, *keys: str):
    """Nested lookup into sample["data"]; None when any level is missing."""
    value = sample.get("data")
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def targets_from_events(events: Sequence[dict]) -> List[Tuple[float, List[float]]]:
    """(ros_time, joint reference) per TARGET line of the reference sender."""
    targets = []
    for event in events:
        if event["message"].startswith("TARGET"):
            values_text = event["message"].split("values=")[1]
            targets.append((event["ros_time"],
                            [float(text) for text in values_text.strip("() \n").split(",")]))
    return targets


def median(values: Sequence[float]) -> float:
    """Median (mean of the two middle values for an even count); NaN when empty."""
    return statistics.median(values) if values else float("nan")


def percentile_95(values: Sequence[float]) -> float:
    """Nearest-rank 95th percentile of a non-empty sequence."""
    ordered = sorted(values)
    return ordered[int(0.95 * (len(ordered) - 1))]


def count_events(events: Sequence[dict]) -> Dict[str, int]:
    """Counts the controller warnings/errors by category."""
    counts = defaultdict(int)  # type: Dict[str, int]
    for event in events:
        text = event["message"]
        if text.startswith("MPC missed"):
            counts["deadline_miss"] += 1
        elif "MPC solve failed" in text:
            counts["solve_failed"] += 1
            # A rejected (infeasible) solution is reported as a solve failure; count it separately too.
            if "violates constraints" in text:
                counts["infeasible_solution"] += 1
        elif "braking fallback" in text:
            counts["fallback_warn"] += 1
        elif "stale" in text:
            counts["stale_reference"] += 1
        elif "saturated" in text:
            counts["torque_saturated_warn"] += 1
        elif "violates constraints" in text:
            counts["infeasible_solution"] += 1
        elif "guard" in text.lower():
            counts["guard_warn"] += 1
    return counts


def settle_time(errors: Sequence[Tuple[float, List[float]]], tolerance: float) -> Optional[float]:
    """Time [s] from which every later max |error| stays below ``tolerance``, or None."""
    last_violation = None
    for index, (_, joint_errors) in enumerate(errors):
        if max(abs(error) for error in joint_errors) >= tolerance:
            last_violation = index
    if last_violation is None:
        return errors[0][0]
    if last_violation + 1 < len(errors):
        return errors[last_violation + 1][0]
    return None


def overshoot(errors: Sequence[Tuple[float, List[float]]]) -> float:
    """Largest |error| [rad] of any joint after its error changed sign (crossed the target)."""
    initial = errors[0][1]
    worst = 0.0
    for _, joint_errors in errors:
        for initial_error, error in zip(initial, joint_errors):
            if initial_error * error < 0:
                worst = max(worst, abs(error))
    return worst


def min_measured_clearance(geometry: Sequence[dict]) -> Optional[float]:
    """Smallest finite min_now_* clearance [m] over the geometry samples (None: group inactive)."""
    minima = []
    for record in geometry:
        finite_values = [record["min_now_" + group] for group in GUARD_GROUPS
                         if _finite(record["min_now_" + group])]
        if finite_values:
            minima.append(min(finite_values))
    return min(minima) if minima else None


def summarize_pose(pose_number: int, reference: List[float], window: List[dict], window_start: float,
                   window_end: float, joint_names: List[str], settle_tolerance: float) -> Optional[dict]:
    """Per-pose statistics over the samples in [window_start, window_end), or None without data."""
    errors = []  # (time since target [s], per-joint error [rad])
    for sample in window:
        position = field(sample, "joint_state", "position")
        if any(position.get(joint) is None for joint in joint_names):
            continue
        errors.append((sample["ros_time"] - window_start,
                       [reference[index] - position[joint] for index, joint in enumerate(joint_names)]))
    if not errors:
        return None
    final_errors = errors[-1][1]
    rms_final = math.sqrt(sum(error * error for error in final_errors) / len(final_errors))
    worst_joint = max(range(len(final_errors)), key=lambda index: abs(final_errors[index]))

    peak_velocity = 0.0
    for sample in window:
        velocities = [abs(velocity) for velocity in field(sample, "joint_state", "velocity").values()
                      if velocity is not None]
        if velocities:
            peak_velocity = max(peak_velocity, max(velocities))

    saturated_joint_samples = 0
    for sample in window:
        kdl = field(sample, "kdl")
        if kdl:
            saturated_joint_samples += sum(
                abs(kdl["raw_torque"][joint] - kdl["commanded_torque"][joint]) > SATURATION_EPSILON_NM
                for joint in joint_names)

    geometry = [field(sample, "geometry") for sample in window if field(sample, "geometry")]
    final_velocities = field(window[-1], "joint_state", "velocity")
    at_rest = all(abs(velocity) < REST_VELOCITY_RAD_S for velocity in final_velocities.values()
                  if velocity is not None)
    return {
        "pose": pose_number, "duration": window_end - window_start, "rms_final": rms_final,
        "max_final": abs(final_errors[worst_joint]), "worst_joint": joint_names[worst_joint][-2:],
        "settle": settle_time(errors, settle_tolerance), "overshoot": overshoot(errors),
        "peak_velocity": peak_velocity, "saturated": saturated_joint_samples,
        "min_clearance": min_measured_clearance(geometry), "at_rest": at_rest,
    }


def summarize_poses(targets, samples, joint_names, settle_tolerance) -> List[dict]:
    rows = []
    for index, (window_start, reference) in enumerate(targets):
        window_end = targets[index + 1][0] if index + 1 < len(targets) else samples[-1]["ros_time"]
        window = [sample for sample in samples
                  if window_start <= sample["ros_time"] < window_end and field(sample, "joint_state", "position")]
        if not window:
            continue
        row = summarize_pose(index + 1, reference, window, window_start, window_end, joint_names, settle_tolerance)
        if row is not None:
            rows.append(row)
    return rows


def print_pose_table(rows: Sequence[dict]) -> None:
    print("\npose  dwell  rms_final  max_final  joint  settle(s)  overshoot  peak|v|  saturated  clearance  rest")
    for row in rows:
        print("%4d %6.1f  %8.4f  %8.4f  %5s  %8s  %8.4f  %6.2f  %5d  %9s  %4s" % (
            row["pose"], row["duration"], row["rms_final"], row["max_final"], row["worst_joint"],
            ("%.2f" % row["settle"]) if row["settle"] is not None else "none",
            row["overshoot"], row["peak_velocity"], row["saturated"],
            ("%.3f" % row["min_clearance"]) if row["min_clearance"] is not None else "n/a",
            "yes" if row["at_rest"] else "no"))


def print_pose_summary(rows: Sequence[dict], args: argparse.Namespace) -> None:
    settled = [row["settle"] for row in rows if row["settle"] is not None]
    print("\nsettled within %.3f rad: %d / %d" % (args.settle_tol, len(settled), len(rows)))
    print("median settle time  : %.2f s" % median(settled))
    print("mean final rms error: %.4f rad" % (sum(row["rms_final"] for row in rows) / len(rows)))
    print("max final error     : %.4f rad (pose %d, %s)" % max(
        (row["max_final"], row["pose"], row["worst_joint"]) for row in rows))
    print("max overshoot       : %.4f rad" % max(row["overshoot"] for row in rows))
    print("torque saturations  : %d joint-samples" % sum(row["saturated"] for row in rows))
    if args.expect_blocked:
        blocked_ok = [row for row in rows if row["at_rest"] and row["min_clearance"] is not None
                      and row["min_clearance"] >= args.clearance_tol]
        print("blocked poses OK    : %d / %d (at rest with clearance >= %.3f m)" % (
            len(blocked_ok), len(rows), args.clearance_tol))


def print_geometry_summary(geometry: Sequence[dict]) -> None:
    print("\nGeometry guard: %d samples" % len(geometry))
    for group in GUARD_GROUPS:
        measured = [record["min_now_" + group] for record in geometry if _finite(record["min_now_" + group])]
        predicted = [record["min_predicted_" + group] for record in geometry
                     if _finite(record["min_predicted_" + group])]
        if measured:
            print("  %-5s min measured clearance %.3f m, min predicted %.3f m" % (
                group, min(measured), min(predicted) if predicted else float("nan")))
    max_slack = [record["max_slack"] for record in geometry if _finite(record["max_slack"])]
    print("  max active rows %d, max slack %.4f, guard rejections %d, linearize median %.2f ms, guard median %.2f ms" % (
        max(record["active_rows"] for record in geometry), max(max_slack) if max_slack else float("nan"),
        max(record["guard_rejections"] for record in geometry),
        median([record["linearize_ms"] for record in geometry]), median([record["guard_ms"] for record in geometry])))


def print_kdl_summary(kdl_samples: Sequence[dict], joint_names: Sequence[str]) -> None:
    loop_dt_ms = [record["loop_dt_ms"] for record in kdl_samples]
    print("\nKDL torque loop: %d samples, loop dt median %.2f ms (p95 %.2f, max %.2f), compute median %.3f ms" % (
        len(kdl_samples), median(loop_dt_ms), percentile_95(loop_dt_ms), max(loop_dt_ms),
        median([record["compute_ms"] for record in kdl_samples])))
    print("max |torque| per joint: " + "  ".join(
        "%s=%.1f" % (joint[-2:], max(abs(record["commanded_torque"][joint]) for record in kdl_samples))
        for joint in joint_names))
    print("max |PD accel| per joint: " + "  ".join(
        "%s=%.2f" % (joint[-2:], max(abs(record["pd_acceleration"][joint]) for record in kdl_samples))
        for joint in joint_names))


def main() -> None:
    args = parse_args()
    joint_names, samples, events = load(args.path)
    if not samples:
        raise SystemExit("no sample records found")
    targets = targets_from_events(events)
    counts = count_events(events)
    rows = summarize_poses(targets, samples, joint_names, args.settle_tol)

    print("file: %s" % args.path)
    print("samples=%d events=%d poses=%d" % (len(samples), len(events), len(rows)))
    if counts:
        print("events: " + ", ".join("%s=%d" % item for item in sorted(counts.items())))
    else:
        print("events: none")
    if args.per_pose:
        print_pose_table(rows)
    if rows:
        print_pose_summary(rows, args)

    geometry = [field(sample, "geometry") for sample in samples if field(sample, "geometry")]
    if geometry:
        print_geometry_summary(geometry)

    kdl_samples = [field(sample, "kdl") for sample in samples if field(sample, "kdl")]
    if kdl_samples:
        print_kdl_summary(kdl_samples, joint_names)

    rates = defaultdict(list)  # type: Dict[str, List[float]]
    for sample in samples:
        for source, rate in sample["rates_hz"].items():
            rates[source].append(rate)
    if rates:
        print("\ntopic rates (median Hz): " + "  ".join(
            "%s=%.1f" % (source, median(values)) for source, values in sorted(rates.items())))

    timing_lines = [event["message"] for event in events if "backend=" in event["message"]]
    if timing_lines:
        print("\nlast MPC timing line: " + timing_lines[-1])


if __name__ == "__main__":
    main()
