#!/usr/bin/env python3
"""Summarise a baxter_control_diagnostics_v1 JSON-lines file written by
record_control_diagnostics.py: per-pose settling, overshoot and final error,
solver failures, fallbacks, torque saturation and loop timing.

    rosrun baxter_mpc_controller summarize_control_diagnostics.py run.txt [--settle-tol 0.02]
"""
import argparse
import json
import math
from collections import defaultdict


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("path")
    parser.add_argument("--settle-tol", type=float, default=0.02,
                        help="|error| threshold in rad for 'settled' [default 0.02]")
    parser.add_argument("--per-pose", action="store_true", help="print one line per pose")
    return parser.parse_args()


def load(path):
    joints, samples, events = None, [], []
    with open(path) as handle:
        for line in handle:
            try:
                record = json.loads(line)
            except ValueError:
                continue
            kind = record.get("type")
            if kind == "metadata":
                joints = record["joint_order"]
            elif kind == "sample":
                samples.append(record)
            elif kind == "event":
                events.append(record)
    if joints is None:
        raise SystemExit("no metadata record found")
    return joints, samples, events


def field(sample, *keys):
    value = sample.get("data")
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def targets_from_events(events):
    targets = []
    for event in events:
        if event["message"].startswith("TARGET"):
            values = event["message"].split("values=")[1]
            targets.append((event["ros_time"],
                            [float(x) for x in values.strip("() \n").split(",")]))
    return targets


def median(values):
    values = sorted(values)
    return values[len(values) // 2] if values else float("nan")


def main():
    args = parse_args()
    joints, samples, events = load(args.path)
    targets = targets_from_events(events)

    counts = defaultdict(int)
    for event in events:
        text = event["message"]
        if text.startswith("MPC missed"):
            counts["deadline_miss"] += 1
        elif "MPC solve failed" in text:
            counts["solve_failed"] += 1
        elif "braking fallback" in text:
            counts["fallback_warn"] += 1
        elif "stale" in text:
            counts["stale_reference"] += 1
        elif "saturated" in text:
            counts["torque_saturated_warn"] += 1
        elif "violates constraints" in text:
            counts["infeasible_solution"] += 1

    rows = []
    for index, (t0, pose) in enumerate(targets):
        t1 = targets[index + 1][0] if index + 1 < len(targets) else samples[-1]["ros_time"]
        window = [s for s in samples if t0 <= s["ros_time"] < t1 and field(s, "joint_state", "position")]
        if not window:
            continue
        errors = []
        for sample in window:
            position = field(sample, "joint_state", "position")
            errors.append((sample["ros_time"] - t0,
                           [pose[i] - position[j] for i, j in enumerate(joints)]))
        final = errors[-1][1]
        rms_final = math.sqrt(sum(e * e for e in final) / len(final))
        worst = max(range(len(final)), key=lambda i: abs(final[i]))
        settle = None
        for k in range(len(errors)):
            if all(max(abs(e) for e in errors[m][1]) < args.settle_tol for m in range(k, len(errors))):
                settle = errors[k][0]
                break
        initial = errors[0][1]
        overshoot = 0.0
        for _, e in errors:
            for i in range(len(e)):
                if initial[i] * e[i] < 0:
                    overshoot = max(overshoot, abs(e[i]))
        peak_velocity = max(max(abs(v) for v in field(s, "joint_state", "velocity").values()
                                if v is not None) for s in window)
        saturated = 0
        for sample in window:
            kdl = field(sample, "kdl")
            if kdl:
                saturated += sum(abs(kdl["raw_torque"][j] - kdl["commanded_torque"][j]) > 1e-6
                                 for j in joints)
        rows.append({
            "pose": index + 1, "duration": t1 - t0, "rms_final": rms_final,
            "max_final": abs(final[worst]), "worst_joint": joints[worst][-2:],
            "settle": settle, "overshoot": overshoot, "peak_velocity": peak_velocity,
            "saturated": saturated,
        })

    print("file: %s" % args.path)
    print("samples=%d events=%d poses=%d" % (len(samples), len(events), len(rows)))
    print("events: " + ", ".join("%s=%d" % kv for kv in sorted(counts.items())) if counts else "events: none")
    if args.per_pose:
        print("\npose  dwell  rms_final  max_final  joint  settle(s)  overshoot  peak|v|  saturated")
        for r in rows:
            print("%4d %6.1f  %8.4f  %8.4f  %5s  %8s  %8.4f  %6.2f  %5d" % (
                r["pose"], r["duration"], r["rms_final"], r["max_final"], r["worst_joint"],
                ("%.2f" % r["settle"]) if r["settle"] is not None else "none",
                r["overshoot"], r["peak_velocity"], r["saturated"]))
    if rows:
        settled = [r["settle"] for r in rows if r["settle"] is not None]
        print("\nsettled within %.3f rad: %d / %d" % (args.settle_tol, len(settled), len(rows)))
        print("median settle time  : %.2f s" % median(settled))
        print("mean final rms error: %.4f rad" % (sum(r["rms_final"] for r in rows) / len(rows)))
        print("max final error     : %.4f rad (pose %d, %s)" % max(
            (r["max_final"], r["pose"], r["worst_joint"]) for r in rows))
        print("max overshoot       : %.4f rad" % max(r["overshoot"] for r in rows))
        print("torque saturations  : %d joint-samples" % sum(r["saturated"] for r in rows))

    kdl_samples = [field(s, "kdl") for s in samples if field(s, "kdl")]
    if kdl_samples:
        print("\nKDL torque loop: %d samples, loop dt median %.2f ms (p95 %.2f, max %.2f), compute median %.3f ms" % (
            len(kdl_samples), median([k["loop_dt_ms"] for k in kdl_samples]),
            sorted(k["loop_dt_ms"] for k in kdl_samples)[int(0.95 * (len(kdl_samples) - 1))],
            max(k["loop_dt_ms"] for k in kdl_samples), median([k["compute_ms"] for k in kdl_samples])))
        print("max |torque| per joint: " + "  ".join(
            "%s=%.1f" % (j[-2:], max(abs(k["commanded_torque"][j]) for k in kdl_samples)) for j in joints))
        print("max |PD accel| per joint: " + "  ".join(
            "%s=%.2f" % (j[-2:], max(abs(k["pd_acceleration"][j]) for k in kdl_samples)) for j in joints))

    rates = defaultdict(list)
    for sample in samples:
        for key, value in sample["rates_hz"].items():
            rates[key].append(value)
    if rates:
        print("\ntopic rates (median Hz): " + "  ".join(
            "%s=%.1f" % (k, median(v)) for k, v in sorted(rates.items())))

    for event in events:
        if "backend=" in event["message"]:
            last = event["message"]
    else:
        last = None
    if last:
        print("\nlast MPC timing line: " + last)


if __name__ == "__main__":
    main()
