#!/usr/bin/env python3
"""Offline metrics computation from a log_recorder_node CSV -- the numbers
the HUMANOID-LAB proposal's "validation dashboard" asks for. Pure
pandas/numpy, no ROS dependency, so it's independently testable.

Usage:
    python3 -m humanoid_benchmarks.metrics /tmp/humanoid_lab_log.csv
"""
import argparse
import sys

import numpy as np


def standing_duration_s(timestamps, safety_states, stand_label="STAND"):
    """Longest unbroken run of `stand_label` in safety_states, in seconds."""
    if len(timestamps) == 0:
        return 0.0
    best = 0.0
    run_start = None
    for i, s in enumerate(safety_states):
        if s == stand_label:
            if run_start is None:
                run_start = timestamps[i]
        else:
            if run_start is not None:
                best = max(best, timestamps[i] - run_start)
                run_start = None
    if run_start is not None:
        best = max(best, timestamps[-1] - run_start)
    return float(best)


def physical_standing_duration_s(timestamps, torso_heights, pitches,
                                 minimum_height_m=0.46, maximum_abs_pitch_rad=0.35):
    """Longest continuous physically upright run based on measured pose.

    Distinct from STAND FSM status; a faulted or fallen robot must never
    be counted as standing solely because a supervisor says STAND.
    """
    import math
    valid = [math.isfinite(h) and math.isfinite(p) and h >= minimum_height_m
             and abs(p) <= maximum_abs_pitch_rad
             for h, p in zip(torso_heights, pitches)]
    return standing_duration_s(timestamps,
                               ["UPRIGHT" if ok else "DOWN" for ok in valid],
                               stand_label="UPRIGHT")


def joint_tracking_rms(commanded, actual):
    """RMS tracking error (rad) between two equal-length sequences."""
    commanded = np.asarray(commanded, dtype=float)
    actual = np.asarray(actual, dtype=float)
    if commanded.shape != actual.shape:
        raise ValueError("commanded/actual must be the same shape")
    return float(np.sqrt(np.mean((commanded - actual) ** 2)))


def push_recovery_rate(outcomes):
    """outcomes: iterable of bool (True = recovered). Returns success rate in [0, 1]."""
    outcomes = list(outcomes)
    if not outcomes:
        return 0.0
    return float(sum(1 for o in outcomes if o) / len(outcomes))


def summarize_csv(path):
    import csv as csv_mod
    timestamps, safety_states = [], []
    heights, pitches = [], []
    errors = []
    with open(path, newline="") as f:
        reader = csv_mod.DictReader(f)
        for row in reader:
            timestamps.append(float(row["stamp_sec"]))
            safety_states.append(row.get("safety_state", ""))
            try:
                heights.append(float(row.get("torso_height_m", "nan")))
                pitches.append(float(row.get("torso_pitch_rad", "nan")))
            except ValueError:
                heights.append(float('nan'))
                pitches.append(float('nan'))
            for key, cmd in row.items():
                if key.startswith("cmd_") and cmd not in ("", None):
                    actual = row.get("pos_" + key[4:])
                    if actual not in ("", None):
                        errors.append((float(cmd)-float(actual))**2)
    return {
        "num_rows": len(timestamps),
        "duration_s": (timestamps[-1] - timestamps[0]) if timestamps else 0.0,
        "longest_stand_state_run_s": standing_duration_s(timestamps, safety_states),
        "longest_physically_upright_run_s": physical_standing_duration_s(timestamps, heights, pitches),
        "joint_tracking_rms_rad": float(np.sqrt(np.mean(errors))) if errors else None,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv_path")
    args = ap.parse_args()
    summary = summarize_csv(args.csv_path)
    for k, v in summary.items():
        print(f"{k}: {v}")


if __name__ == "__main__":
    sys.exit(main())
