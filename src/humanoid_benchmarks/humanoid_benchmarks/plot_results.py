#!/usr/bin/env python3
"""Plots a log_recorder_node CSV with matplotlib: joint positions over time
plus a shaded band for each safety-state segment. Saves a PNG next to the
CSV rather than popping a window, so it works headless over SSH/CI too.

Usage:
    python3 -m humanoid_benchmarks.plot_results /tmp/humanoid_lab_log.csv
"""
import argparse
import csv
import pathlib
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv_path")
    ap.add_argument("--out", default=None, help="output PNG path (default: alongside the CSV)")
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is required: pip install matplotlib", file=sys.stderr)
        return 1

    rows = list(csv.DictReader(open(args.csv_path, newline="")))
    if not rows:
        print("empty log", file=sys.stderr)
        return 1

    t0 = float(rows[0]["stamp_sec"])
    t = [float(r["stamp_sec"]) - t0 for r in rows]
    pos_fields = [k for k in rows[0].keys() if k.startswith("pos_")]

    fig, ax = plt.subplots(figsize=(10, 5))
    for field in pos_fields:
        ax.plot(t, [float(r[field]) for r in rows], label=field.replace("pos_", ""), linewidth=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("joint position (rad)")
    ax.set_title("Joint positions over time")
    ax.legend(fontsize=6, ncol=2, loc="upper right")
    fig.tight_layout()

    out_path = pathlib.Path(args.out) if args.out else pathlib.Path(args.csv_path).with_suffix(".png")
    fig.savefig(out_path, dpi=150)
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
