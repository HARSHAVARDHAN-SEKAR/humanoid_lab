#!/usr/bin/env python3
"""Stage 6D: multi-direction free-floating push benchmark."""

import csv
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from humanoid_motion_controller.command_model import (
    CommandComposer,
    JOINT_NAMES,
)

MODEL_PATH = "/models/humanoid_biped.xml"
OUTPUT_DIR = Path("/results")

DURATION = 8.0
PUSH_START = 2.0
PUSH_END = 2.1
SAMPLE_PERIOD = 0.05


def convex_hull(points):
    points = sorted(set(
        (round(float(p[0]), 7), round(float(p[1]), 7))
        for p in points
    ))

    def cross(a, b, c):
        return (
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0])
        )

    if len(points) < 3:
        return []

    lower = []
    upper = []

    for p in points:
        while len(lower) >= 2 and cross(
            lower[-2], lower[-1], p
        ) <= 0:
            lower.pop()
        lower.append(p)

    for p in reversed(points):
        while len(upper) >= 2 and cross(
            upper[-2], upper[-1], p
        ) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def support_margin(com_xy, polygon):
    if len(polygon) < 3:
        return None

    distances = []

    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]

        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.hypot(dx, dy)

        if length > 1e-10:
            distance = (
                dx * (com_xy[1] - a[1])
                - dy * (com_xy[0] - a[0])
            ) / length
            distances.append(distance)

    return min(distances) if distances else None


def simulate(force_x, force_y):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    assert model.nq == 19, "Expected free-floating biped"

    def get_id(kind, name):
        idx = mujoco.mj_name2id(model, kind, name)
        if idx < 0:
            raise RuntimeError(f"Missing object: {name}")
        return idx

    torso = get_id(
        mujoco.mjtObj.mjOBJ_BODY, "torso"
    )

    floor = get_id(
        mujoco.mjtObj.mjOBJ_GEOM, "floor"
    )

    feet = {
        get_id(mujoco.mjtObj.mjOBJ_GEOM, "left_foot_geom"),
        get_id(mujoco.mjtObj.mjOBJ_GEOM, "right_foot_geom"),
    }

    targets = CommandComposer().standing_command()
    joint_positions = {}

    data.qpos[:7] = [0, 0, 0.58, 1, 0, 0, 0]

    for joint, target in targets.items():
        jid = get_id(
            mujoco.mjtObj.mjOBJ_JOINT, joint
        )
        aid = get_id(
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            joint.replace("_joint", "_act")
        )

        joint_positions[joint] = model.jnt_qposadr[jid]
        data.qpos[joint_positions[joint]] = target
        data.ctrl[aid] = target

    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    steps = round(DURATION / dt)
    sample_every = max(1, round(SAMPLE_PERIOD / dt))

    rows = []
    previous_com = None
    previous_time = None

    for step in range(steps + 1):
        if step % sample_every == 0:
            com = (
                model.body_mass[:, None] * data.xipos
            ).sum(axis=0) / model.body_mass.sum()

            if previous_com is None:
                speed = 0.0
            else:
                elapsed = data.time - previous_time
                speed = float(np.linalg.norm(
                    (com[:2] - previous_com) / elapsed
                ))

            previous_com = com[:2].copy()
            previous_time = float(data.time)

            rotation = data.xmat[torso].reshape(3, 3)
            tilt = math.degrees(math.acos(
                float(np.clip(rotation[2, 2], -1, 1))
            ))

            points = []

            for i in range(data.ncon):
                contact = data.contact[i]

                a = int(contact.geom1)
                b = int(contact.geom2)

                if floor not in (a, b):
                    continue

                other = b if a == floor else a

                if other not in feet or contact.efc_address < 0:
                    continue

                wrench = np.zeros(6)
                mujoco.mj_contactForce(
                    model, data, i, wrench
                )

                if wrench[0] > 0.1:
                    points.append(contact.pos[:2])

            polygon = convex_hull(points)
            margin = support_margin(com[:2], polygon)

            joint_error = max(
                abs(targets[j] - data.qpos[joint_positions[j]])
                for j in JOINT_NAMES
            )

            rows.append({
                "time_s": round(float(data.time), 4),
                "com_x": float(com[0]),
                "com_y": float(com[1]),
                "com_speed": speed,
                "height": float(data.xpos[torso, 2]),
                "tilt": tilt,
                "margin": margin,
                "joint_error": joint_error,
            })

        if step == steps:
            break

        data.xfrc_applied[torso, :] = 0.0

        if PUSH_START <= data.time < PUSH_END:
            data.xfrc_applied[torso, 0] = force_x
            data.xfrc_applied[torso, 1] = force_y

        mujoco.mj_step(model, data)

    return rows


def evaluate(name, rows, baseline):
    post = [
        (r, b)
        for r, b in zip(rows, baseline)
        if r["time_s"] >= PUSH_END
    ]

    deviations = [
        math.hypot(
            r["com_x"] - b["com_x"],
            r["com_y"] - b["com_y"]
        )
        for r, b in post
    ]

    margins = [
        r["margin"] for r, _ in post
        if r["margin"] is not None
    ]

    min_margin = min(margins) if margins else None
    valid_contacts = len(margins) == len(post)

    max_tilt = max(r["tilt"] for r, _ in post)
    min_height = min(r["height"] for r, _ in post)
    max_error = max(r["joint_error"] for r, _ in post)

    final_speed = rows[-1]["com_speed"]
    final_deviation = deviations[-1]
    peak_deviation = max(deviations)

    passed = (
        valid_contacts
        and min_margin is not None
        and min_margin >= 0
        and min_height > 0.48
        and max_tilt < 10.0
        and max_error < 0.700
        and final_speed < 0.03
        and final_deviation < 0.03
    )

    return {
        "test": name,
        "peak_com_deviation_m": peak_deviation,
        "final_com_deviation_m": final_deviation,
        "final_com_speed_mps": final_speed,
        "max_torso_tilt_deg": max_tilt,
        "min_torso_height_m": min_height,
        "min_support_margin_m": min_margin,
        "max_joint_error_rad": max_error,
        "valid_contact_polygons": valid_contacts,
        "diagnostic_pass": passed,
    }


def save_csv(path, rows):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=list(rows[0].keys())
        )
        writer.writeheader()
        writer.writerows(rows)


OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

print("STAGE 6D — MULTI-DIRECTION PUSH TEST")
print("Free-floating biped")
print("Push duration: 0.1 seconds")
print("No active CoM feedback controller")
print()

print("Running baseline...")
baseline = simulate(0.0, 0.0)

cases = []

for force in (20.0, 40.0):
    cases.extend([
        (f"{int(force)}N_forward", force, 0.0),
        (f"{int(force)}N_backward", -force, 0.0),
        (f"{int(force)}N_left", 0.0, force),
        (f"{int(force)}N_right", 0.0, -force),
    ])

results = []

for name, fx, fy in cases:
    print("Running:", name, flush=True)

    rows = simulate(fx, fy)

    result = evaluate(name, rows, baseline)
    results.append(result)

    save_csv(
        OUTPUT_DIR / f"stage6d_{name}.csv",
        rows
    )

print("\n========== STAGE 6D RESULT ==========")
print(
    f"{'TEST':18s} {'PEAK CoM':>9s} "
    f"{'TILT':>8s} {'MARGIN':>9s} {'RESULT':>8s}"
)

for r in results:
    margin = r["min_support_margin_m"]
    margin_text = (
        f"{margin:.4f}"
        if margin is not None else "N/A"
    )

    print(
        f"{r['test']:18s} "
        f"{r['peak_com_deviation_m']:9.4f} "
        f"{r['max_torso_tilt_deg']:8.2f} "
        f"{margin_text:>9s} "
        f"{'PASS' if r['diagnostic_pass'] else 'FAIL':>8s}"
    )

passed = sum(
    1 for r in results if r["diagnostic_pass"]
)

with open(
    OUTPUT_DIR / "stage6d_push_matrix_summary.json", "w"
) as f:
    json.dump({
        "model": "free-floating simplified 12-DOF biped",
        "controller": "constant joint-position references",
        "push_duration_s": PUSH_END - PUSH_START,
        "results": results,
        "passed": passed,
        "total": len(results),
    }, f, indent=2)

print()
print(f"Diagnostic checks passed: {passed}/{len(results)}")
print("STAGE 6D COMPLETE")
