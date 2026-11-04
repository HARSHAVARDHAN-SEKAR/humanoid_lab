#!/usr/bin/env python3
"""Stage 6F: negative CoM-feedback balance regression."""

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
OUT = Path("/results")
OUT.mkdir(parents=True, exist_ok=True)

DURATION = 8.0
PUSH_START = 2.0
PUSH_END = 2.1

KP = 2.5
KD = 0.35
MAX_CORRECTION = 0.12
MAX_RATE = 1.0

CONTROL_PERIOD = 0.01
SAMPLE_PERIOD = 0.02


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


def margin_to_polygon(point, polygon):
    if len(polygon) < 3:
        return None

    values = []

    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]

        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.hypot(dx, dy)

        if length > 1e-9:
            values.append(
                (
                    dx * (point[1] - a[1])
                    - dy * (point[0] - a[0])
                ) / length
            )

    return min(values) if values else None


def simulate(name, fx, fy):

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    assert model.nq == 19, "Expected free-floating model"

    def get_id(kind, name):
        result = mujoco.mj_name2id(model, kind, name)
        if result < 0:
            raise RuntimeError("Missing object: " + name)
        return result

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

    data.qpos[:7] = [0, 0, 0.58, 1, 0, 0, 0]

    qpos_addresses = {}
    actuator_ids = {}

    for joint in JOINT_NAMES:
        jid = get_id(mujoco.mjtObj.mjOBJ_JOINT, joint)
        aid = get_id(
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            joint.replace("_joint", "_act")
        )

        qpos_addresses[joint] = model.jnt_qposadr[jid]
        actuator_ids[joint] = aid

        data.qpos[qpos_addresses[joint]] = targets[joint]
        data.ctrl[aid] = targets[joint]

    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    total_steps = round(DURATION / dt)
    control_steps = max(1, round(CONTROL_PERIOD / dt))
    sample_steps = max(1, round(SAMPLE_PERIOD / dt))

    def center_of_mass():
        return (
            model.body_mass[:, None] * data.xipos
        ).sum(axis=0) / model.body_mass.sum()

    previous_x = None
    previous_time = None
    com_vx = 0.0
    reference_x = None
    correction = 0.0
    rows = []

    ankle_joints = (
        "left_ankle_pitch_joint",
        "right_ankle_pitch_joint",
    )

    for step in range(total_steps + 1):
        t = float(data.time)

        if step % control_steps == 0:
            com = center_of_mass()
            x = float(com[0])

            if previous_x is not None:
                elapsed = t - previous_time
                if elapsed > 0:
                    com_vx = (x - previous_x) / elapsed

            previous_x = x
            previous_time = t

            if reference_x is None and t >= 1.5:
                reference_x = x

            desired = 0.0

            if reference_x is not None:
                desired = -(KP * (x - reference_x)
                            + KD * com_vx)

                desired = float(np.clip(
                    desired,
                    -MAX_CORRECTION,
                    MAX_CORRECTION
                ))

            max_change = MAX_RATE * CONTROL_PERIOD
            correction += float(np.clip(
                desired - correction,
                -max_change,
                max_change
            ))

            for joint in ankle_joints:
                data.ctrl[actuator_ids[joint]] = (
                    targets[joint] + correction
                )

        if step % sample_steps == 0:
            com = center_of_mass()

            rotation = data.xmat[torso].reshape(3, 3)
            tilt = math.degrees(math.acos(
                float(np.clip(rotation[2, 2], -1, 1))
            ))

            contact_points = []

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
                    contact_points.append(contact.pos[:2])

            polygon = convex_hull(contact_points)
            margin = margin_to_polygon(com[:2], polygon)

            error = max(
                abs(
                    data.ctrl[actuator_ids[j]]
                    - data.qpos[qpos_addresses[j]]
                )
                for j in JOINT_NAMES
            )

            rows.append({
                "time_s": round(t, 4),
                "com_x_m": float(com[0]),
                "com_y_m": float(com[1]),
                "com_vx_mps": float(com_vx),
                "torso_height_m": float(data.xpos[torso, 2]),
                "torso_tilt_deg": tilt,
                "support_margin_m": margin,
                "joint_error_rad": float(error),
                "ankle_correction_rad": float(correction),
            })

        if step == total_steps:
            break

        data.xfrc_applied[torso, :] = 0.0

        if PUSH_START <= t < PUSH_END:
            data.xfrc_applied[torso, 0] = fx
            data.xfrc_applied[torso, 1] = fy

        mujoco.mj_step(model, data)

    with open(OUT / f"stage6f_{name}.csv", "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=list(rows[0])
        )
        writer.writeheader()
        writer.writerows(rows)

    return rows


print("\nSTAGE 6F — ACTIVE BALANCE REGRESSION")
print("Controller: negative CoM feedback")
print("Gains: Kp=2.5, Kd=0.35")
print("Maximum ankle correction: 0.12 rad")
print("Original actuator torque limits")
print()

baseline = simulate("no_push", 0.0, 0.0)

cases = []

for force in (20.0, 30.0, 40.0, 45.0):
    cases.append((f"{int(force)}N_forward", force, 0.0))
    cases.append((f"{int(force)}N_backward", -force, 0.0))

for force in (20.0, 40.0):
    cases.append((f"{int(force)}N_left", 0.0, force))
    cases.append((f"{int(force)}N_right", 0.0, -force))

results = []

for name, fx, fy in cases:
    print("Running:", name, flush=True)

    rows = simulate(name, fx, fy)

    pairs = [
        (r, b)
        for r, b in zip(rows, baseline)
        if r["time_s"] >= PUSH_END
    ]

    deviations = [
        math.hypot(
            r["com_x_m"] - b["com_x_m"],
            r["com_y_m"] - b["com_y_m"],
        )
        for r, b in pairs
    ]

    margins = [
        r["support_margin_m"]
        for r, _ in pairs
        if r["support_margin_m"] is not None
    ]

    max_tilt = max(
        r["torso_tilt_deg"] for r, _ in pairs
    )
    min_height = min(
        r["torso_height_m"] for r, _ in pairs
    )
    max_error = max(
        r["joint_error_rad"] for r, _ in pairs
    )

    min_margin = min(margins) if margins else None

    final_velocity = abs(rows[-1]["com_vx_mps"])
    final_deviation = deviations[-1]

    passed = (
        len(margins) == len(pairs)
        and min_margin is not None
        and min_margin >= 0.0
        and max_tilt < 10.0
        and min_height > 0.48
        and max_error < 0.700
        and final_velocity < 0.03
        and final_deviation < 0.03
    )

    results.append({
        "case": name,
        "peak_com_deviation_m": max(deviations),
        "final_com_deviation_m": final_deviation,
        "max_torso_tilt_deg": max_tilt,
        "minimum_support_margin_m": min_margin,
        "max_joint_error_rad": max_error,
        "final_com_x_velocity_mps": final_velocity,
        "pass": passed,
    })

print("\n========== STAGE 6F RESULT ==========")

print(
    f"{'CASE':18s} "
    f"{'PEAK COM':>9s} "
    f"{'MAX TILT':>9s} "
    f"{'MIN MARGIN':>11s} "
    f"{'RESULT':>8s}"
)

for r in results:
    margin = r["minimum_support_margin_m"]

    print(
        f"{r['case']:18s} "
        f"{r['peak_com_deviation_m']:9.4f} "
        f"{r['max_torso_tilt_deg']:9.2f} "
        f"{margin if margin is not None else float('nan'):11.4f} "
        f"{'PASS' if r['pass'] else 'FAIL':>8s}"
    )

passed_count = sum(r["pass"] for r in results)

with open(
    OUT / "stage6f_balance_regression.json", "w"
) as f:
    json.dump({
        "controller": "negative sagittal CoM PD",
        "gains": {
            "kp": KP,
            "kd": KD,
            "max_correction_rad": MAX_CORRECTION,
            "max_rate_rad_s": MAX_RATE,
        },
        "results": results,
        "passed": passed_count,
        "total": len(results),
    }, f, indent=2)

print()
print(
    f"Regression diagnostic: {passed_count}/{len(results)} passed"
)
print("STAGE 6F COMPLETE")
