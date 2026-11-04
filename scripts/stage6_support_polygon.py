#!/usr/bin/env python3
"""Stage 6B: CoM and contact support-polygon benchmark."""

import csv
import json
import math

import mujoco
import numpy as np

from humanoid_motion_controller.command_model import (
    CommandComposer,
    JOINT_NAMES,
)

MODEL = "/models/humanoid_biped.xml"
OUTPUT = "/results/stage6_support_polygon.csv"
SUMMARY = "/results/stage6_support_summary.json"

DURATION_S = 30.0
SAMPLE_PERIOD_S = 0.1


def cross(a, b, c):
    return (
        (b[0] - a[0]) * (c[1] - a[1])
        - (b[1] - a[1]) * (c[0] - a[0])
    )


def convex_hull(points):
    points = sorted(set(
        (round(float(p[0]), 7), round(float(p[1]), 7))
        for p in points
    ))

    if len(points) < 3:
        return []

    lower = []
    for p in points:
        while len(lower) >= 2 and cross(
            lower[-2], lower[-1], p
        ) <= 1e-10:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(
            upper[-2], upper[-1], p
        ) <= 1e-10:
            upper.pop()
        upper.append(p)

    result = lower[:-1] + upper[:-1]
    return result if len(result) >= 3 else []


def support_margin(point, polygon):
    if len(polygon) < 3:
        return None

    margins = []

    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]

        length = math.hypot(
            b[0] - a[0], b[1] - a[1]
        )

        if length > 1e-10:
            margins.append(
                cross(a, b, point) / length
            )

    return min(margins) if margins else None


model = mujoco.MjModel.from_xml_path(MODEL)
data = mujoco.MjData(model)

assert model.nq == 19, "Expected free-floating biped"

def object_id(kind, name):
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise RuntimeError(f"Missing object: {name}")
    return result


floor = object_id(
    mujoco.mjtObj.mjOBJ_GEOM, "floor"
)

feet = {
    "left": object_id(
        mujoco.mjtObj.mjOBJ_GEOM, "left_foot_geom"
    ),
    "right": object_id(
        mujoco.mjtObj.mjOBJ_GEOM, "right_foot_geom"
    ),
}

torso = object_id(
    mujoco.mjtObj.mjOBJ_BODY, "torso"
)

targets = CommandComposer().standing_command()

data.qpos[:7] = [0, 0, 0.58, 1, 0, 0, 0]

for joint in JOINT_NAMES:
    jid = object_id(
        mujoco.mjtObj.mjOBJ_JOINT, joint
    )
    aid = object_id(
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        joint.replace("_joint", "_act")
    )

    data.qpos[model.jnt_qposadr[jid]] = targets[joint]
    data.ctrl[aid] = targets[joint]

mujoco.mj_forward(model, data)

total_mass = float(model.body_mass.sum())
dt = model.opt.timestep

rows = []

print("\nSTAGE 6B — COM AND SUPPORT POLYGON")
print("Duration: 30 simulated seconds")
print("Constant joint-position commands")
print("Actual active foot-floor contacts")
print()

for step in range(round(DURATION_S / dt) + 1):

    if step % max(1, round(SAMPLE_PERIOD_S / dt)) == 0:

        com = (
            model.body_mass[:, None] * data.xipos
        ).sum(axis=0) / total_mass

        contact_points = []
        loads = {"left": 0.0, "right": 0.0}
        other_floor_contacts = 0

        for i in range(data.ncon):
            c = data.contact[i]

            a = int(c.geom1)
            b = int(c.geom2)

            if floor not in (a, b):
                continue

            other = b if a == floor else a

            force = np.zeros(6)

            if c.efc_address < 0:
                continue

            mujoco.mj_contactForce(
                model, data, i, force
            )

            normal_force = max(
                0.0, float(force[0])
            )

            if normal_force < 0.1:
                continue

            matched = False

            for side, geom in feet.items():
                if other == geom:
                    loads[side] += normal_force
                    contact_points.append(
                        tuple(c.pos[:2])
                    )
                    matched = True
                    break

            if not matched:
                other_floor_contacts += 1

        polygon = convex_hull(contact_points)
        margin = support_margin(com[:2], polygon)

        rotation = data.xmat[torso].reshape(3, 3)
        tilt = math.degrees(math.acos(
            float(np.clip(rotation[2, 2], -1, 1))
        ))

        both_feet = (
            loads["left"] > 1.0
            and loads["right"] > 1.0
        )

        row = {
            "time_s": round(float(data.time), 4),
            "com_x_m": float(com[0]),
            "com_y_m": float(com[1]),
            "com_z_m": float(com[2]),
            "torso_height_m": float(data.xpos[torso, 2]),
            "torso_tilt_deg": tilt,
            "support_margin_m": margin,
            "support_vertices": len(polygon),
            "left_normal_force_N": loads["left"],
            "right_normal_force_N": loads["right"],
            "both_feet_loaded": int(both_feet),
            "other_floor_contacts": other_floor_contacts,
        }

        rows.append(row)

        if step % round(5.0 / dt) == 0:
            label = (
                f"{margin:.4f}m"
                if margin is not None else "not available"
            )

            print(
                f"t={data.time:5.1f}s "
                f"CoM=({com[0]:.3f},{com[1]:.3f})m "
                f"margin={label} "
                f"left={loads['left']:.1f}N "
                f"right={loads['right']:.1f}N "
                f"tilt={tilt:.2f}deg",
                flush=True
            )

    if step < round(DURATION_S / dt):
        mujoco.mj_step(model, data)


with open(OUTPUT, "w", newline="") as f:
    writer = csv.DictWriter(
        f, fieldnames=list(rows[0].keys())
    )
    writer.writeheader()
    writer.writerows(rows)

# Exclude initialization and initial settling.
steady = [
    r for r in rows if r["time_s"] >= 0.2
]

margins = [
    r["support_margin_m"]
    for r in steady
    if r["support_margin_m"] is not None
]

both_fraction = sum(
    r["both_feet_loaded"] for r in steady
) / len(steady)

minimum_margin = (
    min(margins) if margins else None
)

max_tilt = max(
    r["torso_tilt_deg"] for r in steady
)

other_contacts = max(
    r["other_floor_contacts"] for r in steady
)

summary = {
    "duration_s": DURATION_S,
    "samples": len(rows),
    "mass_kg": total_mass,
    "minimum_support_margin_m": minimum_margin,
    "double_support_fraction": both_fraction,
    "max_torso_tilt_deg": max_tilt,
    "maximum_nonfoot_floor_contacts": other_contacts,
    "samples_with_polygon": len(margins),
    "steady_samples": len(steady),
}

with open(SUMMARY, "w") as f:
    json.dump(summary, f, indent=2)

passed = (
    len(margins) == len(steady)
    and minimum_margin is not None
    and minimum_margin >= 0.0
    and both_fraction >= 0.95
    and other_contacts == 0
    and max_tilt < 10.0
)

print("\n========== STAGE 6B RESULT ==========")
print("Total mass:", round(total_mass, 3), "kg")
print("Minimum support margin:", minimum_margin, "m")
print("Double-support fraction:", round(both_fraction, 4))
print("Maximum torso tilt:", round(max_tilt, 3), "deg")
print("Non-foot ground contacts:", other_contacts)
print("Samples with valid polygon:", len(margins))

print(
    "SUPPORT GEOMETRY CHECK:",
    "PASS" if passed else "REQUIRES INVESTIGATION"
)

print("STAGE 6B COMPLETE")
