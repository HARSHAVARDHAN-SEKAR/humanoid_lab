#!/usr/bin/env python3
"""Stage 6C: controlled external-push comparison."""

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
DURATION = 8.0
PUSH_START = 2.0
PUSH_END = 2.1
PUSH_FORCE = 20.0


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
    for p in points:
        while len(lower) >= 2 and cross(
            lower[-2], lower[-1], p
        ) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(
            upper[-2], upper[-1], p
        ) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def simulate(force, filename):
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)

    assert model.nq == 19, "Expected free-floating robot"

    torso = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "torso"
    )
    floor = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
    )
    feet = {
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM,
            "left_foot_geom"
        ),
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM,
            "right_foot_geom"
        ),
    }

    targets = CommandComposer().standing_command()
    joint_addresses = {}

    data.qpos[:7] = [0, 0, 0.58, 1, 0, 0, 0]

    for joint, target in targets.items():
        jid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint
        )
        aid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR,
            joint.replace("_joint", "_act")
        )

        joint_addresses[joint] = model.jnt_qposadr[jid]
        data.qpos[joint_addresses[joint]] = target
        data.ctrl[aid] = target

    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    sample_every = max(1, round(0.05 / dt))

    previous_com = None
    previous_time = None
    rows = []

    for step in range(round(DURATION / dt) + 1):

        if step % sample_every == 0:
            com = (
                model.body_mass[:, None] * data.xipos
            ).sum(axis=0) / model.body_mass.sum()

            if previous_com is None:
                velocity = np.zeros(2)
            else:
                velocity = (
                    com[:2] - previous_com
                ) / (data.time - previous_time)

            previous_com = com[:2].copy()
            previous_time = data.time

            rotation = data.xmat[torso].reshape(3, 3)
            tilt = math.degrees(math.acos(
                float(np.clip(rotation[2, 2], -1, 1))
            ))

            joint_error = max(
                abs(
                    targets[j] - data.qpos[joint_addresses[j]]
                )
                for j in JOINT_NAMES
            )

            contact_points = []

            for i in range(data.ncon):
                c = data.contact[i]
                a, b = int(c.geom1), int(c.geom2)

                if floor not in (a, b):
                    continue

                other = b if a == floor else a
                if other not in feet or c.efc_address < 0:
                    continue

                wrench = np.zeros(6)
                mujoco.mj_contactForce(
                    model, data, i, wrench
                )

                if wrench[0] > 0.1:
                    contact_points.append(c.pos[:2])

            polygon = convex_hull(contact_points)
            margins = []

            for i, a in enumerate(polygon):
                b = polygon[(i + 1) % len(polygon)]
                dx = b[0] - a[0]
                dy = b[1] - a[1]
                length = math.hypot(dx, dy)

                if length > 1e-9:
                    margins.append(
                        (
                            dx * (com[1] - a[1])
                            - dy * (com[0] - a[0])
                        ) / length
                    )

            margin = (
                min(margins) if margins else float("nan")
            )

            rows.append({
                "time_s": round(float(data.time), 3),
                "com_x_m": float(com[0]),
                "com_y_m": float(com[1]),
                "com_vx_mps": float(velocity[0]),
                "com_vy_mps": float(velocity[1]),
                "torso_height_m": float(data.xpos[torso, 2]),
                "torso_tilt_deg": tilt,
                "joint_error_rad": float(joint_error),
                "support_margin_m": margin,
                "contacts": int(data.ncon),
            })

        if step == round(DURATION / dt):
            break

        data.xfrc_applied[torso, :] = 0.0

        if PUSH_START <= data.time < PUSH_END:
            data.xfrc_applied[torso, 0] = force

        mujoco.mj_step(model, data)

    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=list(rows[0].keys())
        )
        writer.writeheader()
        writer.writerows(rows)

    return rows


print("STAGE 6C — CONTROLLED PUSH TEST")
print("Simulation: free-floating torso")
print("Push: 20 N for 0.1 s in +X")
print("Controller: constant standing joint targets")
print()

baseline = simulate(
    0.0, "/results/stage6_no_push.csv"
)
pushed = simulate(
    PUSH_FORCE, "/results/stage6_push_20N.csv"
)

print("Time     CoM displacement vs baseline   Tilt    Margin")

peak_difference = 0.0

for base, push in zip(baseline, pushed):
    difference = math.hypot(
        push["com_x_m"] - base["com_x_m"],
        push["com_y_m"] - base["com_y_m"],
    )

    if push["time_s"] >= PUSH_START:
        peak_difference = max(
            peak_difference, difference
        )

    if round(push["time_s"] * 100) in (
        100, 200, 210, 250, 300, 400, 600, 800
    ):
        print(
            f"{push['time_s']:5.2f}s   "
            f"{difference:9.5f} m              "
            f"{push['torso_tilt_deg']:6.2f} deg   "
            f"{push['support_margin_m']:.4f} m"
        )

after_push = [
    r for r in pushed
    if r["time_s"] >= PUSH_END
]

final = pushed[-1]
base_final = baseline[-1]

final_difference = math.hypot(
    final["com_x_m"] - base_final["com_x_m"],
    final["com_y_m"] - base_final["com_y_m"],
)

final_speed = math.hypot(
    final["com_vx_mps"],
    final["com_vy_mps"],
)

min_margin = min(
    r["support_margin_m"]
    for r in after_push
    if math.isfinite(r["support_margin_m"])
)

max_tilt = max(
    r["torso_tilt_deg"] for r in after_push
)

min_height = min(
    r["torso_height_m"] for r in after_push
)

summary = {
    "force_N": PUSH_FORCE,
    "duration_s": PUSH_END - PUSH_START,
    "nominal_impulse_Ns": PUSH_FORCE * (PUSH_END - PUSH_START),
    "peak_com_difference_m": peak_difference,
    "final_com_difference_m": final_difference,
    "final_com_speed_mps": final_speed,
    "maximum_tilt_deg": max_tilt,
    "minimum_torso_height_m": min_height,
    "minimum_support_margin_m": min_margin,
}

with open(
    "/results/stage6_push_summary.json", "w"
) as f:
    json.dump(summary, f, indent=2)

print("\n========== STAGE 6C RESULT ==========")
for key, value in summary.items():
    print(f"{key}: {value:.5f}")

if (
    min_height > 0.48
    and max_tilt < 10.0
    and min_margin >= 0.0
    and final_speed < 0.03
    and final_difference < 0.03
):
    print("PUSH RESPONSE CHECK: PASS")
else:
    print("PUSH RESPONSE CHECK: REQUIRES INVESTIGATION")

print("STAGE 6C COMPLETE")
