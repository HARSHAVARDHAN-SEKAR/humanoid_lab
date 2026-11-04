#!/usr/bin/env python3
"""Stage 6E: offline ankle feedback sign comparison.

This is experimental model-based state feedback, not a
deployable balance or whole-body controller.
"""

import csv
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
PUSH_FORCE = 40.0

KP = 2.5
KD = 0.35
MAX_CORRECTION = 0.12
MAX_RATE = 1.0

CONTROL_PERIOD = 0.01
SAMPLE_PERIOD = 0.05


def run(sign, push_force, name):

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    assert model.nq == 19, "Expected free-floating biped"

    torso = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "torso"
    )

    targets = CommandComposer().standing_command()

    data.qpos[:7] = [0, 0, 0.58, 1, 0, 0, 0]

    joint_addresses = {}
    actuator_ids = {}

    for joint in JOINT_NAMES:
        jid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint
        )
        aid = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            joint.replace("_joint", "_act")
        )

        joint_addresses[joint] = model.jnt_qposadr[jid]
        actuator_ids[joint] = aid

        data.qpos[joint_addresses[joint]] = targets[joint]
        data.ctrl[aid] = targets[joint]

    mujoco.mj_forward(model, data)

    ankle_joints = [
        "left_ankle_pitch_joint",
        "right_ankle_pitch_joint",
    ]

    dt = model.opt.timestep
    control_every = round(CONTROL_PERIOD / dt)
    sample_every = round(SAMPLE_PERIOD / dt)
    total_steps = round(DURATION / dt)

    previous_com = None
    previous_time = None

    reference_com_x = None
    correction = 0.0
    current_vx = 0.0

    rows = []

    def get_com():
        return (
            model.body_mass[:, None] * data.xipos
        ).sum(axis=0) / model.body_mass.sum()

    for step in range(total_steps + 1):

        t = float(data.time)

        if step % control_every == 0:

            com = get_com()
            com_x = float(com[0])

            if previous_com is not None:
                delta_t = t - previous_time
                current_vx = (
                    (com_x - previous_com) / delta_t
                    if delta_t > 0 else 0.0
                )

            previous_com = com_x
            previous_time = t

            if reference_com_x is None and t >= 1.5:
                reference_com_x = com_x

            desired = 0.0

            if sign != 0 and reference_com_x is not None:

                position_error = (
                    com_x - reference_com_x
                )

                desired = sign * (
                    KP * position_error
                    + KD * current_vx
                )

                desired = float(np.clip(
                    desired,
                    -MAX_CORRECTION,
                    MAX_CORRECTION
                ))

            rate_step = MAX_RATE * CONTROL_PERIOD

            correction += float(np.clip(
                desired - correction,
                -rate_step,
                rate_step
            ))

            for joint in ankle_joints:
                aid = actuator_ids[joint]
                data.ctrl[aid] = (
                    targets[joint] + correction
                )

        if step % sample_every == 0:

            com = get_com()

            rotation = data.xmat[torso].reshape(3, 3)

            tilt = math.degrees(math.acos(
                float(np.clip(rotation[2, 2], -1, 1))
            ))

            tracking_error = max(
                abs(
                    data.ctrl[actuator_ids[j]]
                    - data.qpos[joint_addresses[j]]
                )
                for j in JOINT_NAMES
            )

            rows.append({
                "time_s": round(t, 4),
                "com_x_m": float(com[0]),
                "com_vx_mps": current_vx,
                "torso_height_m": float(
                    data.xpos[torso, 2]
                ),
                "torso_tilt_deg": tilt,
                "max_joint_error_rad": tracking_error,
                "ankle_correction_rad": correction,
                "contacts": int(data.ncon),
            })

        if step == total_steps:
            break

        data.xfrc_applied[torso, :] = 0.0

        if PUSH_START <= t < PUSH_END:
            data.xfrc_applied[torso, 0] = push_force

        mujoco.mj_step(model, data)

    filename = OUT / f"stage6e_{name}.csv"

    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=list(rows[0])
        )
        writer.writeheader()
        writer.writerows(rows)

    return rows


def evaluate(rows, no_push):

    post = [
        (r, b)
        for r, b in zip(rows, no_push)
        if r["time_s"] >= PUSH_END
    ]

    deviations = [
        abs(r["com_x_m"] - b["com_x_m"])
        for r, b in post
    ]

    max_tilt = max(
        r["torso_tilt_deg"] for r, _ in post
    )

    min_height = min(
        r["torso_height_m"] for r, _ in post
    )

    max_error = max(
        r["max_joint_error_rad"] for r, _ in post
    )

    peak_deviation = max(deviations)
    final_deviation = deviations[-1]

    final_speed = abs(rows[-1]["com_vx_mps"])

    passed = (
        max_tilt < 10.0
        and min_height > 0.48
        and max_error < 0.700
        and final_deviation < 0.03
        and final_speed < 0.03
    )

    return {
        "peak": peak_deviation,
        "final": final_deviation,
        "speed": final_speed,
        "tilt": max_tilt,
        "height": min_height,
        "error": max_error,
        "passed": passed,
    }


configs = [
    ("baseline", 0),
    ("feedback_positive", 1),
    ("feedback_negative", -1),
]

results = []

print("\nSTAGE 6E — ANKLE BALANCE FEEDBACK")
print("40 N forward and backward pushes")
print("No stepping, no changes to torque limits")
print("Controller feedback uses simulated CoM state")
print()

for name, sign in configs:

    print(f"Testing controller: {name}", flush=True)

    no_push = run(
        sign, 0.0, f"{name}_no_push"
    )

    forward = run(
        sign, PUSH_FORCE, f"{name}_forward"
    )

    backward = run(
        sign, -PUSH_FORCE, f"{name}_backward"
    )

    for direction, data in (
        ("forward", forward),
        ("backward", backward),
    ):
        result = evaluate(data, no_push)
        result["controller"] = name
        result["direction"] = direction
        results.append(result)


print("\n========== STAGE 6E RESULT ==========")

print(
    f"{'CONTROLLER':20s} "
    f"{'PUSH':9s} "
    f"{'TILT':>8s} "
    f"{'PEAK CoM':>10s} "
    f"{'FINAL CoM':>11s} "
    f"{'RESULT':>7s}"
)

for r in results:
    print(
        f"{r['controller']:20s} "
        f"{r['direction']:9s} "
        f"{r['tilt']:8.2f} "
        f"{r['peak']:10.4f} "
        f"{r['final']:11.4f} "
        f"{'PASS' if r['passed'] else 'FAIL':>7s}"
    )

print()
print(
    "Diagnostic checks passed:",
    sum(r["passed"] for r in results),
    "/",
    len(results)
)

print("STAGE 6E COMPLETE")
