#!/usr/bin/env python3

"""Stage 6A: Free-floating biped standing baseline."""

import csv
import math
import numpy as np
import mujoco

from humanoid_motion_controller.command_model import (
    CommandComposer,
    JOINT_NAMES,
)

MODEL_PATH = "/models/humanoid_biped.xml"
OUTPUT_PATH = "/results/stage6_freebase_30s.csv"

DURATION = 30.0

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

assert model.nq == 19, "Expected free-floating 12-DOF biped"

targets = CommandComposer().standing_command()

# Free-floating torso initialization.
data.qpos[0:7] = [0, 0, 0.58, 1, 0, 0, 0]

qpos_addresses = {}
actuator_ids = {}

for joint in JOINT_NAMES:

    jid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint,
    )

    aid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        joint.replace("_joint", "_act"),
    )

    qpos_addresses[joint] = model.jnt_qposadr[jid]
    actuator_ids[joint] = aid

    data.qpos[qpos_addresses[joint]] = targets[joint]
    data.ctrl[aid] = targets[joint]

mujoco.mj_forward(model, data)

torso_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "torso",
)

initial_xy = data.xpos[torso_id, :2].copy()

maximum_tilt = 0.0
minimum_height = float("inf")
maximum_error = 0.0
maximum_xy_drift = 0.0

samples = 0
contact_samples = 0

dt = model.opt.timestep
total_steps = round(DURATION / dt)
sample_interval = max(1, round(0.1 / dt))

print("STAGE 6A — FREE-FLOATING STANDING")
print("Duration: 30 simulated seconds")
print("Model: free-floating torso")
print("Controller: constant standing joint targets")
print("No external push disturbances")
print()

with open(OUTPUT_PATH, "w", newline="") as f:

    writer = csv.writer(f)

    writer.writerow([
        "time_s",
        "torso_height_m",
        "torso_tilt_deg",
        "com_x_m",
        "com_y_m",
        "com_z_m",
        "joint_error_max_rad",
        "torso_xy_drift_m",
        "contact_points",
    ])

    for step in range(total_steps + 1):

        if step % sample_interval == 0:

            height = float(
                data.xpos[torso_id, 2]
            )

            rotation = data.xmat[
                torso_id
            ].reshape(3, 3)

            tilt = math.degrees(
                math.acos(
                    float(
                        np.clip(
                            rotation[2, 2],
                            -1.0,
                            1.0,
                        )
                    )
                )
            )

            com = np.average(
                data.xipos,
                axis=0,
                weights=model.body_mass,
            )

            errors = [
                abs(
                    targets[joint]
                    - data.qpos[qpos_addresses[joint]]
                )
                for joint in JOINT_NAMES
            ]

            error = max(errors)

            xy_drift = float(
                np.linalg.norm(
                    data.xpos[torso_id, :2]
                    - initial_xy
                )
            )

            maximum_tilt = max(maximum_tilt, tilt)
            minimum_height = min(minimum_height, height)
            maximum_error = max(maximum_error, error)
            maximum_xy_drift = max(
                maximum_xy_drift, xy_drift
            )

            samples += 1

            if data.ncon > 0:
                contact_samples += 1

            writer.writerow([
                round(data.time, 4),
                height,
                tilt,
                float(com[0]),
                float(com[1]),
                float(com[2]),
                error,
                xy_drift,
                data.ncon,
            ])

            if step % max(1, round(5.0 / dt)) == 0:
                print(
                    f"t={data.time:5.1f}s "
                    f"height={height:.3f}m "
                    f"tilt={tilt:.2f}deg "
                    f"error={error:.4f}rad "
                    f"xy_drift={xy_drift:.3f}m "
                    f"contacts={data.ncon}",
                    flush=True,
                )

            if not all(
                math.isfinite(v)
                for v in (
                    height, tilt, error, xy_drift
                )
            ):
                raise RuntimeError(
                    "Non-finite simulation measurement"
                )

        if step < total_steps:
            mujoco.mj_step(model, data)

passed = (
    minimum_height >= 0.48
    and maximum_tilt < 10.0
    and maximum_error < 0.700
    and maximum_xy_drift < 0.15
)

print("\n========== STAGE 6A RESULT ==========")
print("Minimum torso height:",
      round(minimum_height, 4), "m")
print("Maximum torso tilt:",
      round(maximum_tilt, 3), "deg")
print("Maximum joint error:",
      round(maximum_error, 5), "rad")
print("Maximum XY drift:",
      round(maximum_xy_drift, 4), "m")
print("Samples:", samples)
print("Samples with contacts:", contact_samples)

print(
    "FREE-BASE BASELINE:",
    "PASS" if passed else "REQUIRES INVESTIGATION"
)

print("STAGE 6A COMPLETE")
