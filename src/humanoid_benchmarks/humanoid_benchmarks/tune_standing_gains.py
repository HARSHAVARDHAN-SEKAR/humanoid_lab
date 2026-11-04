#!/usr/bin/env python3
"""Standalone (no-ROS) gain-sweep tool for the Stage-2 standing-balance
controller. Loads the MJCF directly via the `mujoco` python bindings and
runs the closed-loop ankle-strategy controller for each (kp, kd,
max_correction) combination in the grid, reporting how long the robot
stays upright before falling.

This is exactly the tool used to produce the numbers quoted in
docs/STAGE_STATUS.md -- run it, look at which combinations last longest,
and iterate on humanoid_balance_controller/standing_balance.py from there.
Falling back to a pure PD ankle strategy may not be enough on its own; see
the STAGE_STATUS.md notes on likely next steps (hip strategy, CoM/ZMP
feedback, wider feet).

Usage:
    python3 tune_standing_gains.py [--mjcf PATH] [--seconds 3.0]
"""
import argparse
import itertools
import pathlib
import sys

import numpy as np

try:
    import mujoco
except ImportError:
    print("This tool requires the 'mujoco' python package: pip install mujoco", file=sys.stderr)
    raise

THIS_DIR = pathlib.Path(__file__).resolve().parent
WS_ROOT = THIS_DIR.parents[2]  # .../humanoid_lab_ws
DEFAULT_MJCF = WS_ROOT / "src" / "humanoid_sim" / "mujoco_models" / "humanoid_biped.xml"

JOINT_ORDER = [
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
    "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
    "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
]


def quat_pitch(q):
    w, x, y, z = q
    return float(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))


def run_once(model, standing_pose, kp, kd, max_c, nsteps):
    hip_p, knee_p, ankle_p = standing_pose
    vals = [0.0, 0.0, hip_p, knee_p, ankle_p, 0.0] * 2

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    for jn, v in zip(JOINT_ORDER, vals):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        data.qpos[model.jnt_qposadr[jid]] = v
    base = [0.0, 0.0, hip_p, knee_p, ankle_p, 0.0]
    data.ctrl[:] = base + base
    mujoco.mj_forward(model, data)

    gyro_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "torso_gyro")
    gyro_adr = model.sensor_adr[gyro_sid]

    fell_at = None
    for i in range(nsteps):
        pitch = quat_pitch(data.qpos[3:7])
        pitch_rate = float(data.sensordata[gyro_adr + 1])
        corr = float(np.clip(-(kp * pitch + kd * pitch_rate), -max_c, max_c))
        ankle_cmd = ankle_p + corr
        ctrl = [0.0, 0.0, hip_p, knee_p, ankle_cmd, 0.0]
        data.ctrl[:] = ctrl + ctrl
        mujoco.mj_step(model, data)
        if abs(pitch) > 0.5 or data.qpos[2] < 0.3:
            fell_at = i
            break
    return fell_at, float(data.qpos[2])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mjcf", type=pathlib.Path, default=DEFAULT_MJCF)
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()

    sys.path.insert(0, str(WS_ROOT / "src" / "humanoid_kinematics"))
    sys.path.insert(0, str(WS_ROOT / "src" / "humanoid_description"))
    from humanoid_kinematics.planar_leg import PlanarLegKinematics
    from humanoid_description.robot_params import load_robot_params

    params = load_robot_params()
    leg = PlanarLegKinematics(params.thigh_length_m, params.shank_length_m)
    pose = leg.inverse(params.standing_foot_x_m, params.standing_foot_z_m)
    standing_pose = (pose.hip_pitch, pose.knee_pitch, pose.ankle_pitch)

    model = mujoco.MjModel.from_xml_path(str(args.mjcf))
    nsteps = int(args.seconds / model.opt.timestep)

    kp_grid = [2.0, 6.0, 10.0, 16.0, 24.0]
    kd_grid = [0.1, 0.3, 0.8]
    maxc_grid = [0.15, 0.2, 0.3]

    print(f"{'kp':>6} {'kd':>6} {'max_c':>7} {'result':>10} {'final_z':>8}")
    best = None
    for kp, kd, max_c in itertools.product(kp_grid, kd_grid, maxc_grid):
        fell_at, final_z = run_once(model, standing_pose, kp, kd, max_c, nsteps)
        result = "FULL" if fell_at is None else f"{fell_at * model.opt.timestep:.2f}s"
        print(f"{kp:6.1f} {kd:6.1f} {max_c:7.2f} {result:>10} {final_z:8.3f}")
        score = args.seconds if fell_at is None else fell_at * model.opt.timestep
        if best is None or score > best[0]:
            best = (score, kp, kd, max_c)

    print()
    if best:
        print(f"Best so far: kp={best[1]}, kd={best[2]}, max_correction_rad={best[3]} "
              f"(stood for {best[0]:.2f}s of {args.seconds:.2f}s)")
    print("None of these reliably reach FULL duration as of this scaffold -- see "
          "docs/STAGE_STATUS.md for suggested next steps (hip strategy, CoM/ZMP "
          "feedback, wider feet) before trusting this controller for real standing.")


if __name__ == "__main__":
    main()
