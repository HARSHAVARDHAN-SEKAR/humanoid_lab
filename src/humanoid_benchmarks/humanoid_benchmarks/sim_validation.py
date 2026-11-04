"""Physics-based *supported-torso* regression trial (no ROS required).

This validates motor tracking and a swing-foot reference against MuJoCo
contact and actuator dynamics. Because the pelvis is fixed to the world,
it is NOT evidence of free-standing balance or dynamically stable walking.

Usage (with package available on PYTHONPATH):
    python3 -m humanoid_benchmarks.sim_validation --out results/local

Produces measured CSV + JSON, with an explicit PASS/FAIL and criteria.
Exit nonzero on unmet criteria, missing physics, or bad command targets.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from humanoid_gait_planner.swing_trajectory import SwingFootTrajectory
from humanoid_motion_controller.command_model import CommandComposer, FootTarget, JOINT_NAMES


def run(out_dir: Path, seconds: float = 3.5, stride: float = 0.04):
    import mujoco

    root = Path(__file__).resolve()
    # When installed by ament, the source path is different; use ament share.
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('humanoid_sim')) / 'mujoco_models/humanoid_biped_supported.xml'
    except (ImportError, LookupError):
        path = next((p for p in root.parents
                     if (p / 'src/humanoid_sim/mujoco_models/humanoid_biped_supported.xml').exists()), None)
        if path is None:
            raise FileNotFoundError('Unable to locate supported MJCF model')
        path = path / 'src/humanoid_sim/mujoco_models/humanoid_biped_supported.xml'

    model = mujoco.MjModel.from_xml_path(str(path))
    if model.nq != 12 or model.nu != 12:
        raise ValueError('expected 12 actuators and a *supported* rigid torso')
    data = mujoco.MjData(model)
    composer = CommandComposer()
    nominal = composer.standing_command()
    qpos_index = {}
    actuator_index = {}
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.replace('_joint', '_act'))
        if jid < 0 or aid < 0:
            raise ValueError(f'unmatched actuator for {name}')
        qpos_index[name] = int(model.jnt_qposadr[jid])
        actuator_index[name] = int(aid)
        data.qpos[qpos_index[name]] = nominal[name]
        data.ctrl[aid] = nominal[name]
    mujoco.mj_forward(model, data)

    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'right_foot_site')
    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'left_foot_site')
    torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
    if any(k < 0 for k in (right_id, left_id, torso_id)):
        raise ValueError('missing foot or torso site')
    step_start = 1.0
    step_duration = 1.0
    swing = SwingFootTrajectory((0.0, -0.1, 0.0), (stride, -0.1, 0.0),
                                step_height=0.04, duration=step_duration)
    dt = float(model.opt.timestep)
    rows = []
    errors = []
    max_right_lift = 0.0
    baseline_z = float(data.site_xpos[right_id, 2])
    for i in range(int(seconds / dt)):
        t = i * dt
        if step_start <= t <= step_start + step_duration:
            s = swing.state(t - step_start)
            composer.set_foot('right', FootTarget(s.x, -0.1, s.z))
        elif t > step_start + step_duration:
            composer.set_foot('right', FootTarget(stride, -0.1, 0.0))
        feedback = {name: float(data.qpos[qpos_index[name]]) for name in JOINT_NAMES}
        composer.update_feedback(feedback)
        result = composer.update(safety='WALK', feedback_fresh=True)
        if result.mode != 'TRACKING':
            raise RuntimeError(f'controller rejected command: {result.mode}, {composer.last_fault}')
        for name, target in result.positions.items():
            data.ctrl[actuator_index[name]] = target
        mujoco.mj_step(model, data)
        if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):
            raise RuntimeError(f'nonfinite simulation state at t={t:.3f}s')
        if t >= 0.5:
            errors.extend((float(data.qpos[qpos_index[name]]) - result.positions[name])**2
                          for name in JOINT_NAMES)
        if step_start <= t <= step_start + step_duration:
            max_right_lift = max(max_right_lift,
                                 float(data.site_xpos[right_id, 2] - baseline_z))
        if i % max(1, int(0.01/dt)) == 0:
            row = {
                'time_s': round(t, 4),
                'torso_height_m': float(data.xpos[torso_id, 2]),
                'right_foot_height_m': float(data.site_xpos[right_id, 2]),
                'left_foot_height_m': float(data.site_xpos[left_id, 2]),
                'max_tracking_error_rad': result.max_tracking_error_rad,
                'right_knee_actual_rad': float(data.qpos[qpos_index['right_knee_pitch_joint']]),
                'right_knee_target_rad': float(result.positions['right_knee_pitch_joint']),
            }
            rows.append(row)
    rms = math.sqrt(float(np.mean(errors))) if errors else float('nan')
    # Strictly speaking this verifies supported foot clearance and tracking,
    # not free-base walking distance or push recovery.
    checks = {
        'finite_physics': True,
        'joint_tracking_rms_below_0_2_rad': bool(rms < 0.2),
        'right_foot_lift_above_0_008_m': bool(max_right_lift > 0.008),
    }
    summary = {
        'model': 'simplified_12dof_fixed_torso_support',
        'validation_scope': 'physics joint tracking and one supported swing; not free-base balance or walking',
        'seconds_simulated': seconds,
        'stride_command_m': stride,
        'joint_tracking_rms_rad': rms,
        'measured_right_foot_peak_lift_m': max_right_lift,
        'criteria': checks,
        'passed': all(checks.values()),
        'sample_count': len(rows),
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / 'supported_trial.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    (out_dir / 'supported_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('results/local'))
    parser.add_argument('--seconds', type=float, default=3.5)
    args = parser.parse_args()
    try:
        summary = run(args.out, seconds=args.seconds)
    except ImportError as exc:
        parser.exit(2, f'MuJoCo is required for the physics trial: {exc}\n')
    print(json.dumps(summary, indent=2))
    if not summary['passed']:
        parser.exit(1, 'Supported trial did not meet measured acceptance criteria\n')


if __name__ == '__main__':
    main()
