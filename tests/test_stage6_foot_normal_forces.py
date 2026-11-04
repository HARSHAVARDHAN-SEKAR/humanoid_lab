"""Regression: free-base foot-load topics must reflect contact solver forces."""
from pathlib import Path
import mujoco
from humanoid_sim.contact_force import foot_ground_normal_loads
from humanoid_motion_controller.command_model import CommandComposer, JOINT_NAMES


def test_freebase_standing_foot_forces_are_nonzero():
    path = (Path(__file__).resolve().parent.parent / 'src' /
            'humanoid_sim/mujoco_models/humanoid_biped.xml')
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[:7] = [0.0, 0.0, 0.58, 1.0, 0.0, 0.0, 0.0]
    nominal = CommandComposer().standing_command()
    for joint in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                               joint.replace('_joint', '_act'))
        data.qpos[model.jnt_qposadr[jid]] = nominal[joint]
        data.ctrl[aid] = nominal[joint]
    mujoco.mj_forward(model, data)
    for _ in range(1000):
        mujoco.mj_step(model, data)
    gid = lambda name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    left, right = foot_ground_normal_loads(
        model, data, gid('floor'), gid('left_foot_geom'), gid('right_foot_geom'))
    assert left > 20.0, f'Unexpected left load: {left} N'
    assert right > 20.0, f'Unexpected right load: {right} N'
    assert 100.0 < left + right < 400.0
