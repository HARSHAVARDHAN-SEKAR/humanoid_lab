"""Actual physics checks. Skip automatically only when MuJoCo is absent."""
import pathlib
import pytest

mujoco = pytest.importorskip('mujoco')
from humanoid_motion_controller.command_model import JOINT_NAMES, CommandComposer

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src/humanoid_sim/mujoco_models/humanoid_biped_supported.xml'


def test_supported_model_loads_and_motor_drives_knee():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    assert model.nu == 12 and model.nq == 12
    data = mujoco.MjData(model)
    initial = CommandComposer().standing_command()
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.replace('_joint', '_act'))
        data.qpos[model.jnt_qposadr[jid]] = initial[name]
        data.ctrl[aid] = initial[name]
    mujoco.mj_forward(model, data)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'right_knee_pitch_act')
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'right_knee_pitch_joint')
    q = model.jnt_qposadr[jid]
    before = float(data.qpos[q])
    data.ctrl[aid] += 0.1
    for _ in range(200):
        mujoco.mj_step(model, data)
    assert abs(float(data.qpos[q]) - before) > 0.005
