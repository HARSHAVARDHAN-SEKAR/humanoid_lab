"""Integration checks tying together the MJCF scene (Stage 1 sim), the
robot_params.yaml single source of truth, and the analytic IK (Stage 3
kinematics). These actually load and step the physics model with MuJoCo --
they are the closest thing in this repo to "did it work in simulation"
rather than pure unit tests of isolated math.
"""
import pathlib
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco", reason="pip install mujoco to run sim-integration tests")

from humanoid_description.robot_params import load_robot_params
from humanoid_kinematics.planar_leg import PlanarLegKinematics

ROOT = pathlib.Path(__file__).parent.parent
MJCF_PATH = ROOT / "src" / "humanoid_sim" / "mujoco_models" / "humanoid_biped.xml"


@pytest.fixture(scope="module")
def params():
    return load_robot_params()


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(str(MJCF_PATH))


def test_mjcf_loads_and_has_expected_actuators(model):
    assert model.nu == 12  # 6 joints x 2 legs
    assert model.nq == 19  # 7 (free joint: 3 pos + 4 quat) + 12 hinge joints
    assert model.nv == 18  # 6 (free joint velocity dof) + 12 hinge joints


def test_mjcf_thigh_length_matches_robot_params(model, params):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_thigh_geom")
    length = float(np.linalg.norm(model.geom_size[geom_id]))  # unused for cylinder-fromto geoms
    # For a fromto cylinder, geom_size[0] is the radius; the length lives in
    # the body offset of the child (knee) body instead.
    knee_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_knee_pitch")
    thigh_vec = model.body_pos[knee_body_id]
    thigh_len = float(np.linalg.norm(thigh_vec))
    assert thigh_len == pytest.approx(params.thigh_length_m, abs=1e-6)


def test_mjcf_shank_length_matches_robot_params(model, params):
    ankle_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_pitch")
    shank_vec = model.body_pos[ankle_body_id]
    shank_len = float(np.linalg.norm(shank_vec))
    assert shank_len == pytest.approx(params.shank_length_m, abs=1e-6)


def test_mjcf_hip_width_matches_robot_params(model, params):
    r_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_hip_yaw")
    l_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_hip_yaw")
    width = float(model.body_pos[l_id][1] - model.body_pos[r_id][1])
    assert width == pytest.approx(params.hip_width_m, abs=1e-6)


def test_free_fall_rollout_is_numerically_stable(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))


def test_ik_standing_pose_is_reachable_and_feet_start_near_ground(model, params):
    """Sets BOTH legs directly to the analytic IK solution for the nominal
    standing target (bypassing actuator dynamics) and checks the feet land
    within a few mm of the floor -- i.e. the IK output, the MJCF link
    geometry and the chosen torso spawn height are all mutually consistent.
    This is a kinematic/geometric check only.
    """
    leg = PlanarLegKinematics(params.thigh_length_m, params.shank_length_m)
    pose = leg.inverse(params.standing_foot_x_m, params.standing_foot_z_m)

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    leg_ctrl = [0.0, 0.0, pose.hip_pitch, pose.knee_pitch, pose.ankle_pitch, 0.0]
    for side, jnames in (
        ("right", ["right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
                   "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"]),
        ("left", ["left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
                  "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint"]),
    ):
        for jn, v in zip(jnames, leg_ctrl):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            data.qpos[model.jnt_qposadr[jid]] = v
    mujoco.mj_forward(model, data)

    for foot in ("right_foot_geom", "left_foot_geom"):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, foot)
        foot_z = float(data.geom_xpos[gid][2])
        assert abs(foot_z) < 0.03, f"{foot} is {foot_z:.3f} m from the floor at the IK standing pose"


def test_closed_loop_ankle_balance_loop_is_numerically_stable(model, params):
    """Runs the Stage-2 ankle-strategy balance controller (standing_balance.py)
    in the actual physics loop for 2 s and checks the simulation stays
    numerically well-behaved throughout (no NaN/Inf, no actuator saturation
    error). This is a liveness/integration smoke test, NOT a standing-
    capability claim -- current closed-loop runs keep roll near zero but do
    not yet hold sustained standing height; see docs/STAGE_STATUS.md and
    tools/tune_standing_gains.py for the tuning work still needed.
    """
    from humanoid_balance_controller.standing_balance import AnkleStrategyBalanceController

    leg = PlanarLegKinematics(params.thigh_length_m, params.shank_length_m)
    pose = leg.inverse(params.standing_foot_x_m, params.standing_foot_z_m)
    balance = AnkleStrategyBalanceController()

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    jnames = ["right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
              "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
              "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
              "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint"]
    vals = [0.0, 0.0, pose.hip_pitch, pose.knee_pitch, pose.ankle_pitch, 0.0] * 2
    for jn, v in zip(jnames, vals):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        data.qpos[model.jnt_qposadr[jid]] = v
    base = [0.0, 0.0, pose.hip_pitch, pose.knee_pitch, pose.ankle_pitch, 0.0]
    data.ctrl[:] = base + base
    mujoco.mj_forward(model, data)

    gyro_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "torso_gyro")
    gyro_adr = model.sensor_adr[gyro_sid]

    for _ in range(2000):  # 2 s at dt=0.001
        w, x, y, z = data.qpos[3:7]
        pitch = float(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
        pitch_rate = float(data.sensordata[gyro_adr + 1])
        corr = balance.pitch_correction(pitch, pitch_rate)
        ankle_cmd = pose.ankle_pitch + corr
        ctrl = [0.0, 0.0, pose.hip_pitch, pose.knee_pitch, ankle_cmd, 0.0]
        data.ctrl[:] = ctrl + ctrl
        mujoco.mj_step(model, data)

    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))
    assert data.qpos[2] > 0.0, "torso went through the floor"
