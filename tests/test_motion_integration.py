"""Pure-python regression coverage for the physical command wiring."""
import math
import pathlib
import xml.etree.ElementTree as ET
import pytest

from humanoid_motion_controller.command_model import (
    JOINT_NAMES, CommandComposer, FootTarget, JOINT_LIMITS,
)
from humanoid_kinematics.planar_leg import LegKinematicsError


ROOT = pathlib.Path(__file__).resolve().parents[1]


def test_standing_command_is_complete_and_finite():
    c = CommandComposer()
    command = c.standing_command()
    assert len(command) == 12
    assert set(command) == set(JOINT_NAMES)
    for name, value in command.items():
        key = name.removeprefix('left_').removeprefix('right_').removesuffix('_joint')
        lo, hi = JOINT_LIMITS[key]
        assert lo <= value <= hi
        assert math.isfinite(value)


def test_ankle_balance_corrections_reach_actuator_targets():
    c = CommandComposer()
    base = c.update(safety='STAND', feedback_fresh=True).positions
    shifted = c.update(safety='STAND', feedback_fresh=True, pitch_corr=0.06,
                       roll_corr=0.04).positions
    for side in ('left', 'right'):
        assert shifted[f'{side}_ankle_pitch_joint'] == pytest.approx(
            base[f'{side}_ankle_pitch_joint'] + 0.06)
    assert shifted['right_ankle_roll_joint'] == pytest.approx(0.04)
    assert shifted['left_ankle_roll_joint'] == pytest.approx(-0.04)


@pytest.mark.parametrize('state', ['FAULT_DETECTED', 'RECOVERY', 'SAFE_HOLD', 'invalid'])
def test_safety_gate_holds_measured_joint_state(state):
    c = CommandComposer()
    feedback = {n: 0.1 for n in JOINT_NAMES}
    c.update_feedback(feedback)
    c.set_foot('right', FootTarget(0.04, -0.1, 0.02))
    result = c.update(safety=state, feedback_fresh=True)
    assert result.mode == 'SAFE_HOLD'
    assert result.positions == feedback


def test_stale_feedback_causes_hold():
    c = CommandComposer()
    c.update_feedback({n: 0.2 for n in JOINT_NAMES})
    result = c.update(safety='WALK', feedback_fresh=False)
    assert result.mode == 'SAFE_HOLD'
    assert all(v == 0.2 for v in result.positions.values())


def test_ik_rejection_holds_with_error_not_invalid_actuation():
    c = CommandComposer()
    c.set_foot('left', FootTarget(1.4, 0.1, 0))
    result = c.update(safety='WALK', feedback_fresh=True)
    assert result.mode == 'IK_FAULT_HOLD'
    assert 'unreachable' in c.last_fault


def test_swing_height_converts_to_knee_motion():
    c = CommandComposer()
    before = c.standing_command()
    c.set_foot('right', FootTarget(0.04, -0.1, 0.04))
    after = c.update(safety='WALK', feedback_fresh=True).positions
    assert after['right_knee_pitch_joint'] != pytest.approx(before['right_knee_pitch_joint'])
    assert after['left_knee_pitch_joint'] == pytest.approx(before['left_knee_pitch_joint'])


def test_bad_lateral_and_nan_foot_requests_rejected():
    c = CommandComposer()
    with pytest.raises(ValueError, match='3D IK'):
        c.set_foot('right', FootTarget(0, 0.1, 0))
    with pytest.raises(ValueError, match='nonfinite'):
        c.set_foot('left', FootTarget(float('nan'), 0.1, 0))


def test_sim_models_exist_and_contain_12_matching_actuators():
    for suffix, free_expected in (('humanoid_biped.xml', True),
                                  ('humanoid_biped_supported.xml', False)):
        path = ROOT / 'src/humanoid_sim/mujoco_models' / suffix
        root = ET.parse(path).getroot()
        joints = {j.get('name') for j in root.findall('.//joint') if j.get('name')}
        actuators = root.findall('./actuator/position')
        assert len(actuators) == 12
        assert set(JOINT_NAMES) == joints
        assert (root.find('.//freejoint') is not None) == free_expected


def test_core_ros_topic_chain_source_contract():
    """Static connection check; does not assert that ROS/DDS runtime works."""
    files = {
        'motion': ROOT / 'src/humanoid_motion_controller/humanoid_motion_controller/motion_controller_node.py',
        'bridge': ROOT / 'src/humanoid_sim/humanoid_sim/mujoco_bridge_node.py',
        'estimator': ROOT / 'src/humanoid_state_estimator/humanoid_state_estimator/state_estimator_node.py',
        'balance': ROOT / 'src/humanoid_balance_controller/humanoid_balance_controller/balance_controller_node.py',
        'supervisor': ROOT / 'src/humanoid_safety/humanoid_safety/safety_supervisor_node.py',
    }
    src = {k: p.read_text() for k, p in files.items()}
    assert '"/mujoco_bridge/joint_commands"' in src['motion']
    assert '"/mujoco_bridge/joint_commands"' in src['bridge']
    assert '"/state_estimator/attitude"' in src['balance']
    assert '"/motion_controller/joint_tracking_error"' in src['supervisor']
    assert '"/joint_states"' in src['supervisor']
