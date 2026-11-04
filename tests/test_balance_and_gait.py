import numpy as np
import pytest

from humanoid_gait_planner.swing_trajectory import SwingFootTrajectory
from humanoid_gait_planner.footstep_planner import plan_straight_walk, Footstep
from humanoid_balance_controller.capture_point import (
    ComState, lip_omega0, instantaneous_capture_point,
    is_within_support_polygon, adjust_footstep_for_icp,
)


# ---- swing trajectory ----

def test_swing_starts_and_ends_at_targets():
    traj = SwingFootTrajectory((0, 0, 0), (0.3, 0.05, 0), step_height=0.04, duration=0.5)
    s0 = traj.state(0.0)
    s1 = traj.state(0.5)
    assert (s0.x, s0.y, s0.z) == pytest.approx((0, 0, 0), abs=1e-9)
    assert (s1.x, s1.y, s1.z) == pytest.approx((0.3, 0.05, 0), abs=1e-9)


def test_swing_peaks_at_midpoint():
    traj = SwingFootTrajectory((0, 0, 0), (0.3, 0, 0), step_height=0.04, duration=0.5)
    mid = traj.state(0.25)
    assert mid.z == pytest.approx(0.04, abs=1e-9)
    assert mid.x == pytest.approx(0.15, abs=1e-9)  # minimum-jerk is symmetric at tau=0.5


def test_swing_clearance_never_negative():
    traj = SwingFootTrajectory((0, 0, 0), (0.3, 0, 0), step_height=0.04, duration=0.5)
    zs = [traj.state(t).z for t in np.linspace(0, 0.5, 50)]
    assert min(zs) >= -1e-9


def test_swing_zero_velocity_at_touchdown():
    traj = SwingFootTrajectory((0, 0, 0), (0.3, 0, 0), step_height=0.04, duration=0.5)
    s1 = traj.state(0.5)
    assert (s1.vx, s1.vy, s1.vz) == pytest.approx((0, 0, 0), abs=1e-6)


# ---- footstep planner ----

def test_plan_alternates_feet():
    steps = plan_straight_walk(num_steps=6, stride_length=0.3, step_width=0.2, start_foot="right")
    feet = [s.foot for s in steps]
    assert feet == ["right", "left", "right", "left", "right", "left"]


def test_plan_moves_forward_monotonically():
    steps = plan_straight_walk(num_steps=5, stride_length=0.3, step_width=0.2)
    xs = [s.x for s in steps]
    assert all(b > a for a, b in zip(xs, xs[1:]))


def test_plan_rejects_bad_start_foot():
    with pytest.raises(ValueError):
        plan_straight_walk(num_steps=3, stride_length=0.3, step_width=0.2, start_foot="front")


# ---- capture point ----

def test_omega0_known_value():
    # com_height = g -> omega0 = 1.0
    assert lip_omega0(9.81) == pytest.approx(1.0, abs=1e-6)


def test_icp_equals_com_when_static():
    com = ComState(x=0.1, y=-0.02, vx=0.0, vy=0.0, height=0.8)
    icp_x, icp_y, _ = instantaneous_capture_point(com)
    assert (icp_x, icp_y) == pytest.approx((0.1, -0.02), abs=1e-9)


def test_icp_moves_with_velocity_direction():
    com = ComState(x=0.0, y=0.0, vx=0.5, vy=0.0, height=0.8)
    icp_x, icp_y, omega0 = instantaneous_capture_point(com)
    assert icp_x == pytest.approx(0.5 / omega0, abs=1e-9)
    assert icp_x > 0  # forward velocity -> ICP moves forward


def test_support_polygon_check():
    assert is_within_support_polygon(0.01, 0.0, foot_x=0.0, foot_y=0.0,
                                      foot_half_length=0.1, foot_half_width=0.05) is True
    assert is_within_support_polygon(0.2, 0.0, foot_x=0.0, foot_y=0.0,
                                      foot_half_length=0.1, foot_half_width=0.05) is False


def test_adjust_footstep_blends_toward_icp():
    nominal = Footstep(index=0, foot="left", x=0.3, y=0.1)
    adjusted = adjust_footstep_for_icp(nominal, icp_x=0.5, icp_y=0.1, gain=0.5)
    assert adjusted.x == pytest.approx(0.4, abs=1e-9)   # halfway between 0.3 and 0.5
    assert adjusted.foot == "left"


def test_adjust_footstep_zero_gain_is_nominal():
    nominal = Footstep(index=0, foot="left", x=0.3, y=0.1)
    adjusted = adjust_footstep_for_icp(nominal, icp_x=99.0, icp_y=99.0, gain=0.0)
    assert (adjusted.x, adjusted.y) == (nominal.x, nominal.y)


# ---- standing balance controller (control law only -- see
#      docs/STAGE_STATUS.md for the sim-tuning status of this controller) ----

from humanoid_balance_controller.standing_balance import AnkleStrategyBalanceController, BalanceGains


def test_balance_correction_opposes_tilt_direction():
    ctrl = AnkleStrategyBalanceController(BalanceGains(kp_pitch=10.0, kd_pitch=0.0, max_correction_rad=0.5))
    corr = ctrl.pitch_correction(torso_pitch=0.1, torso_pitch_rate=0.0)
    assert corr < 0  # leaning forward (+pitch) -> correction pushes back (-)


def test_balance_correction_is_clipped():
    ctrl = AnkleStrategyBalanceController(BalanceGains(kp_pitch=100.0, kd_pitch=0.0, max_correction_rad=0.2))
    corr = ctrl.pitch_correction(torso_pitch=1.0, torso_pitch_rate=0.0)
    assert corr == pytest.approx(-0.2, abs=1e-9)


def test_balance_correction_zero_at_zero_tilt():
    ctrl = AnkleStrategyBalanceController()
    assert ctrl.pitch_correction(0.0, 0.0) == 0.0
    assert ctrl.roll_correction(0.0, 0.0) == 0.0
