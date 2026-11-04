import numpy as np
import pytest
from humanoid_kinematics.planar_leg import PlanarLegKinematics, LegKinematicsError

L1, L2 = 0.30, 0.30  # thigh / shank lengths (m), roughly human-proportioned


@pytest.fixture
def leg():
    return PlanarLegKinematics(L1, L2)


def test_straight_leg_reaches_full_extension(leg):
    x, z = leg.forward(0.0, 0.0)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(-(L1 + L2), abs=1e-9)


@pytest.mark.parametrize("x,z", [
    (0.0, -0.50),
    (0.05, -0.45),
    (-0.08, -0.40),
    (0.15, -0.55),
    (0.0, -(L1 + L2)),          # full extension boundary
])
def test_ik_fk_roundtrip(leg, x, z):
    pose = leg.inverse(x, z)
    x2, z2 = leg.forward(pose.hip_pitch, pose.knee_pitch)
    assert x2 == pytest.approx(x, abs=1e-6)
    assert z2 == pytest.approx(z, abs=1e-6)


def test_flat_foot_constraint(leg):
    pose = leg.inverse(0.05, -0.5)
    assert (pose.hip_pitch + pose.knee_pitch + pose.ankle_pitch) == pytest.approx(0.0, abs=1e-9)


def test_unreachable_target_raises(leg):
    with pytest.raises(LegKinematicsError):
        leg.inverse(0.0, -(L1 + L2 + 0.2))  # beyond full extension


def test_knee_flexes_forward_not_backward(leg):
    pose = leg.inverse(0.0, -0.5)
    assert pose.knee_pitch > 0  # positive = physiological flexion direction


def test_reachable_helper_matches_inverse(leg):
    assert leg.reachable(0.0, -0.5) is True
    assert leg.reachable(0.0, -0.9) is False
