"""Linear-Inverted-Pendulum (LIP) capture-point balance math.

This is the standard simplified-model approach used widely in bipedal
balance/push-recovery (Pratt et al., "Capture Point: A Step toward Humanoid
Push Recovery", 2006): approximate the robot as a point mass atop a
massless leg, and use the instantaneous capture point (ICP) -- the point on
the ground where the swing foot should land to bring the CoM to rest -- both
to judge stability and to correct the next footstep.
"""
from dataclasses import dataclass
import numpy as np

from humanoid_gait_planner.footstep_planner import Footstep

G = 9.81


@dataclass
class ComState:
    x: float
    y: float
    vx: float
    vy: float
    height: float  # constant CoM height assumed by the LIP model


def lip_omega0(com_height: float) -> float:
    if com_height <= 0:
        raise ValueError("com_height must be positive")
    return float(np.sqrt(G / com_height))


def instantaneous_capture_point(com: ComState):
    """Returns (icp_x, icp_y, omega0)."""
    omega0 = lip_omega0(com.height)
    icp_x = com.x + com.vx / omega0
    icp_y = com.y + com.vy / omega0
    return icp_x, icp_y, omega0


def is_within_support_polygon(icp_x, icp_y, foot_x, foot_y, foot_half_length, foot_half_width) -> bool:
    """Coarse single-support stability check: is the ICP within the current
    stance foot's footprint (axis-aligned rectangle)?
    """
    return (abs(icp_x - foot_x) <= foot_half_length) and (abs(icp_y - foot_y) <= foot_half_width)


def adjust_footstep_for_icp(nominal: Footstep, icp_x: float, icp_y: float, gain: float = 0.5) -> Footstep:
    """Blend the nominal (pre-planned, Stage 4) footstep toward the current
    ICP so the robot steps to arrest unexpected CoM motion (Stage 5 push
    recovery). gain in [0, 1]: 0 = ignore the disturbance and use the
    nominal plan; 1 = step exactly onto the ICP.
    """
    gain = float(np.clip(gain, 0.0, 1.0))
    x = (1.0 - gain) * nominal.x + gain * icp_x
    y = (1.0 - gain) * nominal.y + gain * icp_y
    return Footstep(index=nominal.index, foot=nominal.foot, x=x, y=y, theta=nominal.theta)
