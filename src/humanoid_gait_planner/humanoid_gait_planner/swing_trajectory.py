"""Minimum-jerk swing-foot trajectory generator (pure math, no ROS dependency).

Horizontal axes (x, y) use a quintic minimum-jerk profile (zero velocity and
acceleration at touch-down/lift-off -> no impact shock from the trajectory
itself). The vertical axis (z) uses a raised sine so foot clearance is zero
at both ends and peaks at `step_height` at mid-swing.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class FootState:
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


def _minimum_jerk(s0: float, s1: float, tau: float):
    """Quintic minimum-jerk scalar interpolation, tau in [0, 1]."""
    tau = np.clip(tau, 0.0, 1.0)
    poly = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
    dpoly_dtau = 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4
    s = s0 + (s1 - s0) * poly
    ds_dtau = (s1 - s0) * dpoly_dtau
    return s, ds_dtau


class SwingFootTrajectory:
    def __init__(self, start_xyz, end_xyz, step_height: float, duration: float):
        if duration <= 0:
            raise ValueError("duration must be positive")
        if step_height < 0:
            raise ValueError("step_height must be non-negative")
        self.start = tuple(start_xyz)
        self.end = tuple(end_xyz)
        self.step_height = float(step_height)
        self.duration = float(duration)

    def state(self, t: float) -> FootState:
        """Foot position/velocity at time t in [0, duration] (clamped)."""
        tau = t / self.duration
        tau_c = float(np.clip(tau, 0.0, 1.0))

        x, dx_dtau = _minimum_jerk(self.start[0], self.end[0], tau_c)
        y, dy_dtau = _minimum_jerk(self.start[1], self.end[1], tau_c)

        z_ground, dzg_dtau = _minimum_jerk(self.start[2], self.end[2], tau_c)
        # Raised-cosine clearance: zero height AND zero slope at both tau=0 and
        # tau=1, so the foot lifts off and lands with no vertical-velocity jump
        # (a plain sine has nonzero slope at the endpoints and would impact
        # the ground at touch-down).
        z_clear = self.step_height / 2.0 * (1.0 - np.cos(2.0 * np.pi * tau_c))
        dzclear_dtau = self.step_height * np.pi * np.sin(2.0 * np.pi * tau_c)
        z = z_ground + z_clear
        dz_dtau = dzg_dtau + dzclear_dtau

        dtau_dt = 1.0 / self.duration
        return FootState(
            x=float(x), y=float(y), z=float(z),
            vx=float(dx_dtau * dtau_dt), vy=float(dy_dtau * dtau_dt), vz=float(dz_dtau * dtau_dt),
        )

    def peak_clearance(self) -> float:
        return self.step_height
