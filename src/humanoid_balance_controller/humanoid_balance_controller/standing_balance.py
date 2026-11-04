"""Ankle-strategy standing-balance PD controller (Stage 2).

STATUS: implemented and unit-tested as a control *law*, but NOT yet tuned
to hold sustained standing in the MuJoCo sim -- see
docs/STAGE_STATUS.md and humanoid_benchmarks.tune_standing_gains. In closed-loop
testing, this controller keeps the robot upright for a few hundred
milliseconds and damps roll almost perfectly, but the commanded bent-knee
stance isn't vertically stiff enough to hold standing height indefinitely
and pitch eventually grows past recovery. Likely next steps (not yet
implemented): a hip strategy in addition to the ankle strategy, CoM/ZMP
feedback rather than pure torso-orientation feedback, a wider support
polygon (longer feet), and a properly tuned gain set via the sweep tool.

The control law itself -- clip(-(kp*angle + kd*angle_rate), +-max) -- is
the standard single-rigid-body "ankle strategy" used for small, fast
posture corrections (Kajita et al., "Introduction to Humanoid Robotics",
ch. 5). It is correct and tested in isolation; what's unverified is
whether these particular gains are enough for this particular robot.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class BalanceGains:
    kp_pitch: float = 10.0
    kd_pitch: float = 0.3
    kp_roll: float = 10.0
    kd_roll: float = 0.3
    max_correction_rad: float = 0.2


class AnkleStrategyBalanceController:
    def __init__(self, gains: BalanceGains = None):
        self.gains = gains or BalanceGains()

    def pitch_correction(self, torso_pitch: float, torso_pitch_rate: float) -> float:
        g = self.gains
        corr = -(g.kp_pitch * torso_pitch + g.kd_pitch * torso_pitch_rate)
        return float(np.clip(corr, -g.max_correction_rad, g.max_correction_rad))

    def roll_correction(self, torso_roll: float, torso_roll_rate: float) -> float:
        g = self.gains
        corr = -(g.kp_roll * torso_roll + g.kd_roll * torso_roll_rate)
        return float(np.clip(corr, -g.max_correction_rad, g.max_correction_rad))
