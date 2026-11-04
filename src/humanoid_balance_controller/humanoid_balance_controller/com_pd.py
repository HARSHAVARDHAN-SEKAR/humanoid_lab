"""Experimental negative-sign sagittal center-of-mass ankle feedback.

Pure Python control law. No ROS runtime required for unit testing.
"""
from dataclasses import dataclass
import math


@dataclass
class ComPitchPD:
    kp: float = 2.5
    kd: float = 0.35
    max_correction: float = 0.12
    max_rate: float = 1.0
    reference_x: float = None
    correction: float = 0.0

    def update(self, x, vx, dt):
        if not all(math.isfinite(v) for v in (x, vx, dt)):
            raise ValueError("Nonfinite CoM state or time step")
        if dt <= 0:
            raise ValueError("Time step must be positive")
        if self.reference_x is None:
            self.reference_x = float(x)
        desired = -(self.kp * (x - self.reference_x) + self.kd * vx)
        desired = max(-self.max_correction, min(self.max_correction, desired))
        step = self.max_rate * dt
        self.correction += max(-step, min(step, desired - self.correction))
        return self.correction
