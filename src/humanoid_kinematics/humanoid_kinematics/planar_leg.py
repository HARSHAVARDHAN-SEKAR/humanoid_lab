"""Analytic kinematics for a 2-link planar (sagittal-plane) humanoid leg.

Pure math, no ROS dependency, so it can be unit tested in isolation and
reused by both the ROS 2 node in this package and offline analysis scripts.

Frame convention
-----------------
Origin at the hip-pitch joint. +x is forward, +z is up. A foot target is
given relative to the hip as (x, z), with z normally negative (foot below
hip). hip_pitch = 0, knee_pitch = 0 means the leg hangs straight down.
Positive knee_pitch is flexion (knee bends, as in a real humanoid knee).
"""
from dataclasses import dataclass
import numpy as np


class LegKinematicsError(Exception):
    """Raised when a requested foot position is outside the leg's reach."""


@dataclass
class LegPose:
    hip_pitch: float
    knee_pitch: float
    ankle_pitch: float  # chosen so the foot sole stays flat (= -(hip+knee))


class PlanarLegKinematics:
    """2-link (thigh/shank) planar leg FK + analytic IK."""

    def __init__(self, thigh_length: float, shank_length: float):
        if thigh_length <= 0 or shank_length <= 0:
            raise ValueError("Link lengths must be positive")
        self.l1 = float(thigh_length)
        self.l2 = float(shank_length)

    def forward(self, hip_pitch: float, knee_pitch: float):
        """Foot (ankle) position (x, z) relative to the hip for given joint angles."""
        x = self.l1 * np.sin(hip_pitch) + self.l2 * np.sin(hip_pitch + knee_pitch)
        z = -self.l1 * np.cos(hip_pitch) - self.l2 * np.cos(hip_pitch + knee_pitch)
        return x, z

    def inverse(self, x: float, z: float, knee_bend_sign: float = 1.0) -> LegPose:
        """Analytic IK for a target foot position relative to the hip.

        knee_bend_sign: +1.0 selects the knee-flexes-forward branch (the
        physically normal humanoid solution), -1.0 the other branch.
        Raises LegKinematicsError if the target is outside [|l1-l2|, l1+l2].
        """
        d_sq = x * x + z * z
        d = np.sqrt(d_sq)
        d_max = self.l1 + self.l2
        d_min = abs(self.l1 - self.l2)
        if d > d_max + 1e-9 or d < d_min - 1e-9 or d < 1e-9:
            raise LegKinematicsError(
                f"Target ({x:.4f}, {z:.4f}) unreachable: dist={d:.4f}, "
                f"reachable range=[{d_min:.4f}, {d_max:.4f}]"
            )
        cos_knee = (d_sq - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        cos_knee = float(np.clip(cos_knee, -1.0, 1.0))
        knee_pitch = knee_bend_sign * np.arccos(cos_knee)

        # hip_pitch: angle of the (x, z) target in the (sin, cos) basis, minus
        # the angle the thigh makes with the hip->foot line.
        phi = np.arctan2(x, -z)
        psi = np.arctan2(self.l2 * np.sin(knee_pitch), self.l1 + self.l2 * np.cos(knee_pitch))
        hip_pitch = phi - psi

        ankle_pitch = -(hip_pitch + knee_pitch)  # keep the sole flat
        return LegPose(hip_pitch=float(hip_pitch), knee_pitch=float(knee_pitch),
                        ankle_pitch=float(ankle_pitch))

    def reachable(self, x: float, z: float) -> bool:
        d = np.hypot(x, z)
        return bool(abs(self.l1 - self.l2) - 1e-9 <= d <= self.l1 + self.l2 + 1e-9)


class BipedKinematics:
    """Pairs two PlanarLegKinematics instances (left/right) with a fixed
    hip-to-hip lateral offset, for whole-leg FK/IK from a pelvis-centred frame.
    """

    def __init__(self, thigh_length: float, shank_length: float, hip_width: float):
        self.leg = PlanarLegKinematics(thigh_length, shank_length)
        self.hip_width = float(hip_width)

    def foot_ik(self, side: str, x: float, z: float) -> LegPose:
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        return self.leg.inverse(x, z)

    def hip_lateral_offset(self, side: str) -> float:
        return self.hip_width / 2.0 if side == "left" else -self.hip_width / 2.0
