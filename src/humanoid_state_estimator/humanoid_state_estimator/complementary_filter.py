"""Gyro/accelerometer complementary filter for torso roll/pitch, plus a
simple foot-contact detector. Pure math, no ROS dependency (see
state_estimator_node.py for the rclpy wrapper that subscribes to /imu and
publishes the fused estimate).
"""
from dataclasses import dataclass
import numpy as np

G = 9.81


@dataclass
class Attitude:
    roll: float
    pitch: float


class ComplementaryFilter:
    """alpha close to 1.0 trusts the (low-noise, slow-drift) gyro integration;
    close to 0.0 trusts the (noisy, but drift-free) accelerometer tilt
    estimate. 0.98 is a common starting point for a ~100-200 Hz IMU.
    """

    def __init__(self, alpha: float = 0.98):
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("alpha must be in [0, 1]")
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0

    def update(self, gyro_xyz, accel_xyz, dt: float) -> Attitude:
        if dt <= 0:
            raise ValueError("dt must be positive")
        gx, gy, _gz = gyro_xyz
        ax, ay, az = accel_xyz

        roll_gyro = self.roll + gx * dt
        pitch_gyro = self.pitch + gy * dt

        roll_acc = np.arctan2(ay, az)
        pitch_acc = np.arctan2(-ax, np.hypot(ay, az))

        self.roll = float(self.alpha * roll_gyro + (1.0 - self.alpha) * roll_acc)
        self.pitch = float(self.alpha * pitch_gyro + (1.0 - self.alpha) * pitch_acc)
        return Attitude(roll=self.roll, pitch=self.pitch)

    def reset(self):
        self.roll = 0.0
        self.pitch = 0.0


def detect_contact(foot_height_m: float, grf_z_n: float,
                    height_threshold_m: float = 0.01, force_threshold_n: float = 20.0) -> bool:
    """A foot is "in contact" if it's near the ground OR already bearing
    significant vertical load -- using both avoids false "lost contact"
    readings from simulator foot-height noise alone.
    """
    return bool(foot_height_m <= height_threshold_m or grf_z_n >= force_threshold_n)
