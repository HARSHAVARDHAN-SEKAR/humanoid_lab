#!/usr/bin/env python3
"""Free-base CoM reconstruction from ROS simulation feedback.

Uses:
  /mujoco_bridge/torso_pose
  /joint_states

Publishes:
  /state_estimator/com_position
  /state_estimator/com_velocity_estimate

Uses model-based forward kinematics, not live access to the
simulator's internal data. Torso pose is simulation ground truth;
this is not yet an IMU-based floating-base estimator.
"""

import math
from pathlib import Path

import mujoco
import numpy as np
import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    Vector3Stamped,
)
from sensor_msgs.msg import JointState


class ComEstimator(Node):

    def __init__(self):
        super().__init__("com_estimator")

        path = (
            Path(get_package_share_directory("humanoid_sim"))
            / "mujoco_models"
            / "humanoid_biped.xml"
        )

        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)

        if self.model.nq != 19:
            raise RuntimeError(
                "CoM estimator requires the 12-DOF free-base model"
            )

        self.joint_addresses = {}

        for jid in range(self.model.njnt):

            if self.model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
                continue

            name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                jid,
            )

            self.joint_addresses[name] = int(
                self.model.jnt_qposadr[jid]
            )

        if len(self.joint_addresses) != 12:
            raise RuntimeError("Expected 12 hinge joints")

        self._pose = None
        self._pose_received = None

        self._previous_com = None
        self._previous_time = None
        self._filtered_velocity = np.zeros(3)

        self.mass = float(self.model.body_mass.sum())

        self.create_subscription(
            PoseStamped,
            "/mujoco_bridge/torso_pose",
            self._on_pose,
            50,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joints,
            50,
        )

        self.position_pub = self.create_publisher(
            PointStamped,
            "/state_estimator/com_position",
            50,
        )

        self.velocity_pub = self.create_publisher(
            Vector3Stamped,
            "/state_estimator/com_velocity_estimate",
            50,
        )

        self.get_logger().info(
            "Free-base CoM estimator initialized"
        )

    def _on_pose(self, msg):

        if msg.header.frame_id != "world":
            return

        self._pose = msg.pose
        self._pose_received = self.get_clock().now()

    def _on_joints(self, msg):

        if self._pose is None:
            return

        now = self.get_clock().now()

        age = (
            now - self._pose_received
        ).nanoseconds * 1e-9

        if age < 0 or age > 0.15:
            return

        if len(msg.name) != len(msg.position):
            return

        measurements = dict(
            zip(msg.name, msg.position)
        )

        if not all(
            name in measurements
            and math.isfinite(measurements[name])
            for name in self.joint_addresses
        ):
            return

        p = self._pose.position
        q = self._pose.orientation

        quaternion = np.array(
            [q.w, q.x, q.y, q.z],
            dtype=float,
        )

        norm = float(np.linalg.norm(quaternion))

        if norm < 1e-8 or not math.isfinite(norm):
            return

        quaternion /= norm

        self.data.qpos[0:3] = [
            p.x, p.y, p.z
        ]

        self.data.qpos[3:7] = quaternion

        for name, address in self.joint_addresses.items():
            self.data.qpos[address] = measurements[name]

        mujoco.mj_forward(self.model, self.data)

        com = (
            self.model.body_mass[:, None]
            * self.data.xipos
        ).sum(axis=0) / self.mass

        if not np.all(np.isfinite(com)):
            return

        timestamp = now.nanoseconds * 1e-9

        if self._previous_com is not None:
            dt = timestamp - self._previous_time

            if 0.002 <= dt <= 0.25:
                raw_velocity = (
                    com - self._previous_com
                ) / dt

                # Light low-pass filter for feedback.
                self._filtered_velocity = (
                    0.8 * self._filtered_velocity
                    + 0.2 * raw_velocity
                )
            else:
                self._filtered_velocity[:] = 0.0

        self._previous_com = com.copy()
        self._previous_time = timestamp

        position = PointStamped()
        position.header.stamp = now.to_msg()
        position.header.frame_id = "world"

        position.point.x = float(com[0])
        position.point.y = float(com[1])
        position.point.z = float(com[2])

        velocity = Vector3Stamped()
        velocity.header = position.header

        velocity.vector.x = float(self._filtered_velocity[0])
        velocity.vector.y = float(self._filtered_velocity[1])
        velocity.vector.z = float(self._filtered_velocity[2])

        self.position_pub.publish(position)
        self.velocity_pub.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    node = ComEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
