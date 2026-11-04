#!/usr/bin/env python3
"""Logs a handful of key topics to a CSV file for offline analysis
(humanoid_benchmarks.metrics / plot_results), as an alternative to a full
rosbag when you just want the numbers the proposal's validation dashboard
asks for (standing duration, tracking error, push-recovery outcomes).

Subscribes:
  /joint_states                 sensor_msgs/JointState
  /imu/torso                    sensor_msgs/Imu
  safety_supervisor/state       std_msgs/String
  mission_manager/phase         std_msgs/String

Writes one row per /joint_states message (the fastest of the above) to
~/log_path (default: /tmp/humanoid_lab_log.csv), carrying forward the most
recent value of the slower topics.
"""
import csv
import pathlib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64, Bool
import math
from std_msgs.msg import String


class LogRecorderNode(Node):
    def __init__(self):
        super().__init__("log_recorder")

        self.declare_parameter("log_path", "/tmp/humanoid_lab_log.csv")
        log_path = pathlib.Path(self.get_parameter("log_path").value)
        log_path.parent.mkdir(parents=True, exist_ok=True)

        self._file = open(log_path, "w", newline="")
        self._writer = None  # created on first joint_states message, once we know joint names
        self._latest_pitch_rad = float('nan')
        self._torso_height_m = float('nan')
        self._commanded = {}
        self._left_contact = False
        self._right_contact = False
        self._safety_state = ""
        self._mission_phase = ""

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 50)
        self.create_subscription(Imu, "/imu/torso", self._on_imu, 50)
        self.create_subscription(JointState, "/mujoco_bridge/joint_commands", self._on_command, 50)
        self.create_subscription(Float64, "/mujoco_bridge/torso_height", self._on_height, 50)
        self.create_subscription(Bool, "/state_estimator/contact_left", self._on_left_contact, 20)
        self.create_subscription(Bool, "/state_estimator/contact_right", self._on_right_contact, 20)
        self.create_subscription(String, "safety_supervisor/state", self._on_safety_state, 10)
        self.create_subscription(String, "mission_manager/phase", self._on_mission_phase, 10)

        self.get_logger().info(f"log_recorder writing to {log_path}")

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self._latest_pitch_rad = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))

    def _on_command(self, msg: JointState):
        self._commanded = dict(zip(msg.name, msg.position))

    def _on_height(self, msg: Float64):
        self._torso_height_m = float(msg.data)

    def _on_left_contact(self, msg: Bool):
        self._left_contact = bool(msg.data)

    def _on_right_contact(self, msg: Bool):
        self._right_contact = bool(msg.data)

    def _on_safety_state(self, msg: String):
        self._safety_state = msg.data

    def _on_mission_phase(self, msg: String):
        self._mission_phase = msg.data

    def _on_joint_states(self, msg: JointState):
        if self._writer is None:
            fieldnames = ["stamp_sec"] + [f"pos_{n}" for n in msg.name] + \
                         [f"cmd_{n}" for n in msg.name] + \
                         ["torso_height_m", "torso_pitch_rad", "contact_left",
                          "contact_right", "safety_state", "mission_phase"]
            self._writer = csv.DictWriter(self._file, fieldnames=fieldnames)
            self._writer.writeheader()

        row = {"stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}
        for n, p in zip(msg.name, msg.position):
            row[f"pos_{n}"] = p
        for n in msg.name:
            row[f"cmd_{n}"] = self._commanded.get(n, "")
        row["torso_height_m"] = self._torso_height_m
        row["torso_pitch_rad"] = self._latest_pitch_rad
        row["contact_left"] = int(self._left_contact)
        row["contact_right"] = int(self._right_contact)
        row["safety_state"] = self._safety_state
        row["mission_phase"] = self._mission_phase
        self._writer.writerow(row)
        self._file.flush()

    def destroy_node(self):
        self._file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LogRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
