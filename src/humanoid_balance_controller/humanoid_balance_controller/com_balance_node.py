#!/usr/bin/env python3
"""Stage 6I: opt-in ROS2 ankle CoM-PD for simplified free-base biped.

This node is the sole actuator publisher in the dedicated Stage 6I launch.
Uses ROS reconstructed CoM and requires live joint and safety feedback.
Last-target hold on fault is not a guarantee of physical balance.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from humanoid_balance_controller.com_pd import ComPitchPD
from humanoid_motion_controller.command_model import CommandComposer, JOINT_NAMES, JOINT_LIMITS


class ComBalanceNode(Node):
    PERIOD = 0.01
    MAX_AGE = 0.25
    TRACKING_LIMIT = 0.70

    def __init__(self):
        super().__init__("com_balance_controller")
        self.nominal = CommandComposer().standing_command()
        self.last_command = dict(self.nominal)
        self.pd = ComPitchPD()
        self.created = self.get_clock().now()
        self.last_seen = {}
        self.position = None
        self.velocity = None
        self.joints = None
        self.safety = None
        self.activated = False
        self.fault_latched = False

        self.create_subscription(PointStamped, "/state_estimator/com_position", self.on_position, 50)
        self.create_subscription(Vector3Stamped, "/state_estimator/com_velocity_estimate", self.on_velocity, 50)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
        self.create_subscription(String, "/safety_supervisor/state", self.on_safety, 20)

        self.command_pub = self.create_publisher(JointState, "/mujoco_bridge/joint_commands", 50)
        self.mode_pub = self.create_publisher(String, "/com_balance_controller/mode", 20)
        self.correction_pub = self.create_publisher(Float64, "/com_balance_controller/pitch_correction", 20)
        self.error_pub = self.create_publisher(Float64, "/motion_controller/joint_tracking_error", 20)
        self.fault_pub = self.create_publisher(Bool, "/motion_controller/ik_fault", 20)
        self.create_timer(self.PERIOD, self.tick)
        self.get_logger().info("Stage 6I CoM-PD controller initialized (experimental)")

    def mark(self, name):
        self.last_seen[name] = self.get_clock().now()

    def on_position(self, msg):
        if msg.header.frame_id == "world" and math.isfinite(msg.point.x):
            self.position = float(msg.point.x)
            self.mark("position")

    def on_velocity(self, msg):
        if msg.header.frame_id == "world" and math.isfinite(msg.vector.x):
            self.velocity = float(msg.vector.x)
            self.mark("velocity")

    def on_joints(self, msg):
        if len(msg.name) != len(msg.position):
            return
        measurements = dict(zip(msg.name, msg.position))
        if all(j in measurements and math.isfinite(measurements[j]) for j in JOINT_NAMES):
            self.joints = measurements
            self.mark("joints")

    def on_safety(self, msg):
        self.safety = msg.data
        self.mark("safety")

    def feedback_ready(self, now):
        for key in ("position", "velocity", "joints", "safety"):
            t = self.last_seen.get(key)
            if t is None:
                return False
            age = (now - t).nanoseconds * 1e-9
            if not 0.0 <= age <= self.MAX_AGE:
                return False
        return True

    def tick(self):
        now = self.get_clock().now()
        ready = self.feedback_ready(now)
        if self.activated and (not ready or self.safety != "STAND"):
            self.fault_latched = True

        elapsed = (now - self.created).nanoseconds * 1e-9
        cmd = dict(self.last_command)
        mode = "WAITING_FOR_FEEDBACK"
        if self.fault_latched:
            mode = "SAFE_HOLD"
        elif not ready or self.safety != "STAND":
            mode = "WAITING_FOR_FEEDBACK"
        elif elapsed < 1.5:
            cmd = dict(self.nominal)
            mode = "SETTLING"
        else:
            if not self.activated:
                self.activated = True
            try:
                correction = self.pd.update(self.position, self.velocity, self.PERIOD)
                cmd = dict(self.nominal)
                lo, hi = JOINT_LIMITS["ankle_pitch"]
                for side in ("left", "right"):
                    joint = f"{side}_ankle_pitch_joint"
                    cmd[joint] = max(lo, min(hi, self.nominal[joint] + correction))
                mode = "ACTIVE_COM_PD"
            except ValueError:
                self.fault_latched = True
                mode = "SAFE_HOLD"
                cmd = dict(self.last_command)

        error = 0.0
        if self.joints is not None:
            error = max(abs(cmd[j] - self.joints[j]) for j in JOINT_NAMES)
            if self.activated and error > self.TRACKING_LIMIT:
                self.fault_latched = True
                mode = "SAFE_HOLD"
                cmd = dict(self.last_command)

        if not self.fault_latched:
            self.last_command = dict(cmd)

        output = JointState()
        output.header.stamp = now.to_msg()
        output.name = list(JOINT_NAMES)
        output.position = [cmd[j] for j in JOINT_NAMES]
        self.command_pub.publish(output)
        self.mode_pub.publish(String(data=mode))
        self.correction_pub.publish(Float64(data=float(self.pd.correction)))
        self.error_pub.publish(Float64(data=float(error)))
        self.fault_pub.publish(Bool(data=self.fault_latched))


def main(args=None):
    rclpy.init(args=args)
    node = ComBalanceNode()
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
