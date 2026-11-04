#!/usr/bin/env python3
"""Authoritative ROS 2 joint command composition and safety gate.

Outputs /mujoco_bridge/joint_commands. No other controller should publish
commands to that topic during stage2-5 launches. Sim bridge also applies an
independent command-timeout hold to its internal motor targets.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64, Bool

from humanoid_motion_controller.command_model import CommandComposer, FootTarget, JOINT_NAMES


class MotionControllerNode(Node):
    def __init__(self):
        super().__init__("motion_controller")
        self.declare_parameter("rate_hz", 100.)
        self.declare_parameter("joint_feedback_timeout_s", 0.35)
        self.declare_parameter("base_height_m", 0.58)
        self.declare_parameter("allow_motion", True)
        self.declare_parameter("safety_required", False)
        self.composer = CommandComposer(base_height_m=float(self.get_parameter("base_height_m").value))
        self._last_feedback = None
        self._last_safety = None
        self._safety = "STAND"
        self._swing_foot = "none"
        self._pitch_corr = 0.0
        self._roll_corr = 0.0
        self._allow_motion = bool(self.get_parameter("allow_motion").value)
        self._feedback_timeout = float(self.get_parameter("joint_feedback_timeout_s").value)
        self._safety_required = bool(self.get_parameter("safety_required").value)

        self.create_subscription(JointState, "/joint_states", self._on_feedback, 50)
        self.create_subscription(String, "/gait_planner/swing_foot", self._on_swing_foot, 10)
        self.create_subscription(Point, "/gait_planner/swing_target", self._on_swing_target, 20)
        self.create_subscription(Float64, "/balance_controller/ankle_pitch_correction", self._on_pitch, 20)
        self.create_subscription(Float64, "/balance_controller/ankle_roll_correction", self._on_roll, 20)
        self.create_subscription(String, "/safety_supervisor/state", self._on_safety, 20)
        self.command_pub = self.create_publisher(JointState, "/mujoco_bridge/joint_commands", 50)
        self.error_pub = self.create_publisher(Float64, "/motion_controller/joint_tracking_error", 20)
        self.mode_pub = self.create_publisher(String, "/motion_controller/mode", 20)
        self.ik_fault_pub = self.create_publisher(Bool, "/motion_controller/ik_fault", 20)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _on_feedback(self, msg):
        self.composer.update_feedback(dict(zip(msg.name, msg.position)))
        self._last_feedback = self.get_clock().now()

    def _on_swing_foot(self, msg):
        self._swing_foot = msg.data

    def _on_swing_target(self, msg):
        if self._swing_foot in ("left", "right"):
            try:
                self.composer.set_foot(self._swing_foot, FootTarget(msg.x, msg.y, msg.z))
            except ValueError as exc:
                self.get_logger().error(f"Invalid gait target: {exc}")
                self._safety = "SAFE_HOLD"

    def _on_pitch(self, msg):
        self._pitch_corr = float(msg.data)

    def _on_roll(self, msg):
        self._roll_corr = float(msg.data)

    def _on_safety(self, msg):
        self._last_safety = self.get_clock().now()
        self._safety = msg.data

    def _tick(self):
        now = self.get_clock().now()
        fresh = self._last_feedback is not None and (
            now - self._last_feedback).nanoseconds * 1e-9 <= self._feedback_timeout
        safety = self._safety
        if self._safety_required and (self._last_safety is None or
                (now - self._last_safety).nanoseconds * 1e-9 > 0.5):
            safety = "SAFE_HOLD"
        result = self.composer.update(
            safety=safety, feedback_fresh=fresh, pitch_corr=self._pitch_corr,
            roll_corr=self._roll_corr, allow_motion=self._allow_motion)
        cmd = JointState()
        cmd.header.stamp = now.to_msg()
        cmd.name = list(JOINT_NAMES)
        cmd.position = [result.positions[j] for j in JOINT_NAMES]
        self.command_pub.publish(cmd)
        self.error_pub.publish(Float64(data=result.max_tracking_error_rad))
        self.mode_pub.publish(String(data=result.mode))
        self.ik_fault_pub.publish(Bool(data=result.mode == "IK_FAULT_HOLD"))


def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
