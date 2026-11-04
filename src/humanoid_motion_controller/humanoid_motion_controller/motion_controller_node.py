
#!/usr/bin/env python3
"""HUMANOID-LAB authoritative joint motion controller.

Pipeline:
  Stamped foot target
      -> Leg inverse kinematics
      -> Balance corrections
      -> Joint limits
      -> Safety gate
      -> MuJoCo actuator commands

The gait target identifies its foot inside one message.

This controller is intended for the simplified supported biped.
It is not a whole-body dynamic walking controller.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Float64, Bool

from humanoid_motion_controller.command_model import (
    CommandComposer,
    FootTarget,
    JOINT_NAMES,
)


class MotionControllerNode(Node):

    def __init__(self):
        super().__init__("motion_controller")

        # Parameters
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter(
            "joint_feedback_timeout_s", 0.35
        )
        self.declare_parameter("base_height_m", 0.58)
        self.declare_parameter("allow_motion", True)
        self.declare_parameter("safety_required", False)

        self.composer = CommandComposer(
            base_height_m=float(
                self.get_parameter("base_height_m").value
            )
        )

        # Controller state
        self._last_feedback = None
        self._last_safety = None
        self._safety = "STAND"

        self._pitch_corr = 0.0
        self._roll_corr = 0.0

        self._allow_motion = bool(
            self.get_parameter("allow_motion").value
        )

        self._feedback_timeout = float(
            self.get_parameter(
                "joint_feedback_timeout_s"
            ).value
        )

        self._safety_required = bool(
            self.get_parameter("safety_required").value
        )

        # Local target faults remain latched until node restart.
        self._target_fault = False

        # Joint feedback
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_feedback,
            50,
        )

        # New atomic gait target.
        self.create_subscription(
            TransformStamped,
            "/gait_planner/swing_target_transform",
            self._on_swing_target_transform,
            20,
        )

        # Balance corrections
        self.create_subscription(
            Float64,
            "/balance_controller/ankle_pitch_correction",
            self._on_pitch,
            20,
        )

        self.create_subscription(
            Float64,
            "/balance_controller/ankle_roll_correction",
            self._on_roll,
            20,
        )

        # Safety supervisor
        self.create_subscription(
            String,
            "/safety_supervisor/state",
            self._on_safety,
            20,
        )

        # Final joint commands
        self.command_pub = self.create_publisher(
            JointState,
            "/mujoco_bridge/joint_commands",
            50,
        )

        # Monitoring
        self.error_pub = self.create_publisher(
            Float64,
            "/motion_controller/joint_tracking_error",
            20,
        )

        self.mode_pub = self.create_publisher(
            String,
            "/motion_controller/mode",
            20,
        )

        self.ik_fault_pub = self.create_publisher(
            Bool,
            "/motion_controller/ik_fault",
            20,
        )

        # Control timer
        rate = float(
            self.get_parameter("rate_hz").value
        )

        self.create_timer(
            1.0 / rate,
            self._tick,
        )

        self.get_logger().info(
            "Motion controller ready: "
            "atomic gait targets + IK + safety enabled"
        )

    def _on_feedback(self, msg: JointState):

        if len(msg.name) != len(msg.position):
            self.get_logger().warn(
                "Ignoring malformed joint feedback"
            )
            return

        feedback = dict(
            zip(msg.name, msg.position)
        )

        self.composer.update_feedback(feedback)

        self._last_feedback = self.get_clock().now()

    def _on_swing_target_transform(
        self,
        msg: TransformStamped,
    ):

        # Avoid accepting new motion after a target fault.
        if self._target_fault:
            return

        # Supported model assumes a fixed world-frame torso.
        if msg.header.frame_id != "world":
            self.get_logger().error(
                f"Unexpected gait frame: {msg.header.frame_id}"
            )
            self._target_fault = True
            return

        foot_map = {
            "left_foot_target": "left",
            "right_foot_target": "right",
        }

        side = foot_map.get(msg.child_frame_id)

        if side is None:
            self.get_logger().error(
                f"Unknown swing foot: {msg.child_frame_id}"
            )
            self._target_fault = True
            return

        position = msg.transform.translation

        target = FootTarget(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
        )

        try:
            self.composer.set_foot(
                side,
                target,
            )

        except ValueError as exc:
            self.get_logger().error(
                f"Invalid gait target: {exc}"
            )
            self._target_fault = True

    def _on_pitch(self, msg: Float64):
        self._pitch_corr = float(msg.data)

    def _on_roll(self, msg: Float64):
        self._roll_corr = float(msg.data)

    def _on_safety(self, msg: String):
        self._last_safety = self.get_clock().now()
        self._safety = msg.data

    def _tick(self):

        now = self.get_clock().now()

        # Check feedback freshness.
        fresh = (
            self._last_feedback is not None
            and (
                now - self._last_feedback
            ).nanoseconds * 1e-9 <= self._feedback_timeout
        )

        safety = self._safety

        # Supervisor required for Stage 4 and Stage 5.
        if self._safety_required:

            supervisor_stale = (
                self._last_safety is None
                or (
                    now - self._last_safety
                ).nanoseconds * 1e-9 > 0.5
            )

            if supervisor_stale:
                safety = "SAFE_HOLD"

        # A malformed or unsupported target inhibits motion.
        if self._target_fault:
            safety = "SAFE_HOLD"

        # Compose all 12 joint targets.
        result = self.composer.update(
            safety=safety,
            feedback_fresh=fresh,
            pitch_corr=self._pitch_corr,
            roll_corr=self._roll_corr,
            allow_motion=self._allow_motion,
        )

        # Publish final actuator reference.
        cmd = JointState()

        cmd.header.stamp = now.to_msg()
        cmd.name = list(JOINT_NAMES)

        cmd.position = [
            result.positions[joint]
            for joint in JOINT_NAMES
        ]

        self.command_pub.publish(cmd)

        # Publish diagnostics.
        self.error_pub.publish(
            Float64(
                data=float(
                    result.max_tracking_error_rad
                )
            )
        )

        self.mode_pub.publish(
            String(data=result.mode)
        )

        self.ik_fault_pub.publish(
            Bool(
                data=(
                    result.mode == "IK_FAULT_HOLD"
                    or self._target_fault
                )
            )
        )


def main(args=None):
    rclpy.init(args=args)

    node = MotionControllerNode()

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

