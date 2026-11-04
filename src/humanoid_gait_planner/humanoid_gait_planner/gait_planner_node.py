#!/usr/bin/env python3
"""ROS 2 wrapper around footstep_planner.py + swing_trajectory.py (Stage 3-4).

On a ~begin_walk (std_msgs/Bool, true) command, plans a straight-line
footstep sequence (humanoid_gait_planner.footstep_planner.plan_straight_walk)
and then, once per step, drives a SwingFootTrajectory for the swinging foot
at ~control_rate_hz, publishing:

  ~/swing_foot        std_msgs/String       "left" | "right" | "none"
  ~/swing_target       geometry_msgs/Point   current swing-foot (x,y,z) target,
                                              in the pelvis/odom frame
  ~/steps_remaining    std_msgs/Int32

This node issues the *nominal* plan only; humanoid_balance_controller is
responsible for blending each nominal footstep toward the capture point
before it's actually commanded to the legs (Stage 5 push recovery) -- that
blend happens downstream, not here, so this node's plan is reusable with
or without reactive correction.

Parameter `autostart_delay_s` (default 0.0, i.e. off): if > 0, this node
fires ~/begin_walk itself after that many seconds instead of waiting for
`ros2 topic pub` from a second terminal -- useful when running Stage 3 on
its own (humanoid_mission_manager, which only exists from Stage 4 on, has
the equivalent `autostart` parameter for the full stack).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import Point

from humanoid_gait_planner.footstep_planner import plan_straight_walk
from humanoid_gait_planner.swing_trajectory import SwingFootTrajectory


class GaitPlannerNode(Node):
    def __init__(self):
        super().__init__("gait_planner")

        self.declare_parameter("num_steps", 6)
        self.declare_parameter("stride_length_m", 0.18)
        self.declare_parameter("step_width_m", 0.20)
        self.declare_parameter("step_height_m", 0.04)
        self.declare_parameter("step_duration_s", 0.6)
        self.declare_parameter("start_foot", "right")
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("autostart_delay_s", 0.0)

        self._steps = []
        self._current_step_idx = -1
        self._swing_traj = None
        self._swing_start_time = None
        self._active = False
        self._feet_xy = {"right": (0.0, -0.1), "left": (0.0, 0.1)}
        self._safety_ok = True

        self.create_subscription(Bool, "~/begin_walk", self._on_begin_walk, 10)
        self.create_subscription(String, "/safety_supervisor/state", self._on_safety, 10)

        self.swing_foot_pub = self.create_publisher(String, "~/swing_foot", 10)
        self.swing_target_pub = self.create_publisher(Point, "~/swing_target", 50)
        self.steps_remaining_pub = self.create_publisher(Int32, "~/steps_remaining", 10)

        rate = self.get_parameter("control_rate_hz").value
        self.create_timer(1.0 / rate, self._tick)

        autostart_delay = self.get_parameter("autostart_delay_s").value
        if autostart_delay > 0.0:
            self._autostart_timer = self.create_timer(autostart_delay, self._autostart_fire)
            self.get_logger().info(f"gait_planner up, autostart armed: ~begin_walk in {autostart_delay}s")
        else:
            self.get_logger().info("gait_planner up, waiting for ~begin_walk")

    def _autostart_fire(self):
        self._autostart_timer.cancel()
        self.get_logger().info("[autostart] firing ~begin_walk")
        self._on_begin_walk(Bool(data=True))

    def _on_safety(self, msg: String):
        self._safety_ok = msg.data in ("STAND", "WALK")
        if not self._safety_ok and self._active:
            self._active = False
            self.swing_foot_pub.publish(String(data="none"))

    def _on_begin_walk(self, msg: Bool):
        if not msg.data:
            self._active = False
            self.swing_foot_pub.publish(String(data="none"))
            return
        if self._active or not self._safety_ok:
            return
        self._feet_xy = {"right": (0.0, -0.1), "left": (0.0, 0.1)}
        self._steps = plan_straight_walk(
            num_steps=self.get_parameter("num_steps").value,
            stride_length=self.get_parameter("stride_length_m").value,
            step_width=self.get_parameter("step_width_m").value,
            start_foot=self.get_parameter("start_foot").value,
        )
        self._current_step_idx = 0
        self._active = True
        self._start_swing(self._steps[0])
        self.get_logger().info(f"planned {len(self._steps)} steps, starting swing")

    def _start_swing(self, step):
        prev_xy = self._feet_xy[step.foot]
        start = (prev_xy[0], prev_xy[1], 0.0)
        end = (step.x, step.y, 0.0)
        self._swing_traj = SwingFootTrajectory(
            start, end,
            step_height=self.get_parameter("step_height_m").value,
            duration=self.get_parameter("step_duration_s").value,
        )
        self._swing_start_time = self.get_clock().now()
        self.swing_foot_pub.publish(String(data=step.foot))

    def _tick(self):
        if not self._active:
            self.steps_remaining_pub.publish(Int32(data=0))
            return

        elapsed = (self.get_clock().now() - self._swing_start_time).nanoseconds * 1e-9
        state = self._swing_traj.state(elapsed)
        pt = Point(x=state.x, y=state.y, z=state.z)
        self.swing_target_pub.publish(pt)

        # Includes the current in-progress step. Mission completion is
        # feedback-driven, not a synthetic 10 Hz countdown.
        remaining = len(self._steps) - self._current_step_idx
        self.steps_remaining_pub.publish(Int32(data=remaining))

        if elapsed >= self._swing_traj.duration:
            prev = self._steps[self._current_step_idx]
            self._feet_xy[prev.foot] = (prev.x, prev.y)
            self._current_step_idx += 1
            if self._current_step_idx >= len(self._steps):
                self._active = False
                self.swing_foot_pub.publish(String(data="none"))
                self.steps_remaining_pub.publish(Int32(data=0))
                self.get_logger().info("planned swing sequence complete (not proof of physical walking)")
                return
            self._start_swing(self._steps[self._current_step_idx])


def main(args=None):
    rclpy.init(args=args)
    node = GaitPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
