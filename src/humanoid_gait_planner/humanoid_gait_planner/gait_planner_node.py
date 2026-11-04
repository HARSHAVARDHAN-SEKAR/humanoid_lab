
#!/usr/bin/env python3

"""ROS 2 humanoid gait planner.

Generates short supported swing-foot trajectories.

Publishes:
    /gait_planner/swing_foot
    /gait_planner/swing_target
    /gait_planner/swing_target_transform
    /gait_planner/steps_remaining

The atomic TransformStamped contains both the foot identity
and target position. This prevents mismatched foot/target
messages from reaching the motion controller.

This planner generates trajectories only.
It does not prove physically successful walking.
"""

import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool, String, Int32

from geometry_msgs.msg import Point, TransformStamped

from humanoid_gait_planner.footstep_planner import (
    plan_straight_walk,
)

from humanoid_gait_planner.swing_trajectory import (
    SwingFootTrajectory,
)


class GaitPlannerNode(Node):

    def __init__(self):

        super().__init__("gait_planner")

        # ----------------------------------------------
        # Parameters
        # ----------------------------------------------

        self.declare_parameter("num_steps", 6)
        self.declare_parameter("stride_length_m", 0.18)
        self.declare_parameter("step_width_m", 0.20)
        self.declare_parameter("step_height_m", 0.04)
        self.declare_parameter("step_duration_s", 0.6)
        self.declare_parameter("start_foot", "right")
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("autostart_delay_s", 0.0)

        # ----------------------------------------------
        # Internal state
        # ----------------------------------------------

        self._steps = []
        self._current_step_idx = -1

        self._swing_traj = None
        self._swing_start_time = None

        self._active = False
        self._safety_ok = True

        self._feet_xy = {
            "right": (0.0, -0.1),
            "left": (0.0, 0.1),
        }

        # Prevent continuously published True messages
        # from restarting a completed sequence.
        #
        # Separate manual one-shot requests remain possible
        # after a quiet interval.

        self._start_latched = False
        self._last_true_request_ns = None
        self._request_quiet_ns = 750_000_000

        # ----------------------------------------------
        # Subscribers
        # ----------------------------------------------

        self.create_subscription(
            Bool,
            "~/begin_walk",
            self._on_begin_walk,
            10,
        )

        self.create_subscription(
            String,
            "/safety_supervisor/state",
            self._on_safety,
            10,
        )

        # ----------------------------------------------
        # Publishers
        # ----------------------------------------------

        self.swing_foot_pub = self.create_publisher(
            String,
            "~/swing_foot",
            10,
        )

        self.swing_target_pub = self.create_publisher(
            Point,
            "~/swing_target",
            50,
        )

        self.swing_target_transform_pub = (
            self.create_publisher(
                TransformStamped,
                "~/swing_target_transform",
                50,
            )
        )

        self.steps_remaining_pub = self.create_publisher(
            Int32,
            "~/steps_remaining",
            10,
        )

        # ----------------------------------------------
        # Control timer
        # ----------------------------------------------

        rate = float(
            self.get_parameter("control_rate_hz").value
        )

        if rate <= 0.0:
            raise ValueError(
                "control_rate_hz must be positive"
            )

        self.create_timer(
            1.0 / rate,
            self._tick,
        )

        # ----------------------------------------------
        # Optional independent autostart
        # ----------------------------------------------

        autostart_delay = float(
            self.get_parameter("autostart_delay_s").value
        )

        if autostart_delay > 0.0:

            self._autostart_timer = self.create_timer(
                autostart_delay,
                self._autostart_fire,
            )

            self.get_logger().info(
                "Gait planner ready. "
                f"Autostart in {autostart_delay}s"
            )

        else:

            self.get_logger().info(
                "Gait planner ready. "
                "Waiting for begin_walk command."
            )

    # --------------------------------------------------
    # Autostart
    # --------------------------------------------------

    def _autostart_fire(self):

        self._autostart_timer.cancel()

        self.get_logger().info(
            "Starting automatic stepping sequence"
        )

        self._on_begin_walk(
            Bool(data=True)
        )

    # --------------------------------------------------
    # Safety
    # --------------------------------------------------

    def _on_safety(self, msg: String):

        self._safety_ok = msg.data in (
            "STAND",
            "WALK",
        )

        if not self._safety_ok and self._active:

            self._active = False

            self.swing_foot_pub.publish(
                String(data="none")
            )

            self.steps_remaining_pub.publish(
                Int32(data=0)
            )

            self.get_logger().warning(
                "Stepping stopped by safety supervisor: "
                f"{msg.data}"
            )

    # --------------------------------------------------
    # Begin walking
    # --------------------------------------------------

    def _on_begin_walk(self, msg: Bool):

        if not msg.data:

            self._start_latched = False
            self._last_true_request_ns = None

            if self._active:
                self._active = False

                self.swing_foot_pub.publish(
                    String(data="none")
                )

                self.steps_remaining_pub.publish(
                    Int32(data=0)
                )

            return

        now_ns = self.get_clock().now().nanoseconds

        # A continuous high signal represents one request.
        # Allow a new one-shot request after a quiet interval.

        if self._last_true_request_ns is not None:

            quiet_ns = (
                now_ns - self._last_true_request_ns
            )

            if quiet_ns >= self._request_quiet_ns:
                self._start_latched = False

        self._last_true_request_ns = now_ns

        if self._start_latched:
            return

        # Ignore a new start when already stepping.
        # No repeated warning messages.

        if self._active:
            return

        if not self._safety_ok:

            self.get_logger().warning(
                "Walking request rejected: "
                "safety supervisor does not allow motion."
            )

            return

        self._start_latched = True

        # Reset the nominal starting-foot positions.

        self._feet_xy = {
            "right": (0.0, -0.1),
            "left": (0.0, 0.1),
        }

        # Generate nominal footsteps.

        self._steps = plan_straight_walk(
            num_steps=int(
                self.get_parameter("num_steps").value
            ),
            stride_length=float(
                self.get_parameter("stride_length_m").value
            ),
            step_width=float(
                self.get_parameter("step_width_m").value
            ),
            start_foot=str(
                self.get_parameter("start_foot").value
            ),
        )

        if not self._steps:

            self._start_latched = False

            self.get_logger().warning(
                "No footsteps were generated."
            )

            return

        self._current_step_idx = 0
        self._active = True

        self._start_swing(
            self._steps[0]
        )

        self.get_logger().info(
            f"Planned {len(self._steps)} supported steps"
        )

    # --------------------------------------------------
    # Start one swing trajectory
    # --------------------------------------------------

    def _start_swing(self, step):

        previous_xy = self._feet_xy[step.foot]

        start = (
            previous_xy[0],
            previous_xy[1],
            0.0,
        )

        end = (
            step.x,
            step.y,
            0.0,
        )

        self._swing_traj = SwingFootTrajectory(
            start,
            end,
            step_height=float(
                self.get_parameter("step_height_m").value
            ),
            duration=float(
                self.get_parameter("step_duration_s").value
            ),
        )

        self._swing_start_time = (
            self.get_clock().now()
        )

        self.swing_foot_pub.publish(
            String(data=step.foot)
        )

        self.get_logger().info(
            f"Step {self._current_step_idx + 1}/"
            f"{len(self._steps)}: {step.foot} foot"
        )

    # --------------------------------------------------
    # Publish atomic foot target
    # --------------------------------------------------

    def _publish_atomic_target(
        self,
        foot,
        x,
        y,
        z,
    ):

        msg = TransformStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = "world"

        msg.child_frame_id = (
            f"{foot}_foot_target"
        )

        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.translation.z = float(z)

        # Identity orientation.
        # TransformStamped is used as an atomic command
        # message, not as a broadcast TF transform.

        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 1.0

        self.swing_target_transform_pub.publish(
            msg
        )

    # --------------------------------------------------
    # Main update loop
    # --------------------------------------------------

    def _tick(self):

        if not self._active:

            self.steps_remaining_pub.publish(
                Int32(data=0)
            )

            return

        now = self.get_clock().now()

        elapsed = (
            now - self._swing_start_time
        ).nanoseconds * 1e-9

        state = self._swing_traj.state(
            elapsed
        )

        current_step = self._steps[
            self._current_step_idx
        ]

        foot = current_step.foot

        # Legacy target topic: retained for monitoring.

        self.swing_target_pub.publish(
            Point(
                x=float(state.x),
                y=float(state.y),
                z=float(state.z),
            )
        )

        # Atomic target topic consumed by the
        # updated motion controller.

        self._publish_atomic_target(
            foot=foot,
            x=state.x,
            y=state.y,
            z=state.z,
        )

        remaining = (
            len(self._steps)
            - self._current_step_idx
        )

        self.steps_remaining_pub.publish(
            Int32(data=remaining)
        )

        # ----------------------------------------------
        # Swing completion
        # ----------------------------------------------

        if elapsed >= self._swing_traj.duration:

            completed_step = self._steps[
                self._current_step_idx
            ]

            self._feet_xy[completed_step.foot] = (
                completed_step.x,
                completed_step.y,
            )

            self._current_step_idx += 1

            # ------------------------------------------
            # Entire sequence completed
            # ------------------------------------------

            if self._current_step_idx >= len(
                self._steps
            ):

                self._active = False

                self.swing_foot_pub.publish(
                    String(data="none")
                )

                self.steps_remaining_pub.publish(
                    Int32(data=0)
                )

                self.get_logger().info(
                    "Commanded swing sequence complete. "
                    "Physical step success requires "
                    "feedback validation."
                )

                return

            # ------------------------------------------
            # Start next step
            # ------------------------------------------

            next_step = self._steps[
                self._current_step_idx
            ]

            self._start_swing(
                next_step
            )


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

