#!/usr/bin/env python3
"""ROS 2 wrapper around mission_fsm.py -- the top-level "Stand -> Shift ->
Step -> Walk -> Stop" sequencer.

Subscribes:
  ~/start          std_msgs/Bool
  ~/begin_walk     std_msgs/Bool
  ~/stop           std_msgs/Bool
  safety_supervisor/state   std_msgs/String   (SafetyState name)

Publishes:
  ~/phase          std_msgs/String   current MissionPhase name
  ~/walk_requested std_msgs/Bool     forwarded to gait_planner / safety_supervisor

steps_remaining is a fixed parameter here (num_steps) rather than tracked
live against the gait planner's own countdown -- good enough to drive the
phase sequencing shown in the architecture diagram, but a tighter
integration would subscribe to gait_planner/steps_remaining directly; see
docs/STAGE_STATUS.md.

Parameters:
  autostart                bool    if true, this node fires ~/start and then
                                    ~/begin_walk itself (no second terminal
                                    needed) instead of waiting for someone to
                                    `ros2 topic pub` them -- see autostart_*_s
                                    below for the timing.
  autostart_delay_s        float   seconds after node start-up before firing
                                    the (simulated) ~/start command
  autostart_begin_walk_delay_s
                            float   additional seconds after `start` before
                                    firing (simulated) ~/begin_walk, giving
                                    the rest of the stack (sim, balance,
                                    state estimator) time to settle first
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32

from humanoid_mission_manager.mission_fsm import MissionManagerFSM, MissionCommand
from humanoid_safety.safety_fsm import SafetyState


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("num_steps", 6)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("autostart", False)
        self.declare_parameter("autostart_delay_s", 2.0)
        self.declare_parameter("autostart_begin_walk_delay_s", 2.0)

        self.fsm = MissionManagerFSM()
        self._safety_state = SafetyState.STAND
        self._start = False
        self._begin_walk = False
        self._stop = False
        self._num_steps = self.get_parameter("num_steps").value
        self._steps_remaining = 0
        self._walk_dispatched = False
        self._gait_started = False

        self.create_subscription(Bool, "~/start", self._on_start, 10)
        self.create_subscription(Bool, "~/begin_walk", self._on_begin_walk, 10)
        self.create_subscription(Bool, "~/stop", self._on_stop, 10)
        self.create_subscription(String, "/safety_supervisor/state", self._on_safety_state, 10)
        self.create_subscription(Int32, "/gait_planner/steps_remaining", self._on_gait_remaining, 10)

        self.phase_pub = self.create_publisher(String, "~/phase", 10)
        self.walk_requested_pub = self.create_publisher(Bool, "~/walk_requested", 10)

        rate = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / rate, self._tick)

        if self.get_parameter("autostart").value:
            delay = self.get_parameter("autostart_delay_s").value
            self._autostart_timer = self.create_timer(delay, self._autostart_fire_start)
            self.get_logger().info(
                f"mission_manager up, autostart armed: ~/start in {delay}s, "
                f"~/begin_walk {self.get_parameter('autostart_begin_walk_delay_s').value}s after that"
            )
        else:
            self.get_logger().info("mission_manager up, starting in IDLE (autostart disabled)")

    def _autostart_fire_start(self):
        self._autostart_timer.cancel()
        self.get_logger().info("[autostart] firing ~/start")
        self._start = True
        delay = self.get_parameter("autostart_begin_walk_delay_s").value
        self._autostart_timer = self.create_timer(delay, self._autostart_fire_begin_walk)

    def _autostart_fire_begin_walk(self):
        self._autostart_timer.cancel()
        self.get_logger().info("[autostart] firing ~/begin_walk")
        self._on_begin_walk(Bool(data=True))

    def _on_start(self, msg: Bool):
        self._start = msg.data

    def _on_begin_walk(self, msg: Bool):
        self._begin_walk = msg.data
        if msg.data and not self._walk_dispatched:
            self._steps_remaining = self._num_steps

    def _on_gait_remaining(self, msg: Int32):
        if self._walk_dispatched:
            if msg.data > 0:
                self._gait_started = True
            if self._gait_started:
                self._steps_remaining = max(0, msg.data)

    def _on_stop(self, msg: Bool):
        self._stop = msg.data

    def _on_safety_state(self, msg: String):
        try:
            self._safety_state = SafetyState[msg.data]
        except KeyError:
            pass  # unknown state name -- ignore rather than crash the mission loop

    def _tick(self):
        cmd = MissionCommand(
            start=self._start,
            begin_walk=self._begin_walk,
            stop=self._stop,
            steps_remaining=self._steps_remaining,
        )
        phase = self.fsm.step(cmd, self._safety_state)
        self.phase_pub.publish(String(data=phase.name))
        walk_allowed = (self._begin_walk and self._steps_remaining > 0 and
                        not self._stop and phase.name != "HOLD" and
                        self._safety_state in (SafetyState.STAND, SafetyState.WALK))
        if walk_allowed:
            self._walk_dispatched = True
        self.walk_requested_pub.publish(Bool(data=walk_allowed))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
