#!/usr/bin/env python3
"""Fault-injection node for Stage 5 validation ("deliberately inject a
delayed IMU message, a missing joint-state update, reduced ground friction
or an actuator torque limit", per the HUMANOID-LAB proposal).

This node does NOT touch the sim directly; it publishes fault *commands*
that other nodes are expected to honor, keeping fault injection decoupled
from the physics bridge:

  ~/inject_imu_delay        std_msgs/Float64   seconds to artificially delay
                                                 the next N IMU messages (the
                                                 mujoco_bridge_node's IMU
                                                 publisher checks this topic)
  ~/inject_joint_dropout     std_msgs/Bool      if true, mujoco_bridge stops
                                                 publishing joint_states until
                                                 cleared
  ~/inject_friction_scale    std_msgs/Float64   multiplier (0-1) the sim
                                                 bridge applies to floor
                                                 friction via
                                                 model.geom_friction at
                                                 reset (requires a sim
                                                 restart to take effect --
                                                 MuJoCo model friction isn't
                                                 safely mutable mid-step)
  ~/inject_torque_limit_scale std_msgs/Float64  multiplier (0-1) applied to
                                                 every actuator's forcerange

A single service-less CLI entry point (trigger_fault) lets you fire one of
these from the command line, e.g.:

    ros2 run humanoid_safety fault_injector --ros-args -p fault:=imu_delay -p value:=0.5
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class FaultInjectorNode(Node):
    def __init__(self):
        super().__init__("fault_injector")

        self.declare_parameter("fault", "")       # "imu_delay" | "joint_dropout" | "friction" | "torque_limit" | ""
        self.declare_parameter("value", 0.0)

        self.imu_delay_pub = self.create_publisher(Float64, "/mujoco_bridge/inject_imu_delay", 10)
        self.joint_dropout_pub = self.create_publisher(Bool, "/mujoco_bridge/inject_joint_dropout", 10)
        self.friction_pub = self.create_publisher(Float64, "/mujoco_bridge/inject_friction_scale", 10)
        self.torque_limit_pub = self.create_publisher(Float64, "/mujoco_bridge/inject_torque_limit_scale", 10)

        fault = self.get_parameter("fault").value
        value = self.get_parameter("value").value
        if fault:
            # Wait for DDS discovery; firing inside __init__ often loses
            # the only message because bridge has not matched subscriber.
            self._fire_count = 0
            self._pending_fault = (fault, value)
            self._timer = self.create_timer(0.5, self._fire_when_ready)

    def _fire_when_ready(self):
        self._fire(*self._pending_fault)
        self._fire_count += 1
        if self._fire_count >= 3:
            self._timer.cancel()

    def _fire(self, fault: str, value: float):
        if fault == "imu_delay":
            self.imu_delay_pub.publish(Float64(data=float(value)))
        elif fault == "joint_dropout":
            self.joint_dropout_pub.publish(Bool(data=bool(value)))
        elif fault == "friction":
            self.friction_pub.publish(Float64(data=float(value)))
        elif fault == "torque_limit":
            self.torque_limit_pub.publish(Float64(data=float(value)))
        else:
            self.get_logger().warn(f"unknown fault type '{fault}' -- no-op")
            return
        self.get_logger().info(f"injected fault={fault} value={value}")


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
