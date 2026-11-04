#!/usr/bin/env python3
"""ROS 2 wrapper around humanoid_state_estimator.complementary_filter.

Subscribes:  sensor_msgs/Imu          on ~imu_topic      (default /imu/torso)
Publishes:   geometry_msgs/Vector3    on ~/attitude       (roll, pitch, 0.0)  [rad]
             std_msgs/Bool            on ~/contact_left
             std_msgs/Bool            on ~/contact_right

Contact is derived from the touch-sensor-style force topics the sim bridge
publishes (~left_foot_force / ~right_foot_force, std_msgs/Float64) rather
than from the IMU -- declared as separate subscriptions below.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3

from humanoid_state_estimator.complementary_filter import ComplementaryFilter, detect_contact


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__("state_estimator")

        self.declare_parameter("alpha", 0.98)
        self.declare_parameter("imu_topic", "/imu/torso")
        self.declare_parameter("contact_force_threshold_n", 20.0)
        self.declare_parameter("contact_height_threshold_m", 0.01)

        self.filter = ComplementaryFilter(alpha=self.get_parameter("alpha").value)
        self._contact_force_threshold = self.get_parameter("contact_force_threshold_n").value
        self._contact_height_threshold = self.get_parameter("contact_height_threshold_m").value

        self._last_imu_time = None

        imu_topic = self.get_parameter("imu_topic").value
        self.create_subscription(Imu, imu_topic, self._on_imu, 50)
        self.create_subscription(Float64, "left_foot_force", self._on_left_force, 10)
        self.create_subscription(Float64, "right_foot_force", self._on_right_force, 10)

        self.attitude_pub = self.create_publisher(Vector3, "~/attitude", 50)
        self.contact_left_pub = self.create_publisher(Bool, "~/contact_left", 10)
        self.contact_right_pub = self.create_publisher(Bool, "~/contact_right", 10)

        self.get_logger().info(f"state_estimator up, subscribing to {imu_topic}")

    def _on_imu(self, msg: Imu):
        now = self.get_clock().now()
        if self._last_imu_time is None:
            dt = 0.001
        else:
            dt = max((now - self._last_imu_time).nanoseconds * 1e-9, 1e-4)
        self._last_imu_time = now

        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        att = self.filter.update(gyro, accel, dt)

        out = Vector3()
        out.x = att.roll
        out.y = att.pitch
        out.z = 0.0
        self.attitude_pub.publish(out)

    def _on_left_force(self, msg: Float64):
        # No height sensor is available in this bridge. Use measured
        # touch force only; passing zero height made contact always true.
        contact = bool(msg.data >= self._contact_force_threshold)
        self.contact_left_pub.publish(Bool(data=contact))

    def _on_right_force(self, msg: Float64):
        # No height sensor is available in this bridge. Use measured
        # touch force only; passing zero height made contact always true.
        contact = bool(msg.data >= self._contact_force_threshold)
        self.contact_right_pub.publish(Bool(data=contact))


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
