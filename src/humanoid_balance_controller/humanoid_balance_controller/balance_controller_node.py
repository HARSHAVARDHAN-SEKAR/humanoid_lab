#!/usr/bin/env python3
"""ROS 2 wrapper combining standing_balance.py (ankle-strategy PD, Stage 2)
and capture_point.py (LIP/ICP push-recovery, Stage 5).

Subscribes:
  ~/attitude            geometry_msgs/Vector3   (roll, pitch, 0) from state_estimator
  ~/com_velocity         geometry_msgs/Vector3   (vx, vy, 0) -- CoM velocity estimate
  ~/nominal_footstep     geometry_msgs/Point     next planned footstep from gait_planner

Publishes:
  ~/ankle_pitch_correction   std_msgs/Float64   added to both ankles' commanded pitch
  ~/ankle_roll_correction    std_msgs/Float64   added to both ankles' commanded roll
  ~/corrected_footstep       geometry_msgs/Point  nominal footstep blended toward the ICP

NOTE on scope: torso angular *rate* (needed for the derivative term) is
read from the IMU directly in this node rather than from the fused
attitude estimate, since state_estimator does not yet publish a rate --
see docs/STAGE_STATUS.md. CoM position/height here is a fixed nominal
value (robot_params.yaml's standing pose height), not yet computed from
the live joint configuration; wiring in humanoid_kinematics for a real
CoM estimate is listed as a next step.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Point
from sensor_msgs.msg import Imu

from humanoid_balance_controller.standing_balance import AnkleStrategyBalanceController, BalanceGains
from humanoid_balance_controller.capture_point import ComState, instantaneous_capture_point, adjust_footstep_for_icp
from humanoid_gait_planner.footstep_planner import Footstep


class BalanceControllerNode(Node):
    def __init__(self):
        super().__init__("balance_controller")

        self.declare_parameter("kp_pitch", 10.0)
        self.declare_parameter("kd_pitch", 0.3)
        self.declare_parameter("kp_roll", 10.0)
        self.declare_parameter("kd_roll", 0.3)
        self.declare_parameter("max_correction_rad", 0.2)
        self.declare_parameter("nominal_com_height_m", 0.55)
        self.declare_parameter("icp_gain", 0.5)

        gains = BalanceGains(
            kp_pitch=self.get_parameter("kp_pitch").value,
            kd_pitch=self.get_parameter("kd_pitch").value,
            kp_roll=self.get_parameter("kp_roll").value,
            kd_roll=self.get_parameter("kd_roll").value,
            max_correction_rad=self.get_parameter("max_correction_rad").value,
        )
        self.controller = AnkleStrategyBalanceController(gains)
        self._com_height = self.get_parameter("nominal_com_height_m").value
        self._icp_gain = self.get_parameter("icp_gain").value

        self._pitch = 0.0
        self._roll = 0.0
        self._pitch_rate = 0.0
        self._roll_rate = 0.0
        self._com_vx = 0.0
        self._com_vy = 0.0

        self.create_subscription(Vector3, "/state_estimator/attitude", self._on_attitude, 50)
        self.create_subscription(Imu, "/imu/torso", self._on_imu, 50)
        self.create_subscription(Vector3, "~/com_velocity", self._on_com_velocity, 50)
        self.create_subscription(Point, "~/nominal_footstep", self._on_nominal_footstep, 10)

        self.pitch_corr_pub = self.create_publisher(Float64, "~/ankle_pitch_correction", 50)
        self.roll_corr_pub = self.create_publisher(Float64, "~/ankle_roll_correction", 50)
        self.footstep_pub = self.create_publisher(Point, "~/corrected_footstep", 10)

        self.create_timer(0.01, self._tick)  # 100 Hz balance loop
        self.get_logger().info("balance_controller up")

    def _on_attitude(self, msg: Vector3):
        self._roll, self._pitch = msg.x, msg.y

    def _on_imu(self, msg: Imu):
        self._roll_rate = msg.angular_velocity.x
        self._pitch_rate = msg.angular_velocity.y

    def _on_com_velocity(self, msg: Vector3):
        self._com_vx, self._com_vy = msg.x, msg.y

    def _on_nominal_footstep(self, msg: Point):
        com = ComState(x=0.0, y=0.0, vx=self._com_vx, vy=self._com_vy, height=self._com_height)
        icp_x, icp_y, _ = instantaneous_capture_point(com)
        nominal = Footstep(index=0, foot="unknown", x=msg.x, y=msg.y)
        corrected = adjust_footstep_for_icp(nominal, icp_x, icp_y, gain=self._icp_gain)
        self.footstep_pub.publish(Point(x=corrected.x, y=corrected.y, z=0.0))

    def _tick(self):
        pitch_corr = self.controller.pitch_correction(self._pitch, self._pitch_rate)
        roll_corr = self.controller.roll_correction(self._roll, self._roll_rate)
        self.pitch_corr_pub.publish(Float64(data=pitch_corr))
        self.roll_corr_pub.publish(Float64(data=roll_corr))


def main(args=None):
    rclpy.init(args=args)
    node = BalanceControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
