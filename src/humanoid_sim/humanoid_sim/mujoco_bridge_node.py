#!/usr/bin/env python3
"""MuJoCo <-> ROS 2 bridge for the HUMANOID-LAB simplified biped (Stage 1).

Loads humanoid_biped.xml directly via the `mujoco` python bindings (rather
than depending on mujoco_ros2_control, which isn't available to verify in
every environment -- see docs/ARCHITECTURE.md for the tradeoff). Steps
physics at the model's own timestep inside a ROS timer, and:

Publishes (at ~sim_publish_rate_hz):
  /joint_states          sensor_msgs/JointState   all 12 leg joints
  /imu/torso             sensor_msgs/Imu          from the MJCF torso IMU site
  left_foot_force,
  right_foot_force       std_msgs/Float64          touch-sensor force (N)

Subscribes:
  ~/joint_commands       sensor_msgs/JointState    .name + .position used to
                                                     set data.ctrl for the
                                                     named actuators; any
                                                     joint not included keeps
                                                     its last commanded value

Fault-injection topics (Stage 5, fed by humanoid_safety/fault_injector):
  ~/inject_imu_delay          std_msgs/Float64
  ~/inject_joint_dropout      std_msgs/Bool
  ~/inject_friction_scale     std_msgs/Float64   (applied to the floor geom)
  ~/inject_torque_limit_scale std_msgs/Float64   (applied to all actuators)
"""
import pathlib
import collections

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool

import mujoco
from humanoid_kinematics.planar_leg import PlanarLegKinematics

ACTUATOR_JOINTS = [
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
    "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
    "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
]


class MujocoBridgeNode(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        self.declare_parameter("mjcf_path", "")
        self.declare_parameter("sim_publish_rate_hz", 100.0)
        self.declare_parameter("supported_base", True)
        self.declare_parameter("joint_command_timeout_s", 0.3)

        mjcf_path = self.get_parameter("mjcf_path").value
        if not mjcf_path:
            # Source-tree fallback only; installed launch passes the package share path.
            pkg_share = pathlib.Path(__file__).resolve().parents[1]
            filename = "humanoid_biped_supported.xml" if self.get_parameter("supported_base").value else "humanoid_biped.xml"
            mjcf_path = str(pkg_share / "mujoco_models" / filename)

        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)
        self._joint_ids = {jn: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                            for jn in ACTUATOR_JOINTS}
        self._joint_qposadr = {jn: self.model.jnt_qposadr[jid] for jn, jid in self._joint_ids.items()}
        self._joint_qveladr = {jn: self.model.jnt_dofadr[jid] for jn, jid in self._joint_ids.items()}

        self._gyro_adr = self.model.sensor_adr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "torso_gyro")]
        self._accel_adr = self.model.sensor_adr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "torso_accel")]
        self._quat_adr = self.model.sensor_adr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "torso_quat")]
        self._right_touch_adr = self.model.sensor_adr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "right_foot_touch")]
        self._left_touch_adr = self.model.sensor_adr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "left_foot_touch")]

        self._floor_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self._base_friction = self.model.geom_friction[self._floor_geom_id].copy()
        self._base_forcerange = self.model.actuator_forcerange.copy()

        # Both models start in a kinematically consistent crouched stance.
        # A fixed-base rig provides mechanical torso support; the dynamic-base
        # model does NOT have proven sustained balance.
        stance = PlanarLegKinematics(0.30, 0.30).inverse(0.0, -0.55)
        initial = (0., 0., stance.hip_pitch, stance.knee_pitch, stance.ankle_pitch, 0.) * 2
        if self.model.nq == 19:  # free floating torso: (x,y,z,w,x,y,z)
            self.data.qpos[0:7] = [0., 0., 0.58, 1., 0., 0., 0.]
        for joint, target in zip(ACTUATOR_JOINTS, initial):
            self.data.qpos[self._joint_qposadr[joint]] = target
        for joint, target in zip(ACTUATOR_JOINTS, initial):
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                   joint.replace("_joint", "_act"))
            if aid < 0:
                raise ValueError(f"missing actuator: {joint}")
            self.data.ctrl[aid] = target
        mujoco.mj_forward(self.model, self.data)
        self._last_command_time = None
        self._command_timeout_s = float(self.get_parameter("joint_command_timeout_s").value)
        self._joint_dropout = False
        self._imu_delay_s = 0.0
        self._imu_queue = collections.deque()  # (publish_time_sec, Imu msg)

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 50)
        self.imu_pub = self.create_publisher(Imu, "/imu/torso", 50)
        self.left_force_pub = self.create_publisher(Float64, "left_foot_force", 50)
        self.right_force_pub = self.create_publisher(Float64, "right_foot_force", 50)
        self.height_pub = self.create_publisher(Float64, "/mujoco_bridge/torso_height", 50)
        self.pose_pub = self.create_publisher(PoseStamped, "/mujoco_bridge/torso_pose", 50)

        self.create_subscription(JointState, "/mujoco_bridge/joint_commands", self._on_joint_commands, 20)
        self.create_subscription(Float64, "~/inject_imu_delay", self._on_inject_imu_delay, 10)
        self.create_subscription(Bool, "~/inject_joint_dropout", self._on_inject_joint_dropout, 10)
        self.create_subscription(Float64, "~/inject_friction_scale", self._on_inject_friction, 10)
        self.create_subscription(Float64, "~/inject_torque_limit_scale", self._on_inject_torque_limit, 10)

        rate = self.get_parameter("sim_publish_rate_hz").value
        self._dt = self.model.opt.timestep
        self._steps_per_tick = max(1, round(1.0 / rate / self._dt))
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"mujoco_bridge up: {mjcf_path}, dt={self._dt}s, {self._steps_per_tick} sub-steps/tick")

    # -- command callbacks --------------------------------------------------

    def _on_joint_commands(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("Ignoring malformed joint command")
            return
        targets = []
        for name, pos in zip(msg.name, msg.position):
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.replace("_joint", "_act"))
            if aid < 0 or not np.isfinite(pos):
                self.get_logger().warn("Ignoring unknown/nonfinite actuator command")
                return
            lo, hi = self.model.actuator_ctrlrange[aid]
            targets.append((aid, float(np.clip(pos, lo, hi))))
        for aid, target in targets:
            self.data.ctrl[aid] = target
        self._last_command_time = self.get_clock().now()

    def _on_inject_imu_delay(self, msg: Float64):
        self._imu_delay_s = max(0.0, float(msg.data))
        if self._imu_delay_s <= 0.0:
            self._imu_queue.clear()
        self.get_logger().warn(f"[fault] IMU delay set to {self._imu_delay_s:.3f}s")

    def _on_inject_joint_dropout(self, msg: Bool):
        self._joint_dropout = bool(msg.data)
        self.get_logger().warn(f"[fault] joint_states dropout = {self._joint_dropout}")

    def _on_inject_friction(self, msg: Float64):
        scale = float(np.clip(msg.data, 0.0, 1.0))
        self.model.geom_friction[self._floor_geom_id] = self._base_friction * scale
        self.get_logger().warn(f"[fault] floor friction scaled to {scale:.2f}x")

    def _on_inject_torque_limit(self, msg: Float64):
        scale = float(np.clip(msg.data, 0.0, 1.0))
        self.model.actuator_forcerange[:] = self._base_forcerange * scale
        self.get_logger().warn(f"[fault] actuator torque limits scaled to {scale:.2f}x")

    # -- sim loop -------------------------------------------------------------

    def _tick(self):
        now = self.get_clock().now()
        # Independent last-resort command watchdog. On stale commands, hold
        # last measured joint pose (not last trajectory reference).
        if (self._last_command_time is not None and
                (now - self._last_command_time).nanoseconds * 1e-9 > self._command_timeout_s):
            for name, qpos_index in self._joint_qposadr.items():
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                        name.replace("_joint", "_act"))
                lo, hi = self.model.actuator_ctrlrange[aid]
                self.data.ctrl[aid] = float(np.clip(self.data.qpos[qpos_index], lo, hi))
            self._last_command_time = now
        for _ in range(self._steps_per_tick):
            mujoco.mj_step(self.model, self.data)

        now = self.get_clock().now()

        if not self._joint_dropout:
            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = list(ACTUATOR_JOINTS)
            js.position = [float(self.data.qpos[self._joint_qposadr[jn]]) for jn in ACTUATOR_JOINTS]
            js.velocity = [float(self.data.qvel[self._joint_qveladr[jn]]) for jn in ACTUATOR_JOINTS]
            self.joint_state_pub.publish(js)

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = "torso"
        qw, qx, qy, qz = self.data.sensordata[self._quat_adr:self._quat_adr + 4]
        imu.orientation.w, imu.orientation.x = float(qw), float(qx)
        imu.orientation.y, imu.orientation.z = float(qy), float(qz)
        gx, gy, gz = self.data.sensordata[self._gyro_adr:self._gyro_adr + 3]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = float(gx), float(gy), float(gz)
        ax, ay, az = self.data.sensordata[self._accel_adr:self._accel_adr + 3]
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)

        if self._imu_delay_s <= 0.0:
            self.imu_pub.publish(imu)
        else:
            now_s = now.nanoseconds * 1e-9
            self._imu_queue.append((now_s + self._imu_delay_s, imu))
            while self._imu_queue and self._imu_queue[0][0] <= now_s:
                _, queued = self._imu_queue.popleft()
                self.imu_pub.publish(queued)

        self.left_force_pub.publish(Float64(data=float(self.data.sensordata[self._left_touch_adr])))
        self.right_force_pub.publish(Float64(data=float(self.data.sensordata[self._right_touch_adr])))
        torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        pos = self.data.xpos[torso_id]
        self.height_pub.publish(Float64(data=float(pos[2])))
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, pos)
        qw, qx, qy, qz = self.data.xquat[torso_id]
        pose.pose.orientation.w, pose.pose.orientation.x = float(qw), float(qx)
        pose.pose.orientation.y, pose.pose.orientation.z = float(qy), float(qz)
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
