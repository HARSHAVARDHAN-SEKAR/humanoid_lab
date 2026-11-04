#!/usr/bin/env python3
"""ROS 2 wrapper around safety_fsm.py (Stage 5).

Subscribes:
  /imu/torso                 sensor_msgs/Imu     -- used only for staleness (message age)
  ~/joint_tracking_error      std_msgs/Float64    -- max |commanded-actual| across joints, rad
  state_estimator/contact_left,  std_msgs/Bool
  state_estimator/contact_right  std_msgs/Bool
  ~/walk_requested            std_msgs/Bool       -- forwarded from mission_manager
  /fault_injection/recovery_success  std_msgs/Bool
  /fault_injection/recovery_failed   std_msgs/Bool

Publishes:
  ~/state         std_msgs/String   current SafetyState name, on every change and at 10 Hz
  ~/fault_reason  std_msgs/String   last_fault_reason, published alongside ~/state

Contact loss here means "both feet report no contact for longer than
~contact_loss_timeout_s" -- a simple heuristic that does not yet account
for the gait planner's expected single-support phases (a foot is *supposed*
to be briefly airborne mid-step). Wiring in the gait planner's swing/stance
schedule so a normal swing phase isn't mistaken for a fault is listed as a
next step in docs/STAGE_STATUS.md.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String, Float64
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger

from humanoid_safety.safety_fsm import SafetySupervisorFSM, SafetyThresholds, SafetyInputs


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__("safety_supervisor")

        self.declare_parameter("max_imu_age_s", 0.15)
        self.declare_parameter("max_joint_tracking_error_rad", 0.70)
        self.declare_parameter("max_joint_age_s", 0.35)
        self.declare_parameter("startup_grace_s", 1.0)
        self.declare_parameter("contact_loss_timeout_s", 0.25)
        self.declare_parameter("publish_rate_hz", 10.0)

        thresholds = SafetyThresholds(
            max_imu_age_s=self.get_parameter("max_imu_age_s").value,
            max_joint_tracking_error_rad=self.get_parameter("max_joint_tracking_error_rad").value,
            max_joint_age_s=self.get_parameter("max_joint_age_s").value,
        )
        self.fsm = SafetySupervisorFSM(thresholds)
        self._contact_timeout = self.get_parameter("contact_loss_timeout_s").value

        self._startup = self.get_clock().now()
        self._startup_grace_s = float(self.get_parameter("startup_grace_s").value)
        self._last_imu_time = None
        self._last_joint_time = None
        self._ever_contacted = False
        self._joint_tracking_error = 0.0
        self._contact_left = True
        self._contact_right = True
        self._last_contact_time = self.get_clock().now()
        self._walk_requested = False
        self._ik_fault = False
        self._recovery_success = False
        self._recovery_failed = False

        self.create_subscription(Imu, "/imu/torso", self._on_imu, 50)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.create_subscription(Float64, "/motion_controller/joint_tracking_error", self._on_tracking_error, 20)
        self.create_subscription(Bool, "/motion_controller/ik_fault", self._on_ik_fault, 20)
        self.create_subscription(Bool, "state_estimator/contact_left", self._on_contact_left, 20)
        self.create_subscription(Bool, "state_estimator/contact_right", self._on_contact_right, 20)
        self.create_subscription(Bool, "/mission_manager/walk_requested", self._on_walk_requested, 10)
        self.create_subscription(Bool, "/fault_injection/recovery_success", self._on_recovery_success, 10)
        self.create_subscription(Bool, "/fault_injection/recovery_failed", self._on_recovery_failed, 10)

        self.state_pub = self.create_publisher(String, "~/state", 10)
        self.reason_pub = self.create_publisher(String, "~/fault_reason", 10)
        self.create_service(Trigger, "/safety_supervisor/reset", self._reset)

        rate = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("safety_supervisor up, starting in STAND")

    def _on_imu(self, _msg: Imu):
        self._last_imu_time = self.get_clock().now()

    def _on_ik_fault(self, msg: Bool):
        self._ik_fault = bool(msg.data)

    def _on_joints(self, _msg: JointState):
        self._last_joint_time = self.get_clock().now()

    def _reset(self, _request, response):
        now = self.get_clock().now()
        imu_ok = self._last_imu_time is not None and (now - self._last_imu_time).nanoseconds * 1e-9 < self.fsm.thresholds.max_imu_age_s
        joint_ok = self._last_joint_time is not None and (now - self._last_joint_time).nanoseconds * 1e-9 < self.fsm.thresholds.max_joint_age_s
        if (imu_ok and joint_ok and not self._ik_fault and
                self._joint_tracking_error <= self.fsm.thresholds.max_joint_tracking_error_rad):
            self.fsm.reset()
            response.success = True
            response.message = "Reset accepted; check safe stance before restarting mission"
        else:
            response.success = False
            response.message = "Reset rejected: feedback stale or tracking-error threshold exceeded"
        return response

    def _on_tracking_error(self, msg: Float64):
        self._joint_tracking_error = msg.data

    def _on_contact_left(self, msg: Bool):
        self._contact_left = msg.data
        self._ever_contacted |= bool(msg.data)
        if msg.data or self._contact_right:
            self._last_contact_time = self.get_clock().now()

    def _on_contact_right(self, msg: Bool):
        self._contact_right = msg.data
        self._ever_contacted |= bool(msg.data)
        if msg.data or self._contact_left:
            self._last_contact_time = self.get_clock().now()

    def _on_walk_requested(self, msg: Bool):
        self._walk_requested = msg.data

    def _on_recovery_success(self, msg: Bool):
        self._recovery_success = msg.data

    def _on_recovery_failed(self, msg: Bool):
        self._recovery_failed = msg.data

    def _tick(self):
        now = self.get_clock().now()
        elapsed = (now - self._startup).nanoseconds * 1e-9
        if elapsed < self._startup_grace_s:
            self.state_pub.publish(String(data=self.fsm.state.name))
            return
        imu_age = 999.0 if self._last_imu_time is None else (now - self._last_imu_time).nanoseconds * 1e-9
        joint_age = 999.0 if self._last_joint_time is None else (now - self._last_joint_time).nanoseconds * 1e-9
        contact_lost = self._ever_contacted and (now - self._last_contact_time).nanoseconds * 1e-9 > self._contact_timeout

        inputs = SafetyInputs(
            imu_age_s=imu_age,
            joint_age_s=joint_age,
            joint_tracking_error_rad=self._joint_tracking_error,
            contact_lost=contact_lost,
            ik_fault=self._ik_fault,
            walk_requested=self._walk_requested,
            recovery_success=self._recovery_success,
            recovery_failed=self._recovery_failed,
        )
        # These are one-shot triggers, not held states.
        self._recovery_success = False
        self._recovery_failed = False

        state = self.fsm.step(inputs)
        self.state_pub.publish(String(data=state.name))
        self.reason_pub.publish(String(data=self.fsm.last_fault_reason or ""))


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
