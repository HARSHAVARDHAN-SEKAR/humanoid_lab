"""Fault-aware safety supervisor state machine (Stage 5).

Implements the state machine sketched in the HUMANOID-LAB proposal:

    STAND / WALK --[fault]--> FAULT_DETECTED --> RECOVERY or SAFE_HOLD
    RECOVERY --[success]--> STAND
    RECOVERY --[failure]--> SAFE_HOLD
    SAFE_HOLD --[operator reset]--> STAND

Pure python, no ROS dependency, so the transition logic is unit-testable in
isolation from rclpy/MuJoCo. The ROS 2 node in safety_supervisor_node.py
wraps this class and feeds it topic data.
"""
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional


class SafetyState(Enum):
    STAND = auto()
    WALK = auto()
    FAULT_DETECTED = auto()
    SAFE_HOLD = auto()
    RECOVERY = auto()


@dataclass
class SafetyThresholds:
    max_imu_age_s: float = 0.15
    max_joint_tracking_error_rad: float = 0.70
    max_joint_age_s: float = 0.35


@dataclass
class SafetyInputs:
    imu_age_s: float = 0.0
    joint_tracking_error_rad: float = 0.0
    joint_age_s: float = 0.0
    contact_lost: bool = False
    ik_fault: bool = False
    walk_requested: bool = False
    recovery_success: bool = False
    recovery_failed: bool = False


@dataclass
class SafetyEvent:
    from_state: SafetyState
    to_state: SafetyState
    reason: Optional[str] = None


class SafetySupervisorFSM:
    def __init__(self, thresholds: SafetyThresholds = None):
        self.state = SafetyState.STAND
        self.thresholds = thresholds or SafetyThresholds()
        self.last_fault_reason: Optional[str] = None
        self.history: List[SafetyEvent] = []

    def _detect_faults(self, inputs: SafetyInputs) -> List[str]:
        reasons = []
        if inputs.ik_fault:
            reasons.append("IK/actuator command invalid")
        if inputs.imu_age_s > self.thresholds.max_imu_age_s:
            reasons.append(f"stale IMU ({inputs.imu_age_s:.3f}s > {self.thresholds.max_imu_age_s:.3f}s)")
        if inputs.joint_age_s > self.thresholds.max_joint_age_s:
            reasons.append(f"stale joints ({inputs.joint_age_s:.3f}s > {self.thresholds.max_joint_age_s:.3f}s)")
        if inputs.joint_tracking_error_rad > self.thresholds.max_joint_tracking_error_rad:
            reasons.append(
                f"joint tracking error ({inputs.joint_tracking_error_rad:.3f} rad > "
                f"{self.thresholds.max_joint_tracking_error_rad:.3f} rad)"
            )
        if inputs.contact_lost:
            reasons.append("unexpected loss of foot contact")
        return reasons

    def _transition(self, to_state: SafetyState, reason: Optional[str] = None):
        self.history.append(SafetyEvent(self.state, to_state, reason))
        self.state = to_state

    def step(self, inputs: SafetyInputs) -> SafetyState:
        reasons = self._detect_faults(inputs)

        if reasons and self.state in (SafetyState.STAND, SafetyState.WALK):
            self.last_fault_reason = "; ".join(reasons)
            self._transition(SafetyState.FAULT_DETECTED, self.last_fault_reason)
            return self.state

        if self.state == SafetyState.FAULT_DETECTED:
            # Contact loss / large tracking error are things a push-recovery
            # step can plausibly fix; anything else goes straight to a safe,
            # stationary hold rather than attempting to keep moving.
            if inputs.contact_lost or inputs.joint_tracking_error_rad > self.thresholds.max_joint_tracking_error_rad:
                self._transition(SafetyState.RECOVERY, "attempting push-recovery step")
            else:
                self._transition(SafetyState.SAFE_HOLD, "fault not recoverable while walking")
            return self.state

        if self.state == SafetyState.RECOVERY:
            if inputs.recovery_success:
                self._transition(SafetyState.STAND, "recovery succeeded")
            elif inputs.recovery_failed:
                self._transition(SafetyState.SAFE_HOLD, "recovery failed")
            return self.state

        if self.state == SafetyState.SAFE_HOLD:
            return self.state  # requires an explicit reset()

        if self.state == SafetyState.STAND:
            if inputs.walk_requested:
                self._transition(SafetyState.WALK, "walk requested")
            return self.state

        if self.state == SafetyState.WALK:
            if not inputs.walk_requested:
                self._transition(SafetyState.STAND, "walk no longer requested")
            return self.state

        return self.state

    def reset(self):
        self._transition(SafetyState.STAND, "operator reset")
        self.last_fault_reason = None
