import pytest
from humanoid_safety.safety_fsm import SafetySupervisorFSM, SafetyState, SafetyInputs
from humanoid_mission_manager.mission_fsm import MissionManagerFSM, MissionPhase, MissionCommand


# ---- safety FSM ----

def test_starts_in_stand():
    fsm = SafetySupervisorFSM()
    assert fsm.state == SafetyState.STAND


def test_stand_to_walk_on_request():
    fsm = SafetySupervisorFSM()
    fsm.step(SafetyInputs(walk_requested=True))
    assert fsm.state == SafetyState.WALK


def test_stale_imu_triggers_fault_from_walk():
    fsm = SafetySupervisorFSM()
    fsm.step(SafetyInputs(walk_requested=True))
    assert fsm.state == SafetyState.WALK
    fsm.step(SafetyInputs(walk_requested=True, imu_age_s=0.5))
    assert fsm.state == SafetyState.FAULT_DETECTED
    assert "stale IMU" in fsm.last_fault_reason


def test_fault_with_contact_lost_attempts_recovery():
    fsm = SafetySupervisorFSM()
    fsm.state = SafetyState.FAULT_DETECTED
    fsm.step(SafetyInputs(contact_lost=True))
    assert fsm.state == SafetyState.RECOVERY


def test_fault_without_recoverable_cause_goes_to_safe_hold():
    fsm = SafetySupervisorFSM()
    fsm.state = SafetyState.FAULT_DETECTED
    fsm.step(SafetyInputs())  # no contact loss, tracking error within bounds
    assert fsm.state == SafetyState.SAFE_HOLD


def test_recovery_success_returns_to_stand():
    fsm = SafetySupervisorFSM()
    fsm.state = SafetyState.RECOVERY
    fsm.step(SafetyInputs(recovery_success=True))
    assert fsm.state == SafetyState.STAND


def test_recovery_failure_goes_to_safe_hold():
    fsm = SafetySupervisorFSM()
    fsm.state = SafetyState.RECOVERY
    fsm.step(SafetyInputs(recovery_failed=True))
    assert fsm.state == SafetyState.SAFE_HOLD


def test_safe_hold_requires_explicit_reset():
    fsm = SafetySupervisorFSM()
    fsm.state = SafetyState.SAFE_HOLD
    fsm.step(SafetyInputs(walk_requested=True))
    assert fsm.state == SafetyState.SAFE_HOLD
    fsm.reset()
    assert fsm.state == SafetyState.STAND


# ---- mission FSM ----

def test_mission_idle_to_stand_on_start():
    fsm = MissionManagerFSM()
    phase = fsm.step(MissionCommand(start=True), SafetyState.STAND)
    assert phase == MissionPhase.STAND


def test_mission_walk_sequence():
    fsm = MissionManagerFSM()
    fsm.step(MissionCommand(start=True), SafetyState.STAND)
    phase = fsm.step(MissionCommand(begin_walk=True, steps_remaining=3), SafetyState.STAND)
    assert phase == MissionPhase.SHIFT
    phase = fsm.step(MissionCommand(begin_walk=True, steps_remaining=3), SafetyState.STAND)
    assert phase == MissionPhase.STEP
    phase = fsm.step(MissionCommand(begin_walk=True, steps_remaining=2), SafetyState.STAND)
    assert phase == MissionPhase.WALK


def test_mission_safety_fault_forces_hold_regardless_of_phase():
    fsm = MissionManagerFSM()
    fsm.step(MissionCommand(start=True), SafetyState.STAND)
    fsm.step(MissionCommand(begin_walk=True, steps_remaining=3), SafetyState.STAND)
    phase = fsm.step(MissionCommand(begin_walk=True, steps_remaining=3), SafetyState.FAULT_DETECTED)
    assert phase == MissionPhase.HOLD


def test_mission_resumes_at_stand_after_safety_clears():
    fsm = MissionManagerFSM()
    fsm.phase = MissionPhase.HOLD
    phase = fsm.step(MissionCommand(), SafetyState.STAND)
    assert phase == MissionPhase.STAND
