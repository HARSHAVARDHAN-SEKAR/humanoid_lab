from humanoid_safety.safety_fsm import SafetySupervisorFSM, SafetyInputs, SafetyState


def test_ik_fault_triggers_safety_fsm():
    fsm = SafetySupervisorFSM()
    assert fsm.step(SafetyInputs(ik_fault=True)) == SafetyState.FAULT_DETECTED
    assert 'IK' in fsm.last_fault_reason


def test_joint_state_dropout_triggers_watchdog():
    fsm = SafetySupervisorFSM()
    assert fsm.step(SafetyInputs(joint_age_s=10.0)) == SafetyState.FAULT_DETECTED
    assert 'stale joints' in fsm.last_fault_reason
