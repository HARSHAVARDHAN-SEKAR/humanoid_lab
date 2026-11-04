import pytest
from humanoid_benchmarks.metrics import standing_duration_s, joint_tracking_rms, push_recovery_rate


def test_standing_duration_picks_longest_run():
    t = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    states = ["STAND", "STAND", "FAULT_DETECTED", "STAND", "STAND", "STAND", "STAND"]
    # runs: [0.0-1.0]=1.0s, [1.5-3.0]=1.5s -> longest is 1.5
    assert standing_duration_s(t, states) == pytest.approx(1.5, abs=1e-9)


def test_standing_duration_empty_input():
    assert standing_duration_s([], []) == 0.0


def test_standing_duration_run_extends_to_end():
    t = [0.0, 1.0, 2.0]
    states = ["STAND", "STAND", "STAND"]
    assert standing_duration_s(t, states) == pytest.approx(2.0, abs=1e-9)


def test_joint_tracking_rms_zero_when_identical():
    assert joint_tracking_rms([0.1, 0.2, 0.3], [0.1, 0.2, 0.3]) == pytest.approx(0.0, abs=1e-9)


def test_joint_tracking_rms_known_value():
    # errors: 1, -1 -> RMS = 1.0
    assert joint_tracking_rms([0.0, 0.0], [1.0, -1.0]) == pytest.approx(1.0, abs=1e-9)


def test_joint_tracking_rms_rejects_mismatched_shapes():
    with pytest.raises(ValueError):
        joint_tracking_rms([0.0, 0.0], [0.0])


def test_push_recovery_rate_mixed():
    assert push_recovery_rate([True, True, False, True]) == pytest.approx(0.75, abs=1e-9)


def test_push_recovery_rate_empty():
    assert push_recovery_rate([]) == 0.0
