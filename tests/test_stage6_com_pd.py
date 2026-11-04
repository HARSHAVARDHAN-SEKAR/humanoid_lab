import pytest
from humanoid_balance_controller.com_pd import ComPitchPD


def test_zero_error():
    assert ComPitchPD(reference_x=-0.032).update(-0.032, 0.0, 0.01) == pytest.approx(0.0)


def test_positive_displacement_negative_correction():
    assert ComPitchPD(reference_x=0.0).update(0.01, 0.0, 0.01) < 0.0


def test_negative_displacement_positive_correction():
    assert ComPitchPD(reference_x=0.0).update(-0.01, 0.0, 0.01) > 0.0


def test_positive_velocity_negative_correction():
    assert ComPitchPD(reference_x=0.0).update(0.0, 0.05, 0.01) < 0.0


def test_rate_limit():
    assert abs(ComPitchPD(reference_x=0.0).update(1.0, 0.0, 0.01)) <= 0.0100001


def test_angle_limit():
    c = ComPitchPD(reference_x=0.0)
    for _ in range(100):
        value = c.update(1.0, 0.0, 0.01)
    assert value == pytest.approx(-0.12)


def test_nan_rejected():
    with pytest.raises(ValueError):
        ComPitchPD().update(float("nan"), 0.0, 0.01)
