import numpy as np
import pytest
from humanoid_state_estimator.complementary_filter import ComplementaryFilter, detect_contact


def test_filter_stays_level_when_static_and_upright():
    f = ComplementaryFilter(alpha=0.98)
    att = f.update(gyro_xyz=(0, 0, 0), accel_xyz=(0, 0, 9.81), dt=0.01)
    assert att.roll == pytest.approx(0.0, abs=1e-6)
    assert att.pitch == pytest.approx(0.0, abs=1e-6)


def test_filter_tracks_known_tilt_with_pure_accel_trust():
    f = ComplementaryFilter(alpha=0.0)  # trust accelerometer fully
    tilt = 0.1
    accel = (0.0, 9.81 * np.sin(tilt), 9.81 * np.cos(tilt))
    att = f.update(gyro_xyz=(0, 0, 0), accel_xyz=accel, dt=0.01)
    assert att.roll == pytest.approx(tilt, abs=1e-6)


def test_filter_rejects_bad_alpha():
    with pytest.raises(ValueError):
        ComplementaryFilter(alpha=1.5)


def test_contact_true_near_ground():
    assert detect_contact(foot_height_m=0.0, grf_z_n=0.0) is True


def test_contact_true_under_load_even_if_slightly_off_ground():
    assert detect_contact(foot_height_m=0.02, grf_z_n=50.0) is True


def test_contact_false_when_airborne_and_unloaded():
    assert detect_contact(foot_height_m=0.15, grf_z_n=0.0) is False
