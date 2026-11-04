"""Stage 6L diagnostics: distinguish 2D support from line contact."""
import pytest
from humanoid_sim.contact_force import polygon_metrics_from_points


def test_inside_rectangle_positive_margin():
    d = polygon_metrics_from_points(
        [(0,0),(0.2,0),(0.2,0.1),(0,0.1)], (.1,.05))
    assert d['valid']
    assert d['margin_m'] == pytest.approx(0.05)


def test_outside_rectangle_negative_margin():
    d = polygon_metrics_from_points(
        [(0,0),(0.2,0),(0.2,0.1),(0,0.1)], (.3,.05))
    assert d['valid']
    assert d['margin_m'] == pytest.approx(-0.1)


def test_four_collinear_contacts_are_not_valid_polygon():
    d = polygon_metrics_from_points(
        [(0,0),(.1,0),(.2,0),(.3,0)], (.1,0))
    assert not d['valid']
    assert d['margin_m'] is None


def test_two_contacts_not_valid():
    d = polygon_metrics_from_points([(0,0),(.1,.1)], (0,0))
    assert not d['valid']
