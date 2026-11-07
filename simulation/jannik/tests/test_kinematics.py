import math

import numpy as np
import pytest

from rpm.core.kinematics import TWO_PI, gravity_in_sample, wrap_angle


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


@pytest.mark.parametrize("theta_outer", np.linspace(-7, 7, 9))
@pytest.mark.parametrize("theta_inner", np.linspace(-7, 7, 9))
def test_matches_rotation_matrices(theta_outer: float, theta_inner: float) -> None:
    # Independent derivation: sample orientation R = Rx(outer) @ Rz(inner),
    # gravity seen by the sample is R^T @ (0, 0, -1).
    r = rot_x(theta_outer) @ rot_z(theta_inner)
    expected = r.T @ np.array([0.0, 0.0, -1.0])
    assert gravity_in_sample(theta_outer, theta_inner) == pytest.approx(expected, abs=1e-12)


def test_rest_position_sees_gravity_straight_down() -> None:
    assert gravity_in_sample(0.0, 0.0) == pytest.approx((0.0, 0.0, -1.0))


def test_quarter_turn_of_outer_puts_gravity_along_minus_y() -> None:
    # Inner frame tilted +90 deg about x: the sample's -y axis now points down.
    assert gravity_in_sample(math.pi / 2, 0.0) == pytest.approx((0.0, -1.0, 0.0), abs=1e-12)


def test_inner_axis_is_vertical_at_rest() -> None:
    # At outer = 0 the inner axis is vertical: turning it does not change gravity.
    for theta_inner in (0.5, 2.0, 4.0):
        assert gravity_in_sample(0.0, theta_inner) == pytest.approx((0.0, 0.0, -1.0))


def test_both_quarter_turns_put_gravity_along_minus_x() -> None:
    expected = (-1.0, 0.0, 0.0)
    assert gravity_in_sample(math.pi / 2, math.pi / 2) == pytest.approx(expected, abs=1e-12)


def test_gravity_is_always_unit_length() -> None:
    rng = np.random.default_rng(0)
    for f, p in rng.uniform(-10, 10, size=(200, 2)):
        assert np.linalg.norm(gravity_in_sample(f, p)) == pytest.approx(1.0)


def test_wrap_angle_range() -> None:
    for theta in (-13.0, -TWO_PI, -0.1, 0.0, 3.0, TWO_PI, 20.0):
        wrapped = wrap_angle(theta)
        assert 0.0 <= wrapped < TWO_PI
        assert math.sin(wrapped) == pytest.approx(math.sin(theta))


def test_estimate_follows_the_first_order_lag_exactly() -> None:
    # From rest to 120 deg/s with lag 0.1 s: after one 0.1 s tick the axis has
    # covered 120 * (0.1 - 0.1 * (1 - e^-1)) = 4.41455 deg, not 12 deg.
    import math

    from rpm.core.controller import OrientationEstimate
    from rpm.core.kinematics import DEG

    estimate = OrientationEstimate(lag_s=0.1)
    estimate.advance(120.0, 0.0, 0.1)
    expected = 120.0 * (0.1 - 0.1 * (1.0 - math.exp(-1.0)))
    assert estimate.theta_outer / DEG == pytest.approx(expected, rel=1e-12)
    assert estimate.speed_outer == pytest.approx(120.0 * (1.0 - math.exp(-1.0)), rel=1e-12)
    # Reversal: from +120 to -120 deg/s.
    estimate = OrientationEstimate(lag_s=0.1)
    estimate.speed_outer = 120.0
    estimate.advance(-120.0, 0.0, 0.1)
    expected = -120.0 * 0.1 + 240.0 * 0.1 * (1.0 - math.exp(-1.0))
    assert estimate.theta_outer / DEG == pytest.approx(expected % 360.0, rel=1e-12)
