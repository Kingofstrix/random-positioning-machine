import numpy as np
import pytest

from sim.metrics import (
    extra_accel,
    residual_g,
    residual_g_curve,
    speed_tracking,
    sphere_coverage,
    windowed_residual,
)

G = 9.81


def circle(n=1000):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return G * np.stack([np.cos(a), np.sin(a), np.zeros(n)], axis=1)


def test_residual_constant_is_one():
    g = np.tile([0, 0, -G], (100, 1))
    assert residual_g(g) == pytest.approx(1.0)


def test_residual_circle_is_zero():
    assert residual_g(circle()) == pytest.approx(0.0, abs=1e-12)


def test_residual_curve_is_cumulative():
    g = np.array([[0, 0, -G], [0, 0, G], [0, 0, -G], [0, 0, G]])
    assert np.allclose(residual_g_curve(g), [1, 0, 1 / 3, 0])


def test_windowed_residual():
    g = np.tile(circle(100), (6, 1))  # 600 samples, one full circle per 100
    med, p95 = windowed_residual(g, window_s=1.0, sample_hz=100)
    assert med == pytest.approx(0) and p95 == pytest.approx(0, abs=1e-12)


def test_sphere_coverage_uniform_vs_circle():
    rng = np.random.default_rng(0)
    v = rng.normal(size=(200_000, 3))
    frac, nonuni = sphere_coverage(v)
    assert frac == 1.0 and nonuni < 0.1
    frac_c, _ = sphere_coverage(circle(10_000))
    assert frac_c <= 0.2  # a great circle touches only the equator bands


def test_extra_accel():
    g = np.tile([0, 0, -G], (10, 1))
    acc = -g.copy()
    acc[3] += [G, 0, 0]
    mx, rms = extra_accel(acc, g)
    assert mx == pytest.approx(1.0)
    assert rms == pytest.approx(np.sqrt(0.1))


def test_speed_tracking():
    cmd = np.array([[10.0, 0.0], [10.0, 0.0]])
    act = np.array([[8.0, 3.0], [12.0, -3.0]])
    assert np.allclose(speed_tracking(cmd, act), [2.0, 3.0])
