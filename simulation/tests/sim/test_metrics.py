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


def test_residual_is_the_vector_norm_of_the_mean():
    # Off-axis mean: the norm (1.0) differs from the largest component (0.8 or 0.71).
    g = np.tile(G * np.array([0.6, 0.0, 0.8]), (50, 1))
    assert residual_g(g) == pytest.approx(1.0)
    g = np.tile(G / np.sqrt(2) * np.array([1.0, 1.0, 0.0]), (50, 1))
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


def test_windowed_residual_median_and_p95_of_differing_windows():
    # 20 windows of 10 samples, window i is a constant vector of magnitude (i/20)^2 g along a
    # changing axis. The skewed spread keeps median, mean, P95 and max apart.
    axes = np.eye(3)
    mags = (np.arange(20) / 20) ** 2
    g = np.concatenate([np.tile(G * m * axes[i % 3], (10, 1)) for i, m in enumerate(mags)])
    med, p95 = windowed_residual(g, window_s=1.0, sample_hz=10)
    assert med == pytest.approx(np.median(mags))
    assert p95 == pytest.approx(np.percentile(mags, 95))
    assert p95 < mags.max()  # P95, not the maximum


def test_windowed_residual_uses_consecutive_samples():
    # Windows of +g and -g alternate: each window averages to 1 g, the whole run to 0 g.
    up, down = np.tile([0, 0, G], (5, 1)), np.tile([0, 0, -G], (5, 1))
    g = np.concatenate([up, down, up, down])
    med, p95 = windowed_residual(g, window_s=1.0, sample_hz=5)
    assert med == pytest.approx(1.0) and p95 == pytest.approx(1.0)


def test_windowed_residual_is_nan_for_a_run_shorter_than_one_window():
    # summarize() relies on nan to leave out windows longer than the run.
    med, p95 = windowed_residual(np.tile([0, 0, -G], (59, 1)), window_s=60.0, sample_hz=1)
    assert np.isnan(med) and np.isnan(p95)


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


def test_speed_tracking_is_rms_not_mean_abs():
    # Errors 1 and 3 on one axis: RMS sqrt(5), mean absolute error 2.
    cmd = np.zeros((2, 2))
    act = np.array([[1.0, 0.0], [3.0, 0.0]])
    assert np.allclose(speed_tracking(cmd, act), [np.sqrt(5.0), 0.0])
