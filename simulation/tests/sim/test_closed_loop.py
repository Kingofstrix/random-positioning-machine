"""Controller from rpmcore + MuJoCo physics (docs/PLAN.md 8.3)."""

import copy
import json

import numpy as np
import pytest

from rpmcore.config import CONFIG
from sim.metrics import G0, extra_accel, residual_g, speed_tracking, sphere_coverage, windowed_residual
from sim.params import MachineParams
from sim.run import SimLog, main, simulate, summarize


def cfg_with(**random_overrides):
    cfg = copy.deepcopy(CONFIG)
    cfg["random"].update(random_overrides)
    return cfg


@pytest.mark.parametrize("mode", ["clino3d"])
def test_signs(mode):
    log = simulate(mode, minutes=0.1, seed=1)
    assert log.q_outer[-1] > 0.3 and log.q_inner[-1] > 0.3


COL = {"inner": 0, "outer": 1}  # column order of SimLog.w_dps, cmd_dps and pulses


@pytest.mark.parametrize("axis, other", [("outer", "inner"), ("inner", "outer")])
def test_sign_and_logging_per_axis(axis, other):
    # PLAN 8.3: a positive setpoint on one axis turns that axis forward, and only that one.
    # Driving one axis at a time also pins every logged column to its axis.
    cfg = copy.deepcopy(CONFIG)
    cfg["clino3d"] = {axis + "_dps": 30.0, other + "_dps": 0.0}
    log = simulate("clino3d", minutes=0.05, seed=1, cfg=cfg)
    q = {"outer": log.q_outer, "inner": log.q_inner}
    a, o = COL[axis], COL[other]
    assert q[axis][-1] > 0.5 and abs(q[other][-1]) < 0.01
    neutral = CONFIG["servo"][other]["neutral_us"]
    assert (log.pulses[:, o] == neutral).all() and (log.pulses[-1, a] != neutral)
    assert log.cmd_dps[-1, a] == pytest.approx(30.0) and (log.cmd_dps[:, o] == 0).all()
    assert log.w_dps[-1, a] > 20.0 and abs(log.w_dps[-1, o]) < 1.0
    # q is in rad and w in deg/s: the angle is the integral of the logged speed.
    turned_deg = np.trapezoid(log.w_dps[:, a], log.t)
    assert np.degrees(q[axis][-1] - q[axis][0]) == pytest.approx(turned_deg, rel=0.05)


@pytest.mark.parametrize("log_hz, actual_hz", [(50.0, 50.0), (30.0, 25.0), (100.0, 50.0)])
def test_meta_log_hz_is_the_rate_actually_logged(log_hz, actual_hz):
    # Logging can only decimate the 50 Hz control rate, so 30 Hz becomes 25 Hz and 100 Hz
    # 50 Hz. summarize() sizes its windows from meta["log_hz"], so it must be the real rate.
    minutes = 0.05
    log = simulate("clino2d", minutes=minutes, seed=1, log_hz=log_hz)
    hz = log.meta["log_hz"]
    assert hz == pytest.approx(actual_hz)
    assert np.allclose(np.diff(log.t), 1.0 / hz)
    assert len(log.t) == round(minutes * 60.0 * hz)


def test_only_inner_axis_keeps_full_gravity():
    # Inner axis is vertical when outer = 0, so spinning it cannot average gravity away.
    cfg = copy.deepcopy(CONFIG)
    cfg["clino3d"] = {"inner_dps": 60.0, "outer_dps": 0.0}
    log = simulate("clino3d", minutes=1.0, seed=1, cfg=cfg)
    assert residual_g(log.g_sample) > 0.95


def test_clino2d_cancels_gravity_on_a_circle():
    log = simulate("clino2d", minutes=1.0, seed=1)
    assert residual_g(log.g_sample) < 0.05
    frac, _ = sphere_coverage(log.g_sample)
    assert frac < 0.3


def test_deterministic():
    a = simulate("random", minutes=0.2, seed=3)
    b = simulate("random", minutes=0.2, seed=3)
    assert np.array_equal(a.q_outer, b.q_outer) and np.array_equal(a.g_sample, b.g_sample)


def test_seed_changes_run():
    a = simulate("random", minutes=0.2, seed=3)
    b = simulate("random", minutes=0.2, seed=4)
    assert not np.array_equal(a.q_outer, b.q_outer)


def test_tracking_in_random_mode():
    log = simulate("random", minutes=1.0, seed=1)
    rms = speed_tracking(log.cmd_dps, log.w_dps)
    assert (rms < 10.0).all(), rms


def test_sensor_convention_payload_at_axis_intersection():
    log = simulate("random", minutes=0.5, seed=2)
    mx, _ = extra_accel(log.acc_sample, log.g_sample)
    assert mx < 0.001


def test_cli_writes_npz_and_summary(tmp_path):
    main(["--mode", "clino3d", "--minutes", "0.05", "--seed", "2", "--out", str(tmp_path)])
    data = np.load(tmp_path / "clino3d_s2.npz")
    assert "g_sample" in data and "t" in data
    summary = json.loads((tmp_path / "summary.json").read_text())
    entry = summary["clino3d_s2"]
    assert {"residual_g", "coverage", "extra_accel_max_g", "tracking_rms_dps"} <= set(entry)


def synthetic_log(n=300, hz=2.0):
    """A SimLog without physics: random gravity directions, different errors per axis."""
    rng = np.random.default_rng(0)
    g = G0 * rng.normal(size=(n, 3))
    cmd = np.zeros((n, 2))
    w = np.tile([1.0, 3.0], (n, 1))  # inner 1 deg/s off, outer 3 deg/s off
    return SimLog(t=np.arange(1, n + 1) / hz, q_outer=np.zeros(n), q_inner=np.zeros(n),
                  w_dps=w, cmd_dps=cmd, pulses=np.zeros((n, 2), int), g_sample=g,
                  acc_sample=-g, meta={"log_hz": hz})


def test_summary_keys_and_windows_follow_meta():
    log = synthetic_log()  # 150 s at 2 Hz: two 60 s windows, no 600 s window
    out = summarize(log)
    assert out["tracking_rms_dps"] == pytest.approx({"inner": 1.0, "outer": 3.0})
    med, p95 = windowed_residual(log.g_sample, 60, log.meta["log_hz"])
    assert out["residual_g_60s"] == pytest.approx({"median": med, "p95": p95})
    assert "residual_g_600s" not in out


def test_summary_reports_accelerometer_mean():
    # PLAN A11: the time-averaged accelerometer reading next to the exact gravity metric.
    # A constant extra acceleration (e.g. a centripetal bias) shows up only there.
    log = synthetic_log()
    bias = np.array([0.0, 0.3 * G0, 0.0])
    log.acc_sample = -log.g_sample + bias
    out = summarize(log)
    expected = np.linalg.norm(np.mean(-log.g_sample, axis=0) + bias) / G0
    assert out["residual_acc_g"] == pytest.approx(expected)
    assert out["residual_acc_g"] != pytest.approx(out["residual_g"], abs=0.1)


def test_cli_keeps_earlier_runs_in_summary(tmp_path):
    main(["--mode", "clino2d", "--minutes", "0.02", "--seed", "1", "--out", str(tmp_path)])
    main(["--mode", "clino3d", "--minutes", "0.02", "--seed", "1", "--out", str(tmp_path)])
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert {"clino2d_s1", "clino3d_s1"} <= set(summary)


@pytest.mark.slow
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_random_mode_simulates_microgravity(seed):
    # PLAN 8.3: start value 0.05, fixed after the first real run. Measured 30 min, seeds 1-3:
    # 0.043 / 0.054 / 0.047 (about 130 random holds, so ~1/sqrt(N) scatter) -> 0.07.
    log = simulate("random", minutes=30.0, seed=seed)
    assert residual_g(log.g_sample) < 0.07
    frac, _ = sphere_coverage(log.g_sample)
    assert frac > 0.9


@pytest.mark.slow
def test_extra_accel_with_offset_payload(tmp_path):
    p = MachineParams(payload_offset=(0.0, 0.0, 0.010))
    log = simulate("random", minutes=10.0, seed=1, params=p)
    mx, rms = extra_accel(log.acc_sample, log.g_sample)
    (tmp_path / "extra.json").write_text(json.dumps({"max_g": mx, "rms_g": rms}))
    assert np.isfinite(mx) and np.isfinite(rms)
