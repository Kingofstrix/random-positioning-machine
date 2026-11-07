"""Controller from rpmcore + MuJoCo physics (docs/PLAN.md 8.3)."""

import copy
import json

import numpy as np
import pytest

from rpmcore.config import CONFIG
from sim.metrics import extra_accel, residual_g, speed_tracking, sphere_coverage
from sim.params import MachineParams
from sim.run import main, simulate


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
