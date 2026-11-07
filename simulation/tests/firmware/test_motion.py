import random

import pytest

from rpmcore.motion import (
    PROFILES,
    Clinostat2DProfile,
    Clinostat3DProfile,
    RandomWalkProfile,
    SlewLimiter,
    dt_to_ms,
)
from rpmcore.prng import XorShift32

DT = 0.02


# --- SlewLimiter ---------------------------------------------------------------------------

def test_slew_never_exceeds_rate():
    lim = SlewLimiter(20.0)
    py = random.Random(0)
    prev = lim.value
    for _ in range(5000):
        target = py.uniform(-100, 100) if py.random() < 0.05 else None
        dt = py.choice([0.0, 0.001, 0.02, 0.05, 0.3])
        cur = lim.update(target if target is not None else prev, dt)
        assert abs(cur - prev) <= 20.0 * dt + 1e-9
        prev = cur


def test_slew_reaches_target_exactly():
    lim = SlewLimiter(20.0)
    for _ in range(200):
        lim.update(30.0, DT)
    assert lim.value == 30.0
    for _ in range(200):
        lim.update(0.0, DT)
    assert lim.value == 0.0


def test_slew_ignores_non_positive_dt():
    lim = SlewLimiter(20.0)
    assert lim.update(50.0, 0.0) == 0.0
    assert lim.update(50.0, -1.0) == 0.0


def test_slew_reset():
    lim = SlewLimiter(20.0)
    lim.update(10.0, 1.0)
    lim.reset()
    assert lim.value == 0.0


# --- RandomWalkProfile ---------------------------------------------------------------------

@pytest.fixture
def rparams(cfg):
    return cfg["random"]


def changes(profile, n_changes, dt=DT):
    """Run a profile until its targets changed n times; return (targets, ticks_between)."""
    seen = [profile.targets(dt)]
    gaps, ticks = [], 0
    while len(seen) <= n_changes:
        t = profile.targets(dt)
        ticks += 1
        if t != seen[-1]:
            seen.append(t)
            gaps.append(ticks)
            ticks = 0
    return seen, gaps


def test_random_magnitudes_in_range(rparams):
    targets, _ = changes(RandomWalkProfile(rparams, XorShift32(1)), 300)
    for inner, outer in targets:
        for v in (inner, outer):
            assert rparams["v_min_dps"] <= abs(v) <= rparams["v_max_dps"]


def test_random_both_signs_on_both_axes(rparams):
    targets, _ = changes(RandomWalkProfile(rparams, XorShift32(2)), 100)
    assert {t[0] > 0 for t in targets} == {True, False}
    assert {t[1] > 0 for t in targets} == {True, False}
    # Axes are drawn independently, so they must not always share the same sign.
    assert any((i > 0) != (o > 0) for i, o in targets)


def test_random_hold_draws_are_int_ms_in_range(rparams):
    p = RandomWalkProfile(rparams, XorShift32(3))
    lo, hi = int(rparams["hold_min_s"] * 1000), int(rparams["hold_max_s"] * 1000)
    holds = set()
    for _ in range(200):
        holds.add(p.hold_ms)
        assert isinstance(p.hold_ms, int)
        assert lo <= p.hold_ms <= hi
        p.redraw()
    assert min(holds) < lo + 2000 and max(holds) > hi - 2000


def test_random_measured_hold_times(rparams):
    _, gaps = changes(RandomWalkProfile(rparams, XorShift32(4)), 100)
    for ticks in gaps:
        held = ticks * DT
        assert rparams["hold_min_s"] - DT <= held <= rparams["hold_max_s"] + DT


def test_random_hold_does_not_drift(rparams):
    # Overshoot past a hold carries into the next one, so long runs keep the planned timing.
    p = RandomWalkProfile(rparams, XorShift32(5))
    planned = 0
    ticks = 0
    for _ in range(50):
        planned += p.hold_ms
        before = p.targets(0.0)
        while p.targets(0.007) == before:
            ticks += 1
        ticks += 1
    assert abs(ticks * 7 - planned) <= 7


def test_random_same_seed_same_targets(rparams):
    a, _ = changes(RandomWalkProfile(rparams, XorShift32(9)), 50)
    b, _ = changes(RandomWalkProfile(rparams, XorShift32(9)), 50)
    c, _ = changes(RandomWalkProfile(rparams, XorShift32(10)), 50)
    assert a == b
    assert a != c


def test_random_reads_params_live(rparams):
    # set_param changes the dict the profile holds; the next draw must use the new limits.
    p = RandomWalkProfile(rparams, XorShift32(6))
    rparams["v_min_dps"], rparams["v_max_dps"] = 40.0, 41.0
    rparams["hold_min_s"], rparams["hold_max_s"] = 1.0, 1.5
    p.redraw()
    assert all(40.0 <= abs(v) <= 41.0 for v in p.targets(0.0))
    assert 1000 <= p.hold_ms <= 1500


def test_random_equal_hold_bounds(rparams):
    rparams["hold_min_s"] = rparams["hold_max_s"] = 2.0
    p = RandomWalkProfile(rparams, XorShift32(7))
    for _ in range(20):
        assert p.hold_ms == 2000
        p.redraw()


# --- Clinostats and registry ---------------------------------------------------------------

def test_clino3d_constant(cfg):
    p = Clinostat3DProfile(cfg["clino3d"], XorShift32(1))
    for _ in range(100):
        assert p.targets(DT) == (18.54, 30.0)


def test_clino2d_only_outer(cfg):
    p = Clinostat2DProfile(cfg["clino2d"], XorShift32(1))
    for _ in range(100):
        assert p.targets(DT) == (0.0, 30.0)


def test_registry():
    assert PROFILES == {
        "random": RandomWalkProfile,
        "clino3d": Clinostat3DProfile,
        "clino2d": Clinostat2DProfile,
    }


# --- Review follow-ups ---------------------------------------------------------------------

@pytest.mark.parametrize(
    "dt, ms", [(0.02, 20), (1.001, 1001), (0.0199999995, 20), (0.0, 0), (-1.0, 0)]
)
def test_dt_to_ms_rounds_to_nearest(dt, ms):
    assert dt_to_ms(dt) == ms


def test_dt_to_ms_ignores_nan():
    assert dt_to_ms(float("nan")) == 0


def test_hold_is_drawn_from_u32_stream(rparams):
    # Pins the float32-safe derivation (PLAN 5.4): four speed/sign draws, then one u32.
    p = RandomWalkProfile(rparams, XorShift32(1))
    ref = XorShift32(1)
    for _ in range(2):
        ref.uniform(0.0, 1.0)
        ref.choice_sign()
    lo, hi = 5000, 20000
    assert p.hold_ms == lo + ref.next_u32() % (hi - lo + 1)


def test_sub_ms_hold_cannot_hang(rparams):
    # A hold that rounds to 0 ms would make targets() loop forever on the Pico.
    rparams["hold_min_s"] = rparams["hold_max_s"] = 0.0001
    p = RandomWalkProfile(rparams, XorShift32(1))
    assert p.hold_ms >= 1
