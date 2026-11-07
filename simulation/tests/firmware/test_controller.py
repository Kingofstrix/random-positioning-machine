import copy
import random

import pytest

from rpmcore.controller import CAL, RUNNING, STOPPED, STOPPING, Controller

DT = 0.02


@pytest.fixture
def c(cfg):
    return Controller(cfg)


def run(ctrl, seconds, dt=DT):
    out = []
    for _ in range(int(round(seconds / dt))):
        out.append(ctrl.tick(dt))
    return out


def speeds(ctrl):
    s = ctrl.status()
    return s["inner_dps"], s["outer_dps"]


def test_starts_stopped_with_pwm_off(c):
    assert c.state == STOPPED
    assert run(c, 1.0) == [(None, None)] * 50


def test_does_not_mutate_given_config(cfg):
    before = copy.deepcopy(cfg)
    ctrl = Controller(cfg)
    ctrl.set_param("v_max", 50)
    ctrl.set_param("seed", 7)
    ctrl.start()
    run(ctrl, 2.0)
    assert cfg == before


def test_start_ramps_up(c, cfg):
    c.start("clino3d")
    assert c.state == RUNNING
    accel = cfg["accel_dps2"]
    prev = (0.0, 0.0)
    for i in range(1, 200):
        c.tick(DT)
        cur = speeds(c)
        for p, v in zip(prev, cur):
            assert abs(v - p) <= accel * DT + 1e-9
            assert abs(v) <= accel * DT * i + 1e-9
        prev = cur
    assert speeds(c) == (cfg["clino3d"]["inner_dps"], cfg["clino3d"]["outer_dps"])


def test_start_unknown_mode_rejected(c, cfg):
    with pytest.raises(ValueError):
        c.start("warp")
    assert c.state == STOPPED
    assert c.mode == cfg["default_mode"]


def test_stop_ramps_down_then_pwm_off(c, cfg):
    c.start("clino2d")
    run(c, 3.0)
    c.stop()
    assert c.state == STOPPING
    prev = abs(speeds(c)[1])
    pulses = []
    for _ in range(int(prev / cfg["accel_dps2"] / DT) + 2):
        pulses.append(c.tick(DT))
        cur = abs(speeds(c)[1])
        assert cur <= prev
        prev = cur
    assert c.state == STOPPED
    assert speeds(c) == (0.0, 0.0)
    # The tick that reaches 0 still sends neutral, every later tick switches the PWM off.
    last_moving = max(i for i, p in enumerate(pulses) if p != (None, None))
    assert pulses[last_moving] == (1500, 1500)
    assert all(p == (None, None) for p in pulses[last_moving + 1:])
    assert c.tick(DT) == (None, None)


def test_stop_reaches_zero_in_expected_time(c, cfg):
    c.start("clino2d")
    run(c, 3.0)  # 30 deg/s needs 1.5 s at 20 deg/s^2
    c.stop()
    n = 0
    while c.state == STOPPING:
        c.tick(DT)
        n += 1
    assert n * DT == pytest.approx(30.0 / cfg["accel_dps2"], abs=2 * DT)


def test_estop_is_immediate(c):
    c.start("clino3d")
    run(c, 2.0)
    c.estop()
    assert c.state == STOPPED
    assert speeds(c) == (0.0, 0.0)
    assert c.tick(DT) == (None, None)


def test_restart_after_estop_ramps_from_zero(c, cfg):
    c.start("clino3d")
    run(c, 2.0)
    c.estop()
    c.start()
    c.tick(DT)
    assert all(abs(v) <= cfg["accel_dps2"] * DT + 1e-9 for v in speeds(c))


def test_start_while_stopping_ramps_from_current_speed(c, cfg):
    c.start("clino2d")
    run(c, 3.0)
    c.stop()
    run(c, 0.5)
    before = speeds(c)[1]
    c.start()
    assert c.state == RUNNING
    c.tick(DT)
    assert abs(speeds(c)[1] - before) <= cfg["accel_dps2"] * DT + 1e-9


def test_mode_switch_is_smooth(c, cfg):
    c.start("clino3d")
    run(c, 3.0)
    prev = speeds(c)
    c.set_mode("clino2d")
    assert c.mode == "clino2d"
    for _ in range(100):
        c.tick(DT)
        cur = speeds(c)
        assert all(abs(a - b) <= cfg["accel_dps2"] * DT + 1e-9 for a, b in zip(prev, cur))
        prev = cur
    assert speeds(c) == (0.0, 30.0)


def test_set_mode_while_stopped_is_remembered(c):
    c.set_mode("clino2d")
    assert c.state == STOPPED
    c.start()
    run(c, 3.0)
    assert speeds(c) == (0.0, 30.0)


def test_set_mode_unknown_rejected(c, cfg):
    with pytest.raises(ValueError):
        c.set_mode("nope")
    assert c.mode == cfg["default_mode"]


def test_targets_clamped_to_servo_range(cfg):
    cfg["clino3d"]["outer_dps"] = 500.0
    ctrl = Controller(cfg)
    ctrl.start("clino3d")
    run(ctrl, 10.0)
    assert speeds(ctrl)[1] == 125.0


# --- Pulses --------------------------------------------------------------------------------

def test_pulses_follow_speed_direction(c):
    c.start("clino2d")
    run(c, 3.0)
    inner_us, outer_us = c.tick(DT)
    assert inner_us == 1500
    assert outer_us > 1545


# --- Calibration ---------------------------------------------------------------------------

def test_cal_pulse_from_stopped(c):
    c.cal_pulse("outer", 1600)
    assert c.state == CAL
    assert c.tick(DT) == (None, 1600)
    c.cal_pulse("inner", 1400)
    assert c.tick(DT) == (1400, 1600)


def test_cal_off_returns_to_stopped(c):
    c.cal_pulse("inner", 1600)
    c.cal_off()
    assert c.state == STOPPED
    assert c.tick(DT) == (None, None)


@pytest.mark.parametrize("state", ["running", "stopping"])
def test_cal_pulse_rejected_while_moving(c, state):
    c.start("clino3d")
    run(c, 1.0)
    if state == "stopping":
        c.stop()
    with pytest.raises(ValueError):
        c.cal_pulse("outer", 1600)
    assert c.state in (RUNNING, STOPPING)


@pytest.mark.parametrize("axis, us", [("middle", 1600), ("outer", 699), ("inner", 2301)])
def test_cal_pulse_invalid_rejected(c, axis, us):
    with pytest.raises(ValueError):
        c.cal_pulse(axis, us)
    assert c.state == STOPPED


def test_start_from_cal_rejected(c):
    c.cal_pulse("outer", 1600)
    with pytest.raises(ValueError):
        c.start()
    assert c.state == CAL


def test_stop_and_estop_end_cal(c):
    c.cal_pulse("outer", 1600)
    c.stop()
    assert c.state == STOPPED and c.tick(DT) == (None, None)
    c.cal_pulse("outer", 1600)
    c.estop()
    assert c.state == STOPPED and c.tick(DT) == (None, None)


# --- Parameters ----------------------------------------------------------------------------

@pytest.mark.parametrize(
    "key, value",
    [
        ("v_min", 0), ("v_min", -5), ("v_min", 100),       # 100 > v_max 90
        ("v_max", 5),                                     # below v_min 30
        ("v_max", 126),                                   # beyond servo range
        ("accel", 0), ("accel", float("nan")), ("accel", float("inf")),
        ("hold_min", 0), ("hold_min", 30),                # 30 > hold_max 20
        ("hold_max", 1),
        ("seed", 1.5), ("seed", "x"),
        ("v_min", "abc"), ("bogus", 1),
    ],
)
def test_invalid_params_rejected_and_nothing_changes(c, key, value):
    before = (copy.deepcopy(c.cfg), c.status())
    with pytest.raises(ValueError):
        c.set_param(key, value)
    assert (c.cfg, c.status()) == before


def test_valid_params_applied(c):
    c.set_param("v_max", 80)
    c.set_param("v_min", 20)
    c.set_param("hold_min", 1)
    c.set_param("hold_max", 2.5)
    r = c.cfg["random"]
    assert (r["v_min_dps"], r["v_max_dps"], r["hold_min_s"], r["hold_max_s"]) == (20, 80, 1, 2.5)


def test_accel_param_changes_ramp(c):
    c.set_param("accel", 50)
    c.start("clino2d")
    c.tick(DT)
    assert speeds(c)[1] == pytest.approx(50 * DT)


def test_seed_param_reproduces_run(cfg):
    a = Controller(cfg)
    a.set_param("seed", 5)
    a.start("random")
    cfg5 = copy.deepcopy(cfg)
    cfg5["seed"] = 5
    b = Controller(cfg5)
    b.start("random")
    assert run(a, 60.0) == run(b, 60.0)


def test_numeric_strings_accepted(c):
    # The serial parser hands values over as text.
    c.set_param("v_max", "55.5")
    c.set_param("seed", "12")
    assert c.cfg["random"]["v_max_dps"] == 55.5
    assert c.cfg["seed"] == 12


# --- Status and time -----------------------------------------------------------------------

def test_status_fields(c):
    s = c.status()
    assert set(s) == {"state", "mode", "inner_dps", "outer_dps", "inner_us", "outer_us", "t_ms"}
    assert (s["state"], s["mode"], s["t_ms"]) == (STOPPED, c.cfg["default_mode"], 0)
    assert s["inner_us"] is None and s["outer_us"] is None


def test_runtime_is_int_ms(c):
    c.start()
    run(c, 1.0)
    t = c.status()["t_ms"]
    assert isinstance(t, int) and t == 1000


# --- Invariants under random command sequences ---------------------------------------------

def test_invariants_under_random_commands(cfg):
    ctrl = Controller(cfg)
    py = random.Random(1234)
    accel = cfg["accel_dps2"]
    prev = speeds(ctrl)
    for _ in range(10_000):
        r = py.random()
        jumped = False
        try:
            if r < 0.01:
                ctrl.start(py.choice([None, "random", "clino3d", "clino2d"]))
            elif r < 0.02:
                ctrl.stop()
            elif r < 0.025:
                ctrl.estop()
                jumped = True
            elif r < 0.03:
                ctrl.set_mode(py.choice(["random", "clino3d", "clino2d"]))
            elif r < 0.035:
                ctrl.cal_pulse(py.choice(["inner", "outer"]), py.randint(600, 2400))
            elif r < 0.04:
                ctrl.cal_off()
            elif r < 0.045:
                ctrl.set_param(
                    py.choice(["v_min", "v_max", "accel", "hold_min", "hold_max", "seed"]),
                    py.uniform(-10, 150),
                )
        except ValueError:
            pass
        accel = ctrl.cfg["accel_dps2"]
        dt = py.choice([0.0, 0.01, 0.02, 0.02, 0.05])
        pulses = ctrl.tick(dt)
        cur = speeds(ctrl)
        for us in pulses:
            assert us is None or 700 <= us <= 2300
        if ctrl.state == STOPPED:
            assert pulses == (None, None) or pulses == (1500, 1500)
        if not jumped:
            for a, b in zip(prev, cur):
                assert abs(b - a) <= accel * dt + 1e-6
        prev = cur


def test_deterministic(cfg):
    def scripted():
        ctrl = Controller(cfg)
        ctrl.start()
        out = run(ctrl, 30.0)
        ctrl.set_mode("clino3d")
        out += run(ctrl, 5.0)
        ctrl.stop()
        return out + run(ctrl, 10.0)

    assert scripted() == scripted()


# --- Review follow-ups ---------------------------------------------------------------------

@pytest.fixture
def asym_cfg(cfg):
    """Inner servo slower than outer, so axis mix-ups become visible."""
    cfg["servo"]["inner"]["pos"] = [(0.0, 45), (60.0, 350)]
    cfg["servo"]["inner"]["neg"] = [(0.0, -45), (60.0, -350)]
    cfg["random"]["v_max_dps"] = 50.0
    return cfg


@pytest.mark.parametrize("sign", [1, -1])
def test_each_axis_clamped_to_own_range(asym_cfg, sign):
    asym_cfg["clino3d"]["inner_dps"] = 500.0 * sign
    asym_cfg["clino3d"]["outer_dps"] = 500.0 * sign
    ctrl = Controller(asym_cfg)
    ctrl.start("clino3d")
    pulses = run(ctrl, 10.0)[-1]
    assert speeds(ctrl) == (60.0 * sign, 125.0 * sign)
    assert pulses == (1500 + 350 * sign, 1500 + 350 * sign)
    assert ctrl.max_dps() == 60.0


def test_pulses_use_each_axis_map(asym_cfg):
    asym_cfg["clino3d"]["inner_dps"] = asym_cfg["clino3d"]["outer_dps"] = 30.0
    ctrl = Controller(asym_cfg)
    ctrl.start("clino3d")
    inner_us, outer_us = run(ctrl, 5.0)[-1]
    assert inner_us == ctrl.maps["inner"].pulse_for(30.0) == 1698  # 45 + 305 * 30/60
    assert outer_us == ctrl.maps["outer"].pulse_for(30.0) == 1618


def test_v_max_limited_by_slower_axis(asym_cfg):
    ctrl = Controller(asym_cfg)
    with pytest.raises(ValueError):
        ctrl.set_param("v_max", 100)
    ctrl.set_param("v_max", 60)


def test_t_ms_resets_on_start_from_stopped(c):
    c.start("clino3d")
    run(c, 1.0)
    c.estop()
    c.start()
    assert c.status()["t_ms"] == 0


def test_t_ms_continues_on_start_while_stopping(c):
    c.start("clino3d")
    run(c, 1.0)
    c.stop()
    run(c, 0.2)
    c.start()
    assert c.status()["t_ms"] == 1200


def test_start_same_mode_while_running_keeps_profile(c):
    c.start("random")
    run(c, 1.0)
    profile, targets = c.profile, c.profile.targets(0.0)
    c.start("random")
    c.start()
    assert c.profile is profile and c.profile.targets(0.0) == targets


def test_start_other_mode_while_running_switches(c):
    c.start("random")
    run(c, 1.0)
    c.start("clino2d")
    assert c.mode == "clino2d"
    run(c, 10.0)
    assert speeds(c) == (0.0, 30.0)


def test_set_param_reaches_running_profile(c):
    c.start("random")
    run(c, 1.0)
    c.set_param("v_max", 41)
    c.set_param("v_min", 40)
    c.set_param("hold_min", 0.5)
    c.set_param("hold_max", 1)
    run(c, 25.0)  # past the old hold (<= 20 s) and several new ones
    assert all(40.0 <= abs(v) <= 41.0 for v in c.profile.targets(0.0))


@pytest.mark.parametrize("end", ["cal_off", "stop", "estop"])
def test_calibration_does_not_leak_into_next_one(c, end):
    c.cal_pulse("inner", 1600)
    getattr(c, end)()
    c.cal_pulse("outer", 1600)
    assert c.tick(DT) == (None, 1600)


def test_status_pulses_match_last_tick(c):
    c.start("clino3d")
    for _ in range(30):
        out = c.tick(DT)
    s = c.status()
    assert (s["inner_us"], s["outer_us"]) == out
    assert out != (None, None)


@pytest.mark.parametrize("axis, us", [("outer", 700), ("inner", 2300)])
def test_cal_pulse_limits_inclusive(c, axis, us):
    c.cal_pulse(axis, us)
    assert c.tick(DT)[0 if axis == "inner" else 1] == us


@pytest.mark.parametrize("key, value", [("v_min", 90), ("v_max", 30), ("hold_min", 20), ("hold_max", 5)])
def test_equal_bounds_accepted(c, key, value):
    c.set_param(key, value)


@pytest.mark.parametrize("value", ["0.0004", "0.01", "1e306", "1e36", "86401"])
@pytest.mark.parametrize("key", ["hold_min", "hold_max"])
def test_hold_outside_sane_range_rejected(c, key, value):
    # Sub-ms holds hung tick() forever, huge ones overflowed inside tick() on the Pico.
    before = copy.deepcopy(c.cfg)
    if key == "hold_min":
        c.set_param("hold_max", 3600)
        before = copy.deepcopy(c.cfg)
    with pytest.raises(ValueError):
        c.set_param(key, value)
    assert c.cfg == before


def test_hold_limits_come_from_config(cfg):
    lo, hi = cfg["limits"]["hold_s"]
    ctrl = Controller(cfg)
    ctrl.set_param("hold_max", hi)
    ctrl.set_param("hold_min", lo)
    with pytest.raises(ValueError):
        ctrl.set_param("hold_min", lo / 2)
