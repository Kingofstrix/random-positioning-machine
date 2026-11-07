"""Zero-g mode under calibration mismatch: each axis gets its own 'real' servo that differs from
the firmware's table (robustness envelope from PR #25: +-20 % speed, +-15 us neutral).

Our sim used identical, perfectly calibrated servos before, so it could not see that the old
clino3d (30 / 18.54 deg/s) phase-locks at 1:1 when outer runs slow and inner runs fast.
"""

import copy
import math

import numpy as np
import pytest

from rpmcore.config import CONFIG
from rpmcore.servo_map import ServoMap
from sim.machine import SimMachine
from sim.metrics import G0, windowed_residual
from sim.params import MachineParams
from sim.run import simulate
from sim.servo_model import u_from_pulse

SCALE, SHIFT = 0.20, 15
# (inner, outer) real-servo deviations; the first one pushes the speed ratio towards 1:1.
CORNERS = {
    "outer_slow_inner_fast": ((1 + SCALE, 1 - SCALE), (-SHIFT, SHIFT)),
    "outer_fast_inner_slow": ((1 - SCALE, 1 + SCALE), (SHIFT, -SHIFT)),
    "both_fast": ((1 + SCALE, 1 + SCALE), (-SHIFT, -SHIFT)),
    "both_slow": ((1 - SCALE, 1 - SCALE), (SHIFT, SHIFT)),
}


def corner_params(name):
    scale, shift = CORNERS[name]
    return MachineParams(servo_speed_scale=scale, servo_neutral_shift_us=shift)


def residual_60s(log):
    return windowed_residual(log.g_sample, 60.0, log.meta["log_hz"])[0]


def test_speed_scale_changes_real_axis_speed_only():
    m = SimMachine(MachineParams(servo_speed_scale=(1.0, 0.5)))
    m.step(1850, 1850, duration=1.5)
    s = m.state()
    p = m.params
    assert s["w_inner"] == pytest.approx(p.omega0 / p.gear_ratio, rel=0.05)
    assert s["w_outer"] == pytest.approx(0.5 * p.omega0 / p.gear_ratio, rel=0.05)


def test_neutral_shift_moves_the_deadband():
    # Real neutral at 1515 us: 1560 us is only 45 us off, the edge of the deadband.
    m = SimMachine(MachineParams(servo_neutral_shift_us=(0, 15)))
    m.step(1560, 1560, duration=1.0)
    s = m.state()
    assert abs(s["w_outer"]) < 1e-3
    assert s["w_inner"] > 0.05


# A mismatch inside the envelope that makes the old clino3d run 1:1: with the nominal table,
# outer 30 deg/s -> 1618 us and inner 18.54 deg/s -> 1590 us; a real outer servo 10 % slow with
# neutral +8 us and a real inner one 10 % fast with neutral -8 us both turn at ~58 % of 1/4 omega0.
LOCK = MachineParams(servo_speed_scale=(1.10, 0.90), servo_neutral_shift_us=(-8, 8))


def real_frame_dps(pulse_us, p, axis):
    i = 0 if axis == "inner" else 1
    u = u_from_pulse(pulse_us, p, p.servo_neutral_shift_us[i])
    return math.degrees(u * p.omega0 * p.servo_speed_scale[i] / p.gear_ratio)


def test_lock_case_really_runs_one_to_one():
    outer = real_frame_dps(ServoMap(CONFIG["servo"]["outer"]).pulse_for(30.0), LOCK, "outer")
    inner = real_frame_dps(ServoMap(CONFIG["servo"]["inner"]).pulse_for(18.54), LOCK, "inner")
    assert inner / outer == pytest.approx(1.0, abs=0.02)


def test_old_clino3d_phase_locks_under_calibration_error():
    # Documents why the ratio changed.
    cfg = copy.deepcopy(CONFIG)
    cfg["clino3d"] = {"outer_dps": 30.0, "inner_dps": 18.54}
    cfg["accel_dps2"] = 20.0
    log = simulate("clino3d", 3.0, 1, params=LOCK, cfg=cfg)
    # Measured 0.26 g after 3 min (an exact 1:1 would keep 0.5 g); the final setting gets ~0.02 g.
    assert np.linalg.norm(log.g_sample.mean(0)) / G0 > 0.2


def test_final_clino3d_ratio_clear_of_one_over_the_whole_envelope():
    # Real speed ratio for every mismatch on a grid over the envelope, using the sim's servo.
    c = CONFIG["clino3d"]
    po = ServoMap(CONFIG["servo"]["outer"]).pulse_for(c["outer_dps"])
    pi = ServoMap(CONFIG["servo"]["inner"]).pulse_for(c["inner_dps"])
    worst = 0.0
    for si in np.linspace(1 - SCALE, 1 + SCALE, 9):
        for so in np.linspace(1 - SCALE, 1 + SCALE, 9):
            for hi in np.linspace(-SHIFT, SHIFT, 7):
                for ho in np.linspace(-SHIFT, SHIFT, 7):
                    p = MachineParams(servo_speed_scale=(si, so), servo_neutral_shift_us=(hi, ho))
                    worst = max(worst, real_frame_dps(pi, p, "inner") / real_frame_dps(po, p, "outer"))
    assert worst < 0.75


def test_final_clino3d_survives_the_worst_corner():
    log = simulate("clino3d", 3.0, 1, params=corner_params("outer_slow_inner_fast"))
    # Measured 0.022 g; most of it is the start-up from rest, which weighs more in a 3-min mean.
    assert np.linalg.norm(log.g_sample.mean(0)) / G0 < 0.03
    assert residual_60s(log) < 0.08


@pytest.mark.slow
@pytest.mark.parametrize("corner", sorted(CORNERS))
def test_final_clino3d_at_every_corner(corner):
    log = simulate("clino3d", 10.0, 1, params=corner_params(corner))
    assert np.linalg.norm(log.g_sample.mean(0)) / G0 < 0.01
    assert residual_60s(log) < 0.08
