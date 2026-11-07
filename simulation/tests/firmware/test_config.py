"""CONFIG carries team interfaces (docs/interfaces.md), so drift must fail loudly."""

import pytest

from rpmcore.servo_map import ServoMap


def test_pins_match_assembly_guide(cfg):
    # The original Core Electronics code swaps these; the assembly guide is authoritative.
    assert cfg["pins"] == {"outer": 15, "inner": 16}


def test_tick_matches_servo_pwm(cfg):
    assert cfg["tick_ms"] == 20


def test_both_servo_tables_are_valid(cfg):
    for axis in ("outer", "inner"):
        ServoMap(cfg["servo"][axis])


def test_pulse_limits_are_datasheet_range(cfg):
    # Safety bound from the FS90R datasheet; calibration (M7) changes tables, not limits.
    for axis in ("outer", "inner"):
        s = cfg["servo"][axis]
        assert (s["min_us"], s["max_us"]) == (700, 2300)


def test_servo_tables_are_independent_objects(cfg):
    # A calibration written for one axis must not silently change the other.
    from rpmcore.config import CONFIG

    assert CONFIG["servo"]["outer"] is not CONFIG["servo"]["inner"]
    assert CONFIG["servo"]["outer"]["pos"] is not CONFIG["servo"]["inner"]["pos"]


def test_random_mode_never_commands_deadband_speed(cfg):
    r = cfg["random"]
    assert 0 < r["v_min_dps"] < r["v_max_dps"]
    assert 0 < r["hold_min_s"] < r["hold_max_s"]
    for axis in ("outer", "inner"):
        assert r["v_max_dps"] <= ServoMap(cfg["servo"][axis]).max_dps(), axis


# --- Final zero-g logic (sim comparison 2026-11-07, robustness envelope from PR #25) ---------

# Worst calibration mismatch per axis the zero-g mode must survive: real servo speed off by
# +-20 % and its neutral off by +-15 us (about 6 deg/s near the deadband with the default table).
ENVELOPE_SCALE = 0.20
ENVELOPE_SHIFT_DPS = 15 * 125.0 / (350 - 45)


def test_zero_g_is_the_default_mode(cfg):
    assert cfg["default_mode"] == "clino3d"


def test_clino3d_ratio_is_one_over_phi_squared(cfg):
    c = cfg["clino3d"]
    assert c["inner_dps"] / c["outer_dps"] == pytest.approx(0.381966, abs=0.002)


def test_clino3d_ratio_stays_clear_of_resonance_under_calibration_error(cfg):
    # At 1:1 the inner and outer rotations phase-lock and 0.5 g stays for good.
    c = cfg["clino3d"]
    inner_hi = c["inner_dps"] * (1 + ENVELOPE_SCALE) + ENVELOPE_SHIFT_DPS
    outer_lo = c["outer_dps"] * (1 - ENVELOPE_SCALE) - ENVELOPE_SHIFT_DPS
    assert inner_hi / outer_lo < 0.85


def test_speeds_leave_headroom_below_servo_maximum(cfg):
    # Near the servo's no-load maximum it saturates under load and the gears wear.
    top = ServoMap(cfg["servo"]["outer"]).max_dps()
    assert cfg["clino3d"]["outer_dps"] <= 0.75 * top
    assert cfg["random"]["v_max_dps"] <= 0.75 * top
    assert 60.0 <= cfg["clino3d"]["outer_dps"]
