"""CONFIG carries team interfaces (docs/interfaces.md), so drift must fail loudly."""

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
