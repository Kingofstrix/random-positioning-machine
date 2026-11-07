"""All firmware parameters in one place (docs/PLAN.md 5.1).

Values marked [Annahme] in the plan are estimates until the machine is calibrated (M7).
"""


def _servo_cal():
    # A fresh dict per axis, so calibrating one axis can never change the other.
    return {
        "neutral_us": 1500,
        "min_us": 700,
        "max_us": 2300,
        # Calibration table: (frame speed in deg/s, pulse offset in us), per direction,
        # ascending speed. The first point is the edge of the deadband (FS90R: +-45 us).
        "pos": [(0.0, 45), (125.0, 350)],
        "neg": [(0.0, -45), (125.0, -350)],
    }


CONFIG = {
    # Assembly guide: outer servo GP15, inner servo (via slip ring) GP16.
    "pins": {"outer": 15, "inner": 16},
    "tick_ms": 20,
    "gear_ratio": 4.0,
    "autostart": True,
    # Zero g: the 3D clinostat cancels gravity far faster than the random walk (sim, 2026-11-07).
    "default_mode": "clino3d",
    "seed": 1,
    "accel_dps2": 60.0,
    "random": {"v_min_dps": 30.0, "v_max_dps": 90.0, "hold_min_s": 5.0, "hold_max_s": 20.0},
    # Accepted range for `set hold_min/hold_max`. Holds that round to 0 ms would hang the
    # random profile, huge ones overflow the int-ms conversion on the Pico's float32.
    "limits": {"hold_s": (0.1, 3600.0)},
    # Ratio 1/phi^2 = 0.382: irrational, so the rotations never phase-lock, and far enough from
    # 1:1 that +-20 % / +-15 us calibration error per axis cannot pull them together (0.618 at
    # 30 deg/s could). 80 deg/s leaves about a third of headroom below the servo's maximum.
    "clino3d": {"outer_dps": 80.0, "inner_dps": 30.56},
    "clino2d": {"outer_dps": 30.0},
    "servo": {"outer": _servo_cal(), "inner": _servo_cal()},
}
