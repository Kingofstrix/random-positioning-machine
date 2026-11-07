"""State machine that turns commands and profiles into servo pulses (docs/PLAN.md 5.5).

The same object runs on the Pico (firmware/main.py) and in the simulation, so everything that
decides what the servos do lives here and not in the hardware or physics adapters.
Starting automatically (CONFIG["autostart"]) is left to the caller.
"""

import math

from rpmcore.motion import PROFILES, SlewLimiter, dt_to_ms
from rpmcore.prng import XorShift32
from rpmcore.servo_map import ServoMap

STOPPED = "STOPPED"
RUNNING = "RUNNING"
STOPPING = "STOPPING"
CAL = "CAL"

AXES = ("inner", "outer")

# `set <key>` name -> (CONFIG section or None for top level, CONFIG field)
_PARAMS = {
    "v_min": ("random", "v_min_dps"),
    "v_max": ("random", "v_max_dps"),
    "hold_min": ("random", "hold_min_s"),
    "hold_max": ("random", "hold_max_s"),
    "accel": (None, "accel_dps2"),
    "seed": (None, "seed"),
}


def _clone(obj):
    # MicroPython has no `copy` module; CONFIG only holds dicts, lists and immutables.
    if isinstance(obj, dict):
        return {k: _clone(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_clone(v) for v in obj]
    return obj


def _parse_int(value):
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            raise ValueError("not an integer: %s" % value)
    try:
        f = float(value)
        i = int(f)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("not an integer: %s" % (value,))
    if f != i:
        raise ValueError("not an integer: %s" % (value,))
    return i


def _parse_positive(value):
    try:
        f = float(value)
    except (TypeError, ValueError):
        raise ValueError("not a number: %s" % (value,))
    if not (math.isfinite(f) and f > 0):
        raise ValueError("must be a positive finite number: %s" % (value,))
    return f


class Controller:
    def __init__(self, cfg):
        self.cfg = _clone(cfg)
        self.maps = {}
        self.limiters = {}
        for axis in AXES:
            self.maps[axis] = ServoMap(self.cfg["servo"][axis])
            self.limiters[axis] = SlewLimiter(self.cfg["accel_dps2"])
        self._check_mode(self.cfg["default_mode"])
        self.mode = self.cfg["default_mode"]
        self.rng = XorShift32(self.cfg["seed"])
        self.state = STOPPED
        self.profile = None
        self.t_ms = 0
        self._cal = {"inner": None, "outer": None}
        self._pulses = (None, None)

    # --- commands --------------------------------------------------------------------------

    def start(self, mode=None):
        if self.state == CAL:
            raise ValueError("calibration active, send 'pulse off' first")
        if mode is not None:
            self._check_mode(mode)
        if self.state == RUNNING and (mode is None or mode == self.mode):
            return
        if self.state == STOPPED:
            self.t_ms = 0
        if mode is not None:
            self.mode = mode
        # The limiters keep their current value, so a restart while STOPPING ramps from the
        # speed the frames still have instead of jumping.
        self.profile = PROFILES[self.mode](self.cfg[self.mode], self.rng)
        self.state = RUNNING

    def stop(self):
        if self.state == RUNNING:
            self.state = STOPPING
            self.profile = None
        elif self.state == CAL:
            self._off()

    def estop(self):
        self._off()

    def set_mode(self, name):
        self._check_mode(name)
        self.mode = name
        if self.state == RUNNING:
            self.profile = PROFILES[name](self.cfg[name], self.rng)

    def set_param(self, key, value):
        if key not in _PARAMS:
            raise ValueError("unknown parameter: %s" % key)
        section, field = _PARAMS[key]
        if key == "seed":
            seed = _parse_int(value)
            self.cfg["seed"] = seed
            self.rng.seed(seed)
            return
        v = _parse_positive(value)
        if key == "accel":
            self.cfg["accel_dps2"] = v
            for axis in AXES:
                self.limiters[axis].rate = v
            return
        if key in ("hold_min", "hold_max"):
            lo, hi = self.cfg["limits"]["hold_s"]
            if not lo <= v <= hi:
                raise ValueError("hold time outside %.1f..%.1f s" % (lo, hi))
        # Check the combination before writing anything, so a rejected value changes nothing.
        r = dict(self.cfg[section])
        r[field] = v
        if r["v_min_dps"] > r["v_max_dps"]:
            raise ValueError("v_min must not exceed v_max")
        if r["v_max_dps"] > self.max_dps():
            raise ValueError("v_max beyond servo range (%.1f deg/s)" % self.max_dps())
        if r["hold_min_s"] > r["hold_max_s"]:
            raise ValueError("hold_min must not exceed hold_max")
        self.cfg[section][field] = v

    def cal_pulse(self, axis, us):
        if self.state not in (STOPPED, CAL):
            raise ValueError("stop the machine before calibrating")
        if axis not in self.maps:
            raise ValueError("unknown axis: %s" % (axis,))
        us = _parse_int(us)
        m = self.maps[axis]
        if not m.min_us <= us <= m.max_us:
            raise ValueError("pulse outside %d..%d us" % (m.min_us, m.max_us))
        self._cal[axis] = us
        self.state = CAL
        # status() must show what the new state sends even before the next tick.
        self._pulses = (self._cal["inner"], self._cal["outer"])

    def cal_off(self):
        if self.state == CAL:
            self._off()

    # --- loop ------------------------------------------------------------------------------

    def tick(self, dt):
        """Advance by dt seconds; returns (inner_us, outer_us), None meaning PWM off."""
        state = self.state
        if state == STOPPED:
            out = (None, None)
        elif state == CAL:
            out = (self._cal["inner"], self._cal["outer"])
        else:
            if state == RUNNING:
                t_inner, t_outer = self.profile.targets(dt)
                t_inner = self._clamp("inner", t_inner)
                t_outer = self._clamp("outer", t_outer)
            else:
                t_inner = t_outer = 0.0
            v_inner = self.limiters["inner"].update(t_inner, dt)
            v_outer = self.limiters["outer"].update(t_outer, dt)
            self.t_ms += dt_to_ms(dt)
            out = (self.maps["inner"].pulse_for(v_inner), self.maps["outer"].pulse_for(v_outer))
            if state == STOPPING and v_inner == 0 and v_outer == 0:
                # This tick still sends neutral; from the next tick on the PWM is off.
                self.state = STOPPED
        self._pulses = out
        return out

    def status(self):
        return {
            "state": self.state,
            "mode": self.mode,
            "inner_dps": self.limiters["inner"].value,
            "outer_dps": self.limiters["outer"].value,
            "inner_us": self._pulses[0],
            "outer_us": self._pulses[1],
            "t_ms": self.t_ms,
        }

    def max_dps(self):
        return min(self.maps["inner"].max_dps(), self.maps["outer"].max_dps())

    # --- helpers ---------------------------------------------------------------------------

    def _check_mode(self, name):
        if name not in PROFILES:
            raise ValueError("unknown mode: %s" % (name,))

    def _clamp(self, axis, dps):
        limit = self.maps[axis].max_dps()
        if dps > limit:
            return limit
        if dps < -limit:
            return -limit
        return dps

    def _off(self):
        self.state = STOPPED
        self.profile = None
        self._cal = {"inner": None, "outer": None}
        self._pulses = (None, None)
        for axis in AXES:
            self.limiters[axis].reset()
