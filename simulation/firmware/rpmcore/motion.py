"""Speed ramps and motion profiles (docs/PLAN.md 5.4).

Profiles return target frame speeds in deg/s as (inner, outer). They read their parameter dict
on every draw, so `set` commands take effect without rebuilding the profile.
"""


def dt_to_ms(dt):
    """Tick length in whole ms. Timers count int ms because the Pico has only float32."""
    if not dt > 0:
        return 0
    return int(dt * 1000.0 + 0.5)


class SlewLimiter:
    def __init__(self, rate):
        self.rate = rate
        self.value = 0.0

    def update(self, target, dt):
        if not dt > 0:
            return self.value
        step = self.rate * dt
        diff = target - self.value
        if diff > step:
            self.value += step
        elif diff < -step:
            self.value -= step
        else:
            # Land exactly on the target, so a stop really reaches 0.0.
            self.value = target
        return self.value

    def reset(self):
        self.value = 0.0


class RandomWalkProfile:
    def __init__(self, params, rng):
        self.params = params
        self.rng = rng
        self.redraw()

    def _draw(self):
        p = self.params
        rng = self.rng
        inner = rng.uniform(p["v_min_dps"], p["v_max_dps"]) * rng.choice_sign()
        outer = rng.uniform(p["v_min_dps"], p["v_max_dps"]) * rng.choice_sign()
        self._targets = (inner, outer)
        # Drawn as int ms straight from the u32 stream, so the Pico (float32) and the sim
        # (float64) get identical hold times for the same seed.
        lo = dt_to_ms(p["hold_min_s"])
        hi = dt_to_ms(p["hold_max_s"])
        self.hold_ms = lo + rng.next_u32() % (hi - lo + 1)
        if self.hold_ms < 1:
            # A 0 ms hold would make the loop in targets() spin forever.
            self.hold_ms = 1

    def redraw(self):
        self._draw()
        self._left_ms = self.hold_ms

    def targets(self, dt):
        self._left_ms -= dt_to_ms(dt)
        while self._left_ms <= 0:
            self._draw()
            # Carry the overshoot, so long runs keep the planned hold times on average.
            self._left_ms += self.hold_ms
        return self._targets


class Clinostat3DProfile:
    def __init__(self, params, rng):
        self.params = params

    def targets(self, dt):
        return (float(self.params["inner_dps"]), float(self.params["outer_dps"]))


class Clinostat2DProfile:
    def __init__(self, params, rng):
        self.params = params

    def targets(self, dt):
        return (0.0, float(self.params["outer_dps"]))


PROFILES = {
    "random": RandomWalkProfile,
    "clino3d": Clinostat3DProfile,
    "clino2d": Clinostat2DProfile,
}
