"""Motion strategies: which speeds should the two axes run at right now?

All strategies share one interface so the simulation can compare them and the
firmware can run any of them:

    strategy.step(t, dt, estimate) -> (speed_outer, speed_inner)

``t`` is the time since start in seconds, ``dt`` the control period and
``estimate`` the controller's ``OrientationEstimate`` (dead-reckoned angles and
the metrics of the estimated gravity). Speeds are axis deg/s, limited to
``+-max_speed``.

- ``RandomSetpoints``: the original Core Electronics firmware, reproduced as
  the baseline.
- ``Clinostat``: both axes at constant, incommensurate speeds. Recommended:
  it averages gravity out best and is robust to servo errors (docs/adr/0002).
- ``RandomWalk``: independent random speeds and directions per axis.
"""

import math

from rpm.core.rng import Rng

TYPE_CHECKING = False
if TYPE_CHECKING:
    from rpm.core.controller import OrientationEstimate

DEFAULT_MAX_SPEED = 120.0  # axis deg/s (20 rpm); the nominal FS90R + 4:1 gear reaches ~150


class Strategy:
    """Base class. Subclasses implement ``step``."""

    name = "strategy"

    def __init__(self, max_speed: float = DEFAULT_MAX_SPEED, seed: int = 1) -> None:
        if max_speed <= 0.0:
            raise ValueError("max_speed must be positive")
        self.max_speed = max_speed
        self.rng = Rng(seed)

    def step(self, t: float, dt: float, estimate: "OrientationEstimate") -> "tuple[float, float]":
        raise NotImplementedError


class RandomSetpoints(Strategy):
    """Baseline: the original Core Electronics ``main.py``.

    Every ``hold_s`` seconds each axis draws a setpoint u in [0, 1). The
    internal speed follows it with a ~10 s time constant (originally
    ``speed += 0.01 * (setpoint - speed)`` every 100 ms) and the drive gets
    ``sin(2*pi*speed)``, pushed out of a dead band of 12/25 = 0.48. Reproduced
    on a normalised scale where 1.0 is ``max_speed``.
    """

    name = "random_setpoints"

    def __init__(
        self,
        max_speed: float = DEFAULT_MAX_SPEED,
        seed: int = 1,
        hold_s: float = 10.0,
        smoothing_s: float = 10.0,
        dead_band: float = 0.48,
    ) -> None:
        Strategy.__init__(self, max_speed, seed)
        self.hold_s = hold_s
        self.smoothing_s = smoothing_s
        self.dead_band = dead_band
        self.speed = [0.5, 0.5]
        self.setpoint = [0.5, 0.5]
        self.next_change = 0.0

    def _drive(self, speed: float) -> float:
        value = math.sin(2.0 * math.pi * speed)
        if abs(value) < self.dead_band:
            value = self.dead_band if value >= 0.0 else -self.dead_band
        return value

    def step(self, t: float, dt: float, estimate: "OrientationEstimate") -> "tuple[float, float]":
        if t >= self.next_change:
            self.next_change = t + self.hold_s
            self.setpoint = [self.rng.random(), self.rng.random()]
        k = min(1.0, dt / self.smoothing_s)
        for i in (0, 1):
            self.speed[i] += k * (self.setpoint[i] - self.speed[i])
        return (
            self.max_speed * self._drive(self.speed[0]),
            self.max_speed * self._drive(self.speed[1]),
        )


class Clinostat(Strategy):
    """Classic two-axis clinostat: constant speeds with an irrational ratio.

    Gravity averages out for every speed ratio except |ratio| = 1: then the
    inner axis keeps a fixed phase to the outer one and 0.5 g remains.
    The default 1/phi^2 = 0.382 is irrational (the path never closes) and
    stays below 0.6 even if the two servos err by 20 % in opposite directions.

    A constant outer speed lets gravity linger along the inner (platform) axis,
    so coverage is not uniform (about 2x share there); see docs/adr/0002.
    """

    name = "clinostat"

    def __init__(
        self,
        max_speed: float = DEFAULT_MAX_SPEED,
        seed: int = 1,
        ratio: float = 0.3819660113,
    ) -> None:
        Strategy.__init__(self, max_speed, seed)
        self.ratio = ratio

    def step(self, t: float, dt: float, estimate: "OrientationEstimate") -> "tuple[float, float]":
        return (self.max_speed, self.max_speed * self.ratio)


class RandomWalk(Strategy):
    """Each axis independently: random speed and direction, random hold time.

    New targets come after exponentially distributed holds (mean ``hold_s``);
    speeds ramp with at most ``accel`` deg/s^2 to spare the plastic gears.
    """

    name = "random_walk"

    def __init__(
        self,
        max_speed: float = DEFAULT_MAX_SPEED,
        seed: int = 1,
        hold_s: float = 6.0,
        min_fraction: float = 0.3,
        accel: float = 60.0,
    ) -> None:
        Strategy.__init__(self, max_speed, seed)
        self.hold_s = hold_s
        self.min_fraction = min_fraction
        self.accel = accel
        self.target = [0.0, 0.0]
        self.current = [0.0, 0.0]
        self.next_change = [0.0, 0.0]

    def step(self, t: float, dt: float, estimate: "OrientationEstimate") -> "tuple[float, float]":
        max_delta = self.accel * dt
        for i in (0, 1):
            if t >= self.next_change[i]:
                self.next_change[i] = t + self.rng.exponential(self.hold_s)
                magnitude = self.rng.uniform(self.min_fraction, 1.0) * self.max_speed
                self.target[i] = self.rng.sign() * magnitude
            delta = self.target[i] - self.current[i]
            delta = max(-max_delta, min(max_delta, delta))
            self.current[i] += delta
        return (self.current[0], self.current[1])


STRATEGIES = {
    RandomSetpoints.name: RandomSetpoints,
    Clinostat.name: Clinostat,
    RandomWalk.name: RandomWalk,
}


def make_strategy(name: str, max_speed: float = DEFAULT_MAX_SPEED, seed: int = 1) -> "Strategy":
    """Create a strategy by its ``name`` (see ``STRATEGIES``)."""
    if name not in STRATEGIES:
        raise ValueError("unknown strategy %r, choose from %s" % (name, sorted(STRATEGIES)))
    return STRATEGIES[name](max_speed=max_speed, seed=seed)
