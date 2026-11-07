"""The control loop shared by the Pico firmware and the simulation.

Every tick the controller

1. asks the motion strategy for target axis speeds (deg/s),
2. turns them into whole-microsecond pulses with the drives' ``ServoModel``,
3. advances its *estimate* of the orientation by dead reckoning, because the
   FS90R servos report no position (open loop, see docs/adr/0001).

The simulation feeds the returned pulses into a model of the real servos; the
firmware writes them to the PWM pins. Both run exactly this code.
"""

import math

from rpm.core.kinematics import DEG, gravity_in_sample, wrap_angle
from rpm.core.metrics import GravityStats

TYPE_CHECKING = False
if TYPE_CHECKING:
    from rpm.core.servo import ServoModel
    from rpm.core.strategies import Strategy

RESPONSE_LAG_S = 0.1  # assumed first-order speed response of an FS90R


def first_order(speed: float, target: float, t: float, lag_s: float) -> "tuple[float, float]":
    """Speed after ``t`` seconds of a first-order lag towards ``target``, and the distance.

    Exact solution, so the result does not depend on how long the step is.
    Distance is in units of ``speed`` times seconds.
    """
    if lag_s <= 0.0:
        return target, target * t
    decay = math.exp(-t / lag_s)
    return target + (speed - target) * decay, target * t + (speed - target) * lag_s * (1.0 - decay)


class OrientationEstimate:
    """Dead-reckoned orientation plus metrics of the *estimated* gravity.

    Without position feedback the error of this estimate grows with time; the
    simulation measures how much that matters (docs/adr/0001).
    """

    def __init__(self, lag_s: float = RESPONSE_LAG_S) -> None:
        self.lag_s = lag_s
        self.theta_outer = 0.0  # rad
        self.theta_inner = 0.0  # rad
        self.speed_outer = 0.0  # deg/s
        self.speed_inner = 0.0  # deg/s
        self.stats = GravityStats()

    def gravity(self) -> "tuple[float, float, float]":
        return gravity_in_sample(self.theta_outer, self.theta_inner)

    def advance(self, target_outer: float, target_inner: float, dt: float) -> None:
        """Move the estimate forward by ``dt`` seconds at the commanded speeds (deg/s)."""
        lag = self.lag_s
        # Gravity is sampled where the axes are halfway through the step in time.
        _, mid_o = first_order(self.speed_outer, target_outer, 0.5 * dt, lag)
        _, mid_i = first_order(self.speed_inner, target_inner, 0.5 * dt, lag)
        self.speed_outer, step_o = first_order(self.speed_outer, target_outer, dt, lag)
        self.speed_inner, step_i = first_order(self.speed_inner, target_inner, dt, lag)
        self.stats.add(
            gravity_in_sample(self.theta_outer + mid_o * DEG, self.theta_inner + mid_i * DEG), dt
        )
        self.theta_outer = wrap_angle(self.theta_outer + step_o * DEG)
        self.theta_inner = wrap_angle(self.theta_inner + step_i * DEG)


class Controller:
    """Strategy + servo models + estimate, one ``tick`` per control period."""

    def __init__(
        self,
        strategy: "Strategy",
        outer_model: "ServoModel",
        inner_model: "ServoModel",
        lag_s: float = RESPONSE_LAG_S,
    ) -> None:
        self.strategy = strategy
        self.outer_model = outer_model
        self.inner_model = inner_model
        self.estimate = OrientationEstimate(lag_s)
        self.ticks = 0
        self.time = 0.0

    def tick(self, dt: float) -> "tuple[int, int]":
        """Advance one control period; return (outer_pulse_us, inner_pulse_us)."""
        # Time from an int tick count, not a running float sum: the Pico's
        # 32-bit floats would drift by about a second per half hour.
        self.time = self.ticks * dt
        speed_o, speed_i = self.strategy.step(self.time, dt, self.estimate)
        pulse_o = self.outer_model.pulse_for(speed_o)
        pulse_i = self.inner_model.pulse_for(speed_i)
        # Estimate with what the model says these pulses really do (rounding,
        # clamping and the dead band included), not with what was asked for.
        self.estimate.advance(
            self.outer_model.speed_at(pulse_o), self.inner_model.speed_at(pulse_i), dt
        )
        self.ticks += 1
        return pulse_o, pulse_i
