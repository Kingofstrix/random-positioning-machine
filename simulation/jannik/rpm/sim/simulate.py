"""Run the shared controller against a model of the real machine.

The controller only knows its *assumed* servo models and dead-reckons the
orientation. The plant here plays the real servos, which may differ:

- ``speed_error``: the real speed is ``(1 + speed_error)`` times the model's,
- ``stop_offset_us``: the real dead band is shifted (trim pot not centred),
- ``noise``: random speed jitter, re-drawn every control tick,
- ``lag_s``: first-order response time of the real servo.

The true angles are integrated with ``physics_substeps`` steps per control
tick and give the *true* metrics, which the acceptance criteria judge.
"""

import itertools
import random
from dataclasses import dataclass, field

import numpy as np

from rpm.core.controller import RESPONSE_LAG_S, Controller, first_order, make_controller
from rpm.core.kinematics import DEG, gravity_in_sample, wrap_angle
from rpm.core.metrics import GravityStats
from rpm.core.servo import ServoModel
from rpm.core.strategies import DEFAULT_MAX_SPEED, Strategy

CONTROL_DT_S = 0.1  # the Pico firmware ticks every 100 ms


@dataclass(frozen=True)
class ServoPlant:
    """How a real drive deviates from the controller's ``ServoModel``."""

    speed_error: float = 0.0
    stop_offset_us: float = 0.0
    noise: float = 0.0
    lag_s: float = RESPONSE_LAG_S

    def true_speed(self, model: ServoModel, pulse_us: float, rng: random.Random) -> float:
        """Real axis speed in deg/s for a commanded pulse."""
        speed = model.speed_at(pulse_us - self.stop_offset_us) * (1.0 + self.speed_error)
        if self.noise and speed:
            speed *= 1.0 + rng.gauss(0.0, self.noise)
        return speed


def error_envelope() -> list[tuple[ServoPlant, ServoPlant]]:
    """Corners of the robustness envelope used by the acceptance criteria.

    Each drive errs by +-20 % in speed with its dead band shifted by 15 us,
    plus 3 % speed noise; all four sign combinations of the two drives.
    """
    corners = []
    for e_f, e_p in itertools.product((-0.2, 0.2), repeat=2):
        corners.append(
            (
                ServoPlant(speed_error=e_f, stop_offset_us=15.0 if e_f > 0 else -15.0, noise=0.03),
                ServoPlant(speed_error=e_p, stop_offset_us=-15.0 if e_p > 0 else 15.0, noise=0.03),
            )
        )
    return corners


@dataclass
class SimResult:
    """Outcome of one run. Arrays are sampled once per control tick."""

    strategy: str
    seed: int
    duration_s: float
    true_stats: GravityStats
    estimated_stats: GravityStats
    time_s: np.ndarray
    residual_g: np.ndarray  # running |mean gravity| (true) at each tick
    theta_outer: np.ndarray  # true angles, radians
    theta_inner: np.ndarray
    gravity: np.ndarray  # (n, 3) true gravity in the sample frame
    pulses_us: np.ndarray  # (n, 2) commanded pulses (outer, inner)
    mean_g: np.ndarray  # (n, 3) running mean gravity (true), residual_g is its norm
    params: dict = field(default_factory=dict)

    @property
    def residual_gravity(self) -> float:
        return self.true_stats.residual_gravity()

    @property
    def coverage_range(self) -> tuple[float, float]:
        return self.true_stats.coverage_range()


def run(
    strategy: str | Strategy,
    duration_s: float = 600.0,
    seed: int = 1,
    max_speed: float = DEFAULT_MAX_SPEED,
    outer_model: ServoModel | None = None,
    inner_model: ServoModel | None = None,
    outer_plant: ServoPlant | None = None,
    inner_plant: ServoPlant | None = None,
    dt: float = CONTROL_DT_S,
    physics_substeps: int = 10,
    initial_angles: tuple[float, float] = (0.0, 0.0),
) -> SimResult:
    """Simulate ``duration_s`` seconds of one strategy and return the metrics.

    ``strategy`` is a name from ``STRATEGIES`` or a ready ``Strategy``
    instance. A name is built exactly like the firmware does it
    (``make_controller``): with ``seed`` and ``max_speed`` in deg/s, capped at
    what both servo models reach.
    """
    outer_model = outer_model or ServoModel.nominal()
    inner_model = inner_model or ServoModel.nominal()
    outer_plant = outer_plant or ServoPlant()
    inner_plant = inner_plant or ServoPlant()
    if isinstance(strategy, str):
        controller = make_controller(strategy, max_speed, seed, outer_model, inner_model)
    else:
        controller = Controller(strategy, outer_model, inner_model)
    strategy = controller.strategy
    noise_rng = random.Random(seed * 7919 + 17)

    n = int(round(duration_s / dt))
    h = dt / physics_substeps
    th_o, th_i = initial_angles
    w_o = w_i = 0.0  # true speeds, rad/s
    true_stats = GravityStats()
    out_t = np.empty(n)
    out_res = np.empty(n)
    out_o = np.empty(n)
    out_i = np.empty(n)
    out_g = np.empty((n, 3))
    out_pulse = np.empty((n, 2))
    out_mean = np.empty((n, 3))

    for i in range(n):
        pulse_o, pulse_i = controller.tick(dt)
        target_o = outer_plant.true_speed(outer_model, pulse_o, noise_rng) * DEG
        target_i = inner_plant.true_speed(inner_model, pulse_i, noise_rng) * DEG
        for _ in range(physics_substeps):
            # Exact first-order response; gravity sampled at the temporal midpoint.
            _, mid_o = first_order(w_o, target_o, 0.5 * h, outer_plant.lag_s)
            _, mid_i = first_order(w_i, target_i, 0.5 * h, inner_plant.lag_s)
            true_stats.add(gravity_in_sample(th_o + mid_o, th_i + mid_i), h)
            w_o, step_o = first_order(w_o, target_o, h, outer_plant.lag_s)
            w_i, step_i = first_order(w_i, target_i, h, inner_plant.lag_s)
            th_o = wrap_angle(th_o + step_o)
            th_i = wrap_angle(th_i + step_i)
        out_t[i] = (i + 1) * dt
        out_res[i] = true_stats.residual_gravity()
        out_o[i] = th_o
        out_i[i] = th_i
        out_g[i] = gravity_in_sample(th_o, th_i)
        out_pulse[i] = (pulse_o, pulse_i)
        out_mean[i] = true_stats.mean_vector()

    return SimResult(
        strategy=strategy.name,
        seed=seed,
        duration_s=duration_s,
        true_stats=true_stats,
        estimated_stats=controller.estimate.stats,
        time_s=out_t,
        residual_g=out_res,
        theta_outer=out_o,
        theta_inner=out_i,
        gravity=out_g,
        pulses_us=out_pulse,
        mean_g=out_mean,
        params={
            "max_speed": strategy.max_speed,
            "dt": dt,
            "outer_plant": outer_plant,
            "inner_plant": inner_plant,
        },
    )
