import pytest

from rpm.core.controller import Controller, OrientationEstimate
from rpm.core.servo import ServoModel
from rpm.core.strategies import STRATEGIES, Clinostat, RandomSetpoints, make_strategy

DT = 0.1


MAX = 120.0  # deg/s


def drive(strategy_name: str, seconds: float = 300.0, seed: int = 1) -> list[tuple[float, float]]:
    strategy = make_strategy(strategy_name, max_speed=MAX, seed=seed)
    estimate = OrientationEstimate()
    out = []
    for i in range(int(seconds / DT)):
        rpm = strategy.step(i * DT, DT, estimate)
        estimate.advance(rpm[0], rpm[1], DT)
        out.append(rpm)
    return out


@pytest.mark.parametrize("name", sorted(STRATEGIES))
def test_speeds_stay_within_max_speed(name: str) -> None:
    for speed_o, speed_i in drive(name):
        assert abs(speed_o) <= MAX + 1e-9
        assert abs(speed_i) <= MAX + 1e-9


@pytest.mark.parametrize("name", sorted(STRATEGIES))
def test_same_seed_same_motion(name: str) -> None:
    assert drive(name, 60, seed=5) == drive(name, 60, seed=5)


@pytest.mark.parametrize("name", ["random_setpoints", "random_walk"])
def test_random_strategies_depend_on_seed(name: str) -> None:
    assert drive(name, 60, seed=1) != drive(name, 60, seed=2)


@pytest.mark.parametrize("name", ["random_setpoints", "random_walk"])
def test_random_strategies_use_both_directions(name: str) -> None:
    speeds = drive(name, 600)
    for axis in (0, 1):
        assert min(s[axis] for s in speeds) < 0.0 < max(s[axis] for s in speeds)


def test_baseline_never_drives_slower_than_its_dead_band() -> None:
    # Original firmware: |drive| >= 12 of amplitude 25.
    for speed_o, speed_i in drive("random_setpoints"):
        assert abs(speed_o) >= 0.48 * MAX - 1e-9
        assert abs(speed_i) >= 0.48 * MAX - 1e-9


def test_baseline_draws_new_setpoints_every_hold_period() -> None:
    strategy = RandomSetpoints(seed=1, hold_s=10.0)
    estimate = OrientationEstimate()
    seen = set()
    for i in range(int(35 / DT)):
        strategy.step(i * DT, DT, estimate)
        seen.add(tuple(strategy.setpoint))
    assert len(seen) == 4  # t = 0, 10, 20, 30


def test_clinostat_speed_ratio() -> None:
    speed_o, speed_i = Clinostat(max_speed=MAX).step(0.0, DT, OrientationEstimate())
    assert speed_o == pytest.approx(MAX)
    assert speed_i / speed_o == pytest.approx(0.381966, abs=1e-6)


def test_unknown_strategy_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown strategy"):
        make_strategy("warp_drive")


def test_max_speed_must_be_positive() -> None:
    with pytest.raises(ValueError):
        Clinostat(max_speed=0.0)


# One simulated hour per strategy: a hash of the ticks at which the timers
# fire and the sum of all pulses. tests/micropython_smoke.py checks the same
# reference on MicroPython, including a single-precision build like the
# Pico's: there the timers must match exactly and the pulse sum within a few
# microseconds (float smoothing may round a pulse differently now and then).
REFERENCE_RUNS = {
    "clinostat": (0, 125856000),
    "random_setpoints": (92546640, 108226584),
    "random_walk": (3808759477, 108476452),
}


def reference_run(name: str, ticks: int = 36000) -> tuple[int, int]:
    model = ServoModel.nominal()
    controller = Controller(make_strategy(name, 120.0, 1), model, model)
    strategy = controller.strategy
    events = pulse_sum = 0
    last = None
    for k in range(ticks):
        pulse_o, pulse_i = controller.tick(0.1)
        pulse_sum += pulse_o + pulse_i
        timers = getattr(strategy, "next_change_ms", None)
        key = tuple(timers) if isinstance(timers, list) else timers
        if key != last:
            events = (events * 31 + k) & 0xFFFFFFFF
            last = key
    return events, pulse_sum


@pytest.mark.parametrize("name", sorted(STRATEGIES))
def test_reference_run(name: str) -> None:
    assert reference_run(name) == REFERENCE_RUNS[name]


def test_random_walk_holds_are_whole_milliseconds() -> None:
    strategy = make_strategy("random_walk", MAX, 1)
    estimate = OrientationEstimate()
    for i in range(600):
        strategy.step(i * DT, DT, estimate)
        assert all(isinstance(t, int) for t in strategy.next_change_ms)
    assert strategy.elapsed_ms == 60000
