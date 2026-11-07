"""End-to-end checks of controller + plant, including the acceptance criteria.

The criteria themselves are defined in docs/api.md. The testing team
owns the formal acceptance tests in testing/; these are the simulation team's
own guard rails so a change that breaks the criteria fails here first.
"""

import numpy as np
import pytest

from rpm.core.strategies import STRATEGIES, Clinostat
from rpm.sim.simulate import ServoPlant, error_envelope, run

RESIDUAL_LIMIT_G = 0.01
COVERAGE_MIN = 0.5
COVERAGE_MAX = 2.5


@pytest.mark.parametrize("name", sorted(STRATEGIES))
def test_exact_model_gives_exact_estimate(name: str) -> None:
    # Plant identical to the controller's model: dead reckoning must agree with
    # the truth up to integration error.
    result = run(name, duration_s=120.0, seed=3)
    assert result.estimated_stats.residual_gravity() == pytest.approx(
        result.residual_gravity, abs=2e-3
    )


def test_wrong_model_makes_the_estimate_drift() -> None:
    result = run("random_walk", duration_s=300.0, outer_plant=ServoPlant(speed_error=0.2))
    true_g = np.array(result.true_stats.mean_vector())
    est_g = np.array(result.estimated_stats.mean_vector())
    assert np.linalg.norm(true_g - est_g) > 1e-3


def test_result_arrays_are_consistent() -> None:
    result = run("clinostat", duration_s=10.0)
    assert result.time_s.shape == (100,)
    assert result.gravity.shape == (100, 3)
    assert result.time_s[-1] == pytest.approx(10.0)
    assert result.true_stats.total_time == pytest.approx(10.0)
    assert np.all((result.pulses_us >= 700) & (result.pulses_us <= 2300))


def test_equal_axis_speeds_do_not_average_out() -> None:
    # The one resonance of this gimbal: with |ratio| = 1 the mean stays at
    # 0.5 g. This is why the clinostat ratio keeps away from 1.
    assert run(Clinostat(ratio=1.0), duration_s=600.0).residual_gravity == pytest.approx(
        0.5, abs=0.02
    )


@pytest.mark.parametrize("plants", error_envelope())
@pytest.mark.parametrize("seed", range(1, 6))
def test_clinostat_meets_acceptance_criteria(
    plants: tuple[ServoPlant, ServoPlant], seed: int
) -> None:
    result = run(
        "clinostat", duration_s=600.0, seed=seed, outer_plant=plants[0], inner_plant=plants[1]
    )
    lo, hi = result.coverage_range
    assert result.residual_gravity < RESIDUAL_LIMIT_G
    assert lo >= COVERAGE_MIN
    assert hi <= COVERAGE_MAX


def test_plant_follows_the_first_order_lag_exactly() -> None:
    # One tick from rest: the true axis covers speed * (dt - lag * (1 - e^(-dt/lag))),
    # independent of the number of physics substeps (Codex review: was 4.79 deg
    # instead of 4.41 deg at 120 deg/s).
    import math

    from rpm.core.kinematics import DEG
    from rpm.core.servo import ServoModel

    model = ServoModel.nominal()
    for substeps in (1, 10, 50):
        result = run(Clinostat(), duration_s=0.1, physics_substeps=substeps)
        speed = model.speed_at(result.pulses_us[0, 0])
        expected = speed * (0.1 - 0.1 * (1.0 - math.exp(-1.0)))
        assert result.theta_outer[0] / DEG == pytest.approx(expected, rel=1e-9)


def test_slow_calibrated_servos_cap_the_strategy_like_the_firmware() -> None:
    # Codex review: with drives that only reach 45 deg/s the simulation used to
    # command 120 deg/s on both axes, saturating both at the same speed (ratio 1,
    # 0.5 g left), while the firmware caps max_speed to what the drives reach.
    from rpm.core.servo import ServoModel

    slow = ServoModel.nominal(servo_rpm=30.0)
    result = run("clinostat", duration_s=600.0, outer_model=slow, inner_model=slow)
    assert result.params["max_speed"] == pytest.approx(slow.max_speed)
    assert result.residual_gravity < RESIDUAL_LIMIT_G
