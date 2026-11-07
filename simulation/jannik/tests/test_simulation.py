"""End-to-end checks of controller + plant, including the acceptance criteria.

The criteria themselves are defined in docs/interfaces.md. The testing team
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
