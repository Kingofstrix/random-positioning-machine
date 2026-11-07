import itertools
import math

import numpy as np
import pytest

from rpm.core.metrics import CELL_CENTRES, N_CELLS, GravityStats, cell_index
from rpm.core.rng import Rng

# First outputs for seed 1. tests/micropython_smoke.py checks the same values,
# which proves the Pico and the PC produce identical random sequences.
SEED1_U32 = [270369, 67634689, 2647435461]


def test_rng_reference_sequence() -> None:
    rng = Rng(1)
    assert [rng.next_u32() for _ in range(3)] == SEED1_U32


def test_rng_is_reproducible_and_in_range() -> None:
    a, b = Rng(42), Rng(42)
    values = [a.random() for _ in range(5000)]
    assert values == [b.random() for _ in range(5000)]
    assert min(values) >= 0.0
    assert max(values) < 1.0
    assert np.mean(values) == pytest.approx(0.5, abs=0.02)


def test_rng_zero_seed_does_not_get_stuck() -> None:
    rng = Rng(0)
    assert len({rng.next_u32() for _ in range(10)}) == 10


def test_rng_exponential_mean() -> None:
    rng = Rng(3)
    assert np.mean([rng.exponential(6.0) for _ in range(20000)]) == pytest.approx(6.0, rel=0.05)


def test_cell_centres_form_a_regular_dodecahedron() -> None:
    centres = np.array(CELL_CENTRES)
    assert centres.shape == (N_CELLS, 3)
    assert np.linalg.norm(centres, axis=1) == pytest.approx(np.ones(N_CELLS))
    # Every centre has exactly three nearest neighbours at the same angle:
    # all 20 cells are congruent, hence equal in area.
    dots = centres @ centres.T
    np.fill_diagonal(dots, -2.0)
    nearest = np.sort(dots, axis=1)[:, -3:]
    assert nearest == pytest.approx(np.full((N_CELLS, 3), nearest[0, 0]))


def test_cell_index_of_each_centre_is_itself() -> None:
    for i, centre in enumerate(CELL_CENTRES):
        assert cell_index(centre) == i


def test_uniform_directions_fill_all_cells_equally() -> None:
    rng = np.random.default_rng(1)
    v = rng.normal(size=(100_000, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    stats = GravityStats()
    for g in v:
        stats.add(tuple(g), 0.01)
    lo, hi = stats.coverage_range()
    assert lo > 0.95
    assert hi < 1.05
    assert stats.residual_gravity() < 0.01


def test_stats_time_weighting() -> None:
    stats = GravityStats()
    stats.add((0.0, 0.0, -1.0), 3.0)
    stats.add((0.0, 0.0, 1.0), 1.0)
    assert stats.total_time == pytest.approx(4.0)
    assert stats.mean_vector() == pytest.approx((0.0, 0.0, -0.5))
    assert stats.residual_gravity() == pytest.approx(0.5)
    ratios = stats.coverage_ratios()
    assert sum(ratios) == pytest.approx(N_CELLS)
    assert sorted(ratios)[-2:] == pytest.approx([5.0, 15.0])


def test_empty_stats() -> None:
    stats = GravityStats()
    assert stats.residual_gravity() == 0.0
    assert stats.coverage_range() == (0.0, 0.0)


def test_opposite_pairs_cancel() -> None:
    stats = GravityStats()
    for x, y, z in itertools.product((-1.0, 1.0), repeat=3):
        stats.add((x / math.sqrt(3), y / math.sqrt(3), z / math.sqrt(3)), 1.0)
    assert stats.residual_gravity() == pytest.approx(0.0, abs=1e-12)
