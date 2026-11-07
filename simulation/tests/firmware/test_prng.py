import pytest

from rpmcore.prng import XorShift32

N = 10_000


def take(rng, n, fn="next_u32"):
    return [getattr(rng, fn)() for _ in range(n)]


def test_matches_marsaglia_reference():
    # First outputs for the seed used in Marsaglia (2003), "Xorshift RNGs", section 3.
    rng = XorShift32(2463534242)
    assert take(rng, 3) == [723471715, 2497366906, 2064144800]


def test_seed_1_sequence_is_pinned():
    # tools/pico_selftest.py compares against these values on the real Pico.
    assert take(XorShift32(1), 5) == [270369, 67634689, 2647435461, 307599695, 2398689233]


def test_same_seed_same_sequence():
    assert take(XorShift32(42), 100) == take(XorShift32(42), 100)


def test_different_seeds_differ():
    assert take(XorShift32(1), 10) != take(XorShift32(2), 10)


def test_outputs_are_u32():
    assert all(0 < x <= 0xFFFFFFFF for x in take(XorShift32(7), N))


@pytest.mark.parametrize("seed", [0, 2**32, -(2**32)])
def test_seeds_that_mask_to_zero_still_work(seed):
    # xorshift is stuck at 0 forever if the state is ever 0.
    values = take(XorShift32(seed), 10)
    assert all(v != 0 for v in values)
    assert len(set(values)) == 10
    # Seed 0 maps to one fixed state, so `set seed 0` is reproducible like any other seed.
    assert values == take(XorShift32(0), 10)


def test_seed_is_masked_to_32_bit():
    assert take(XorShift32(1 + 2**32), 5) == take(XorShift32(1), 5)
    assert take(XorShift32(-1), 5) == take(XorShift32(0xFFFFFFFF), 5)


def test_reseed_restarts_sequence():
    rng = XorShift32(5)
    first = take(rng, 10)
    rng.seed(5)
    assert take(rng, 10) == first


def test_random_in_unit_interval():
    values = take(XorShift32(3), N, "random")
    assert all(0.0 <= v < 1.0 for v in values)


def test_random_is_exact_in_float32():
    # The Pico computes in float32. Values must be multiples of 2**-24 so that the Pico and
    # CPython produce the same numbers and random() can never round up to 1.0.
    for v in take(XorShift32(9), 1000, "random"):
        scaled = v * 16777216
        assert scaled == int(scaled)


def test_random_uses_top_24_bits():
    # Ties random() to the pinned u32 stream; the low bits of xorshift are the weakest.
    expected = [(u >> 8) / 2**24 for u in take(XorShift32(1), 5)]
    assert take(XorShift32(1), 5, "random") == expected


def test_random_mean():
    values = take(XorShift32(1), N, "random")
    assert sum(values) / N == pytest.approx(0.5, abs=0.02)


def test_uniform_range():
    rng = XorShift32(11)
    values = [rng.uniform(5.0, 20.0) for _ in range(N)]
    assert all(5.0 <= v < 20.0 for v in values)
    assert min(values) < 5.5 and max(values) > 19.5


def test_choice_sign_balanced():
    signs = take(XorShift32(13), N, "choice_sign")
    assert set(signs) == {-1, 1}
    assert abs(sum(signs)) < 0.05 * N
