import pytest

from rpmcore.servo_map import ServoMap

DEFAULT = {
    "neutral_us": 1500, "min_us": 700, "max_us": 2300,
    "pos": [(0.0, 45), (125.0, 350)], "neg": [(0.0, -45), (125.0, -350)],
}
ASYM = {
    "neutral_us": 1510, "min_us": 700, "max_us": 2300,
    "pos": [(0.0, 40), (60.0, 150), (125.0, 360)],
    "neg": [(0.0, -50), (100.0, -300)],
}
def roundtrip_tol(cal):
    """Rounding to whole µs moves the speed by at most half a µs times the steepest slope."""
    slopes = [
        abs((b[0] - a[0]) / (b[1] - a[1]))
        for table in (cal["pos"], cal["neg"])
        for a, b in zip(table, table[1:])
    ]
    return 0.5 * max(slopes) + 1e-9


def grid(lo, hi, step):
    n = int(round((hi - lo) / step))
    return [lo + i * step for i in range(n + 1)]


@pytest.fixture
def sm():
    return ServoMap(DEFAULT)


def test_zero_is_exact_neutral(sm):
    assert sm.pulse_for(0.0) == 1500
    assert sm.pulse_for(-0.0) == 1500
    assert ServoMap(ASYM).pulse_for(0.0) == 1510


def test_pulse_is_int(sm):
    assert all(isinstance(sm.pulse_for(x), int) for x in (-50.3, 0.0, 0.7, 99.9))


@pytest.mark.parametrize("dps", [1e-12, 1e-6, 0.01, 0.1])
@pytest.mark.parametrize(
    "cal, first_pos, first_neg",
    # Exactly one us past each direction's own deadband edge, not more (over-drive) and
    # not the other direction's edge (asymmetric servos).
    [(DEFAULT, 1546, 1454), (ASYM, 1551, 1459)],
    ids=["default", "asym"],
)
def test_smallest_speeds_skip_deadband(cal, first_pos, first_neg, dps):
    sm = ServoMap(cal)
    assert sm.pulse_for(dps) == first_pos
    assert sm.pulse_for(-dps) == first_neg


def test_table_points_hit_exactly(sm):
    # Float32 on the Pico must not push an exact table point to the next µs.
    assert sm.pulse_for(125.0) == 1850
    assert sm.pulse_for(-125.0) == 1150


def test_monotonic(sm):
    pulses = [sm.pulse_for(x) for x in grid(-200.0, 200.0, 0.25)]
    assert pulses == sorted(pulses)


def test_beyond_table_holds_saturation(sm):
    # More pulse than the last calibrated point gives no more speed on a saturated servo.
    assert sm.pulse_for(1e6) == 1850
    assert sm.pulse_for(-1e6) == 1150


def test_clamped_to_pulse_limits():
    wide = dict(DEFAULT, pos=[(0.0, 45), (125.0, 1200)], neg=[(0.0, -45), (125.0, -1200)])
    sm = ServoMap(wide)
    assert sm.pulse_for(125.0) == 2300
    assert sm.pulse_for(-125.0) == 700
    assert all(700 <= sm.pulse_for(x) <= 2300 for x in grid(-300.0, 300.0, 1.0))


@pytest.mark.parametrize("cal", [DEFAULT, ASYM], ids=["default", "asym"])
def test_roundtrip(cal):
    sm = ServoMap(cal)
    tol = roundtrip_tol(cal)
    assert tol <= 0.3  # docs/PLAN.md 8.1
    for x in grid(-sm.max_dps(), sm.max_dps(), 0.37):
        if abs(x) < 1.0:
            continue
        assert sm.dps_for(sm.pulse_for(x)) == pytest.approx(x, abs=tol)


@pytest.mark.parametrize(
    "cal, lo, hi", [(DEFAULT, 1455, 1545), (ASYM, 1460, 1550)], ids=["default", "asym"]
)
def test_dps_for_deadband_is_zero(cal, lo, hi):
    sm = ServoMap(cal)
    for us in range(lo, hi + 1):
        assert sm.dps_for(us) == 0.0
    assert sm.dps_for(hi + 1) > 0.0
    assert sm.dps_for(lo - 1) < 0.0


def test_dps_for_saturates(sm):
    assert sm.dps_for(2300) == 125.0
    assert sm.dps_for(700) == -125.0


def test_asymmetric_tables_respected():
    sm = ServoMap(ASYM)
    assert sm.pulse_for(60.0) == 1510 + 150
    assert sm.pulse_for(-100.0) == 1510 - 300
    assert sm.pulse_for(30.0) == 1510 + 95
    assert sm.max_dps() == 100.0


def test_max_dps_is_smaller_direction(sm):
    assert sm.max_dps() == 125.0


def test_accepts_lists_instead_of_tuples():
    as_lists = dict(DEFAULT, pos=[[0.0, 45], [125.0, 350]], neg=[[0.0, -45], [125.0, -350]])
    assert ServoMap(as_lists).pulse_for(62.5) == ServoMap(DEFAULT).pulse_for(62.5)


@pytest.mark.parametrize(
    "change",
    [
        {"pos": [(0.0, 45)]},                                  # one point only
        {"pos": [(5.0, 45), (125.0, 350)]},                    # does not start at 0
        {"pos": [(0.0, 45), (125.0, 350), (100.0, 400)]},      # dps not ascending
        {"pos": [(0.0, 45), (125.0, 45)]},                     # offset not increasing
        {"pos": [(0.0, 45), (float("nan"), 350)]},             # NaN slips past <= checks
        {"pos": [(0.0, 45), (60.0, float("nan")), (125.0, 350)]},
        {"pos": [(0.0, -45), (125.0, 350)]},                   # wrong sign
        {"neg": [(0.0, -45), (125.0, -40)]},                   # neg offsets must decrease
        {"neg": [(0.0, 45), (125.0, 350)]},                    # neg table with + offsets
        {"neutral_us": 2400},                                  # neutral outside limits
        {"min_us": 2300, "max_us": 700},                       # limits swapped
    ],
)
def test_invalid_tables_rejected(change):
    with pytest.raises(ValueError):
        ServoMap(dict(DEFAULT, **change))
