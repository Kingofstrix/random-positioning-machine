"""PC side of the Nano calibration (rpm/nano/calibrate.py)."""

import math
import re

import pytest

from rpm.nano.calibrate import calibration_header, measure, parse_seconds, speed_from_seconds

GOOD = [(1150, -120.0), (1455, 0.0), (1545, 0.0), (1850, 118.0)]


@pytest.mark.parametrize(
    ("text", "seconds"),
    [("4.8", 4.8), ("-5,1", -5.1), (" 0 ", 0.0), ("nan", None), ("inf", None), ("x", None)],
)
def test_parse_seconds(text: str, seconds: float | None) -> None:
    assert parse_seconds(text) == seconds


def test_speed_from_seconds() -> None:
    assert speed_from_seconds(3.0) == pytest.approx(120.0)
    assert speed_from_seconds(-4.0) == pytest.approx(-90.0)
    assert speed_from_seconds(0.0) == 0.0


def test_header_holds_both_curves() -> None:
    text = calibration_header(GOOD, list(reversed(GOOD)), "test run")
    assert "const int CAL_OUTER_N = 4;" in text
    assert "const int16_t CAL_INNER_US[] = {1150, 1455, 1545, 1850};" in text
    dps = re.search(r"CAL_OUTER_DPS\[\] = \{(.*)\};", text)
    assert dps is not None
    assert [float(v.rstrip("f")) for v in dps.group(1).split(", ")] == [-120.0, 0.0, 0.0, 118.0]
    assert '#define RPM_CALIBRATION "test run"' in text


@pytest.mark.parametrize(
    "bad",
    [
        [(1000, -5.0), (1500, 3.0), (2000, 1.0)],  # not monotone: a slipped stopwatch
        [(1000, -5.0), (1500, math.nan), (2000, 5.0)],
        [(2400, -1.0), (2500, 0.0), (2600, 1.0)],  # stop pulse outside 700..2300
        [(1000 + i, float(i - 8)) for i in range(17)],  # more points than the Nano holds
    ],
)
def test_invalid_curves_never_become_a_header(bad: list[tuple[int, float]]) -> None:
    with pytest.raises(ValueError):
        calibration_header(GOOD, bad, "x")


def test_measure_steps_through_the_pulses_and_always_ends_with_pulse_off() -> None:
    sent: list[str] = []
    answers = iter(["oops", "-3"] + ["0"] * 13 + ["3"])

    def send(line: str) -> str:
        sent.append(line)
        return "ok"

    points = measure(send, lambda prompt: next(answers), "outer")
    assert points[0] == (1000, -120.0)
    assert points[-1] == (2000, 120.0)
    assert sent[0] == "pulse outer 1000"
    assert sent[-1] == "pulse off"

    def refusing(line: str) -> str:
        sent.append(line)
        return "err stop first"

    sent.clear()
    with pytest.raises(RuntimeError):
        measure(refusing, lambda prompt: "1", "inner")
    assert sent[-1] == "pulse off"
