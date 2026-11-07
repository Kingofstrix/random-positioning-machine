import numpy as np
import pytest

from rpm.core.servo import GEAR_RATIO, ServoModel


@pytest.fixture
def nominal() -> ServoModel:
    return ServoModel.nominal(servo_rpm=100.0)


def test_nominal_curve_shape(nominal: ServoModel) -> None:
    full = 100.0 * 6.0 / GEAR_RATIO  # 100 servo rpm -> 150 axis deg/s
    assert nominal.stop_us == 1500
    assert nominal.speed_at(1500) == 0.0
    assert nominal.speed_at(1460) == 0.0  # inside the +-45 us dead band
    assert nominal.speed_at(1540) == 0.0
    assert nominal.speed_at(2000) == pytest.approx(full)
    assert nominal.speed_at(1000) == pytest.approx(-full)
    assert nominal.speed_at(2300) == pytest.approx(full)  # saturated
    assert nominal.max_speed == pytest.approx(150.0)


def test_curve_is_monotone(nominal: ServoModel) -> None:
    speeds = [nominal.speed_at(us) for us in np.linspace(600, 2400, 500)]
    assert all(b >= a for a, b in zip(speeds, speeds[1:], strict=False))


@pytest.mark.parametrize("speed", [-150.0, -73.0, -10.0, 10.0, 42.0, 149.0])
def test_pulse_for_inverts_speed_at(nominal: ServoModel, speed: float) -> None:
    pulse = nominal.pulse_for(speed)
    assert isinstance(pulse, int)
    # Whole microseconds: off by at most half a microsecond's worth of speed.
    us_step = 150.0 / (500 - 45)
    assert nominal.speed_at(pulse) == pytest.approx(speed, abs=0.5 * us_step + 1e-9)


def test_zero_speed_is_the_dead_band_centre(nominal: ServoModel) -> None:
    assert nominal.pulse_for(0.0) == 1500


def test_requests_beyond_the_curve_are_clamped(nominal: ServoModel) -> None:
    assert nominal.speed_at(nominal.pulse_for(500.0)) == pytest.approx(150.0)
    assert nominal.speed_at(nominal.pulse_for(-500.0)) == pytest.approx(-150.0)


def test_reversed_mounting_flips_the_sign() -> None:
    model = ServoModel.nominal(direction=-1.0)
    assert model.speed_at(2000) < 0.0
    assert model.speed_at(model.pulse_for(60.0)) == pytest.approx(60.0, abs=0.2)
    assert model.pulse_for(60.0) < 1500


def test_calibration_round_trip() -> None:
    measured = {
        "points": [
            [1100, -120.0],
            [1300, -57.0],
            [1470, 0.0],
            [1530, 0.0],
            [1700, 48.6],
            [1900, 114.0],
        ]
    }
    model = ServoModel.from_dict(measured)
    assert model.stop_us == 1500
    assert model.max_speed == pytest.approx(114.0)
    assert ServoModel.from_dict(model.to_dict()).points == model.points
    assert model.speed_at(1600) == pytest.approx(48.6 * 70 / 170)


@pytest.mark.parametrize(
    "points",
    [
        [(1500, 0.0)],  # too few
        [(1000, -5.0), (1000, 5.0)],  # duplicate pulse
        [(1000, -5.0), (1500, 3.0), (2000, 1.0)],  # not monotone
        [(1000, 1.0), (2000, 5.0)],  # one direction only
    ],
)
def test_invalid_calibrations_are_rejected(points: list[tuple[float, float]]) -> None:
    with pytest.raises(ValueError):
        ServoModel(points)
