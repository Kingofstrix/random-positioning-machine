"""Mapping between servo pulse width and the speed of the driven axis.

The FS90R is a continuous-rotation servo: the pulse width sets a *speed*, not
an angle, and nothing reports the actual position back. A ``ServoModel``
describes one drive (servo + 15:60 gear) as a monotone piecewise-linear curve
of points ``(pulse_us, axis_deg_s)``:

- ``axis_deg_s`` is the speed of the driven axis (outer or inner), not of the
  servo shaft, in degrees per second and signed by the right-hand rule of
  docs/interfaces.md. Gear ratio and mounting direction are therefore folded
  into the curve.
- Between the points with speed 0 around the stop pulse lies the dead band.

``ServoModel.nominal()`` uses datasheet values. ``calibration.json`` written
by ``firmware/calibrate.py`` replaces them with measured points per drive.
Pulse widths are whole microseconds, as written to the PWM.
"""

GEAR_RATIO = 4.0  # 15-tooth servo gear driving a 60-tooth axis gear

STOP_US = 1500  # FS90R datasheet: stop at 1500 us
DEAD_BAND_US = 45  # datasheet: 90 us dead band in total
FULL_SPEED_US = 500  # full speed at roughly 1000 / 2000 us
SERVO_RPM_NOMINAL = 100.0  # no-load at ~5 V; sources say 80-110 rpm
MIN_PULSE_US = 700  # datasheet pulse range
MAX_PULSE_US = 2300


class ServoModel:
    """Monotone pulse <-> axis speed curve for one drive."""

    def __init__(self, points: "list[tuple[float, float]]") -> None:
        pts = sorted((float(us), float(speed)) for us, speed in points)
        if len(pts) < 2:
            raise ValueError("need at least two calibration points")
        for (us0, _), (us1, _) in zip(pts, pts[1:]):
            if us1 <= us0:
                raise ValueError("pulse widths must be distinct")
        increasing = pts[-1][1] > pts[0][1]
        for (_, v0), (_, v1) in zip(pts, pts[1:]):
            if (increasing and v1 < v0) or (not increasing and v1 > v0):
                raise ValueError("speed must be monotone in pulse width")
        if pts[0][1] * pts[-1][1] >= 0.0:
            raise ValueError("curve must reach both directions")
        self.points = pts
        self._increasing = increasing
        zeros = [us for us, speed in pts if speed == 0.0]
        stop = (min(zeros) + max(zeros)) / 2.0 if zeros else self._interp_pulse(0.0)
        self.stop_us = int(round(stop))
        self.max_forward = max(pts[0][1], pts[-1][1])
        self.max_reverse = min(pts[0][1], pts[-1][1])

    @classmethod
    def nominal(
        cls, servo_rpm: float = SERVO_RPM_NOMINAL, direction: float = 1.0, stop_us: int = STOP_US
    ) -> "ServoModel":
        """Datasheet curve: linear outside the dead band, saturated beyond."""
        axis_deg_s = servo_rpm * 6.0 / GEAR_RATIO * direction  # 1 rpm = 6 deg/s
        return cls(
            [
                (stop_us - FULL_SPEED_US, -axis_deg_s),
                (stop_us - DEAD_BAND_US, 0.0),
                (stop_us + DEAD_BAND_US, 0.0),
                (stop_us + FULL_SPEED_US, axis_deg_s),
            ]
        )

    @classmethod
    def from_dict(cls, data: dict) -> "ServoModel":
        """Build from a calibration entry: ``{"points": [[us, deg_s], ...]}``."""
        return cls([(p[0], p[1]) for p in data["points"]])

    def to_dict(self) -> dict:
        return {"points": [[us, speed] for us, speed in self.points]}

    @property
    def max_speed(self) -> float:
        """Largest axis speed (deg/s) reachable in *both* directions."""
        return min(self.max_forward, -self.max_reverse)

    def speed_at(self, pulse_us: float) -> float:
        """Axis speed in deg/s for a pulse width (held constant outside the curve)."""
        pts = self.points
        if pulse_us <= pts[0][0]:
            return pts[0][1]
        if pulse_us >= pts[-1][0]:
            return pts[-1][1]
        for (us0, v0), (us1, v1) in zip(pts, pts[1:]):
            if pulse_us <= us1:
                return v0 + (v1 - v0) * (pulse_us - us0) / (us1 - us0)
        return pts[-1][1]  # unreachable, keeps type checkers happy

    def pulse_for(self, speed: float) -> int:
        """Whole-microsecond pulse for ``speed`` deg/s; clamped to the reachable range.

        Zero maps to the centre of the dead band so the drive really stops.
        """
        if speed == 0.0:
            return self.stop_us
        speed = max(self.max_reverse, min(self.max_forward, speed))
        return int(round(self._interp_pulse(speed)))

    def _interp_pulse(self, speed: float) -> float:
        # The curve is monotone, so a non-flat segment containing ``speed``
        # gives the unique pulse (flat segments, e.g. the dead band, are skipped).
        pts = self.points
        for (us0, v0), (us1, v1) in zip(pts, pts[1:]):
            lo, hi = (v0, v1) if v0 <= v1 else (v1, v0)
            if lo <= speed <= hi and v1 != v0:
                return us0 + (us1 - us0) * (speed - v0) / (v1 - v0)
        return pts[0][0] if (speed < 0.0) == self._increasing else pts[-1][0]
