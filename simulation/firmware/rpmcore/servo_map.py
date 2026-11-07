"""Frame speed (deg/s) <-> servo pulse (us) via a per-direction calibration table.

The table is filled with real measurements (M7), so it is piecewise linear and separate for each
direction: continuous-rotation servos are rarely symmetric around neutral.
"""

import math


def _check_table(table, sign, name):
    if len(table) < 2:
        raise ValueError("%s: need at least 2 points" % name)
    if table[0][0] != 0:
        raise ValueError("%s: first point must be at 0 deg/s" % name)
    prev_dps = -1.0
    prev_mag = 0
    for point in table:
        dps, off = point[0], point[1]
        mag = off * sign
        # Written as `not >` so that NaN, which fails every comparison, is rejected too.
        if not dps > prev_dps:
            raise ValueError("%s: speeds must be ascending" % name)
        if not mag > prev_mag:
            raise ValueError("%s: offsets must grow away from neutral" % name)
        prev_dps = dps
        prev_mag = mag


def _interp(xs, ys, x):
    # Beyond the last point the servo is saturated: more input gives no more output.
    if x >= xs[-1]:
        return ys[-1]
    i = 1
    while xs[i] < x:
        i += 1
    x0, x1 = xs[i - 1], xs[i]
    y0, y1 = ys[i - 1], ys[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


class ServoMap:
    def __init__(self, cal):
        neutral, lo, hi = cal["neutral_us"], cal["min_us"], cal["max_us"]
        if not lo <= neutral <= hi:
            raise ValueError("neutral_us must lie within [min_us, max_us]")
        _check_table(cal["pos"], 1, "pos")
        _check_table(cal["neg"], -1, "neg")
        self.neutral_us = int(neutral)
        self.min_us = int(lo)
        self.max_us = int(hi)
        # Both directions are stored as magnitudes; the sign is applied at the end.
        self._dps = {}
        self._mag = {}
        for key, sign in (("pos", 1), ("neg", -1)):
            self._dps[key] = [float(p[0]) for p in cal[key]]
            self._mag[key] = [float(p[1] * sign) for p in cal[key]]

    def pulse_for(self, frame_dps):
        if frame_dps == 0:
            return self.neutral_us
        if frame_dps > 0:
            key, sign = "pos", 1
        else:
            key, sign = "neg", -1
        mags = self._mag[key]
        mag = _interp(self._dps[key], mags, abs(frame_dps))
        steps = int(math.floor(mag + 0.5))
        # The first table point is the last pulse that does not move the servo, so any
        # non-zero speed needs at least one us more, or it would stall in the deadband.
        edge = int(math.floor(mags[0] + 0.5))
        if steps <= edge:
            steps = edge + 1
        us = self.neutral_us + sign * steps
        if us < self.min_us:
            return self.min_us
        if us > self.max_us:
            return self.max_us
        return us

    def dps_for(self, pulse_us):
        off = pulse_us - self.neutral_us
        if off >= 0:
            key, sign = "pos", 1
        else:
            key, sign = "neg", -1
        mags = self._mag[key]
        mag = off * sign
        if mag <= mags[0]:
            return 0.0
        return sign * _interp(mags, self._dps[key], mag)

    def max_dps(self):
        return min(self._dps["pos"][-1], self._dps["neg"][-1])
