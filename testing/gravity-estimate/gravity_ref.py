"""Reference implementation of the gres/cov gravity estimate for the Arduino sketch.

The sketch has no sensors, so it estimates the gravity the sample has seen from the
*commanded* frame speeds. This file is the readable spec: `gravity_estimate.h` must behave
the same, and `python gravity_ref.py` writes the test vectors (`vectors.csv`) to check it.

Integer parts (sums, counter, bit array) are reproduced exactly, including the int32 limit
of an Arduino `long`. Angles use Python floats (64 bit) while the Nano uses 32-bit floats,
so compare against the vectors with a small tolerance, not for equality.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

SAMPLE_MS = 100  # one sample every 100 ms while RUNNING or STOPPING
N_BANDS = 10  # equal-area bands in gy (Archimedes: area is linear in gy)
N_SECTORS = 20  # equal sectors in azimuth atan2(gz, gx)
N_BINS = N_BANDS * N_SECTORS  # 200 bins -> 25 bytes of bits
INT32_MAX = 2**31 - 1


def round_half_away(x: float) -> int:
    """Round like the C code `(long)(x >= 0 ? x + 0.5 : x - 0.5)`, not like Python's round()."""
    return int(x + 0.5) if x >= 0 else -int(-x + 0.5)


def gravity_in_sample(a_o_deg: float, a_i_deg: float) -> tuple[float, float, float]:
    """Unit gravity vector in the platform frame, STL coordinates (+Y up).

    g_sample = R^T * (0, -1, 0) with R = Rx(a_o) * Ry(a_i).
    """
    so, co = math.sin(math.radians(a_o_deg)), math.cos(math.radians(a_o_deg))
    si, ci = math.sin(math.radians(a_i_deg)), math.cos(math.radians(a_i_deg))
    return (-so * si, -co, so * ci)


def bin_index(ix: int, iy: int, iz: int) -> int:
    """Orientation bin 0..199 from the rounded components (units of 0.001 g).

    Using the integers instead of the floats gives the same result as the contract formula
    on the 0.001-quantised vector, but avoids -0.0 (atan2(+0, -0) = pi) and makes the
    Arduino and this reference agree at bin edges.
    """
    band = min(N_BANDS - 1, (iy + 1000) * N_BANDS // 2000)  # == floor((gy + 1) / 2 * 10)
    phi = math.atan2(iz, ix)  # atan2(0, 0) = 0 for integer zeros
    sector = min(N_SECTORS - 1, int((phi + math.pi) / (2 * math.pi) * N_SECTORS))
    return band * N_SECTORS + sector


class GravityEstimate:
    """Same API as the C header: start(), tick(), gres(), cov()."""

    def __init__(self) -> None:
        self.a_o = 0.0  # outer angle, degrees, [0, 360)
        self.a_i = 0.0  # inner angle, degrees, [0, 360)
        self.start()

    def start(self) -> None:
        """Reset the statistics (not the angles), called on the `start` command."""
        self.sx = self.sy = self.sz = 0
        self.n = 0
        self.ms_since_sample = 0
        self.bits = bytearray(N_BINS // 8)

    def tick(self, outer_dps: float, inner_dps: float, dt_ms: int, running: bool) -> None:
        """Call every control tick with the speeds the servos actually get (after ramping).

        `running` is True while the state is RUNNING or STOPPING. Angles are integrated
        always, so the estimate knows where the frames stopped.
        """
        self.a_o = math.fmod(self.a_o + outer_dps * dt_ms / 1000.0, 360.0)
        self.a_i = math.fmod(self.a_i + inner_dps * dt_ms / 1000.0, 360.0)
        if self.a_o < 0:
            self.a_o += 360.0
        if self.a_i < 0:
            self.a_i += 360.0
        if not running:
            return
        self.ms_since_sample += dt_ms
        while self.ms_since_sample >= SAMPLE_MS:
            self.ms_since_sample -= SAMPLE_MS
            self._sample()

    def _sample(self) -> None:
        gx, gy, gz = gravity_in_sample(self.a_o, self.a_i)
        ix, iy, iz = round_half_away(gx * 1000), round_half_away(gy * 1000), round_half_away(gz * 1000)
        self.sx += ix
        self.sy += iy
        self.sz += iz
        self.n += 1
        for s in (self.sx, self.sy, self.sz):
            assert -INT32_MAX <= s <= INT32_MAX, "int32 overflow (run longer than ~59 h)"
        b = bin_index(ix, iy, iz)
        self.bits[b >> 3] |= 1 << (b & 7)

    def gres(self) -> float:
        """|time-averaged gravity| in g. 1.000 before the first sample."""
        if self.n == 0:
            return 1.0
        return math.sqrt(self.sx**2 + self.sy**2 + self.sz**2) / (1000.0 * self.n)

    def cov(self) -> float:
        """Share of the 200 orientation bins visited since start, 0..1."""
        return sum(bin(b).count("1") for b in self.bits) / N_BINS


TICK_MS = 20
VECTOR_STEP_S = 10


def scenarios():
    """(name, duration_s, speeds(t_ms) -> (outer_dps, inner_dps), reset_at_s or None)."""
    return [
        ("still", 60, lambda t: (0.0, 0.0), None),
        ("inner_only", 60, lambda t: (0.0, 30.0), None),
        ("clino3d", 600, lambda t: (30.0, 18.54), None),
        ("clino3d_reset", 300, lambda t: (30.0, 18.54), 150),
    ]


def run_scenario(duration_s, speeds, reset_at_s=None, step_s=VECTOR_STEP_S):
    """Yield (t_s, gres, cov) every step_s seconds, starting at t=0 right after start()."""
    g = GravityEstimate()
    g.start()
    yield 0, g.gres(), g.cov()
    for k in range(1, duration_s * 1000 // TICK_MS + 1):
        t_ms = k * TICK_MS
        o, i = speeds(t_ms)
        g.tick(o, i, TICK_MS, True)
        if reset_at_s is not None and t_ms == reset_at_s * 1000:
            g.start()  # `start` sent again mid-run: stats reset, angles kept
        if t_ms % (step_s * 1000) == 0:
            yield t_ms // 1000, g.gres(), g.cov()


def write_vectors(path: Path) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["scenario", "t_s", "gres", "cov"])
        for name, dur, speeds, reset in scenarios():
            for t, gr, cv in run_scenario(dur, speeds, reset):
                w.writerow([name, t, "%.3f" % gr, "%.3f" % cv])


if __name__ == "__main__":
    out = Path(__file__).with_name("vectors.csv")
    write_vectors(out)
    print("wrote", out)
