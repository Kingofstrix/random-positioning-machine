"""How good is the simulated microgravity? Streaming metrics over a run.

Two numbers decide (the acceptance criteria are in simulation/jannik/docs/api.md):

- **Residual gravity**: the norm of the time-averaged gravity vector in the
  sample frame, in g. Perfect averaging gives 0.
- **Coverage**: the sphere of gravity directions is split into 20 congruent
  cells (the faces of an icosahedron). Each cell should get 1/20 of the time.
  ``coverage_ratios`` reports each cell's share divided by 1/20, so a perfectly
  uniform run gives 1.0 everywhere.

Everything is incremental and allocation free per sample, so the same code
runs on the Pico (where the controller tracks its own estimate) and in the
simulation (which tracks the true orientation).
"""

import math

N_CELLS = 20

_PHI = (1.0 + math.sqrt(5.0)) / 2.0
_INV_PHI = 1.0 / _PHI


def _icosahedron_face_centres() -> "list[tuple[float, float, float]]":
    # The 20 face centres of an icosahedron are the 20 vertices of the dual
    # dodecahedron: (+-1, +-1, +-1), (0, +-1/phi, +-phi) and cyclic shifts.
    raw = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                raw.append((sx, sy, sz))
    for a in (-_INV_PHI, _INV_PHI):
        for b in (-_PHI, _PHI):
            raw.append((0.0, a, b))
            raw.append((a, b, 0.0))
            raw.append((b, 0.0, a))
    norm = math.sqrt(3.0)
    return [(x / norm, y / norm, z / norm) for (x, y, z) in raw]


CELL_CENTRES = _icosahedron_face_centres()


def cell_index(g: "tuple[float, float, float]") -> int:
    """Index of the coverage cell that contains direction ``g``."""
    gx, gy, gz = g
    best = 0
    best_dot = -2.0
    for i, (cx, cy, cz) in enumerate(CELL_CENTRES):
        dot = gx * cx + gy * cy + gz * cz
        if dot > best_dot:
            best_dot = dot
            best = i
    return best


class GravityStats:
    """Time-weighted accumulator for gravity samples in the sample frame.

    The Pico has 32-bit floats: adding 0.1 s to a plain running sum loses
    about a second per hour. Time and vector sums therefore use compensated
    (Kahan) summation; the coverage cells, which only feed a coarse ratio, do not.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.total_time = 0.0
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_z = 0.0
        self._carry = [0.0, 0.0, 0.0, 0.0]  # lost low-order parts of the four sums
        self.cell_time = [0.0] * N_CELLS

    def add(self, g: "tuple[float, float, float]", dt: float) -> None:
        """Record that gravity pointed along ``g`` for ``dt`` seconds."""
        carry = self._carry
        y = dt - carry[0]
        s = self.total_time + y
        carry[0] = (s - self.total_time) - y
        self.total_time = s
        y = g[0] * dt - carry[1]
        s = self.sum_x + y
        carry[1] = (s - self.sum_x) - y
        self.sum_x = s
        y = g[1] * dt - carry[2]
        s = self.sum_y + y
        carry[2] = (s - self.sum_y) - y
        self.sum_y = s
        y = g[2] * dt - carry[3]
        s = self.sum_z + y
        carry[3] = (s - self.sum_z) - y
        self.sum_z = s
        self.cell_time[cell_index(g)] += dt

    def mean_vector(self) -> "tuple[float, float, float]":
        """Time-averaged gravity vector in g (zero vector before any sample)."""
        if self.total_time <= 0.0:
            return (0.0, 0.0, 0.0)
        t = self.total_time
        return (self.sum_x / t, self.sum_y / t, self.sum_z / t)

    def residual_gravity(self) -> float:
        """Norm of the time-averaged gravity vector, in g."""
        mx, my, mz = self.mean_vector()
        return math.sqrt(mx * mx + my * my + mz * mz)

    def coverage_ratios(self) -> "list[float]":
        """Time share of each cell divided by the uniform share 1/20."""
        if self.total_time <= 0.0:
            return [0.0] * N_CELLS
        scale = N_CELLS / self.total_time
        return [c * scale for c in self.cell_time]

    def coverage_range(self) -> "tuple[float, float]":
        """(min, max) of ``coverage_ratios``; (1, 1) means perfectly uniform."""
        ratios = self.coverage_ratios()
        return (min(ratios), max(ratios))
