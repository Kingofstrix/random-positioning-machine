"""Runs rpm.core under the real MicroPython interpreter (no pytest there).

    cd simulation && MICROPYPATH=. micropython tests/micropython_smoke.py

CI runs this so nothing in rpm/core can quietly depend on CPython-only
features. Reference values must match tests/test_rng_metrics.py.
"""

import math

from rpm.core.controller import Controller
from rpm.core.kinematics import gravity_in_sample
from rpm.core.metrics import CELL_CENTRES, N_CELLS, cell_index
from rpm.core.rng import Rng
from rpm.core.servo import ServoModel
from rpm.core.strategies import STRATEGIES, make_strategy

rng = Rng(1)
assert [rng.next_u32() for _ in range(3)] == [270369, 67634689, 2647435461]

g = gravity_in_sample(math.pi / 2, 0.0)
assert abs(g[1] + 1.0) < 1e-6
assert len(CELL_CENTRES) == N_CELLS
assert all(cell_index(c) == i for i, c in enumerate(CELL_CENTRES))

model = ServoModel.from_dict(ServoModel.nominal().to_dict())
assert abs(model.speed_at(model.pulse_for(60.0)) - 60.0) < 0.2

for name in sorted(STRATEGIES):
    controller = Controller(make_strategy(name, 120.0, 1), model, model)
    for _ in range(600):  # one simulated minute at 10 Hz
        pulse_o, pulse_i = controller.tick(0.1)
        assert 700 <= pulse_o <= 2300 and 700 <= pulse_i <= 2300
        assert isinstance(pulse_o, int) and isinstance(pulse_i, int)
    residual = controller.estimate.stats.residual_gravity()
    assert 0.0 <= residual <= 1.0
    print("ok", name, f"residual(est) {residual:.4f} g")

print("micropython smoke test passed")
