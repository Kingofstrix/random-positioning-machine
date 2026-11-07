"""Runs rpm.core under the real MicroPython interpreter (no pytest there).

    cd simulation/jannik
    MICROPYPATH=.:firmware:tests/fake_pico micropython tests/micropython_smoke.py

CI runs this so nothing in rpm/core can quietly depend on CPython-only
features, and boots firmware/main.py against tests/fake_pico/machine.py.
Reference values must match tests/test_rng_metrics.py and tests/test_strategies.py.
"""

import math
import os

import config  # firmware/config.py; firmware imports are unconditional on purpose:
import machine  # a firmware that cannot be imported must fail this test
import main as firmware

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

# Same seed, same motion as CPython (reference from tests/test_strategies.py),
# also on a single-precision build like the Pico's: timer events exactly, the
# sum of 72000 pulses to within 0.1 % of them being 1 us off.
REFERENCE_RUNS = {
    "clinostat": (0, 125856000),
    "random_setpoints": (92546640, 108226584),
    "random_walk": (3808759477, 108476452),
}
nominal = ServoModel.nominal()
for name in sorted(REFERENCE_RUNS):
    controller = Controller(make_strategy(name, 120.0, 1), nominal, nominal)
    strategy = controller.strategy
    events = pulse_sum = 0
    last = None
    for k in range(36000):  # one simulated hour
        pulse_o, pulse_i = controller.tick(0.1)
        pulse_sum += pulse_o + pulse_i
        timers = getattr(strategy, "next_change_ms", None)
        key = tuple(timers) if isinstance(timers, list) else timers
        if key != last:
            events = (events * 31 + k) & 0xFFFFFFFF
            last = key
    ref_events, ref_sum = REFERENCE_RUNS[name]
    assert events == ref_events, (name, events)
    assert abs(pulse_sum - ref_sum) <= 72, (name, pulse_sum - ref_sum)
    total = controller.estimate.stats.total_time
    assert abs(total - 3600.0) < 0.01, (name, total)  # compensated sum, also in float32
    print("ok", name, "1 h matches CPython: pulse sum off by", pulse_sum - ref_sum, "us")

print("micropython smoke test passed")

# Firmware loop against the fake machine module (tests/fake_pico/machine.py).
firmware.run(max_ticks=20)
outer_writes = machine.writes[config.OUTER_PIN]
assert len(outer_writes) == 20 + 2  # 20 ticks, then stop pulse and off
assert all(700_000 <= ns <= 2_300_000 for ns in outer_writes[:-1])
assert outer_writes[-1] == 0
print("firmware loop passed")

# A broken calibration must not stop the Pico at boot (review finding B1):
# the firmware falls back to the datasheet curve and runs.
bad = "tests/_bad_calibration.json"
with open(bad, "w") as f:
    f.write('{"outer": {"points": [[1000, -5], [1500, 3], [2000, 1]]}, "inner": {}}')
good_file = config.CALIBRATION_FILE
config.CALIBRATION_FILE = bad
try:
    firmware.run(max_ticks=5)
finally:
    config.CALIBRATION_FILE = good_file
    os.remove(bad)
assert len(machine.writes[config.OUTER_PIN]) == 5 + 2
print("broken calibration falls back to datasheet values")

# Pulses beyond the servo range never reach the PWM, whatever asks for them.
probe = firmware.ServoOutput(config.OUTER_PIN)
probe.write_us(2500)
probe.write_us(400)
assert machine.writes[config.OUTER_PIN] == [2_300_000, 700_000]

# A fault while stopping must not leave the other servo, or this one, powered.
machine.fail_on[config.OUTER_PIN] = 1_500_000  # the outer stop pulse fails
try:
    firmware.run(max_ticks=3)
    raise AssertionError("the injected fault should propagate")
except OSError:
    pass
machine.fail_on.clear()
assert machine.writes[config.OUTER_PIN][-1] == 0, machine.writes[config.OUTER_PIN]
assert machine.writes[config.INNER_PIN][-2:] == [1_500_000, 0]
print("servos switched off despite a fault during shutdown")
