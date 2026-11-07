# API and acceptance criteria (for the testing team)

What `simulation/jannik/` promises, so tests can be written against it. Conventions (axis names,
zero pose, units, pins) are the team's, from [`docs/interfaces.md`](../../../docs/interfaces.md);
nothing here redefines them.

## Conventions used

- World frame: x along the horizontal **outer** axis, z up, gravity `(0, 0, -1)` g.
- `theta_outer`: inner frame inside the outer frame, about x. `theta_inner`: platform inside the
  inner frame, about the inner frame's z axis (vertical at 0/0). Radians, right-hand rule.
- Gravity in the sample frame: `g = (-sin(i)·sin(o), -cos(i)·sin(o), -cos(o))` for
  `o = theta_outer`, `i = theta_inner` (`R = Rx(o)·Rz(i)`, `g_s = Rᵀ·(0, 0, -1)`).
- Speeds: axis deg/s (after the 4:1 gear). Pulses: whole µs. Time: s.

## Python API

Everything in `rpm.core` also runs on the Pico (MicroPython). Install with `uv sync` in
`simulation/jannik/`.

```python
from rpm.core.kinematics import gravity_in_sample        # (theta_outer, theta_inner) -> (gx, gy, gz)
from rpm.core.metrics import GravityStats, CELL_CENTRES  # residual gravity and coverage of a run
from rpm.core.servo import ServoModel                    # pulse_us <-> axis deg/s of one drive
from rpm.core.strategies import STRATEGIES, make_strategy
from rpm.core.controller import Controller               # strategy + models + dead reckoning
from rpm.sim.simulate import run, ServoPlant, error_envelope

result = run("clinostat", duration_s=600, seed=1,
             outer_plant=ServoPlant(speed_error=0.2, stop_offset_us=15, noise=0.03))
result.residual_gravity     # float, g, true orientation
result.coverage_range       # (min, max) coverage ratio
result.true_stats           # GravityStats of the true orientation
result.estimated_stats      # GravityStats of what the Pico believes (dead reckoning)
result.time_s, result.gravity, result.pulses_us   # numpy arrays, one row per 100 ms tick
result.mean_g, result.residual_g                  # running mean gravity (n, 3) and its norm

from rpm.sim.export import result_to_dict          # the viewer's JSON, schema in rpm/sim/export.py
```

`ServoPlant` describes how a real servo deviates from the controller's model: `speed_error`
(0.2 = 20 % faster), `stop_offset_us` (dead band shifted), `noise` (relative, per tick),
`lag_s` (response time).

Strategies: `clinostat` (default on the Pico), `random_walk`, `random_setpoints` (the original Core
Electronics behaviour, kept as the baseline).

## Acceptance criteria

Measured on the **true** orientation over **600 s**:

1. **Residual gravity**: `|mean(g)| < 0.01 g`.
2. **Coverage**: gravity directions are binned into the 20 congruent faces of an icosahedron
   (`CELL_CENTRES`); each cell's time share divided by 1/20 lies in **[0.5, 2.5]**.

Both must hold for the default strategy in **every** run of the robustness envelope
(`error_envelope()`: each drive ±20 % speed, dead band shifted by ±15 µs, 3 % noise, all four sign
combinations) for seeds 1–5. `tests/test_simulation.py` checks this on every PR; the formal
acceptance tests belong to the testing team (issue #14).

Coverage is deliberately not required to be uniform: without position feedback no strategy can do
that, the geometry gives the directions along the inner axis about twice their share
([ADR 0002](adr/0002-clinostat-default-strategy.md)).

## Serial commands (Nano)

USB serial, 115200 baud, one command per line, as in `docs/PLAN.md` 5.6. Every reply is one line
starting with `ok` or `err`.

| Command | Effect |
|---|---|
| `help` | list of commands |
| `status` | `ok state=… mode=… outer_dps=… inner_dps=… outer_us=…\|off inner_us=…\|off t=<ms> residual_est=<g>` |
| `start [mode]` | start or restart with a fresh seed: same motion as in the simulation |
| `stop` | ramp both axes down at 60 °/s² (120 °/s: 2 s), then switch the servos off |
| `estop` | switch the servos off at once (no pulses) |
| `mode <name>` | strategy for the next start; restarts if running |
| `set speed <deg/s>` / `set seed <n>` | applies on the next start; speed is capped at what the servos reach |
| `pulse <outer\|inner> <us>` / `pulse off` | raw pulse 700–2300 µs for calibration, only when stopped |

Modes: `clinostat`, `random_walk`, `random_setpoints`; PLAN's names `clino3d` and `random` are
accepted as aliases. States: `STOPPED` (no pulses), `RUNNING`, `STOPPING`, `CAL`. While running it
prints `status` every 10 s on its own.

## Calibration file

On the Nano: `firmware/nano/rpm_nano/calibration.h`, written by `rpm-nano-calibrate` (same
points as below, as C arrays). On the legacy Pico: `calibration.json`, written by
`firmware/calibrate.py`:

```json
{
  "outer": {"points": [[1000, -129.0], [1460, 0.0], [1540, 0.0], [2000, 132.0]]},
  "inner": {"points": [[1000, 139.0], [1455, 0.0], [1545, 0.0], [2000, -134.0]]}
}
```

Each point is `[pulse_us, axis_deg_s]`, signed by the right-hand rule. Points with `0.0` mark the
dead band, the drive stops in its middle. Speeds must be monotone in pulse width and reach both
directions. Without the file, or if it is invalid, the firmware prints why and uses the PLAN 2.5
assumptions (83 rpm, about 125 deg/s at full speed); a bad file never stops the boot.
`calibrate.py` writes an invalid measurement to `calibration.rejected.json` instead, so it cannot
replace a good `calibration.json`. Pulses are always clamped to 700–2300 µs.
