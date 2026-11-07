# gravity-estimate

Spec, reference code and test vectors for the `gres=` and `cov=` fields in the Arduino
sketch's `status` reply. Owner: Testing team. The sketch itself lives outside this repo.

| File | Content |
|---|---|
| `gravity_estimate.h` | Drop-in header for the sketch (no heap, no `String`, ~51 bytes RAM) |
| `gravity_ref.py` | Readable Python reference; `python gravity_ref.py` rewrites `vectors.csv` |
| `vectors.csv` | Expected `gres`/`cov` every 10 s for four scenarios, 20 ms tick |
| `test_gravity_ref.py` | pytest: math, bins, scenarios, and the header compiled on the host |

## What the two numbers mean
- **gres**: length of the time-averaged gravity vector in the sample frame since `start`, in g.
  1.000 right after `start`, near 0 once gravity has averaged out.
- **cov**: share of 200 equal-area orientation bins (10 bands in gy × 20 azimuth sectors) that
  gravity has pointed into since `start`, 0..1.

The UI treats the run as initialised at `gres < 0.05` and `cov >= 0.90` (provisional, PLAN.md §8.3).
With the clino3d speeds (outer 30 °/s, inner 18.54 °/s) the reference gets there after 117 s.

## Why it is only an estimate
There is no accelerometer. The angles are integrated from the *commanded* (ramped) speeds.
FS90R continuous servos have no position feedback, their real speed drifts with load,
voltage and temperature, and the start angle is assumed to be 0°/0°. Read the numbers as
"what the motion profile should achieve", not as a measurement.

## Integrate into the sketch
1. Copy `gravity_estimate.h` next to the `.ino` and `#include "gravity_estimate.h"`.
2. One global `GravityEstimate grav;`.
3. Call `grav.start()` in the `start` handler (resets the statistics, keeps the angles).
4. Every control tick, in every state:
   `grav.tick(outer_dps, inner_dps, dt_ms, state == RUNNING || state == STOPPING);`
   with the ramped speeds the servos really get.
5. In the status reply: `Serial.print(F(" gres=")); Serial.print(grav.gres(), 3);` and the same
   for `cov`. Arduino's `sprintf` has no `%f`; use `Serial.print(x, 3)` or `dtostrf`.

The example at the bottom of the header shows all of this.

## Check against the vectors
On the host (needs `numpy`, `pytest`, and a C++ compiler for the last test):

```sh
pytest testing/gravity-estimate
```

On the Nano: run each scenario from `vectors.csv` (speeds from `scenarios()` in
`gravity_ref.py`, fixed 20 ms tick, no ramp) and print `gres`/`cov` every 10 s. Expect
`gres` within ±0.005 and `cov` within ±0.02 of the file. Exact equality is not expected: the Nano
uses 32-bit floats, the reference 64-bit, and the angle drift moves a few samples across bin edges.

One detail beyond the contract: the bin is computed from the rounded components
(`round(g*1000)`), not the raw floats. This is the same formula on a 0.001 grid, but avoids
`atan2(+0, -0) = π` at the poles and makes both implementations agree at bin edges.
