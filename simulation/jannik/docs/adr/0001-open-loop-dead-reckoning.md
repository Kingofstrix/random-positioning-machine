# ADR 0001: Open loop with dead reckoning, no position sensor

- Status: accepted
- Date: 2026-11-07
- Author: Jannik Greiner

## Context

The FS90R is a continuous-rotation servo: the pulse width sets a speed, and nothing reports the
angle back. The real speed differs from the datasheet (sources give 80–130 rpm), from servo to
servo, with supply voltage and load, and the stop point can be off by tens of µs.

## Decision

- The controller **dead-reckons** both angles from the commanded speeds (`OrientationEstimate`).
- Each drive has a **servo model**: datasheet values by default, replaced by measured points from
  `firmware/calibrate.py`. Commands are whole-µs pulse widths, not "degrees": the
  micropython-servo library used by the original code maps `write(90)` to 1472 µs, not to the
  FS90R's 1500 µs stop.
- **No sensor**, the machine stays as built.

## Alternatives considered

| Option | Result | Why not now |
|---|---|---|
| IMU on the platform (2 free slip-ring wires) | would measure gravity directly | more wiring and code, not needed for the residual-gravity goal |
| Index sensor on the outer frame (hall sensor + magnet, ~2 €) | first experiment (3 seeds, ±20 % servo error): an angle-aware strategy reached coverage 0.87–1.32 and < 0.01 g | team decision: keep the hardware as built |

The index-sensor result is recorded because it is the cheapest way to *uniform* coverage, should
an experiment ever need it.

## Consequences

- Strategies must not rely on the true angle. The default clinostat does not
  ([ADR 0002](0002-clinostat-default-strategy.md)).
- The simulation reports the estimate separately from the truth, so its drift is visible.
- Calibrating once per machine makes the estimate better, but the acceptance criteria hold even
  with an uncalibrated ±20 % error.
