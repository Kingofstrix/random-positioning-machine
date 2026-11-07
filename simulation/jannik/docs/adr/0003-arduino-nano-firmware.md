# ADR 0003: Arduino Nano instead of the Pico, with a C++ port checked against Python

Status: accepted by Jannik (2026-11-07); the wiring change in `docs/interfaces.md` needs the other
teams' agreement.

## Context

The machine is controlled by an **Arduino Nano** clone (CH340 USB chip, so an ATmega328P: 16 MHz,
2 KB RAM, 32 KB flash, `double` is 32-bit `float`), not the Raspberry Pi Pico the plan assumed.
MicroPython does not run on it, so "the simulation tests exactly the code that drives the servos"
can no longer hold literally.

## Decision

- `rpm/core` (Python) stays the reference. The simulation, the acceptance criteria and the
  strategy comparison keep testing it.
- `firmware/nano/rpm_nano/` is a line-by-line C++ port of it, with no Arduino headers in
  `rpm_core.h` and `machine.h`, so the PC can compile it. It uses float only, no heap and no
  virtual classes.
- Equivalence is a test, not a promise. `tests/test_nano_core.py` compiles the port on the PC and
  compares it with the Python reference runs over one simulated hour per strategy:
  - timer events must be identical;
  - the pulse sum may differ by at most 72 µs over 72 000 pulses, for float rounding (measured:
    0, 0 and 6 µs);
  - the estimated residual must agree to 0.002 g.
- The size check makes sure the sketch fits: 61 % flash, 49 % RAM, ≥ 700 bytes kept for the stack.
- The sketch also implements the serial protocol of PLAN 5.6, so the testing team's steps
  (`testing/steps/`) work with it. Pins are D9 (outer) and D10 (inner), the two Timer1 outputs
  the Servo library uses.
- Calibration moves to the PC (`rpm-nano-calibrate`). It drives `pulse` over USB, validates the
  curve with the Python `ServoModel` and writes `calibration.h`.

## Consequences

- A change to `rpm/core` must be mirrored in `rpm_core.h`. The reference test fails otherwise,
  because the pulses no longer match.
- Option A (`rpmcore`, MicroPython) is affected the same way: it also needs a C++ firmware for the
  Nano.
- Not yet verified on real Nano hardware. avr-libc's `sinf`/`expf` may round differently from
  the PC's libm; the tolerances above leave room for that, but only a run on the machine can
  show it.
