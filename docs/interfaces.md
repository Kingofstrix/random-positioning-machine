# Interfaces between teams

Pins, units, data formats and coordinate frames shared by hardware, simulation and testing.
Changing anything here needs a PR approved by every affected team (see `CONTRIBUTING.md`).
Background and derivations: `docs/PLAN.md`, sections 2 and 5.

## Wiring (Raspberry Pi Pico)

From the assembly instructions, page 8. The original Core Electronics code swaps the two
signal pins; the values below are the ones the firmware uses.

| Signal | Pico pin | Cable |
|---|---|---|
| Outer servo signal | GP15 (pin 20) | orange |
| Inner servo signal (via slip ring) | GP16 (pin 21) | orange → yellow |
| 5 V | VBUS (pin 40) | red |
| GND | GND (pin 38) | brown → black |

Servo PWM: 50 Hz, pulse 700…2300 µs, 1500 µs = stop, > 1500 µs = CCW.

## Units

| Quantity | Unit | Where |
|---|---|---|
| Frame speed (setpoints, profiles) | °/s | firmware |
| Servo pulse width | µs (int) | firmware → PWM |
| Time per control tick | s (float `dt`) | firmware |
| Everything in MuJoCo | SI (m, kg, s, rad, N·m) | simulation |
| STL coordinates | mm, +Y = up | `rpm_printables/` |

## Axes and frames

- **outer** axis: inner frame turns inside the fixed outer frame. Horizontal, along STL X,
  through (0, 0, 2.5) mm.
- **inner** axis: platform turns inside the inner frame. Along STL Y through (0, 0, 2.5) mm,
  vertical when both angles are 0.
- Positive speed = positive rotation about the axis (right-hand rule).
- Gear ratio 4:1 on both axes (15-tooth pinion on the servo, 60-tooth gear on the frame).
- Simulation world frame: Z up, floor at z = 0. STL Y maps to world Z.

## Serial command protocol (USB, line based)

Defined in `docs/PLAN.md` section 5.6. Replies start with `ok` or `err`. The same protocol runs on
the Pico and in the simulation viewer.

The web interface (`interface/`) sends `start`, `stop`, `estop` and `status` (every 0.5 s) and reads
`state`, `inner_dps`, `outer_dps`, `inner_us`, `outer_us` and `t` (seconds) from the `status` reply.
Changing that reply format breaks the interface; tag the Testing team.
