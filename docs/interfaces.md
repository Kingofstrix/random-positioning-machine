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

The control page (`interface/`, run through its launcher) talks to the Arduino Nano (CH340, 115200
baud). It sends `start`, `stop`, `estop` and `status` (every 0.5 s) and reads `state`, `t` (seconds),
`outer_dps`, `inner_dps`, `gres` and `cov` from the `status` reply. Unknown keys are ignored.
Changing that reply format breaks the page; tag the Testing team.

Example: `ok state=RUNNING mode=random inner_dps=-21.00 outer_dps=32.00 inner_us=1441 outer_us=1590 t=12.345 gres=0.412 cov=0.370`

The `status` reply also carries `gres` and `cov` (3 decimals each). `gres` is the magnitude in g of
the time-averaged gravity vector in the platform frame since `start`. `cov` is the share (0..1) of
200 equal-area orientation bins visited since `start`. The Arduino estimates both from the
commanded, ramped speeds, as there is no sensor. `start` resets them to `gres=1.000 cov=0.000`. The
spec, reference code and test values are in `testing/gravity-estimate/`. Changing these fields
affects Testing (control page) and the author of the Arduino sketch.
