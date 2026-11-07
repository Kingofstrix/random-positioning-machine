# Interfaces between teams

Pins, units, data formats and coordinate frames shared by hardware, simulation and testing.
Changing anything here needs a PR approved by every affected team (see `CONTRIBUTING.md`).
Background and derivations: `docs/PLAN.md`, sections 2 and 5.

## Wiring (Arduino Nano)

The controller is an Arduino Nano (ATmega328P; CH340 clones work, usually with the "old
bootloader" board setting). It replaces the Raspberry Pi Pico of the assembly instructions, which
had the outer servo on GP15 and the inner servo on GP16.

| Signal | Nano pin | Cable |
|---|---|---|
| Outer servo signal | D9 | orange |
| Inner servo signal (via slip ring) | D10 | orange → yellow |
| Servo 5 V | external 5 V supply, ≥ 2 A (not the Nano's 5V pin) | red |
| GND | Nano GND **and** supply GND, connected | brown → black |

D9 and D10 are the two Timer1 outputs that the Arduino Servo library drives. Two stalled FS90R draw
up to 1.6 A, which neither USB nor the Nano's regulator can deliver, so the servos get their own
supply with a common ground.

Servo PWM: 50 Hz, pulse 700…2300 µs, 1500 µs = stop, > 1500 µs = CCW.
USB serial: 115200 baud, one command per line (newline).

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
the Nano and in the simulation viewer.
