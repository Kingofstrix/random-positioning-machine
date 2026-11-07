# Random Positioning Machine (RPM)

A 3D-printed **Random Positioning Machine**: a two-axis gimbal that continuously reorients a sample along random paths so that the gravity vector averages out over time. This approximates microgravity (a *clinostat*-style device). No concrete experiment is planned yet.

Course project *Disciplined Vibe Coding*, DHBW Ravensburg.

## How it works

A fixed outer frame holds two nested rotating parts on perpendicular axes, each driven by its own continuous-rotation servo through a 4:1 gear:

```
 outer frame (fixed)
   └─ inner frame    turns on the horizontal "outer" axis   (outer servo, GP15)
        └─ platform  turns on the "inner" axis               (inner servo, GP16, via slip ring)
             └─ sample
```

Both axes meet in one point, where the sample sits. The control software picks random speeds and directions for both axes, so the direction of gravity seen by the sample keeps changing and averages out over time. A slip ring carries power and the inner servo's signal across the rotating outer axis, so the frames can turn without limit and without tangling cables.

Axes, units and wiring shared between the teams: [`docs/interfaces.md`](docs/interfaces.md).

## Repository structure

| Path | Content | Team |
|---|---|---|
| `rpm_printables/` | STL files, assembly guide (PDF), print and buy list | Hardware |
| `hardware/` | CAD, wiring, BOM *(not created yet)* | Hardware |
| `simulation/` | Pico firmware (`firmware/`), MuJoCo simulation (`sim/`), pytest suite (`tests/`) | Simulation |
| `testing/` | Interactive test runner, test steps per topic, test reports | Testing |
| `interface/` | Web interface for experimenters, deployed to [GitHub Pages](https://kingofstrix.github.io/random-positioning-machine/) | Testing |
| `docs/` | Project plan (`PLAN.md`), team interfaces (`interfaces.md`) | Everyone |
| `CLAUDE.md` | Rules for AI agents working on the firmware and simulation | Everyone |
| `.github/` | PR template, branch ruleset, Pages deploy workflow | – |

## Hardware

Parts to print (see [`rpm_printables/README.txt`](rpm_printables/README.txt)):

| File | Qty |
|---|---|
| `01_outer_frame.stl` | 1 |
| `02_inner_frame.stl` | 1 |
| `03_experiment_platform.stl` | 1 |
| `04_spur_gear_large_outer.stl` | 1 |
| `05_spur_gear_large_inner.stl` | 1 |
| `06_spur_gear_small.stl` | 2 (one per servo) |
| `07_end_cap.stl` | 1 |
| `optional_stand/Stand.stl` | 2 (only for a free-standing machine) |

Parts to buy: 2 continuous-rotation servos (FS90R), 1 slip ring, 4 servo screws, a Raspberry Pi Pico, a breadboard, jumper wires, cable ties, and 4 M3x6 bolts for the stands.

The servos are powered from the Pico's VBUS. Two stalled servos draw up to 1.6 A, more than many USB ports deliver, so use a 5 V supply that can handle it.

Build with the [assembly instructions](rpm_printables/Assembly%20Instructions%20-%20Random%20Positioning%20Machine.pdf).

### Changes from the original design
- `01_outer_frame.stl`: four 3.0 mm corner holes added.
- `03_experiment_platform.stl`: collar replaced by an open frame with six M2.5 nut seats.
- All other parts are unchanged apart from renaming.

## Software

- **Firmware:** **MicroPython** on the Raspberry Pi Pico. The control logic in `simulation/firmware/rpmcore/` is pure Python without hardware imports. `simulation/firmware/main.py` (milestone M2, not written yet) will connect it to PWM and the USB serial console.
- **Simulation:** **MuJoCo** (Python 3.11+) with the real STL parts. It imports the same `rpmcore` code that runs on the Pico, so what is tested in the simulation is what gets flashed.
- **Modes:** `random` (main mode), `clino3d` and `clino2d` as references. Controlled over a line-based serial protocol (`start`, `stop`, `estop`, `status`, …), the same on the Pico and in the simulation viewer.

The control code is written from scratch by this team; it does not reuse the code of the original [CE-Random-Positioning-Machine](https://github.com/CoreElectronics/CE-Random-Positioning-Machine) project. Full plan with hardware facts, architecture and test plan: [`docs/PLAN.md`](docs/PLAN.md) (German).

### Status

Work follows the milestones in `docs/PLAN.md`, section 9.

| Milestone | Content | Status |
|---|---|---|
| M0 | Project scaffold under `simulation/` | done |
| M1 | Control logic: PRNG, servo mapping ✓ · motion profiles, controller · serial commands | in progress |
| M2 | `main.py` on the Pico, self-test | open |
| M3–M4 | MuJoCo model, servo model, actuators | open |
| M5–M6 | Simulation runs, metrics, viewer, reports | open |
| M7 | Hardware commissioning and calibration | open |

## Getting started

Simulation and firmware tests (in `simulation/`, details in [`simulation/README.md`](simulation/README.md)):

```sh
cd simulation
python3 -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"
pytest -m "not slow"
```

Manual test path for all teams (details in [`testing/README.md`](testing/README.md)):

```sh
pip install pyyaml
python testing/run_tests.py --topic hardware   # or software, test; no --topic runs all
```

## Contributing

Never push to `main`. Use branches named `<team>/<type>/<description>` and open a pull request with at least one approval. Details in [CONTRIBUTING.md](CONTRIBUTING.md), including the rules for working with AI agents. Agents working on `simulation/` also follow [CLAUDE.md](CLAUDE.md).

## License and attribution

The printable parts are based on the design by **Core Electronics and Unconvengineering** ([source](https://github.com/CoreElectronics/CE-Random-Positioning-Machine)), licensed under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/). Modifications by Mark Schutera, DHBW Ravensburg. Non-commercial use only; derivatives must keep the same license. A license for our own code is not decided yet.
