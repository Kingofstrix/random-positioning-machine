# Random Positioning Machine (RPM)

A 3D-printed **Random Positioning Machine**: a two-axis gimbal that continuously reorients a sample along random paths so that the gravity vector averages out over time. This approximates microgravity (a *clinostat*-style device) for biology experiments such as plant growth or cell cultures.

Course project *Disciplined Vibe Coding*, DHBW Ravensburg.

## How it works

Two nested frames rotate on perpendicular axes, each driven by its own servo:

```
 outer frame  ── axis 1 ──►  rotates the whole assembly
   └─ inner frame ── axis 2 ──►  rotates the experiment platform
        └─ experiment platform  (sample)
```

Control software picks random speeds and directions for both axes so the sample's orientation relative to gravity covers the sphere evenly. A slip ring carries power and signals across the rotating joint, so the frames can turn without limit and without tangling cables.

## Repository structure

| Path | Content | Team |
|---|---|---|
| `rpm_printables/` | STL files, assembly guide (PDF), print and buy list | Hardware |
| `hardware/` | CAD, wiring, BOM *(planned)* | Hardware |
| `simulation/` | Control logic and motion models *(planned)* | Simulation |
| `testing/` | Test plans, scripts, results *(planned)* | Testing |
| `docs/` | Documentation, `interfaces.md` *(planned)* | Everyone |
| `.github/` | PR template, branch ruleset | – |

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

Parts to buy: 2 servos, 1 slip ring, 4 servo screws, a Raspberry Pi Pico, a breadboard, jumper wires, cable ties, and 4 M3x6 bolts for the stands.

Build with the [assembly instructions](rpm_printables/Assembly%20Instructions%20-%20Random%20Positioning%20Machine.pdf).

### Changes from the original design
- `01_outer_frame.stl`: four 3.0 mm corner holes added.
- `03_experiment_platform.stl`: collar replaced by an open frame with six M2.5 nut seats.
- All other parts are unchanged apart from renaming.

## Software

Planned stack: **C++ / Arduino** on the Raspberry Pi Pico, plus a **simulation** of the motion algorithm so it can be developed and checked without hardware. Control code of the original project: [CE-Random-Positioning-Machine](https://github.com/CoreElectronics/CE-Random-Positioning-Machine).

*Status: not implemented yet.*

## Contributing

Never push to `main`. Use branches named `<team>/<type>/<description>` and open a pull request with at least one approval. Details in [CONTRIBUTING.md](CONTRIBUTING.md).

## License and attribution

The printable parts are based on the design by **Core Electronics and Unconvengineering** ([source](https://github.com/CoreElectronics/CE-Random-Positioning-Machine)), licensed under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/). Modifications by Mark Schutera, DHBW Ravensburg. Non-commercial use only; derivatives must keep the same license. A license for our own code is not decided yet.
