# simulation

Control logic for the Random Positioning Machine and a MuJoCo simulation that runs the same logic.
Owner: simulation team. Plan and background: [`docs/PLAN.md`](../docs/PLAN.md), interfaces:
[`docs/interfaces.md`](../docs/interfaces.md).

| Folder | Content |
|---|---|
| `firmware/` | Copied 1:1 onto the Pico. `rpmcore/` is pure logic (MicroPython compatible), `main.py` the hardware adapter |
| `sim/` | MuJoCo model, servo model, simulation runs, metrics, viewer, reports |
| `tests/` | pytest suite for firmware logic and simulation |

The STL meshes are loaded from [`rpm_printables/`](../rpm_printables/), not copied.

## Setup

Python 3.11 or newer. All commands run in this folder.

```bash
python3 -m venv .venv
```

```bash
. .venv/bin/activate
```

(fish: `. .venv/bin/activate.fish`)

```bash
pip install -e ".[dev]"
```

## Tests

```bash
pytest -m "not slow"
```

Fast tests, under 30 s. Plain `pytest` also runs the long simulation runs.

## Simulation, flashing, calibration

Added with milestones M2 to M7 (see `docs/PLAN.md`, section 9).

Power note for later: two stalled servos draw up to 1.6 A from VBUS, more than many USB ports
deliver. Use a supply that can handle it.
