# sim

MuJoCo simulation of the RPM. It runs the controller from `firmware/rpmcore` and never keeps its
own copy of the control logic.

| Module | Content | Milestone |
|---|---|---|
| `geometry.py` | STL path and geometry constants from the printed parts | M0, M3 |
| `params.py`, `mjcf.py` | Physical parameters and model builder | M3 |
| `servo_model.py`, `machine.py` | FS90R model, actuators, stepping the physics | M4 |
| `run.py`, `metrics.py` | Simulation runs and evaluation | M5 |
| `viewer.py`, `report.py` | Interactive viewer, plots and video | M6 |
