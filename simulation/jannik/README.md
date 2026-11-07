# Simulation – Jannik's implementation

Motion logic for the RPM, the Pico firmware that runs it, and a simulation that checks it against
measurable acceptance criteria. Author: Jannik Greiner (simulation team).

This is one of two independent implementations in `simulation/`, so the testing team can test and
compare both; the other one lives directly in `simulation/` (`rpmcore` + MuJoCo). Both use the
team conventions from [`docs/interfaces.md`](../../docs/interfaces.md): axes `outer`/`inner`,
speeds in deg/s, pulses in µs, GP15/GP16.

## Layout

| Path | What | Runs on |
|---|---|---|
| `rpm/core/` | shared core: everything that also runs on the Pico | Pico **and** PC |
| `tests/` | pytest suite + `micropython_smoke.py` for the real MicroPython interpreter | PC |

One core for both sides: the simulation tests exactly the code that drives the servos, and CI runs
that core under the real MicroPython interpreter.

## Checks

```bash
cd simulation/jannik && uv sync
uv run pytest                    # unit tests (~1 s)
uv run ruff check . && uv run ruff format --check .
uv run mypy
MICROPYPATH=. micropython tests/micropython_smoke.py
```

CI runs all of them on every PR that touches `simulation/jannik/`
([workflow](../../.github/workflows/simulation-jannik.yml)).

## Strategies

| Name | Motion | Use |
|---|---|---|
| `clinostat` | outer 120 deg/s, inner 0.382 × that, constant | default |
| `random_walk` | random speed and direction per axis, ramped | comparison |
| `random_setpoints` | original Core Electronics behaviour | baseline |
