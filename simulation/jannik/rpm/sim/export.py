"""Export a simulation run as JSON for the browser viewer (``viewer/index.html``).

Schema (one entry per 100 ms control tick in every list)::

    {"strategy": "clinostat", "dt": 0.1, "target_g": [0, 0, 0],
     "params": {...}, "t": [...], "theta_outer": [...], "theta_inner": [...],
     "gravity": [[gx, gy, gz], ...], "mean_g": [[mx, my, mz], ...], "residual_g": [...]}

- ``t`` in s, angles in rad (true orientation), vectors in g in the sample frame.
- ``mean_g`` is the running time average of gravity, ``residual_g`` its norm.
- ``target_g`` is the mean gravity the run aims for: zero for microgravity, e.g.
  ``[0, 0, -0.379]`` for Mars. The viewer shows ``|mean_g - target_g|``.

Floats are rounded to 5 significant digits so a 10-minute run stays well under 1 MB.
"""

import dataclasses
import json
import math
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from rpm.core.strategies import DEFAULT_MAX_SPEED, make_strategy
from rpm.sim.simulate import ServoPlant, SimResult, run

SIGNIFICANT_DIGITS = 5
ZERO_BELOW = 1e-9  # |x| below this is float noise (e.g. sin(pi)); written as 0
INDEX_FILE = "index.json"
PROJECT_DIR = Path(__file__).resolve().parents[2]  # simulation/jannik
DATA_DIR = PROJECT_DIR / "viewer" / "data"


def round_sig(x: float, digits: int = SIGNIFICANT_DIGITS) -> float:
    """Round ``x`` to ``digits`` significant digits (0 for float noise, inf/nan rejected)."""
    if not math.isfinite(x):
        raise ValueError(f"cannot export non-finite value {x!r}")
    if abs(x) < ZERO_BELOW:
        return 0.0
    return float(f"{x:.{digits}g}")


def _rounded(values: Iterable[float]) -> list[float]:
    return [round_sig(float(v)) for v in values]


def _rounded_rows(rows: np.ndarray) -> list[list[float]]:
    return [_rounded(row) for row in rows]


def _json_safe(value: Any) -> Any:
    """Turn ``SimResult.params`` entries (dataclasses, numpy scalars) into JSON types."""
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {k: _json_safe(v) for k, v in dataclasses.asdict(value).items()}
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, list | tuple):
        return [_json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def result_to_dict(
    result: SimResult,
    target_g: Sequence[float] = (0.0, 0.0, 0.0),
    extra_params: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build the viewer JSON document for one ``SimResult``."""
    if len(target_g) != 3:
        raise ValueError("target_g must have three components")
    params = _json_safe(result.params)
    dt = float(params.pop("dt"))
    lo, hi = result.coverage_range
    params.update(
        {
            "seed": result.seed,
            "duration_s": result.duration_s,
            "final_residual_g": round_sig(result.residual_gravity),
            "coverage_range": [round_sig(lo), round_sig(hi)],
        }
    )
    if extra_params:
        params.update(_json_safe(extra_params))
    return {
        "strategy": result.strategy,
        "dt": dt,
        "target_g": _rounded(target_g),
        "params": params,
        "t": _rounded(result.time_s),
        "theta_outer": _rounded(result.theta_outer),
        "theta_inner": _rounded(result.theta_inner),
        "gravity": _rounded_rows(result.gravity),
        "mean_g": _rounded_rows(result.mean_g),
        "residual_g": _rounded(result.residual_g),
    }


def write_json(data: dict[str, Any], path: Path) -> Path:
    """Write ``data`` compactly to ``path`` and refresh the folder's ``index.json``."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":"), allow_nan=False) + "\n")
    update_index(path.parent)
    return path


def update_index(folder: Path) -> Path:
    """List the exported runs in ``folder/index.json`` so the viewer can offer them."""
    runs = sorted(p.name for p in folder.glob("*.json") if p.name != INDEX_FILE)
    index = folder / INDEX_FILE
    index.write_text(json.dumps({"runs": runs}, indent=1) + "\n")
    return index


def export(
    strategy: str,
    minutes: float = 10.0,
    seed: int = 1,
    speed_error: float = 0.0,
    noise: float = 0.0,
    out: Path | None = None,
    max_speed: float = DEFAULT_MAX_SPEED,
) -> Path:
    """Simulate ``strategy`` for ``minutes`` and write the viewer JSON; returns its path.

    A strategy may aim for partial gravity through a ``target_g`` attribute (in g, meaning a
    target mean of ``(0, 0, -target_g)`` in the sample frame); without one the target is zero.
    """
    instance = make_strategy(strategy, max_speed, seed)
    target = (0.0, 0.0, -float(getattr(instance, "target_g", 0.0)))
    plant = ServoPlant(speed_error=speed_error, noise=noise)
    result = run(instance, minutes * 60.0, seed=seed, outer_plant=plant, inner_plant=plant)
    data = result_to_dict(result, target, {"speed_error": speed_error, "noise": noise})
    return write_json(data, out or DATA_DIR / f"{strategy}.json")
