import json
import math
from pathlib import Path

import numpy as np
import pytest

from rpm.core.kinematics import gravity_in_sample
from rpm.core.strategies import Clinostat
from rpm.sim.export import export, result_to_dict, round_sig, update_index, write_json
from rpm.sim.simulate import run

KEYS = {
    "strategy",
    "dt",
    "target_g",
    "params",
    "t",
    "theta_outer",
    "theta_inner",
    "gravity",
    "mean_g",
    "residual_g",
}


@pytest.fixture(scope="module")
def doc() -> dict:
    # Through JSON and back, exactly what the viewer reads.
    data = result_to_dict(run("random_walk", duration_s=30.0, seed=2))
    return json.loads(json.dumps(data, allow_nan=False))


def test_schema(doc: dict) -> None:
    assert set(doc) == KEYS
    assert doc["strategy"] == "random_walk"
    assert doc["dt"] == pytest.approx(0.1)
    assert doc["target_g"] == [0.0, 0.0, 0.0]
    assert doc["params"]["seed"] == 2
    assert doc["params"]["duration_s"] == pytest.approx(30.0)
    assert set(doc["params"]["outer_plant"]) == {"speed_error", "stop_offset_us", "noise", "lag_s"}


def test_lengths(doc: dict) -> None:
    n = 300
    for key in ("t", "theta_outer", "theta_inner", "residual_g"):
        assert len(doc[key]) == n, key
    for key in ("gravity", "mean_g"):
        assert np.asarray(doc[key]).shape == (n, 3), key
    assert doc["t"][0] == pytest.approx(0.1)
    assert doc["t"][-1] == pytest.approx(30.0)


def test_gravity_matches_angles(doc: dict) -> None:
    expected = [
        gravity_in_sample(o, i) for o, i in zip(doc["theta_outer"], doc["theta_inner"], strict=True)
    ]
    # Angles and vectors are each rounded to 5 significant digits.
    assert np.asarray(doc["gravity"]) == pytest.approx(np.asarray(expected), abs=2e-4)
    norms = np.linalg.norm(np.asarray(doc["gravity"]), axis=1)
    assert norms == pytest.approx(np.ones(len(norms)), abs=1e-4)


def test_residual_is_norm_of_mean(doc: dict) -> None:
    mean = np.asarray(doc["mean_g"])
    assert np.linalg.norm(mean, axis=1) == pytest.approx(
        np.asarray(doc["residual_g"]), rel=1e-3, abs=1e-6
    )


def test_mean_roughly_matches_tick_samples(doc: dict) -> None:
    # mean_g integrates the sub-stepped physics, the tick samples are coarser:
    # both must still agree on where the average ends up.
    tick_mean = np.asarray(doc["gravity"]).mean(axis=0)
    assert np.asarray(doc["mean_g"][-1]) == pytest.approx(tick_mean, abs=0.02)


def test_target_is_written() -> None:
    data = result_to_dict(run("clinostat", duration_s=1.0), target_g=(0.0, 0.0, -0.379))
    assert data["target_g"] == [0.0, 0.0, -0.379]
    with pytest.raises(ValueError):
        result_to_dict(run("clinostat", duration_s=1.0), target_g=(0.0, 1.0))


@pytest.mark.parametrize(
    ("value", "expected"),
    [(math.pi, 3.1416), (-0.000123456, -0.00012346), (1.2246e-16, 0.0), (600.0, 600.0)],
)
def test_round_sig(value: float, expected: float) -> None:
    assert round_sig(value) == expected


def test_round_sig_rejects_nan() -> None:
    with pytest.raises(ValueError):
        round_sig(float("nan"))


def test_export_writes_file_and_index(tmp_path: Path) -> None:
    out = export("clinostat", minutes=0.1, seed=3, speed_error=0.1, out=tmp_path / "c.json")
    data = json.loads(out.read_text())
    assert data["strategy"] == "clinostat"
    assert len(data["t"]) == 60
    assert data["params"]["speed_error"] == pytest.approx(0.1)
    assert json.loads((tmp_path / "index.json").read_text()) == {"runs": ["c.json"]}


def test_index_lists_runs_only(tmp_path: Path) -> None:
    write_json({"strategy": "b"}, tmp_path / "b.json")
    write_json({"strategy": "a"}, tmp_path / "a.json")
    index = json.loads(update_index(tmp_path).read_text())
    assert index == {"runs": ["a.json", "b.json"]}


def test_export_takes_target_from_strategy(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # Partial-gravity strategies carry target_g (in g); the export turns it into the vector.
    from rpm.sim import export as export_module

    def mars_like(name: str, max_speed: float, seed: int) -> Clinostat:
        strategy = Clinostat(max_speed, seed)
        strategy.target_g = 0.379  # type: ignore[attr-defined]
        return strategy

    monkeypatch.setattr(export_module, "make_strategy", mars_like)
    data = json.loads(export("clinostat", minutes=0.05, out=tmp_path / "m.json").read_text())
    assert data["target_g"] == [0.0, 0.0, -0.379]
